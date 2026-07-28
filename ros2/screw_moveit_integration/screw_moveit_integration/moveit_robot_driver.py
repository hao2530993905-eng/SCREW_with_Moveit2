from __future__ import annotations

import math
import threading
import time
from typing import Iterable

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, WrenchStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

TRAJECTORY_CONTROLLERS = (
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
)
SERVO_CONTROLLER = "forward_position_controller"


def _as_six(values: Iterable[float], name: str) -> list[float]:
    result = np.asarray(values, dtype=float).reshape(-1).tolist()
    if len(result) != 6:
        raise ValueError(f"{name} must contain 6 values, got {len(result)}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains non-finite values")
    return result


def _rotation_vector_to_quaternion(rotation: Iterable[float]) -> list[float]:
    vector = np.asarray(rotation, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    axis = vector / angle
    scale = math.sin(angle / 2.0)
    return [
        float(axis[0] * scale),
        float(axis[1] * scale),
        float(axis[2] * scale),
        math.cos(angle / 2.0),
    ]


def _normalize_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    result = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(result))
    if norm < 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0])
    result /= norm
    if result[3] < 0.0:
        result = -result
    return result


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def _quaternion_to_rotation_vector(quaternion: Iterable[float]) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    vector_norm = float(np.linalg.norm(q[:3]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(q[3]))
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return q[:3] / vector_norm * angle


def _orientation_error(target: Iterable[float], current: Iterable[float]) -> np.ndarray:
    target_q = _normalize_quaternion(target)
    current_q = _normalize_quaternion(current)
    inverse_current = np.asarray(
        [-current_q[0], -current_q[1], -current_q[2], current_q[3]]
    )
    return _quaternion_to_rotation_vector(
        _quaternion_multiply(target_q, inverse_current)
    )


def _pose_message(values: Iterable[float]) -> Pose:
    x, y, z, rx, ry, rz = _as_six(values, "pose")
    qx, qy, qz, qw = _rotation_vector_to_quaternion([rx, ry, rz])
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class MoveItRobotDriver:
    """Drop-in robot adapter backed by MoveIt and ros2_control.

    Long approach motions are planned and executed through MoveGroup. Fine
    Cartesian motion is streamed through MoveIt Servo, which retains collision,
    singularity, and joint-limit checks while the screwdriver torque loop runs.
    """

    def __init__(
        self,
        host: str = "",
        *,
        group_name: str = "ur_manipulator",
        pose_frame: str = "base",
        planning_frame: str = "base_link",
        ee_link: str = "tool0",
        planning_time: float = 5.0,
        connect_timeout_s: float = 15.0,
        max_servo_linear_speed: float = 0.02,
        max_servo_angular_speed: float = 0.3,
    ):
        del host
        self.group_name = group_name
        self.pose_frame = pose_frame
        self.planning_frame = planning_frame
        self.ee_link = ee_link
        self.planning_time = planning_time
        self.connect_timeout_s = connect_timeout_s
        self.max_servo_linear_speed = max_servo_linear_speed
        self.max_servo_angular_speed = max_servo_angular_speed

        self.connected = False
        self._owns_rclpy = False
        self._node: Node | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._executor_thread: threading.Thread | None = None
        self._state_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._wrench = np.zeros(6)
        self._servo_active = False
        self._preferred_trajectory_controller = TRAJECTORY_CONTROLLERS[0]
        self._last_pose_sample: tuple[float, np.ndarray] | None = None

    def connect(self) -> bool:
        if self.connected:
            return True
        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self._node = Node("screw_moveit_robot_driver")
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self._node,
            spin_thread=False,
        )
        self._move_group = ActionClient(self._node, MoveGroup, "/move_action")
        self._ik_client = self._node.create_client(
            GetPositionIK,
            "/compute_ik",
        )
        self._list_controllers = self._node.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )
        self._switch_controller = self._node.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
        )
        self._start_servo = self._node.create_client(
            Trigger,
            "/servo_node/start_servo",
        )
        self._stop_servo_client = self._node.create_client(
            Trigger,
            "/servo_node/stop_servo",
        )
        self._twist_publisher = self._node.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )
        self._node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
        )
        self._node.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self._on_wrench,
            20,
        )

        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            name="screw-moveit-executor",
            daemon=True,
        )
        self._executor_thread.start()

        deadline = time.monotonic() + self.connect_timeout_s
        checks = (
            (self._move_group.wait_for_server, "MoveGroup action /move_action"),
            (self._ik_client.wait_for_service, "MoveIt service /compute_ik"),
            (
                self._list_controllers.wait_for_service,
                "controller manager list service",
            ),
            (
                self._switch_controller.wait_for_service,
                "controller manager switch service",
            ),
            (self._start_servo.wait_for_service, "MoveIt Servo start service"),
        )
        for wait_function, description in checks:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not wait_function(timeout_sec=remaining):
                self.close()
                raise RuntimeError(f"Timed out waiting for {description}")

        self._wait_for_robot_state(deadline)
        controllers = self._controller_states()
        for name in TRAJECTORY_CONTROLLERS:
            if controllers.get(name) == "active":
                self._preferred_trajectory_controller = name
                break
        self.connected = True
        self._node.get_logger().info(
            "MoveIt robot adapter connected; approach=MoveGroup, fine motion=Servo"
        )
        return True

    def _on_joint_state(self, message: JointState) -> None:
        with self._state_lock:
            self._joint_positions.update(
                dict(zip(message.name, message.position))
            )

    def _on_wrench(self, message: WrenchStamped) -> None:
        with self._state_lock:
            self._wrench = np.asarray([
                message.wrench.force.x,
                message.wrench.force.y,
                message.wrench.force.z,
                message.wrench.torque.x,
                message.wrench.torque.y,
                message.wrench.torque.z,
            ])

    def _wait_for_robot_state(self, deadline: float) -> None:
        while time.monotonic() < deadline:
            with self._state_lock:
                have_joints = all(
                    name in self._joint_positions for name in JOINT_NAMES
                )
            if have_joints and self._can_read_pose():
                return
            time.sleep(0.05)
        raise RuntimeError("Timed out waiting for /joint_states and robot TF")

    def _can_read_pose(self) -> bool:
        try:
            self._tf_buffer.lookup_transform(
                self.pose_frame,
                self.ee_link,
                Time(),
            )
            return True
        except Exception:
            return False

    @staticmethod
    def _wait_future(future, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            raise RuntimeError(f"Timed out waiting for {description}")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{description} failed: {exception}") from exception
        return future.result()

    def get_joint_positions(self) -> list[float]:
        self._require_connected()
        with self._state_lock:
            return [self._joint_positions[name] for name in JOINT_NAMES]

    def get_tcp_pose(self) -> list[float]:
        self._require_connected()
        transform = self._tf_buffer.lookup_transform(
            self.pose_frame,
            self.ee_link,
            Time(),
        ).transform
        quaternion = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        rotation = _quaternion_to_rotation_vector(quaternion)
        pose = np.asarray([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            rotation[0],
            rotation[1],
            rotation[2],
        ])
        with self._state_lock:
            self._last_pose_sample = (time.monotonic(), pose.copy())
        return pose.tolist()

    def get_tcp_speed(self) -> list[float]:
        self._require_connected()
        with self._state_lock:
            previous = self._last_pose_sample
        current_time = time.monotonic()
        current = np.asarray(self.get_tcp_pose())
        if previous is None:
            return [0.0] * 6
        dt = max(current_time - previous[0], 1e-4)
        return ((current - previous[1]) / dt).tolist()

    def get_tcp_force(self) -> list[float]:
        self._require_connected()
        with self._state_lock:
            return self._wrench.copy().tolist()

    def inverse_kinematics(
        self,
        pose: Iterable[float],
        qnear: Iterable[float] | None = None,
    ) -> list[float]:
        self._require_connected()
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self.pose_frame
        request.ik_request.pose_stamped.pose = _pose_message(pose)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        if qnear is not None:
            request.ik_request.robot_state.joint_state.name = JOINT_NAMES
            request.ik_request.robot_state.joint_state.position = _as_six(
                qnear,
                "qnear",
            )
        request.ik_request.robot_state.is_diff = True

        response = self._wait_future(
            self._ik_client.call_async(request),
            3.0,
            "MoveIt IK",
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt IK failed with code {response.error_code.val}"
            )
        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        return [solution[name] for name in JOINT_NAMES]

    def move_j(
        self,
        joints: Iterable[float],
        speed: float = 0.5,
        acceleration: float = 0.5,
        timeout: float = 20.0,
        joint_tolerance: float = 0.001,
    ) -> bool:
        self._require_connected()
        self._ensure_trajectory_controller()
        target = _as_six(joints, "joints")

        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=joint_tolerance,
                tolerance_below=joint_tolerance,
                weight=1.0,
            )
            for name, value in zip(JOINT_NAMES, target)
        ]
        return self._execute_move_group(
            constraints,
            speed,
            acceleration,
            timeout,
        )

    def move_to_pose(
        self,
        pose: Iterable[float],
        *,
        speed_scaling: float = 0.05,
        acceleration_scaling: float = 0.05,
        timeout: float = 20.0,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
    ) -> bool:
        self._require_connected()
        self._ensure_trajectory_controller()
        target_pose = _pose_message(pose)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]
        position = PositionConstraint()
        position.header.frame_id = self.pose_frame
        position.link_name = self.ee_link
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [target_pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.pose_frame
        orientation.link_name = self.ee_link
        orientation.orientation = target_pose.orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return self._execute_move_group(
            constraints,
            speed_scaling,
            acceleration_scaling,
            timeout,
        )

    def _execute_move_group(
        self,
        constraints: Constraints,
        speed: float,
        acceleration: float,
        timeout: float,
    ) -> bool:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = min(
            max(speed / math.pi, 0.01),
            1.0,
        )
        goal.request.max_acceleration_scaling_factor = min(
            max(acceleration / math.pi, 0.01),
            1.0,
        )
        goal.request.start_state = RobotState(is_diff=True)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        goal_handle = self._wait_future(
            self._move_group.send_goal_async(goal),
            min(timeout, self.planning_time + 5.0),
            "MoveGroup goal acceptance",
        )
        if not goal_handle.accepted:
            return False
        wrapped_result = self._wait_future(
            goal_handle.get_result_async(),
            timeout,
            "MoveGroup execution",
        )
        return wrapped_result.result.error_code.val == MoveItErrorCodes.SUCCESS

    def move_l(
        self,
        pose: Iterable[float],
        speed: float = 0.003,
        acceleration: float = 0.03,
        timeout: float = 20.0,
        position_tolerance: float = 0.001,
        rotation_tolerance: float = 0.01,
        cancel_event: threading.Event | None = None,
    ) -> bool:
        del acceleration
        self._prepare_servo()
        target = np.asarray(_as_six(pose, "pose"))
        target_quaternion = _rotation_vector_to_quaternion(target[3:])
        deadline = time.monotonic() + timeout
        period = 0.02

        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                self._publish_zero_twist()
                return False
            current = np.asarray(self.get_tcp_pose())
            current_quaternion = _rotation_vector_to_quaternion(current[3:])
            linear_error = target[:3] - current[:3]
            angular_error = _orientation_error(
                target_quaternion,
                current_quaternion,
            )
            if (
                np.linalg.norm(linear_error) <= position_tolerance
                and np.linalg.norm(angular_error) <= rotation_tolerance
            ):
                self._publish_zero_twist()
                return True
            self._publish_servo_error(
                linear_error,
                angular_error,
                linear_limit=speed,
                angular_limit=self.max_servo_angular_speed,
            )
            time.sleep(period)

        self._publish_zero_twist()
        return False

    def servo_l(
        self,
        pose: Iterable[float],
        dt: float = 0.02,
        speed: float = 0.0,
        acceleration: float = 0.0,
        lookahead_time: float = 0.1,
        gain: float = 300.0,
    ) -> bool:
        del speed, acceleration, lookahead_time, gain
        self._prepare_servo()
        target = np.asarray(_as_six(pose, "pose"))
        current = np.asarray(self.get_tcp_pose())
        linear_error = target[:3] - current[:3]
        angular_error = _orientation_error(
            _rotation_vector_to_quaternion(target[3:]),
            _rotation_vector_to_quaternion(current[3:]),
        )
        self._publish_servo_error(
            linear_error,
            angular_error,
            linear_limit=self.max_servo_linear_speed,
            angular_limit=self.max_servo_angular_speed,
            proportional_period=max(dt, 0.004),
        )
        return True

    def _publish_servo_error(
        self,
        linear_error: np.ndarray,
        angular_error: np.ndarray,
        *,
        linear_limit: float,
        angular_limit: float,
        proportional_period: float = 0.05,
    ) -> None:
        linear = linear_error / proportional_period
        angular = angular_error / proportional_period
        linear_norm = float(np.linalg.norm(linear))
        angular_norm = float(np.linalg.norm(angular))
        if linear_norm > linear_limit > 0.0:
            linear *= linear_limit / linear_norm
        if angular_norm > angular_limit > 0.0:
            angular *= angular_limit / angular_norm

        message = TwistStamped()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = self.pose_frame
        message.twist.linear.x = float(linear[0])
        message.twist.linear.y = float(linear[1])
        message.twist.linear.z = float(linear[2])
        message.twist.angular.x = float(angular[0])
        message.twist.angular.y = float(angular[1])
        message.twist.angular.z = float(angular[2])
        self._twist_publisher.publish(message)

    def _publish_zero_twist(self) -> None:
        if self._node is None:
            return
        for _ in range(4):
            message = TwistStamped()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.header.frame_id = self.pose_frame
            self._twist_publisher.publish(message)
            time.sleep(0.004)

    def ensure_control_script_ready(self, timeout_s: float = 2.0) -> None:
        del timeout_s
        self._prepare_servo()

    def _prepare_servo(self) -> None:
        if self._servo_active:
            return
        self._switch_motion_controller(SERVO_CONTROLLER)
        response = self._wait_future(
            self._start_servo.call_async(Trigger.Request()),
            3.0,
            "MoveIt Servo start",
        )
        if not response.success:
            raise RuntimeError(f"MoveIt Servo start failed: {response.message}")
        self._servo_active = True

    def _controller_states(self) -> dict[str, str]:
        response = self._wait_future(
            self._list_controllers.call_async(ListControllers.Request()),
            3.0,
            "controller list",
        )
        return {item.name: item.state for item in response.controller}

    def _switch_motion_controller(self, target: str) -> None:
        states = self._controller_states()
        if states.get(target) == "active":
            return
        deactivate = [
            name
            for name in (*TRAJECTORY_CONTROLLERS, SERVO_CONTROLLER)
            if name != target and states.get(name) == "active"
        ]
        if states.get(target) not in {"inactive", "active"}:
            raise RuntimeError(f"Controller {target} is not loaded")

        request = SwitchController.Request()
        request.activate_controllers = [target]
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 3
        response = self._wait_future(
            self._switch_controller.call_async(request),
            5.0,
            f"switch controller to {target}",
        )
        if not response.ok:
            raise RuntimeError(f"Failed switching controller to {target}")

    def _ensure_trajectory_controller(self) -> None:
        if self._servo_active:
            self.stop_servo()
        self._switch_motion_controller(self._preferred_trajectory_controller)

    def stop_servo(self) -> None:
        if self._node is None:
            return
        self._publish_zero_twist()
        if self._servo_active:
            try:
                self._wait_future(
                    self._stop_servo_client.call_async(Trigger.Request()),
                    2.0,
                    "MoveIt Servo stop",
                )
            finally:
                self._servo_active = False
        self._switch_motion_controller(self._preferred_trajectory_controller)

    def stop_linear_motion(self, acceleration: float = 0.5) -> None:
        del acceleration
        if self.connected:
            self.stop_servo()

    def stop_joint_motion(self, acceleration: float = 1.5) -> None:
        del acceleration
        # MoveGroup cancellation is handled by action timeout/cancel paths.
        if self._servo_active:
            self.stop_servo()

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("MoveItRobotDriver is not connected")

    def close(self) -> None:
        if self._node is None:
            return
        try:
            if self.connected and self._servo_active:
                self.stop_servo()
        except Exception:
            pass
        self.connected = False
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=2.0)
        self._node.destroy_node()
        self._node = None
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
        self._owns_rclpy = False

    def __enter__(self) -> "MoveItRobotDriver":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
