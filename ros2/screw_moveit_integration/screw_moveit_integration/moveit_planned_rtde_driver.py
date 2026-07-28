from __future__ import annotations

import math
import threading
import time
from bisect import bisect_right
from typing import Any, Iterable

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .moveit_robot_driver import JOINT_NAMES, _as_six, _pose_message


class MoveItPlannedRTDEDriver:
    """Plan collision-free approaches with MoveIt and execute them via ur_rtde.

    Only move_j is replaced. All Cartesian alignment, adaptive servoL insertion,
    stopL behavior, TCP/force feedback, and RTDE script recovery are delegated
    to the project's original URDriver.
    """

    def __init__(
        self,
        host: str,
        *,
        group_name: str = "ur_manipulator",
        pose_frame: str = "base",
        planning_frame: str = "base_link",
        ee_link: str = "tool0",
        planning_time: float = 5.0,
        connect_timeout_s: float = 20.0,
        trajectory_period_s: float = 0.008,
        rtde_driver: Any | None = None,
        **_: Any,
    ):
        if rtde_driver is None:
            from apps.screw_demo.ur_driver import URDriver

            rtde_driver = URDriver(
                host,
                connect_timeout_s=connect_timeout_s,
            )
        self._rtde = rtde_driver
        self.group_name = group_name
        self.pose_frame = pose_frame
        self.planning_frame = planning_frame
        self.ee_link = ee_link
        self.planning_time = planning_time
        self.connect_timeout_s = connect_timeout_s
        self.trajectory_period_s = trajectory_period_s

        self.connected = False
        self._owns_rclpy = False
        self._node: Node | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._executor_thread: threading.Thread | None = None
        self._state_thread: threading.Thread | None = None
        self._stop_state = threading.Event()
        self._rtde_lock = threading.RLock()

    @property
    def rtde_c(self):
        return self._rtde.rtde_c

    @property
    def rtde_r(self):
        return self._rtde.rtde_r

    def connect(self) -> bool:
        if self.connected:
            return True
        if not self._rtde.connect():
            return False

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        self._node = Node("screw_moveit_rtde_driver")
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._node)
        self._joint_publisher = self._node.create_publisher(
            JointState,
            "/joint_states",
            20,
        )
        self._move_group = ActionClient(self._node, MoveGroup, "/move_action")
        self._ik_client = self._node.create_client(
            GetPositionIK,
            "/compute_ik",
        )
        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            name="moveit-rtde-executor",
            daemon=True,
        )
        self._executor_thread.start()
        self._state_thread = threading.Thread(
            target=self._publish_robot_state_loop,
            name="moveit-rtde-joint-state",
            daemon=True,
        )
        self._state_thread.start()

        deadline = time.monotonic() + self.connect_timeout_s
        remaining = deadline - time.monotonic()
        if not self._move_group.wait_for_server(timeout_sec=max(remaining, 0.0)):
            self.close()
            raise RuntimeError("Timed out waiting for MoveGroup action /move_action")
        remaining = deadline - time.monotonic()
        if not self._ik_client.wait_for_service(timeout_sec=max(remaining, 0.0)):
            self.close()
            raise RuntimeError("Timed out waiting for MoveIt service /compute_ik")

        self.connected = True
        self._node.get_logger().info(
            "Hybrid motion ready: MoveIt-planned RTDE approach; "
            "original RTDE moveL/servoL insertion"
        )
        return True

    def _publish_robot_state_loop(self) -> None:
        while not self._stop_state.is_set():
            try:
                with self._rtde_lock:
                    positions = list(self._rtde.get_joint_positions())
                message = JointState()
                message.header.stamp = self._node.get_clock().now().to_msg()
                message.name = list(JOINT_NAMES)
                message.position = positions
                self._joint_publisher.publish(message)
            except Exception as exc:
                if self._node is not None:
                    self._node.get_logger().warning(
                        f"Could not publish RTDE joint state: {exc}"
                    )
            self._stop_state.wait(0.02)

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

    def inverse_kinematics(
        self,
        pose: Iterable[float],
        qnear: Iterable[float] | None = None,
    ) -> list[float]:
        self._require_connected()
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped.header.frame_id = self.pose_frame
        request.ik_request.pose_stamped.pose = _pose_message(pose)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        seed = list(qnear) if qnear is not None else self.get_joint_positions()
        request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.ik_request.robot_state.joint_state.position = seed

        response = self._wait_future(
            self._ik_client.call_async(request),
            4.0,
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

        start_positions = self.get_joint_positions()
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
        goal.request.start_state = RobotState()
        goal.request.start_state.joint_state.name = list(JOINT_NAMES)
        goal.request.start_state.joint_state.position = start_positions
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        goal_handle = self._wait_future(
            self._move_group.send_goal_async(goal),
            min(timeout, self.planning_time + 5.0),
            "MoveGroup plan acceptance",
        )
        if not goal_handle.accepted:
            return False
        wrapped = self._wait_future(
            goal_handle.get_result_async(),
            min(timeout, self.planning_time + 8.0),
            "MoveGroup planning",
        )
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False
        return self._execute_rtde_trajectory(
            result.planned_trajectory.joint_trajectory,
            timeout=timeout,
            acceleration=acceleration,
            joint_tolerance=joint_tolerance,
        )

    def _execute_rtde_trajectory(
        self,
        trajectory,
        *,
        timeout: float,
        acceleration: float,
        joint_tolerance: float,
    ) -> bool:
        if not trajectory.points:
            return False
        name_to_index = {
            name: index for index, name in enumerate(trajectory.joint_names)
        }
        indices = [name_to_index[name] for name in JOINT_NAMES]
        positions = np.asarray([
            [point.positions[index] for index in indices]
            for point in trajectory.points
        ])
        times = np.asarray([
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
            for point in trajectory.points
        ])
        if len(times) == 1 or times[-1] <= 0.0:
            times = np.asarray([0.0, max(self.trajectory_period_s, 0.1)])
            positions = np.vstack([self.get_joint_positions(), positions[-1]])
        total_time = float(times[-1])
        if total_time > timeout:
            raise RuntimeError(
                f"Planned approach duration {total_time:.2f}s exceeds "
                f"motion timeout {timeout:.2f}s"
            )

        control = self._rtde.rtde_c
        if control is None:
            raise RuntimeError("RTDE control interface is unavailable")
        period = self.trajectory_period_s
        started = time.perf_counter()
        try:
            while True:
                cycle_started = time.perf_counter()
                elapsed = min(cycle_started - started, total_time)
                upper = min(
                    max(bisect_right(times, elapsed), 1),
                    len(times) - 1,
                )
                lower = upper - 1
                span = max(times[upper] - times[lower], 1e-9)
                ratio = (elapsed - times[lower]) / span
                command = (
                    positions[lower]
                    + ratio * (positions[upper] - positions[lower])
                )
                with self._rtde_lock:
                    accepted = control.servoJ(
                        command.tolist(),
                        0.0,
                        0.0,
                        period,
                        0.1,
                        300.0,
                    )
                if not accepted:
                    raise RuntimeError("RTDE servoJ rejected the MoveIt trajectory")
                if elapsed >= total_time:
                    break
                remaining = period - (time.perf_counter() - cycle_started)
                if remaining > 0.0:
                    time.sleep(remaining)
        finally:
            with self._rtde_lock:
                control.servoStop(acceleration)

        actual = np.asarray(self.get_joint_positions())
        return bool(
            np.max(np.abs(actual - positions[-1]))
            <= max(joint_tolerance, 0.002)
        )

    def get_tcp_pose(self):
        with self._rtde_lock:
            return self._rtde.get_tcp_pose()

    def get_tcp_speed(self):
        with self._rtde_lock:
            return self._rtde.get_tcp_speed()

    def get_tcp_force(self):
        with self._rtde_lock:
            return self._rtde.get_tcp_force()

    def get_joint_positions(self):
        with self._rtde_lock:
            return self._rtde.get_joint_positions()

    def move_l(self, *args, **kwargs):
        with self._rtde_lock:
            return self._rtde.move_l(*args, **kwargs)

    def servo_l(self, *args, **kwargs):
        with self._rtde_lock:
            return self._rtde.servo_l(*args, **kwargs)

    def stop_servo(self, *args, **kwargs):
        with self._rtde_lock:
            return self._rtde.stop_servo(*args, **kwargs)

    def stop_linear_motion(self, *args, **kwargs):
        with self._rtde_lock:
            return self._rtde.stop_linear_motion(*args, **kwargs)

    def stop_joint_motion(self, *args, **kwargs):
        with self._rtde_lock:
            return self._rtde.stop_joint_motion(*args, **kwargs)

    def ensure_control_script_ready(self, *args, **kwargs):
        with self._rtde_lock:
            return self._rtde.ensure_control_script_ready(*args, **kwargs)

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("Call connect() before using the robot")

    def close(self) -> None:
        self.connected = False
        self._stop_state.set()
        if self._state_thread is not None:
            self._state_thread.join(timeout=1.0)
            self._state_thread = None
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)
            self._executor_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._executor = None
        with self._rtde_lock:
            self._rtde.close()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
        self._owns_rclpy = False
