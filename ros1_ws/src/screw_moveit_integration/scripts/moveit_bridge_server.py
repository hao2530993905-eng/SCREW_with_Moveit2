#!/usr/bin/python3

import json
import math
import socketserver
import sys
import threading

import moveit_commander
import rospy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import MoveItErrorCodes, PlanningSceneComponents, RobotState
from moveit_msgs.srv import (
    GetPlanningScene,
    GetPositionIK,
    GetPositionIKRequest,
    GetStateValidity,
    GetStateValidityRequest,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _pose(values):
    message = Pose()
    message.position.x, message.position.y, message.position.z = values[:3]
    rx, ry, rz = values[3:]
    angle = (rx * rx + ry * ry + rz * rz) ** 0.5
    if angle < 1e-12:
        message.orientation.w = 1.0
    else:
        import math

        scale = math.sin(angle / 2.0) / angle
        message.orientation.x = rx * scale
        message.orientation.y = ry * scale
        message.orientation.z = rz * scale
        message.orientation.w = math.cos(angle / 2.0)
    return message


def _robot_state(positions):
    state = RobotState()
    state.is_diff = True
    state.joint_state.name = list(JOINT_NAMES)
    state.joint_state.position = list(positions)
    return state


class MoveItBridge(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._groups = {}
        self._joint_publisher = rospy.Publisher(
            "/joint_states",
            JointState,
            queue_size=20,
        )
        self._target_marker_publisher = rospy.Publisher(
            "/moveit_target_pose_marker",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self._current_tcp_marker_publisher = rospy.Publisher(
            "/moveit_current_tcp_marker",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        rospy.wait_for_service("/compute_ik", timeout=30.0)
        rospy.wait_for_service("/get_planning_scene", timeout=30.0)
        rospy.wait_for_service("/check_state_validity", timeout=30.0)
        self._ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self._scene = rospy.ServiceProxy(
            "/get_planning_scene",
            GetPlanningScene,
        )
        self._state_validity = rospy.ServiceProxy(
            "/check_state_validity",
            GetStateValidity,
        )

    def _group(self, name):
        with self._lock:
            if name not in self._groups:
                self._groups[name] = moveit_commander.MoveGroupCommander(name)
            return self._groups[name]

    def handle(self, request):
        command = request.get("command")
        if command == "ping":
            return {"ok": True, "ros_version": 1}
        if command == "update_joint_state":
            message = JointState()
            message.header.stamp = rospy.Time.now()
            message.name = request["names"]
            message.position = request["positions"]
            self._joint_publisher.publish(message)
            if "tcp_pose" in request:
                self._publish_current_tcp_pose(
                    request["tcp_pose"],
                    request.get("tcp_frame", "base"),
                )
            return {"ok": True}
        if command == "ik":
            return self._handle_ik(request)
        if command == "publish_tcp_target":
            self._publish_target_pose(request)
            return {"ok": True}
        if command == "plan":
            return self._handle_plan(request)
        if command == "check_state":
            return self._handle_state_validity(request)
        if command == "status":
            return self._handle_status()
        raise ValueError("unknown command: %s" % command)

    def _handle_ik(self, request):
        self._publish_target_pose(request)
        message = GetPositionIKRequest()
        ik = message.ik_request
        ik.group_name = request.get("group", "manipulator")
        ik.ik_link_name = request.get("ee_link", "tool0")
        ik.pose_stamped.header.frame_id = request.get("frame", "base")
        ik.pose_stamped.header.stamp = rospy.Time.now()
        ik.pose_stamped.pose = _pose(request["pose"])
        ik.avoid_collisions = bool(request.get("avoid_collisions", True))
        # KDL is seed-sensitive. Try several bounded, elbow/wrist-distinct
        # seeds while retaining collision checking for every candidate.
        seeds = []
        preferred_seed = request.get("preferred_seed")
        if preferred_seed is not None:
            seeds.append(list(preferred_seed))
        seeds.append(list(request["seed"]))
        seeds.extend([
            [0.0, -1.3, 2.0, -1.5, 0.5, math.pi],
            [0.0, -1.3, 2.0, -1.5, -0.5, -math.pi],
            [0.0, -2.0, 2.0, -1.5, 1.0, 0.0],
            [0.8, -1.0, 1.5, -1.5, 0.5, 0.0],
            [-0.8, -1.0, 1.5, -1.5, -0.5, 0.0],
        ])
        unique_seeds = []
        for seed in seeds:
            if len(seed) == 6 and seed not in unique_seeds:
                unique_seeds.append(seed)
        last_error = None
        for seed in unique_seeds:
            ik.robot_state = _robot_state(seed)
            ik.timeout = rospy.Duration(0.8)
            with self._lock:
                response = self._ik(message)
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                last_error = response.error_code.val
                continue
            solution = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ))
            rospy.loginfo("MoveIt IK succeeded using seed %s", seed)
            return {
                "ok": True,
                "joints": [solution[name] for name in JOINT_NAMES],
            }
        raise RuntimeError(
            "MoveIt IK failed with code %d after %d seed attempts"
            % (last_error, len(unique_seeds))
        )

    def _publish_target_pose(self, request):
        """Show the camera-derived TCP target in RViz, even when IK fails."""
        target = request.get("tcp_target")
        if target is None:
            target = request["pose"]
        pose = _pose(target)
        frame_id = request.get("tcp_frame", request.get("frame", "base"))
        stamp = rospy.Time.now()
        origin = Point(pose.position.x, pose.position.y, pose.position.z)
        quaternion = pose.orientation
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        axes = (
            ("x", (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)), ColorRGBA(1.0, 0.1, 0.1, 1.0)),
            ("y", (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)), ColorRGBA(0.1, 1.0, 0.1, 1.0)),
            ("z", (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)), ColorRGBA(0.2, 0.4, 1.0, 1.0)),
        )

        markers = MarkerArray()
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "target_tcp"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = origin
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.060
        sphere.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        markers.markers.append(sphere)

        for marker_id, (axis_name, direction, color) in enumerate(axes, start=1):
            arrow = Marker()
            arrow.header.frame_id = frame_id
            arrow.header.stamp = stamp
            arrow.ns = "target_tcp_axes"
            arrow.id = marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.points = [
                origin,
                Point(
                    origin.x + 0.18 * direction[0],
                    origin.y + 0.18 * direction[1],
                    origin.z + 0.18 * direction[2],
                ),
            ]
            arrow.scale.x = 0.012
            arrow.scale.y = 0.024
            arrow.scale.z = 0.036
            arrow.color = color
            markers.markers.append(arrow)

        guide = Marker()
        guide.header.frame_id = frame_id
        guide.header.stamp = stamp
        guide.ns = "target_tcp"
        guide.id = 4
        guide.type = Marker.CYLINDER
        guide.action = Marker.ADD
        guide.pose.position = Point(origin.x, origin.y, origin.z + 0.125)
        guide.pose.orientation.w = 1.0
        guide.scale.x = guide.scale.y = 0.012
        guide.scale.z = 0.25
        guide.color = ColorRGBA(1.0, 0.0, 1.0, 0.85)
        markers.markers.append(guide)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "target_tcp"
        label.id = 5
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(origin.x, origin.y, origin.z + 0.275)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.035
        label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        label.text = "Target TCP"
        markers.markers.append(label)
        self._target_marker_publisher.publish(markers)
        rospy.loginfo("Published target TCP marker in frame '%s'", frame_id)

    def _publish_current_tcp_pose(self, values, frame_id):
        """Show UR's active, teach-pendant-calibrated TCP in RViz."""
        pose = _pose(values)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_calibrated_tcp"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.040
        marker.color = ColorRGBA(0.0, 1.0, 1.0, 0.95)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = marker.header.stamp
        label.ns = "current_calibrated_tcp"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            pose.position.x,
            pose.position.y,
            pose.position.z + 0.030,
        )
        label.pose.orientation.w = 1.0
        label.scale.z = 0.030
        label.color = ColorRGBA(0.0, 1.0, 1.0, 1.0)
        label.text = "Current calibrated TCP"

        markers = MarkerArray()
        markers.markers.extend((marker, label))
        self._current_tcp_marker_publisher.publish(markers)

    def _handle_plan(self, request):
        group = self._group(request.get("group", "manipulator"))
        base_planning_time = float(request.get("planning_time", 5.0))
        base_attempts = int(request.get("attempts", 10))
        # RRTConnect is fast, but can miss a valid route in a cluttered UR3
        # workspace. Try two complementary OMPL planners before declaring the
        # target unreachable. All candidates still use MoveIt's collision check.
        planner_attempts = (
            ("RRTConnect", base_planning_time, base_attempts),
            ("RRTstar", max(base_planning_time, 12.0), max(base_attempts, 30)),
            ("PRM", max(base_planning_time, 12.0), max(base_attempts, 30)),
        )
        trajectory = None
        planner_used = None
        with self._lock:
            group.stop()
            group.clear_pose_targets()
            group.set_max_velocity_scaling_factor(
                float(request.get("velocity_scaling", 0.1))
            )
            group.set_max_acceleration_scaling_factor(
                float(request.get("acceleration_scaling", 0.1))
            )
            for planner_id, planning_time, attempts in planner_attempts:
                group.set_start_state(_robot_state(request["start"]))
                group.set_planner_id(planner_id)
                group.set_planning_time(planning_time)
                group.set_num_planning_attempts(attempts)
                group.set_joint_value_target(
                    dict(zip(JOINT_NAMES, request["target"]))
                )
                rospy.loginfo(
                    "Planning with %s (%.1fs, %d attempts)",
                    planner_id,
                    planning_time,
                    attempts,
                )
                planned = group.plan()
                if isinstance(planned, tuple):
                    success = bool(planned[0])
                    candidate = planned[1]
                else:
                    candidate = planned
                    success = bool(candidate.joint_trajectory.points)
                if success and candidate.joint_trajectory.points:
                    trajectory = candidate
                    planner_used = planner_id
                    break
            group.clear_pose_targets()
        if trajectory is None or not trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt could not find a collision-free path")
        rospy.loginfo("Found collision-free path with %s", planner_used)
        joint_trajectory = trajectory.joint_trajectory
        return {
            "ok": True,
            "planner_id": planner_used,
            "trajectory": {
                "joint_names": list(joint_trajectory.joint_names),
                "points": [
                    {
                        "positions": list(point.positions),
                        "time_from_start": point.time_from_start.to_sec(),
                    }
                    for point in joint_trajectory.points
                ],
            },
        }

    def _handle_state_validity(self, request):
        message = GetStateValidityRequest()
        message.group_name = request.get("group", "manipulator")
        message.robot_state = _robot_state(request["positions"])
        with self._lock:
            response = self._state_validity(message)
        result = {"ok": True, "valid": bool(response.valid)}
        if request.get("contacts", False):
            result["contacts"] = [
                {
                    "first": contact.contact_body_1,
                    "second": contact.contact_body_2,
                    "depth": contact.depth,
                }
                for contact in response.contacts
            ]
        return result

    def _handle_status(self):
        components = PlanningSceneComponents()
        components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        with self._lock:
            scene = self._scene(components).scene
        return {
            "ok": True,
            "world_objects": [
                collision_object.id
                for collision_object in scene.world.collision_objects
            ],
            "attached_objects": [
                attached.object.id
                for attached in scene.robot_state.attached_collision_objects
            ],
        }


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while not rospy.is_shutdown():
            line = self.rfile.readline()
            if not line:
                return
            try:
                request = json.loads(line.decode("utf-8"))
                response = self.server.bridge.handle(request)
            except Exception as exc:
                rospy.logerr("MoveIt bridge request failed: %s", exc)
                response = {"ok": False, "error": str(exc)}
            self.wfile.write(
                json.dumps(response, separators=(",", ":")).encode("utf-8")
                + b"\n"
            )
            self.wfile.flush()


class ThreadingServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("screw_moveit_bridge")
    port = int(rospy.get_param("~port", 5061))
    server = ThreadingServer(("127.0.0.1", port), RequestHandler)
    server.bridge = MoveItBridge()
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    rospy.loginfo("ROS1 MoveIt bridge listening on 127.0.0.1:%d", port)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
