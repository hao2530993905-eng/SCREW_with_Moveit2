from __future__ import annotations

import json
import math
import socket
import threading
import time
from bisect import bisect_right
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from tcp_geometry import tcp_target_to_ee_target
from ur_driver import URDriver


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

_REVOLUTE_PERIOD = 2.0 * math.pi
_JOINT_LIMIT_KEYS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_joint",
    "wrist_1",
    "wrist_2",
    "wrist_3",
]
_JOINT_LIMITS_FILE = (
    Path(__file__).resolve().parents[2]
    / "ros1_ws/src/screw_moveit_integration/config/ur3_joint_limits.yaml"
)


def _load_joint_limits() -> np.ndarray:
    """Load the same position bounds passed to the URDF used by MoveIt."""
    try:
        data = yaml.safe_load(_JOINT_LIMITS_FILE.read_text(encoding="utf-8"))
        configured = data["joint_limits"]
        limits = [
            (
                float(configured[key]["min_position"]),
                float(configured[key]["max_position"]),
            )
            for key in _JOINT_LIMIT_KEYS
        ]
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f"Cannot load shared UR3 joint limits from {_JOINT_LIMITS_FILE}: {exc}"
        ) from exc
    return np.asarray(limits, dtype=float)


# Keep a small buffer so RTDE is never asked to drive further into a controller
# limit. The bounds themselves come from the shared MoveIt/teach-pendant file.
_JOINT_LIMITS = _load_joint_limits()
_JOINT_LIMIT_MARGIN_RAD = 0.05
def _as_six(values: Iterable[float], name: str) -> list[float]:
    result = np.asarray(list(values), dtype=float).reshape(-1)
    if result.size != 6 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain 6 finite values")
    return result.tolist()


def _moveit_equivalent_joints(values: Iterable[float]) -> np.ndarray:
    """Map UR multi-turn feedback to an equivalent state accepted by MoveIt."""
    joints = np.asarray(_as_six(values, "joint positions"), dtype=float)
    return (joints + math.pi) % _REVOLUTE_PERIOD - math.pi


def _equivalent_within_joint_limits(
    values: Iterable[float],
    reference: Iterable[float],
) -> np.ndarray:
    """Choose the nearest 2*pi-equivalent position that is safe for UR3."""
    result = np.empty(6, dtype=float)
    for index, (value, ref) in enumerate(zip(values, reference, strict=True)):
        low, high = _JOINT_LIMITS[index]
        candidates = np.asarray(
            [value + turns * _REVOLUTE_PERIOD for turns in range(-2, 3)],
            dtype=float,
        )
        valid = candidates[
            (candidates >= low + _JOINT_LIMIT_MARGIN_RAD)
            & (candidates <= high - _JOINT_LIMIT_MARGIN_RAD)
        ]
        if valid.size == 0:
            raise RuntimeError(
                f"No safe {JOINT_NAMES[index]} equivalent exists within "
                f"[{low:.3f}, {high:.3f}] rad"
            )
        result[index] = valid[np.argmin(np.abs(valid - ref))]
    return result


def _require_actual_joint_state_is_safe(actual: np.ndarray) -> None:
    for index, value in enumerate(actual):
        low, high = _JOINT_LIMITS[index]
        if value < low + _JOINT_LIMIT_MARGIN_RAD or value > high - _JOINT_LIMIT_MARGIN_RAD:
            raise RuntimeError(
                f"Refusing MoveIt execution: actual {JOINT_NAMES[index]} is "
                f"{value:.3f} rad, at/near its configured limit "
                f"[{low:.3f}, {high:.3f}]. Manually jog the robot to a "
                "central pose, clear the protective stop, then retry."
            )


def _execution_equivalent_near_actual(
    value: float,
    reference: float,
    index: int,
) -> float:
    """Map a MoveIt waypoint onto the current RTDE turn count safely."""
    low, high = _JOINT_LIMITS[index]
    candidates = np.asarray(
        [value + turns * _REVOLUTE_PERIOD for turns in range(-2, 3)],
        dtype=float,
    )
    valid = candidates[
        (candidates >= low + _JOINT_LIMIT_MARGIN_RAD)
        & (candidates <= high - _JOINT_LIMIT_MARGIN_RAD)
    ]
    if valid.size == 0:
        raise RuntimeError(
            f"No safe {JOINT_NAMES[index]} trajectory waypoint exists"
        )
    return float(valid[np.argmin(np.abs(valid - reference))])


def _unwrap_trajectory_near_actual(
    positions: np.ndarray,
    actual_start: np.ndarray,
) -> np.ndarray:
    """Choose safe 2π-equivalent waypoints nearest to the real joint state."""
    unwrapped = np.asarray(positions, dtype=float).copy()
    reference = np.asarray(actual_start, dtype=float).copy()
    for row in unwrapped:
        row[:] = [
            _execution_equivalent_near_actual(value, reference[index], index)
            for index, value in enumerate(row)
        ]
        reference = row
    return unwrapped


class _BridgeClient:
    def __init__(self, host: str, port: int, timeout_s: float):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self._socket: socket.socket | None = None
        self._reader = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        self.close()
        self._socket = socket.create_connection(
            (self.host, self.port),
            timeout=self.timeout_s,
        )
        self._socket.settimeout(self.timeout_s)
        self._reader = self._socket.makefile("rb")

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._socket is None or self._reader is None:
                raise RuntimeError("ROS1 MoveIt bridge is not connected")
            encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            try:
                self._socket.sendall(encoded + b"\n")
                line = self._reader.readline()
            except OSError:
                self.close()
                raise
            if not line:
                self.close()
                raise RuntimeError("ROS1 MoveIt bridge closed the connection")
            response = json.loads(line.decode("utf-8"))
            if not response.get("ok", False):
                raise RuntimeError(
                    "ROS1 MoveIt bridge request failed: "
                    + str(response.get("error", "unknown error"))
                )
            return response

    def close(self) -> None:
        reader, self._reader = self._reader, None
        sock, self._socket = self._socket, None
        if reader is not None:
            try:
                reader.close()
            except OSError:
                pass
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


class MoveItROS1PlannedRTDEDriver:
    """Use ROS1 MoveIt for moveJ planning and retain original RTDE execution."""

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
        bridge_host: str = "127.0.0.1",
        bridge_port: int = 5061,
        rtde_driver: Any | None = None,
        **_: Any,
    ):
        self._rtde = rtde_driver or URDriver(
            host,
            connect_timeout_s=connect_timeout_s,
        )
        self.group_name = (
            "manipulator" if group_name == "ur_manipulator" else group_name
        )
        self.pose_frame = pose_frame
        self.planning_frame = planning_frame
        self.ee_link = ee_link
        self.planning_time = planning_time
        self.connect_timeout_s = connect_timeout_s
        self.trajectory_period_s = trajectory_period_s
        self._bridge = _BridgeClient(
            bridge_host,
            bridge_port,
            timeout_s=max(connect_timeout_s, planning_time + 10.0),
        )
        self._rtde_lock = threading.RLock()
        self._stop_state = threading.Event()
        self._state_thread: threading.Thread | None = None
        self._last_ik_tcp_offset: np.ndarray | None = None
        self._ik_seed_cache_path = (
            Path(__file__).resolve().parents[2]
            / "artifacts"
            / "moveit_last_successful_ik.json"
        )
        self._last_successful_ik = self._load_last_successful_ik()
        self.connected = False

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
        deadline = time.monotonic() + self.connect_timeout_s
        while True:
            try:
                self._bridge.connect()
                self._bridge.request({"command": "ping"})
                break
            except (OSError, RuntimeError) as exc:
                if time.monotonic() >= deadline:
                    self._rtde.close()
                    raise RuntimeError(
                        "Cannot connect to the ROS1 MoveIt bridge at "
                        f"{self._bridge.host}:{self._bridge.port}. "
                        "Start the ROS1 launch file and source its catkin "
                        f"workspace first: {exc}"
                    ) from exc
                time.sleep(0.25)
        self.connected = True
        self._stop_state.clear()
        self._state_thread = threading.Thread(
            target=self._publish_robot_state_loop,
            name="ros1-moveit-joint-state",
            daemon=True,
        )
        self._state_thread.start()
        print(
            "[MoveIt ROS1] Ready: MoveIt plans moveJ; RTDE executes the "
            "trajectory; original moveL/servoL insertion is unchanged."
        )
        print(
            "[MoveIt ROS1] Cartesian targets use the active controller TCP: "
            f"{self.get_tcp_offset()}"
        )
        if self._last_successful_ik is not None:
            print(
                "[MoveIt ROS1] Loaded the previous successful IK solution "
                "as the preferred seed."
            )
        return True

    def _publish_robot_state_loop(self) -> None:
        while not self._stop_state.is_set():
            try:
                joints = self.get_joint_positions()
                tcp_pose = self.get_tcp_pose()
                self._bridge.request({
                    "command": "update_joint_state",
                    "names": JOINT_NAMES,
                    "positions": _moveit_equivalent_joints(joints).tolist(),
                    # This is getActualTCPPose from the UR controller. It
                    # already includes the active TCP set on the teach pendant.
                    "tcp_pose": _as_six(tcp_pose, "current TCP pose"),
                    "tcp_frame": self.pose_frame,
                })
            except Exception as exc:
                if not self._stop_state.is_set():
                    print(f"[MoveIt ROS1] Joint-state update failed: {exc}")
            self._stop_state.wait(0.02)

    def inverse_kinematics(
        self,
        pose: Iterable[float],
        qnear: Iterable[float] | None = None,
    ) -> list[float]:
        self._require_connected()
        tcp_target = _as_six(pose, "TCP target")
        tcp_offset = np.asarray(self.get_tcp_offset(), dtype=float)
        self._last_ik_tcp_offset = tcp_offset
        tool0_target = tcp_target_to_ee_target(tcp_target, tcp_offset)
        response = self._bridge.request({
            "command": "ik",
            "group": self.group_name,
            "frame": self.pose_frame,
            "ee_link": self.ee_link,
            "pose": tool0_target,
            "tcp_target": tcp_target,
            "tcp_frame": self.pose_frame,
            "preferred_seed": self._last_successful_ik,
            "seed": (
                _moveit_equivalent_joints(qnear).tolist()
                if qnear is not None
                else _moveit_equivalent_joints(
                    self.get_joint_positions()
                ).tolist()
            ),
            "avoid_collisions": True,
        })
        solution = _as_six(response["joints"], "IK result")
        self._last_successful_ik = solution
        self._save_last_successful_ik(solution)
        return solution

    def publish_tcp_target(self, pose: Iterable[float]) -> None:
        """Publish a camera-derived TCP target before the operator confirms motion."""
        self._require_connected()
        self._bridge.request({
            "command": "publish_tcp_target",
            "tcp_target": _as_six(pose, "TCP target"),
            "tcp_frame": self.pose_frame,
        })

    def move_j(
        self,
        joints: Iterable[float],
        speed: float = 0.5,
        acceleration: float = 0.5,
        timeout: float = 20.0,
        joint_tolerance: float = 0.001,
    ) -> bool:
        self._require_connected()
        if self._last_ik_tcp_offset is not None:
            current_tcp = np.asarray(self.get_tcp_offset(), dtype=float)
            if not np.allclose(
                current_tcp,
                self._last_ik_tcp_offset,
                rtol=0.0,
                atol=1e-9,
            ):
                raise RuntimeError(
                    "The active UR TCP changed after IK. Recompute the target "
                    "before moving so the intended TCP point remains valid."
                )
        actual_start = np.asarray(self.get_joint_positions(), dtype=float)
        _require_actual_joint_state_is_safe(actual_start)
        planning_start = _moveit_equivalent_joints(actual_start)
        planning_target = _equivalent_within_joint_limits(
            joints,
            planning_start,
        )
        if not np.allclose(actual_start, planning_start, atol=1e-9, rtol=0.0):
            print(
                "[MoveIt ROS1] Normalized multi-turn joint feedback for "
                "planning: "
                f"{actual_start.tolist()} -> {planning_start.tolist()}"
            )
        response = self._bridge.request({
            "command": "plan",
            "group": self.group_name,
            "start": planning_start.tolist(),
            "target": planning_target.tolist(),
            "planning_time": self.planning_time,
            "attempts": 10,
            "velocity_scaling": min(max(speed / math.pi, 0.01), 1.0),
            "acceleration_scaling": min(
                max(acceleration / math.pi, 0.01),
                1.0,
            ),
            "joint_tolerance": joint_tolerance,
        })
        return self._execute_rtde_trajectory(
            response["trajectory"],
            timeout=timeout,
            acceleration=acceleration,
            joint_tolerance=joint_tolerance,
            actual_start=actual_start,
        )

    def _execute_rtde_trajectory(
        self,
        trajectory: dict[str, Any],
        *,
        timeout: float,
        acceleration: float,
        joint_tolerance: float,
        actual_start: np.ndarray,
    ) -> bool:
        points = trajectory.get("points", [])
        if not points:
            return False
        name_to_index = {
            name: index
            for index, name in enumerate(trajectory["joint_names"])
        }
        indices = [name_to_index[name] for name in JOINT_NAMES]
        positions = np.asarray([
            [point["positions"][index] for index in indices]
            for point in points
        ])
        positions = _unwrap_trajectory_near_actual(positions, actual_start)
        times = np.asarray([point["time_from_start"] for point in points])
        if len(times) == 1 or times[-1] <= 0.0:
            times = np.asarray([0.0, max(self.trajectory_period_s, 0.1)])
            positions = np.vstack([actual_start, positions[-1]])
        total_time = float(times[-1])
        if total_time > timeout:
            raise RuntimeError(
                f"Planned motion takes {total_time:.2f}s, exceeding "
                f"the {timeout:.2f}s timeout"
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
                    raise RuntimeError("RTDE servoJ rejected the MoveIt path")
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

    def get_tcp_offset(self):
        with self._rtde_lock:
            return self._rtde.get_tcp_offset()

    def _load_last_successful_ik(self) -> list[float] | None:
        try:
            payload = json.loads(self._ik_seed_cache_path.read_text("utf-8"))
            return _as_six(payload["joints"], "saved IK seed")
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return None

    def _save_last_successful_ik(self, joints: Iterable[float]) -> None:
        try:
            self._ik_seed_cache_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = self._ik_seed_cache_path.with_suffix(".tmp")
            temporary_path.write_text(
                json.dumps({"joints": _as_six(joints, "IK solution")}),
                encoding="utf-8",
            )
            temporary_path.replace(self._ik_seed_cache_path)
        except OSError as exc:
            print(f"[MoveIt ROS1] Could not save IK seed cache: {exc}")

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
        self._bridge.close()
        if self._state_thread is not None:
            self._state_thread.join(timeout=1.0)
            self._state_thread = None
        with self._rtde_lock:
            self._rtde.close()
