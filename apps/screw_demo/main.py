# PROJECT FILE HEADER
# 文件：apps/screw_demo/main.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

"""Mixed motion variant of the manual alignment demo.

This script keeps the operator workflow from main.py, but changes only the
first long approach move. The approach pose is solved with inverse kinematics
and executed with moveJ. Once the tool is near the hole, manual alignment and
pre-insertion still use moveL.

Use this to test whether protective stops are caused by the original long
Cartesian moveL path into the approach pose.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from enum import Enum, auto
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np

from ur_driver import URDriver


DEMO_APPROACH_POSE = [0.0, 0.0, 0.30, 0.0, 3.14, 0.0]
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "default.json"


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def load_config(path: Optional[Path]) -> dict:
    if path is not None:
        path = resolve_repo_path(path)
    if path is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def config_get(config: dict, dotted_key: str, fallback):
    current = config
    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return fallback
        current = current[part]
    return current


class DemoState(Enum):
    INIT = auto()
    MOVE_APPROACH = auto()
    MANUAL_ALIGN = auto()
    PRE_INSERT = auto()
    DONE = auto()
    ERROR = auto()
    ABORTED = auto()


class DryRunRobot:
    def __init__(self):
        self.pose = np.asarray(DEMO_APPROACH_POSE, dtype=float)
        self.joints = np.zeros(6, dtype=float)
        self._pending_pose = self.pose.copy()
        self.connected = False

    def connect(self) -> bool:
        self.connected = True
        return True

    def get_tcp_pose(self) -> List[float]:
        return self.pose.tolist()

    def get_tcp_force(self) -> List[float]:
        return [0.0] * 6

    def get_joint_positions(self) -> List[float]:
        return self.joints.tolist()

    def inverse_kinematics(self, pose: Iterable[float], qnear: Iterable[float]) -> List[float]:
        self._pending_pose = np.asarray(pose, dtype=float)
        q = np.asarray(qnear, dtype=float).copy()
        q[:3] = self._pending_pose[:3]
        q[3:] = self._pending_pose[3:]
        return q.tolist()

    def move_j(self, joints: Iterable[float], **kwargs) -> bool:
        self.joints = np.asarray(joints, dtype=float)
        self.pose = self._pending_pose.copy()
        time.sleep(0.05)
        return True

    def move_l(self, pose: Iterable[float], **kwargs) -> bool:
        self.pose = np.asarray(pose, dtype=float)
        self._pending_pose = self.pose.copy()
        time.sleep(0.05)
        return True

    def servo_l(self, pose: Iterable[float], **kwargs) -> bool:
        """Simulate the permanently enabled adaptive servoL path."""
        self.pose = np.asarray(pose, dtype=float)
        self._pending_pose = self.pose.copy()
        return True

    def stop_servo(self) -> None:
        return None

    def stop_joint_motion(self, acceleration: float = 1.5) -> None:
        return None

    def stop_linear_motion(self, acceleration: float = 0.5) -> None:
        return None

    def close(self) -> None:
        self.connected = False


def read_key() -> str:
    """Read one key and normalize arrow keys across Windows and Linux."""
    if sys.platform.startswith("win"):
        import msvcrt

        key = msvcrt.getwch()
        if key in ("\x00", "\xe0"):
            return {
                "H": "UP",
                "P": "DOWN",
                "K": "LEFT",
                "M": "RIGHT",
            }.get(msvcrt.getwch(), "UNKNOWN")
        if key == "\r":
            return "ENTER"
        return key.lower()

    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
        if key == "\x1b":
            second = sys.stdin.read(1)
            third = sys.stdin.read(1) if second == "[" else ""
            return {
                "A": "UP",
                "B": "DOWN",
                "C": "RIGHT",
                "D": "LEFT",
            }.get(third, "UNKNOWN")
        if key in ("\r", "\n"):
            return "ENTER"
        return key.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def rotation_vector_to_matrix(rotation_vector: Iterable[float]) -> np.ndarray:
    vector = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)

    axis = vector / angle
    x, y, z = axis
    skew = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def offset_in_tcp_frame(pose: Iterable[float], local_offset: Iterable[float]) -> np.ndarray:
    result = np.asarray(pose, dtype=float).copy()
    rotation = rotation_vector_to_matrix(result[3:])
    result[:3] += rotation @ np.asarray(local_offset, dtype=float)
    return result


def format_values(values: Iterable[float]) -> str:
    array = np.asarray(values, dtype=float)
    return "[" + ", ".join(f"{value:.6f}" for value in array) + "]"


def split_axial_lateral_force(
    pose: Iterable[float],
    wrench: Iterable[float],
    insert_sign: float,
) -> tuple[float, float]:
    pose_array = np.asarray(pose, dtype=float)
    force = np.asarray(wrench, dtype=float)[:3]
    tool_z = rotation_vector_to_matrix(pose_array[3:])[:, 2] * insert_sign
    axial = float(np.dot(force, tool_z))
    lateral = float(np.linalg.norm(force - axial * tool_z))
    return axial, lateral


class ForceRecorder:
    def __init__(self, robot, args):
        self.robot = robot
        self.args = args
        self.period = 1.0 / args.force_log_hz
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.first_sample_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.path: Optional[Path] = None
        self.error: Optional[BaseException] = None
        self.sample_count = 0
        self.current_event = "pre_insert"
        self.bias = np.zeros(6, dtype=float)
        self._lock = threading.Lock()

    def start(self) -> Optional[Path]:
        if not self.args.record_force:
            return None

        log_dir = resolve_repo_path(self.args.force_log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        name = self.args.force_log_name or f"main_force_{time.strftime('%Y%m%d_%H%M%S')}"
        self.path = log_dir / f"{name}.csv"
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self.path

    def wait_until_recording(self, timeout: float) -> bool:
        if self.thread is None:
            return False
        return self.ready_event.wait(timeout) and self.first_sample_event.wait(timeout)

    def samples(self) -> int:
        with self._lock:
            return self.sample_count

    def set_event(self, event: str) -> None:
        with self._lock:
            self.current_event = event

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def _run(self) -> None:
        assert self.path is not None
        fieldnames = [
            "time",
            "elapsed",
            "x",
            "y",
            "z",
            "rx",
            "ry",
            "rz",
            "raw_Fx",
            "raw_Fy",
            "raw_Fz",
            "raw_Tx",
            "raw_Ty",
            "raw_Tz",
            "bias_Fx",
            "bias_Fy",
            "bias_Fz",
            "bias_Tx",
            "bias_Ty",
            "bias_Tz",
            "Fx",
            "Fy",
            "Fz",
            "Tx",
            "Ty",
            "Tz",
            "f_axial",
            "f_lateral",
            "event",
        ]
        start = time.time()
        next_time = start
        try:
            with self.path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                f.flush()
                self.bias = self._measure_bias()
                self.ready_event.set()
                while not self.stop_event.is_set():
                    now = time.time()
                    pose = np.asarray(self.robot.get_tcp_pose(), dtype=float)
                    raw_wrench = np.asarray(self.robot.get_tcp_force(), dtype=float)
                    wrench = raw_wrench - self.bias
                    f_axial, f_lateral = split_axial_lateral_force(
                        pose,
                        wrench,
                        self.args.insert_sign,
                    )
                    with self._lock:
                        event = self.current_event
                    writer.writerow(
                        {
                            "time": now,
                            "elapsed": now - start,
                            "x": pose[0],
                            "y": pose[1],
                            "z": pose[2],
                            "rx": pose[3],
                            "ry": pose[4],
                            "rz": pose[5],
                            "raw_Fx": raw_wrench[0],
                            "raw_Fy": raw_wrench[1],
                            "raw_Fz": raw_wrench[2],
                            "raw_Tx": raw_wrench[3],
                            "raw_Ty": raw_wrench[4],
                            "raw_Tz": raw_wrench[5],
                            "bias_Fx": self.bias[0],
                            "bias_Fy": self.bias[1],
                            "bias_Fz": self.bias[2],
                            "bias_Tx": self.bias[3],
                            "bias_Ty": self.bias[4],
                            "bias_Tz": self.bias[5],
                            "Fx": wrench[0],
                            "Fy": wrench[1],
                            "Fz": wrench[2],
                            "Tx": wrench[3],
                            "Ty": wrench[4],
                            "Tz": wrench[5],
                            "f_axial": f_axial,
                            "f_lateral": f_lateral,
                            "event": event,
                        }
                    )
                    f.flush()
                    with self._lock:
                        self.sample_count += 1
                    self.first_sample_event.set()
                    next_time += self.period
                    time.sleep(max(0.0, next_time - time.time()))
        except BaseException as exc:
            self.error = exc
            self.ready_event.set()
            self.stop_event.set()

    def _measure_bias(self) -> np.ndarray:
        samples = int(self.args.force_bias_samples)
        if samples <= 0:
            return np.zeros(6, dtype=float)
        values = []
        dt = self.args.force_bias_dt
        for _ in range(samples):
            if self.stop_event.is_set():
                break
            values.append(self.robot.get_tcp_force())
            if dt > 0.0:
                time.sleep(dt)
        if not values:
            return np.zeros(6, dtype=float)
        return np.mean(np.asarray(values, dtype=float), axis=0)


def wait_for_enter_or_abort(message: str) -> bool:
    print(message)
    print("Press Enter to continue, or Q to abort.")
    while True:
        key = read_key()
        if key == "ENTER":
            return True
        if key == "q":
            return False


def move_linear(
    robot,
    target: Iterable[float],
    args,
    speed: float,
    label: str,
    cancel_event=None,
) -> bool:
    target = np.asarray(target, dtype=float)
    print(f"\n[{label}]")
    print("Target pose:", format_values(target))
    kwargs = {
        "speed": speed,
        "acceleration": args.linear_acc,
        "timeout": args.motion_timeout,
    }
    if cancel_event is not None:
        kwargs["cancel_event"] = cancel_event
    return robot.move_l(target, **kwargs)


def solve_ik(robot, target_pose: Iterable[float], args) -> List[float]:
    target = np.asarray(target_pose, dtype=float).reshape(-1).tolist()
    if len(target) != 6:
        raise ValueError(f"target pose must contain 6 values, got {len(target)}")

    qnear = robot.get_joint_positions()
    if hasattr(robot, "inverse_kinematics"):
        joints = robot.inverse_kinematics(target, qnear)
    else:
        rtde_c = getattr(robot, "rtde_c", None)
        if rtde_c is None or not hasattr(rtde_c, "getInverseKinematics"):
            raise RuntimeError("RTDE control object does not expose getInverseKinematics")
        try:
            joints = rtde_c.getInverseKinematics(
                target,
                qnear,
                args.ik_position_error,
                args.ik_orientation_error,
            )
        except TypeError:
            try:
                joints = rtde_c.getInverseKinematics(target, qnear)
            except TypeError:
                joints = rtde_c.getInverseKinematics(target)

    joints = np.asarray(joints, dtype=float).reshape(-1).tolist()
    if len(joints) != 6:
        raise RuntimeError(f"IK failed for target pose {format_values(target)}")
    if not np.all(np.isfinite(joints)):
        raise RuntimeError(f"IK returned non-finite joints: {joints}")
    return joints


def move_joint_to_pose(robot, target_pose: Iterable[float], args, label: str) -> bool:
    current_pose = np.asarray(robot.get_tcp_pose(), dtype=float)
    target_pose = np.asarray(target_pose, dtype=float)
    distance = float(np.linalg.norm(target_pose[:3] - current_pose[:3]))
    joints = solve_ik(robot, target_pose, args)

    print(f"\n[{label}]")
    print("Current pose:", format_values(current_pose))
    print("Target pose:", format_values(target_pose))
    print(f"TCP position delta: {distance * 1000.0:.1f} mm")
    print("Current joints:", format_values(robot.get_joint_positions()))
    print("IK target joints:", format_values(joints))

    if distance > args.warn_distance:
        print(f"WARNING: target is farther than {args.warn_distance * 1000.0:.1f} mm.")
        if not wait_for_enter_or_abort("Confirm this joint-space move is safe."):
            return False

    return robot.move_j(
        joints,
        speed=args.joint_speed,
        acceleration=args.joint_acc,
        timeout=args.motion_timeout,
    )


def manual_align(robot, approach_pose: np.ndarray, args) -> bool:
    print("\n[MANUAL_ALIGN]")
    print("Arrow keys jog in the TCP local X/Y plane.")
    print("Left/Right: local -X/+X; Down/Up: local -Y/+Y.")
    print("This test version executes each jog with moveL.")
    print("Press Enter to accept alignment, or Q to abort.")

    local_xy = np.zeros(2, dtype=float)
    key_to_delta = {
        "LEFT": np.array([-args.jog_step, 0.0]),
        "RIGHT": np.array([args.jog_step, 0.0]),
        "DOWN": np.array([0.0, -args.jog_step]),
        "UP": np.array([0.0, args.jog_step]),
    }

    while True:
        key = read_key()
        if key == "q":
            return False
        if key == "ENTER":
            print("Accepted aligned pose:", format_values(robot.get_tcp_pose()))
            return True
        if key not in key_to_delta:
            continue

        candidate_xy = local_xy + key_to_delta[key]
        if np.linalg.norm(candidate_xy) > args.max_align_offset:
            print(f"Rejected: alignment offset exceeds {args.max_align_offset * 1000.0:.1f} mm.")
            continue

        target = offset_in_tcp_frame(approach_pose, [candidate_xy[0], candidate_xy[1], 0.0])
        print(
            f"Jog local XY: [{candidate_xy[0] * 1000.0:.1f}, "
            f"{candidate_xy[1] * 1000.0:.1f}] mm"
        )
        if not move_linear(robot, target, args, speed=args.jog_speed, label="JOG_MOVEL"):
            raise RuntimeError("Manual alignment moveL failed or timed out")
        local_xy = candidate_xy


def resolve_approach_pose(args) -> np.ndarray:
    if args.approach_pose is not None:
        return np.asarray(args.approach_pose, dtype=float)
    if args.enable_robot and not args.use_hardcoded_pose:
        raise RuntimeError(
            "Real robot mode requires --approach-pose X Y Z RX RY RZ, or "
            "--use-hardcoded-pose after editing DEMO_APPROACH_POSE."
        )
    return np.asarray(DEMO_APPROACH_POSE, dtype=float)


def validate_args(args) -> None:
    if args.jog_step <= 0.0:
        raise ValueError("--jog-step must be positive")
    if args.max_align_offset <= 0.0:
        raise ValueError("--max-align-offset must be positive")
    if hasattr(args, "insert_depth") and args.insert_depth is not None:
        if not 0.0 < args.insert_depth <= args.max_insert_depth:
            raise ValueError("--insert-depth must be positive and no larger than --max-insert-depth")
    elif args.max_insert_depth <= 0.0:
        raise ValueError("--max-insert-depth must be positive")
    if hasattr(args, "torque_stop_nm"):
        if args.enable_robot and args.torque_stop_nm <= 0.0:
            raise ValueError("real robot mode requires --torque-stop-nm > 0")
        if args.torque_stop_nm < 0.0:
            raise ValueError("--torque-stop-nm must be non-negative")
        if args.torque_poll_s <= 0.0:
            raise ValueError("--torque-poll-s must be positive")
        if args.stop_linear_acc <= 0.0:
            raise ValueError("--stop-linear-acc must be positive")
        if getattr(args, "torque_filter_window", 1) <= 0:
            raise ValueError("--torque-filter-window must be positive")
        if getattr(args, "torque_confirm_samples", 1) <= 0:
            raise ValueError("--torque-confirm-samples must be positive")
        if getattr(args, "torque_min_active_s", 0.0) < 0.0:
            raise ValueError("--torque-min-active-s must be non-negative")
    if args.joint_speed <= 0.0 or args.joint_acc <= 0.0:
        raise ValueError("--joint-speed and --joint-acc must be positive")
    if args.force_log_hz <= 0.0:
        raise ValueError("--force-log-hz must be positive")
    if args.force_log_start_timeout < 0.0:
        raise ValueError("--force-log-start-timeout must be non-negative")
    if args.force_log_tail < 0.0:
        raise ValueError("--force-log-tail must be non-negative")
    if args.force_log_success_tail < 0.0:
        raise ValueError("--force-log-success-tail must be non-negative")
    if args.force_bias_samples < 0:
        raise ValueError("--force-bias-samples must be non-negative")
    if args.force_bias_dt < 0.0:
        raise ValueError("--force-bias-dt must be non-negative")


def run(args) -> DemoState:
    validate_args(args)
    approach_pose = resolve_approach_pose(args)
    robot = URDriver(args.robot_host) if args.enable_robot else DryRunRobot()
    force_recorder: Optional[ForceRecorder] = None

    try:
        print("Connecting to robot..." if args.enable_robot else "Starting dry-run robot...")
        if not robot.connect():
            raise RuntimeError("Robot connection failed")

        print("\n[INIT]")
        print("This test script uses moveJ for approach, then moveL for jog/pre-insert.")
        print("Current pose:", format_values(robot.get_tcp_pose()))
        print("Current joints:", format_values(robot.get_joint_positions()))
        print("Configured approach pose:", format_values(approach_pose))
        if not wait_for_enter_or_abort("Confirm the workspace is clear."):
            return DemoState.ABORTED

        if not move_joint_to_pose(robot, approach_pose, args, "MOVE_APPROACH_MOVEJ"):
            return DemoState.ABORTED
        print("Approach pose reached by joint-space motion.")
        if not wait_for_enter_or_abort("Confirm the tool is near the intended hole."):
            return DemoState.ABORTED

        if not manual_align(robot, approach_pose, args):
            return DemoState.ABORTED

        aligned_pose = np.asarray(robot.get_tcp_pose(), dtype=float)
        local_insert = [0.0, 0.0, args.insert_sign * args.insert_depth]
        insert_target = offset_in_tcp_frame(aligned_pose, local_insert)
        axis = rotation_vector_to_matrix(aligned_pose[3:])[:, 2] * args.insert_sign

        print("\n[PRE_INSERT]")
        print(f"Insertion depth: {args.insert_depth * 1000.0:.1f} mm")
        print("Insertion direction in robot base frame:", format_values(axis))
        print("Pre-insertion target pose:", format_values(insert_target))
        print("Press P to execute pre-insertion with moveL, or Q to abort.")
        while True:
            key = read_key()
            if key == "q":
                return DemoState.ABORTED
            if key == "p":
                break

        force_recorder = ForceRecorder(robot, args)
        force_log_path = force_recorder.start()
        if force_log_path is not None:
            print("Force log:", force_log_path)
            if not force_recorder.wait_until_recording(args.force_log_start_timeout):
                raise RuntimeError(
                    "Force recorder did not write an initial sample before pre-insertion"
                )
            print("Force bias [Fx,Fy,Fz,Tx,Ty,Tz]:", format_values(force_recorder.bias))
            print(f"Force recorder started with {force_recorder.samples()} sample(s).")

        pre_insert_error: Optional[BaseException] = None
        try:
            if not move_linear(robot, insert_target, args, speed=args.insert_speed, label="PRE_INSERT_MOVEL"):
                pre_insert_error = RuntimeError("Pre-insertion moveL failed or timed out")
        except BaseException as exc:
            pre_insert_error = exc
        finally:
            if force_recorder.path is not None:
                if pre_insert_error is not None:
                    force_recorder.set_event("pre_insert_error")
                    print(
                        "Pre-insertion stopped before reaching target; keeping force log "
                        f"for {args.force_log_tail:.2f} s."
                    )
                    time.sleep(args.force_log_tail)
                elif args.force_log_success_tail > 0.0:
                    force_recorder.set_event("post_pre_insert")
                    print(
                        "Pre-insertion target reached; keeping force log "
                        f"for {args.force_log_success_tail:.2f} s."
                    )
                    time.sleep(args.force_log_success_tail)
            force_recorder.stop()
            if force_recorder.error is not None:
                print(f"Force recorder stopped with error: {force_recorder.error}")
            if force_log_path is not None:
                print(f"Saved {force_recorder.samples()} force sample(s) to {force_log_path}")
            force_recorder = None

        if pre_insert_error is not None:
            raise pre_insert_error

        print("\n[DONE]")
        print("Pre-insertion target reached. No screwdriver command was sent.")
        print("Final pose:", format_values(robot.get_tcp_pose()))
        print("Final joints:", format_values(robot.get_joint_positions()))
        return DemoState.DONE

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping joint motion.")
        robot.stop_linear_motion()
        robot.stop_joint_motion()
        return DemoState.ABORTED
    except Exception as exc:
        print(f"\n[ERROR] {exc}")
        robot.stop_linear_motion()
        robot.stop_joint_motion()
        return DemoState.ERROR
    finally:
        if force_recorder is not None:
            force_recorder.stop()
            if force_recorder.error is not None:
                print(f"Force recorder stopped with error: {force_recorder.error}")
        robot.close()


def parse_args():
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="JSON config file loaded before command line overrides",
    )
    config_args, remaining = config_parser.parse_known_args()
    config = load_config(config_args.config)

    def cfg(key: str, fallback):
        return config_get(config, key, fallback)

    parser = argparse.ArgumentParser(
        description="Interactive manual alignment demo using moveJ approach and moveL fine motion.",
        parents=[config_parser],
    )
    parser.add_argument("--robot-host", default=cfg("robot.host", "192.168.1.5"))
    parser.add_argument(
        "--enable-robot",
        action="store_true",
        default=cfg("robot.enable_robot", False),
        help="connect to and move the real UR",
    )
    parser.add_argument(
        "--approach-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        default=cfg("motion.approach_pose", None),
        help="approach TCP pose in robot base coordinates",
    )
    parser.add_argument(
        "--use-hardcoded-pose",
        action="store_true",
        default=cfg("motion.use_hardcoded_pose", False),
        help="allow DEMO_APPROACH_POSE in real robot mode",
    )
    parser.add_argument(
        "--jog-step",
        type=float,
        default=cfg("motion.jog_step", 0.0005),
        help="arrow-key jog step in meters",
    )
    parser.add_argument(
        "--max-align-offset",
        type=float,
        default=cfg("motion.max_align_offset", 0.010),
        help="maximum manual XY offset radius from approach pose in meters",
    )
    parser.add_argument(
        "--insert-depth",
        type=float,
        default=cfg("motion.insert_depth", 0.002),
        help="pre-insertion depth in meters",
    )
    parser.add_argument(
        "--max-insert-depth",
        type=float,
        default=cfg("motion.max_insert_depth", 0.60),
        help="maximum accepted pre-insertion depth in meters",
    )
    parser.add_argument(
        "--insert-sign",
        type=int,
        choices=(-1, 1),
        default=cfg("motion.insert_sign", 1),
        help="TCP local Z direction: +1 or -1",
    )
    parser.add_argument("--joint-speed", type=float, default=cfg("motion.joint_speed", 0.15), help="moveJ speed in rad/s")
    parser.add_argument("--joint-acc", type=float, default=cfg("motion.joint_acc", 0.15), help="moveJ acceleration in rad/s^2")
    parser.add_argument("--jog-speed", type=float, default=cfg("motion.jog_speed", 0.005))
    parser.add_argument("--insert-speed", type=float, default=cfg("motion.insert_speed", 0.003))
    parser.add_argument("--linear-acc", type=float, default=cfg("motion.linear_acc", 0.03))
    parser.add_argument("--motion-timeout", type=float, default=cfg("motion.motion_timeout", 20.0))
    parser.add_argument(
        "--warn-distance",
        type=float,
        default=cfg("motion.warn_distance", 0.050),
        help="ask for extra confirmation when TCP target is farther than this many meters",
    )
    parser.add_argument("--ik-position-error", type=float, default=cfg("motion.ik_position_error", 1e-4))
    parser.add_argument("--ik-orientation-error", type=float, default=cfg("motion.ik_orientation_error", 1e-3))
    parser.add_argument(
        "--no-force-log",
        dest="record_force",
        action="store_false",
        help="disable background TCP force logging",
    )
    parser.set_defaults(record_force=cfg("force.record", True))
    parser.add_argument("--force-log-hz", type=float, default=cfg("force.log_hz", 50.0), help="TCP force logging rate in Hz")
    parser.add_argument(
        "--force-log-dir",
        default=cfg("force.log_dir", "artifacts/logs/screw_demo"),
        help="directory for force CSV logs",
    )
    parser.add_argument("--force-log-name", default=cfg("force.log_name", None), help="optional force CSV file name without extension")
    parser.add_argument(
        "--force-log-start-timeout",
        type=float,
        default=cfg("force.log_start_timeout", 2.0),
        help="seconds to wait for the first force sample before pre-insertion",
    )
    parser.add_argument(
        "--force-log-tail",
        type=float,
        default=cfg("force.log_tail", 0.5),
        help="seconds to keep logging after a failed pre-insertion move",
    )
    parser.add_argument(
        "--force-log-success-tail",
        type=float,
        default=cfg("force.log_success_tail", 20.0),
        help="seconds to keep logging after a successful pre-insertion move",
    )
    parser.add_argument(
        "--force-bias-samples",
        type=int,
        default=cfg("force.bias_samples", 50),
        help="stationary force samples used as static bias before pre-insertion logging",
    )
    parser.add_argument(
        "--force-bias-dt",
        type=float,
        default=cfg("force.bias_dt", None),
        help="seconds between force bias samples; defaults to 1 / force log Hz",
    )
    args = parser.parse_args(remaining)
    args.config = config_args.config
    if args.force_bias_dt is None:
        args.force_bias_dt = 1.0 / args.force_log_hz
    return args


def main() -> None:
    result = run(parse_args())
    if result not in (DemoState.DONE, DemoState.ABORTED):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
