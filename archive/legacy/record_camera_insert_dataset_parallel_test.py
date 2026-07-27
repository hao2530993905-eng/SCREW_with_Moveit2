#!/usr/bin/env python
from __future__ import annotations

import argparse
import ast
import importlib.util
import math
import json
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Any

import requests


REPO_ROOT = Path(__file__).resolve().parents[2]
CAMERA_DIR = REPO_ROOT / "run"
CAMERA_SCRIPT = CAMERA_DIR / "ring.py"
RECORD_SCRIPT = REPO_ROOT / "lerobot_robot_screw" / "scripts" / "record_main_insert_dataset.py"
DEFAULT_LOG_ROOT = REPO_ROOT / "test_log"

DEFAULT_BASE_APPROACH_POSE_MM_RAD = [
    47.52324639603356,
    -571.8177031033332,
    -61.35649117906536,
    0.009630858678279864,
    2.2044226449971296,
    -2.1350999569901568,
]

DEFAULT_CAMERA_TO_ROBOT_ROTATION = [
    [-0.9855, -0.0026, 0.1698],
    [-0.1697, -0.0147, -0.9854],
    [0.0051, -0.9999, 0.0141],
]
DEFAULT_CAMERA_TO_ROBOT_TRANSLATION_MM = [-308.1396, 102.8816, 43.0891]


class TeeStream:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data: str) -> int:
        for stream in self.streams:
            stream.write(data)
            stream.flush()
        return len(data)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


class StageTimer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.marks: dict[str, float] = {}
        self.mark("脚本启动")

    def mark(self, name: str) -> float:
        now = time.perf_counter()
        with self._lock:
            self.marks[name] = now
        return now

    def mark_once(self, name: str) -> float:
        now = time.perf_counter()
        with self._lock:
            return self.marks.setdefault(name, now)

    def elapsed(self, start: str, end: str) -> float | None:
        with self._lock:
            if start not in self.marks or end not in self.marks:
                return None
            return self.marks[end] - self.marks[start]


def make_experiment_dir(log_root: Path) -> Path:
    base = log_root / f"parallel_{datetime.now():%Y%m%d_%H%M%S}"
    path = base
    suffix = 1
    while path.exists():
        path = Path(f"{base}_{suffix:02d}")
        suffix += 1
    path.mkdir(parents=True, exist_ok=False)
    return path


def format_seconds(value: float | None) -> str:
    if value is None:
        return "未记录"
    return f"{value:.3f} 秒"


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=(
            "Run camera ring detection first, convert the detected ring center to an approach "
            "pose placeholder, then start record_main_insert_dataset.py. If --dataset-root "
            "and --repo-id are omitted after the separator, the recorder creates timestamped "
            "defaults automatically."
        )
    )
    parser.add_argument(
        "--camera-ip",
        default="192.168.1.66",
        help=(
            "camera IP passed to run/ring.py. Default is 'auto'; ring.py will search "
            "for the camera and use the detected IP."
        ),
    )
    parser.add_argument(
        "--camera-python",
        default=sys.executable,
        help="Python executable used to run run/ring.py",
    )
    parser.add_argument(
        "--record-python",
        default=sys.executable,
        help="Python executable used to run record_main_insert_dataset.py",
    )
    parser.add_argument(
        "--base-approach-pose",
        nargs=6,
        type=float,
        metavar=("X_MM", "Y_MM", "Z_MM", "RX", "RY", "RZ"),
        default=DEFAULT_BASE_APPROACH_POSE_MM_RAD,
        help=(
            "temporary nominal approach pose. XYZ are millimeters, rotation is radians. "
            "This is used until hand-eye calibration is filled in."
        ),
    )
    parser.add_argument(
        "--target-ring",
        choices=("inner", "outer", "midpoint"),
        default="outer",
        help="which detected ring center should drive the target pose",
    )
    parser.add_argument(
        "--approach-distance-mm",
        type=float,
        default=0.0,
        help=(
            "deprecated: stand-off distance is disabled; the generated target now uses "
            "the camera center position directly"
        ),
    )
    parser.add_argument(
        "--normal-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
        help="deprecated: plane-normal orientation is disabled",
    )
    parser.add_argument(
        "--calibration-json",
        type=Path,
        default=None,
        help=(
            "optional calibration file. Supported fields: camera_to_robot_rotation, "
            "camera_to_robot_translation_mm, camera_point_unit_to_mm."
        ),
    )
    parser.add_argument(
        "--camera-result-json",
        type=Path,
        default=None,
        help="use an existing camera result JSON instead of running the camera, useful for dry tests",
    )
    parser.add_argument(
        "--camera-service-url",
        default=None,
        help=(
            "optional persistent camera service endpoint, for example "
            "http://127.0.0.1:5060/capture. If omitted, run/ring.py is launched as before."
        ),
    )
    parser.add_argument(
        "--camera-service-timeout-s",
        type=float,
        default=30.0,
        help="timeout for one camera service capture request",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="only print the generated record command, do not execute it",
    )
    parser.add_argument(
        "--full-auto",
        "--full_auto",
        dest="full_auto",
        action="store_true",
        help="automatically confirm all prompts, skip manual alignment, and start insertion",
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=DEFAULT_LOG_ROOT,
        help="root directory for per-experiment run.log files",
    )
    parser.add_argument(
        "--",
        dest="separator",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args, record_args = parser.parse_known_args()
    if record_args and record_args[0] == "--":
        record_args = record_args[1:]
    return args, record_args


def run_camera(args: argparse.Namespace) -> dict[str, Any]:
    if args.camera_result_json is not None:
        with args.camera_result_json.open("r", encoding="utf-8-sig") as f:
            return json.load(f)
    if args.camera_service_url:
        print("正在请求相机常驻服务：", args.camera_service_url)
        session = requests.Session()
        session.trust_env = False
        response = session.get(
            args.camera_service_url,
            params={"ip": args.camera_ip},
            timeout=args.camera_service_timeout_s,
        )
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", False):
            raise RuntimeError(f"Camera service failed: {payload}")
        result = payload.get("result")
        if not isinstance(result, dict):
            raise RuntimeError(f"Camera service response has no result dict: {payload}")
        print("相机服务返回结果:", result)
        return result

    command = [args.camera_python, str(CAMERA_SCRIPT), args.camera_ip]
    print("正在启动相机：", " ".join(command))
    completed = subprocess.run(
        command,
        cwd=str(CAMERA_DIR),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    print(completed.stdout)
    if completed.returncode != 0:
        raise RuntimeError(f"Camera script failed with exit code {completed.returncode}")
    return parse_camera_result(completed.stdout)


def load_record_module():
    spec = importlib.util.spec_from_file_location(
        "record_main_insert_dataset_module",
        RECORD_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load record script: {RECORD_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def connect_robot_and_screw(record_module, record_args: list[str]):
    connection_args = record_module.parse_args(record_args)
    robot = (
        record_module.URDriver(connection_args.robot_host)
        if connection_args.enable_robot
        else record_module.DryRunRobot()
    )
    screw_client = record_module.make_screw_client(
        host=connection_args.screw_host,
        port=connection_args.screw_port,
        timeout=connection_args.screw_timeout_s,
        dry_run=not connection_args.enable_robot,
    )
    try:
        print("正在与相机并行启动机械臂和电批连接...")
        if not robot.connect():
            raise RuntimeError("Robot connection failed")
        screw_client.connect()
        print("机械臂和电批连接已准备就绪。")
        return robot, screw_client
    except BaseException:
        try:
            if hasattr(screw_client, "close"):
                screw_client.close()
        finally:
            if hasattr(robot, "close"):
                robot.close()
        raise


def prepare_record_module_and_connections(record_args: list[str], full_auto: bool = False):
    """Load the recorder and establish hardware connections in one background task."""
    record_module = load_record_module()
    if full_auto:
        install_full_auto(record_module)
    robot, screw_client = connect_robot_and_screw(record_module, record_args)
    return record_module, robot, screw_client


def parse_camera_result(stdout: str) -> dict[str, Any]:
    for line in reversed(stdout.splitlines()):
        if "outer_center" not in line and "inner_center" not in line:
            continue
        payload = line
        if ":" in payload:
            payload = payload.split(":", 1)[1].strip()
        try:
            result = ast.literal_eval(payload)
        except (SyntaxError, ValueError):
            continue
        if isinstance(result, dict):
            return result
    raise RuntimeError("Could not parse camera output. Expected a final result dict with ring centers.")


def vector3(values: Any, name: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"{name} must contain 3 values, got {values}")
    return [float(value) for value in values]


def matrix3(values: Any, name: str) -> list[list[float]]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"{name} must be a 3x3 matrix")
    return [vector3(row, f"{name}[{index}]") for index, row in enumerate(values)]


def dot(a: list[float], b: list[float]) -> float:
    return sum(a[i] * b[i] for i in range(3))


def cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm(v: list[float]) -> float:
    return math.sqrt(dot(v, v))


def normalize(v: list[float], name: str) -> list[float]:
    length = norm(v)
    if length < 1e-12:
        raise ValueError(f"{name} is too small to normalize: {v}")
    return [value / length for value in v]


def mat_vec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [dot(row, vector) for row in matrix]


def rotvec_to_matrix(rotvec: list[float]) -> list[list[float]]:
    theta = norm(rotvec)
    if theta < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = [value / theta for value in rotvec]
    c = math.cos(theta)
    s = math.sin(theta)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ]


def matrix_to_rotvec(matrix: list[list[float]]) -> list[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return [0.0, 0.0, 0.0]
    if abs(math.pi - theta) < 1e-6:
        axis = [
            math.sqrt(max(0.0, (matrix[0][0] + 1.0) / 2.0)),
            math.sqrt(max(0.0, (matrix[1][1] + 1.0) / 2.0)),
            math.sqrt(max(0.0, (matrix[2][2] + 1.0) / 2.0)),
        ]
        if matrix[0][1] < 0.0:
            axis[1] = -axis[1]
        if matrix[0][2] < 0.0:
            axis[2] = -axis[2]
        axis = normalize(axis, "rotation axis")
        return [axis[i] * theta for i in range(3)]
    scale = theta / (2.0 * math.sin(theta))
    return [
        (matrix[2][1] - matrix[1][2]) * scale,
        (matrix[0][2] - matrix[2][0]) * scale,
        (matrix[1][0] - matrix[0][1]) * scale,
    ]


def orientation_from_tool_z(tool_z: list[float], reference_rotvec: list[float]) -> list[float]:
    z_axis = normalize(tool_z, "tool z axis")
    reference_matrix = rotvec_to_matrix(reference_rotvec)
    reference_x = [reference_matrix[0][0], reference_matrix[1][0], reference_matrix[2][0]]
    x_axis = [reference_x[i] - dot(reference_x, z_axis) * z_axis[i] for i in range(3)]
    if norm(x_axis) < 1e-6:
        reference_y = [reference_matrix[0][1], reference_matrix[1][1], reference_matrix[2][1]]
        x_axis = [reference_y[i] - dot(reference_y, z_axis) * z_axis[i] for i in range(3)]
    x_axis = normalize(x_axis, "tool x axis")
    y_axis = cross(z_axis, x_axis)
    rotation = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return matrix_to_rotvec(rotation)


def select_ring_center(camera_result: dict[str, Any], target_ring: str) -> list[float]:
    outer = camera_result.get("outer_center")
    inner = camera_result.get("inner_center")
    if target_ring == "outer":
        center = outer
    elif target_ring == "inner":
        center = inner if inner is not None else outer
    else:
        if outer is None or inner is None:
            center = inner if inner is not None else outer
        else:
            outer_vector = vector3(outer, "outer_center")
            inner_vector = vector3(inner, "inner_center")
            center = [(outer_vector[i] + inner_vector[i]) / 2.0 for i in range(3)]
    if center is None:
        raise RuntimeError(f"Camera result has no usable {target_ring} ring center: {camera_result}")
    return vector3(center, "ring center")


def outer_pointcloud_path(camera_result: dict[str, Any]) -> Path:
    crop_info = camera_result.get("crop_info") or {}
    output_files = crop_info.get("output_files") or {}
    outer_path = output_files.get("outer")
    if not outer_path:
        raise RuntimeError("Camera result did not include crop_info.output_files.outer")
    path = Path(outer_path)
    if not path.is_absolute():
        path = CAMERA_DIR / path
    return path


def outer_plane_normal(camera_result: dict[str, Any]) -> list[float]:
    normal = normalize(
        vector3(camera_result.get("outer_normal"), "outer_normal"),
        "outer plane normal",
    )
    outer_center = vector3(camera_result.get("outer_center"), "outer_center")
    if dot(normal, outer_center) < 0.0:
        normal = [-value for value in normal]
    return normal


def load_calibration(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def record_arg_value(record_args: list[str], name: str, default: str | None = None) -> str | None:
    try:
        index = record_args.index(name)
    except ValueError:
        return default
    if index + 1 >= len(record_args):
        return default
    return record_args[index + 1]


def current_robot_rotvec(record_args: list[str], fallback_rotvec: list[float]) -> list[float]:
    if "--enable-robot" not in record_args:
        print("Robot is not enabled; using fallback orientation from --base-approach-pose.")
        return fallback_rotvec

    host = record_arg_value(record_args, "--robot-host", "192.168.1.5")
    try:
        import rtde_receive
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Keeping the current robot orientation requires rtde_receive in this environment"
        ) from exc

    receiver = rtde_receive.RTDEReceiveInterface(host)
    try:
        current_pose = receiver.getActualTCPPose()
    finally:
        disconnect = getattr(receiver, "disconnect", None)
        if callable(disconnect):
            disconnect()

    current_pose = [float(value) for value in current_pose]
    if len(current_pose) != 6:
        raise RuntimeError(f"Unexpected robot TCP pose from {host}: {current_pose}")
    print("Current robot TCP pose [m, rad]:", current_pose)
    return current_pose[3:]


def camera_center_to_approach_pose(
    camera_center: list[float],
    camera_normal: list[float],
    base_approach_pose: list[float],
    calibration: dict[str, Any],
) -> list[float]:
    rotation = matrix3(
        calibration.get("camera_to_robot_rotation", DEFAULT_CAMERA_TO_ROBOT_ROTATION),
        "camera_to_robot_rotation",
    )
    translation = vector3(
        calibration.get(
            "camera_to_robot_translation_mm",
            DEFAULT_CAMERA_TO_ROBOT_TRANSLATION_MM,
        ),
        "camera_to_robot_translation_mm",
    )
    unit_to_mm = float(calibration.get("camera_point_unit_to_mm", 1.0))
    center_mm = [value * unit_to_mm for value in camera_center]
    center_robot = [
        mat_vec(rotation, center_mm)[i] + translation[i]
        for i in range(3)
    ]
    normal_robot = normalize(mat_vec(rotation, camera_normal), "robot plane normal")
    print("Plane normal in camera frame:", camera_normal)
    print("Plane normal in robot base frame:", normal_robot)
    tool_z = normal_robot
    tool_rotvec = orientation_from_tool_z(tool_z, base_approach_pose[3:])
    return [*center_robot, *tool_rotvec]


def strip_approach_pose(record_args: list[str]) -> list[str]:
    stripped: list[str] = []
    index = 0
    while index < len(record_args):
        if record_args[index] == "--approach-pose":
            index += 7
            continue
        stripped.append(record_args[index])
        index += 1
    return stripped


def install_full_auto(record_module) -> None:
    def auto_confirm(message: str) -> bool:
        print(f"[FULL_AUTO] 自动确认: {message}")
        return True

    def auto_manual_align(robot, approach_pose, args) -> bool:
        print("[FULL_AUTO] 跳过手动微调，直接接受当前 TCP 位姿。")
        print("Accepted aligned pose:", record_module.format_values(robot.get_tcp_pose()))
        return True

    def auto_read_key() -> str:
        print("[FULL_AUTO] 自动输入 P，启动下一步。")
        return "p"

    record_module.wait_for_enter_or_abort = auto_confirm
    record_module.manual_align = auto_manual_align
    record_module.read_key = auto_read_key
    for function_name in ("move_joint_to_pose", "move_linear"):
        function = getattr(record_module, function_name, None)
        globals_dict = getattr(function, "__globals__", None)
        if isinstance(globals_dict, dict):
            globals_dict["wait_for_enter_or_abort"] = auto_confirm
            globals_dict["read_key"] = auto_read_key


def install_timing_hooks(record_module, robot, screw_client, timer: StageTimer) -> None:
    original_get_joint_positions = robot.get_joint_positions

    def timed_get_joint_positions(*args, **kwargs):
        timer.mark_once("机械臂初始化完成")
        return original_get_joint_positions(*args, **kwargs)

    robot.get_joint_positions = timed_get_joint_positions

    original_move_joint_to_pose = record_module.move_joint_to_pose

    def timed_move_joint_to_pose(robot_arg, target_pose, args, label: str):
        timer.mark(f"{label}开始")
        print(f"[计时] {label} 开始")
        try:
            return original_move_joint_to_pose(robot_arg, target_pose, args, label)
        finally:
            timer.mark(f"{label}结束")
            print(f"[计时] {label} 结束")

    record_module.move_joint_to_pose = timed_move_joint_to_pose

    original_move_linear = record_module.move_linear

    def timed_move_linear(robot_arg, target, args, speed: float, label: str):
        timer.mark(f"{label}开始")
        print(f"[计时] {label} 开始")
        try:
            return original_move_linear(robot_arg, target, args, speed, label)
        finally:
            timer.mark(f"{label}结束")
            print(f"[计时] {label} 结束")

    record_module.move_linear = timed_move_linear

    if hasattr(screw_client, "set_speed"):
        original_set_speed = screw_client.set_speed

        def timed_set_speed(speed: float):
            timer.mark("电批启动命令发送")
            print(f"[计时] 电批启动命令发送: {speed} rpm")
            return original_set_speed(speed)

        screw_client.set_speed = timed_set_speed

    if hasattr(screw_client, "hold"):
        original_hold = screw_client.hold

        def timed_hold():
            timer.mark("电批停止命令发送")
            print("[计时] 电批停止命令发送")
            return original_hold()

        screw_client.hold = timed_hold


def print_timing_summary(timer: StageTimer) -> None:
    timer.mark("脚本结束")
    pairs = [
        ("相机启动 -> 相机拍摄和计算完成", "相机启动", "相机处理完成"),
        ("机械臂连接启动 -> 机械臂连接完成", "机械臂连接启动", "机械臂连接完成"),
        ("相机处理完成 -> 机械臂连接完成", "相机处理完成", "机械臂连接完成"),
        ("复用连接 -> 机械臂初始化完成", "记录流程启动", "机械臂初始化完成"),
        ("MOVE_APPROACH_MOVEJ 耗时", "MOVE_APPROACH_MOVEJ开始", "MOVE_APPROACH_MOVEJ结束"),
        ("INSERT_MOVEL 耗时", "INSERT_MOVEL开始", "INSERT_MOVEL结束"),
        ("记录流程总耗时", "记录流程启动", "记录流程结束"),
        ("脚本启动 -> 机械臂初始化完成", "脚本启动", "机械臂初始化完成"),
        ("脚本启动 -> 记录流程结束", "脚本启动", "记录流程结束"),
        ("全脚本实际总耗时", "脚本启动", "脚本结束"),
    ]
    print("")
    print("===== 并行耗时汇总 =====")
    for label, start, end in pairs:
        value = timer.elapsed(start, end)
        if value is not None:
            print(f"{label}: {format_seconds(value)}")
    print("===== 并行启动耗时测试结束 =====")


def main() -> None:
    args, record_args = parse_args()
    log_dir = make_experiment_dir(args.log_root)
    log_path = log_dir / "run.log"
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    timer = StageTimer()

    with log_path.open("w", encoding="utf-8") as log_file:
        sys.stdout = TeeStream(original_stdout, log_file)
        sys.stderr = TeeStream(original_stderr, log_file)
        try:
            _main_with_logging(args, record_args, timer, log_dir, log_path)
        finally:
            sys.stdout = original_stdout
            sys.stderr = original_stderr


def _main_with_logging(
    args: argparse.Namespace,
    record_args: list[str],
    timer: StageTimer,
    log_dir: Path,
    log_path: Path,
) -> None:
    print("===== parallel 全流程测试开始 =====")
    print(f"测试脚本: {Path(__file__).resolve()}")
    print(f"日志文件: {log_path}")
    print(f"实验目录: {log_dir}")
    print(f"开始时间: {datetime.now():%Y-%m-%d %H:%M:%S}")
    print(f"full_auto: {args.full_auto}")
    robot = None
    screw_client = None
    handed_off_connections = False

    with ThreadPoolExecutor(max_workers=2) as executor:
        # Start the camera immediately. Loading the recorder module used to happen
        # before this pool was created, delaying both the camera and robot startup.
        print("[阶段1] 正在同时启动相机和机械臂连接...")

        def camera_task():
            timer.mark("相机启动")
            print("[相机] 已启动相机任务")
            result = run_camera(args)
            timer.mark("相机处理完成")
            print("[相机] 拍摄、点云处理和目标位姿计算已完成")
            return result

        def connection_task():
            timer.mark("机械臂连接启动")
            print("[机械臂] 已启动机械臂和电批连接任务")
            result = prepare_record_module_and_connections(record_args, args.full_auto)
            timer.mark("机械臂连接完成")
            print("[机械臂] 机械臂和电批连接已完成")
            return result

        camera_future = executor.submit(camera_task)
        connection_future = executor.submit(
            connection_task,
        )
        try:
            camera_result = camera_future.result()
            record_module, robot, screw_client = connection_future.result()
        except BaseException:
            camera_future.cancel()
            if connection_future.done() and not connection_future.cancelled():
                connection_future.result()
            raise

    try:
        camera_center = select_ring_center(camera_result, args.target_ring)
        camera_normal = outer_plane_normal(camera_result)
        calibration = load_calibration(args.calibration_json)
        approach_pose = camera_center_to_approach_pose(
            camera_center,
            camera_normal,
            args.base_approach_pose,
            calibration,
        )

        print("Selected camera center:", camera_center)
        print("Generated approach pose [mm, rad]:", approach_pose)

        record_args = strip_approach_pose(record_args)
        record_module_args = [
            "--approach-pose",
            *[f"{value:.12g}" for value in approach_pose],
            *record_args,
        ]
        record_args = record_module.parse_args(record_module_args)
        print("正在当前进程中复用已连接的机械臂和电批执行记录流程。")
        if args.print_only:
            return

        install_timing_hooks(record_module, robot, screw_client, timer)
        handed_off_connections = True
        timer.mark("记录流程启动")
        print("[阶段2] 开始复用已连接设备执行记录流程")
        result = record_module.run(
            record_args,
            robot=robot,
            screw_client=screw_client,
        )
        timer.mark("记录流程结束")
        if result not in (record_module.DemoState.DONE, record_module.DemoState.ABORTED):
            raise SystemExit(1)
    finally:
        if not handed_off_connections:
            if screw_client is not None:
                screw_client.close()
            if robot is not None:
                robot.close()
        print_timing_summary(timer)


if __name__ == "__main__":
    main()

# /home/sjh/anaconda3/envs/lerobot_eye/bin/python   "lerobot_robot_screw/scripts/record_camera_insert_dataset.py"   --camera-ip auto   --target-ring outer   --   --enable-robot   --robot-host 192.168.1.5   --insert-depth 40   --insert-speed 3   --insert-sign 1   --screw-host 127.0.0.1   --screw-port 5055   --screw-speed-rpm 60   --dataset-root "lerobot_robot_screw/output/real_insert_dataset_${RUN_ID}"   --repo-id "local/real_screw_insert_${RUN_ID}"   --fps 20
