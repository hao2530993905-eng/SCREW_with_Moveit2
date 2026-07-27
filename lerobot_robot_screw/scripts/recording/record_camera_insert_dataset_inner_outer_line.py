#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/recording/record_camera_insert_dataset_inner_outer_line.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
import ast
import importlib.util
import math
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
CAMERA_DIR = REPO_ROOT / "apps" / "camera"
CAMERA_SCRIPT = CAMERA_DIR / "ring.py"
RECORD_SCRIPT = REPO_ROOT / "lerobot_robot_screw" / "scripts" / "recording" / "record_main_insert_dataset.py"

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


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def resolve_optional_repo_path(path: Path | None) -> Path | None:
    if path is None:
        return None
    return resolve_repo_path(path)


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
        default="auto",
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
        "--print-only",
        action="store_true",
        help="only print the generated record command, do not execute it",
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
    args.calibration_json = resolve_optional_repo_path(args.calibration_json)
    args.camera_result_json = resolve_optional_repo_path(args.camera_result_json)
    return args, record_args


def run_camera(args: argparse.Namespace) -> dict[str, Any]:
    if args.camera_result_json is not None:
        with args.camera_result_json.open("r", encoding="utf-8-sig") as f:
            return json.load(f)

    command = [args.camera_python, str(CAMERA_SCRIPT), args.camera_ip]
    print("Running camera command:", " ".join(command))
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
        print("Starting robot and screwdriver connections in parallel with the camera...")
        if not robot.connect():
            raise RuntimeError("Robot connection failed")
        screw_client.connect()
        print("Robot and screwdriver connections are ready.")
        return robot, screw_client
    except BaseException:
        try:
            if hasattr(screw_client, "close"):
                screw_client.close()
        finally:
            if hasattr(robot, "close"):
                robot.close()
        raise


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


def inner_outer_line_normal(camera_result: dict[str, Any]) -> list[float]:
    outer_center = vector3(camera_result.get("outer_center"), "outer_center")
    inner_center = vector3(camera_result.get("inner_center"), "inner_center")
    line = [inner_center[i] - outer_center[i] for i in range(3)]
    normal = normalize(line, "inner-to-outer center line")

    # Choose the sign whose direction points away from the camera origin and into the hole.
    if dot(normal, inner_center) < 0.0:
        normal = [-value for value in normal]

    print("Outer ring center in camera frame:", outer_center)
    print("Inner ring center in camera frame:", inner_center)
    print("Final direction in camera frame (outer center to inner center):", normal)
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
    normal_robot = normalize(mat_vec(rotation, camera_normal), "robot target direction")
    print("Final direction in robot base frame:", normal_robot)
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


def main() -> None:
    args, record_args = parse_args()
    record_module = load_record_module()
    robot = None
    screw_client = None
    handed_off_connections = False

    with ThreadPoolExecutor(max_workers=2) as executor:
        camera_future = executor.submit(run_camera, args)
        connection_future = executor.submit(
            connect_robot_and_screw,
            record_module,
            record_args,
        )
        try:
            camera_result = camera_future.result()
            robot, screw_client = connection_future.result()
        except BaseException:
            camera_future.cancel()
            if connection_future.done() and not connection_future.cancelled():
                connection_future.result()
            raise

    try:
        camera_center = select_ring_center(camera_result, args.target_ring)
        camera_normal = inner_outer_line_normal(camera_result)
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
        print("Running record flow in the current process with the preconnected robot.")
        if args.print_only:
            return

        handed_off_connections = True
        result = record_module.run(
            record_args,
            robot=robot,
            screw_client=screw_client,
        )
        if result not in (record_module.DemoState.DONE, record_module.DemoState.ABORTED):
            raise SystemExit(1)
    finally:
        if not handed_off_connections:
            if screw_client is not None:
                screw_client.close()
            if robot is not None:
                robot.close()


if __name__ == "__main__":
    main()

# /home/sjh/anaconda3/envs/lerobot_eye/bin/python   "lerobot_robot_screw/scripts/recording/record_camera_insert_dataset.py"   --camera-ip auto   --target-ring outer   --   --enable-robot   --robot-host 192.168.1.5   --insert-depth 40   --insert-speed 3   --insert-sign 1   --screw-host 127.0.0.1   --screw-port 5055   --screw-speed-rpm 60   --dataset-root "artifacts/datasets/real_insert_dataset_${RUN_ID}"   --repo-id "local/real_screw_insert_${RUN_ID}"   --fps 20
