#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/calc_robot_frame_pointcloud_pose.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


REPO_ROOT = Path(__file__).resolve().parents[3]
CAMERA_RECORD_SCRIPT = REPO_ROOT / "lerobot_robot_screw" / "scripts" / "recording" / "record_camera_insert_dataset.py"


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def resolve_optional_repo_path(path: Path | None) -> Path | None:
    if path is None:
        return None
    return resolve_repo_path(path)


def load_camera_record_module():
    spec = importlib.util.spec_from_file_location("record_camera_insert_dataset_copy", CAMERA_RECORD_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {CAMERA_RECORD_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run camera detection, transform the outer-ring point cloud into the robot base frame, "
            "fit the plane normal in robot coordinates, and generate a target pose."
        )
    )
    parser.add_argument("--camera-ip", default="auto")
    parser.add_argument("--target-ring", choices=("inner", "outer", "midpoint"), default="outer")
    parser.add_argument("--normal-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--calibration-json", type=Path, default=None)
    parser.add_argument("--camera-result-json", type=Path, default=None)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--camera-python", default=sys.executable)
    args = parser.parse_args()
    args.calibration_json = resolve_optional_repo_path(args.calibration_json)
    args.camera_result_json = resolve_optional_repo_path(args.camera_result_json)
    args.output_json = resolve_optional_repo_path(args.output_json)
    return args


def transform_points_to_robot(points_camera: np.ndarray, rotation, translation, unit_to_mm: float) -> np.ndarray:
    rotation_np = np.asarray(rotation, dtype=float)
    translation_np = np.asarray(translation, dtype=float)
    points_mm = points_camera.astype(float) * unit_to_mm
    return points_mm @ rotation_np.T + translation_np


def fit_normal(points_robot: np.ndarray) -> list[float]:
    if points_robot.shape[0] < 3:
        raise RuntimeError("Need at least 3 points to fit a plane normal")
    centered = points_robot - points_robot.mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1].astype(float)
    normal /= np.linalg.norm(normal)
    return normal.tolist()


def main() -> None:
    args = parse_args()
    camera_record = load_camera_record_module()

    camera_args = argparse.Namespace(
        camera_ip=args.camera_ip,
        camera_python=args.camera_python,
        camera_result_json=args.camera_result_json,
    )
    camera_result = camera_record.run_camera(camera_args)
    camera_center = camera_record.select_ring_center(camera_result, args.target_ring)
    pointcloud_path = camera_record.outer_pointcloud_path(camera_result)
    calibration = camera_record.load_calibration(args.calibration_json)

    rotation = camera_record.matrix3(
        calibration.get("camera_to_robot_rotation", camera_record.DEFAULT_CAMERA_TO_ROBOT_ROTATION),
        "camera_to_robot_rotation",
    )
    translation = camera_record.vector3(
        calibration.get(
            "camera_to_robot_translation_mm",
            camera_record.DEFAULT_CAMERA_TO_ROBOT_TRANSLATION_MM,
        ),
        "camera_to_robot_translation_mm",
    )
    unit_to_mm = float(calibration.get("camera_point_unit_to_mm", 1.0))

    pcd_camera = o3d.io.read_point_cloud(str(pointcloud_path))
    points_camera = np.asarray(pcd_camera.points, dtype=float)
    points_robot = transform_points_to_robot(points_camera, rotation, translation, unit_to_mm)
    normal_robot = fit_normal(points_robot)
    normal_robot = [args.normal_sign * value for value in normal_robot]

    center_robot = camera_record.mat_vec(rotation, [value * unit_to_mm for value in camera_center])
    center_robot = [center_robot[i] + translation[i] for i in range(3)]

    target_rotvec = camera_record.orientation_from_tool_z(
        normal_robot,
        camera_record.DEFAULT_BASE_APPROACH_POSE_MM_RAD[3:],
    )
    target_pose = [*center_robot, *target_rotvec]

    result = {
        "camera_center": camera_center,
        "pointcloud_path": str(pointcloud_path),
        "robot_center_mm": center_robot,
        "robot_normal": normal_robot,
        "target_pose_mm_rad": target_pose,
        "target_xyz_mm": target_pose[:3],
        "target_rotvec_rad": target_pose[3:],
    }

    print("Selected camera center:", camera_center)
    print("Outer pointcloud:", pointcloud_path)
    print("Robot-frame center [mm]:", center_robot)
    print("Robot-frame fitted normal:", normal_robot)
    print("Generated target pose [mm, rad]:", target_pose)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("Saved result JSON:", args.output_json)


if __name__ == "__main__":
    main()
