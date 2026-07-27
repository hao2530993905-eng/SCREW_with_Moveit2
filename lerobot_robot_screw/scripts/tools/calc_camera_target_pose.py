#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/calc_camera_target_pose.py
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
    parser = argparse.ArgumentParser(description="Calculate the target robot pose for the detected hole only.")
    parser.add_argument("--camera-ip", default="auto")
    parser.add_argument("--target-ring", choices=("inner", "outer", "midpoint"), default="outer")
    parser.add_argument("--calibration-json", type=Path, default=None)
    parser.add_argument("--camera-result-json", type=Path, default=None)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--camera-python", default=sys.executable)
    args = parser.parse_args()
    args.calibration_json = resolve_optional_repo_path(args.calibration_json)
    args.camera_result_json = resolve_optional_repo_path(args.camera_result_json)
    args.output_json = resolve_optional_repo_path(args.output_json)
    return args


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
    camera_normal = camera_record.outer_plane_normal(camera_result)
    calibration = camera_record.load_calibration(args.calibration_json)
    target_pose = camera_record.camera_center_to_approach_pose(
        camera_center,
        camera_normal,
        camera_record.DEFAULT_BASE_APPROACH_POSE_MM_RAD,
        calibration,
    )

    result = {
        "camera_center": camera_center,
        "camera_normal": camera_normal,
        "target_pose_mm_rad": target_pose,
        "target_xyz_mm": target_pose[:3],
        "target_rotvec_rad": target_pose[3:],
    }
    print("Selected camera center:", camera_center)
    print("Fitted outer plane normal in camera frame:", camera_normal)
    print("Generated target pose [mm, rad]:", target_pose)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("Saved result JSON:", args.output_json)


if __name__ == "__main__":
    main()
