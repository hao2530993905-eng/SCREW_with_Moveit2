#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/sample_lerobot_dataset.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
import random
from pathlib import Path

import torch

from lerobot.datasets import LeRobotDataset

from lerobot_robot_screw.dataset import ACTION, OBS_IMAGE, OBS_SCREW_STATE, OBS_STATE


REPO_ROOT = Path(__file__).resolve().parents[3]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def describe(value) -> str:
    if isinstance(value, torch.Tensor):
        return f"torch {tuple(value.shape)} {value.dtype}"
    shape = getattr(value, "shape", None)
    dtype = getattr(value, "dtype", None)
    return f"{type(value).__name__} shape={shape} dtype={dtype}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", type=str, default="local/screw_robot_smoke")
    parser.add_argument("--video-backend", type=str, default=None, choices=["pyav", "torchcodec"])
    args = parser.parse_args()

    dataset = LeRobotDataset(args.repo_id, root=resolve_repo_path(args.root), tolerance_s=1e-4, video_backend=args.video_backend)
    index = random.randrange(len(dataset))
    sample = dataset[index]
    print(f"dataset_len: {len(dataset)}")
    print(f"sample_index: {index}")
    print(f"{OBS_STATE}: {describe(sample[OBS_STATE])} first={sample[OBS_STATE][:4]}")
    print(f"{OBS_SCREW_STATE}: {describe(sample[OBS_SCREW_STATE])} value={sample[OBS_SCREW_STATE]}")
    print(f"{ACTION}: {describe(sample[ACTION])} value={sample[ACTION]}")
    print(f"{OBS_IMAGE}: {describe(sample[OBS_IMAGE])}")
    print(f"timestamp: {sample['timestamp']}")
    print(f"phase_id: {sample['phase_id']}")
    print(f"hole_id: {sample['hole_id']}")


if __name__ == "__main__":
    main()
