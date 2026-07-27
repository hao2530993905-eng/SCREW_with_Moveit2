#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/visualize_lerobot_dataset.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
from pathlib import Path

from lerobot.datasets import LeRobotDataset
from lerobot.scripts.lerobot_dataset_viz import visualize_dataset
from lerobot.utils.utils import init_logging


REPO_ROOT = Path(__file__).resolve().parents[3]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", type=str, default="local/screw_robot_video_smoke")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--video-backend", type=str, default="pyav", choices=["pyav", "torchcodec"])
    args = parser.parse_args()

    init_logging()
    dataset = LeRobotDataset(
        args.repo_id,
        episodes=[args.episode_index],
        root=resolve_repo_path(args.root),
        tolerance_s=1e-4,
        video_backend=args.video_backend,
    )
    rrd_path = visualize_dataset(
        dataset,
        episode_index=args.episode_index,
        batch_size=8,
        num_workers=0,
        mode="local",
        save=True,
        output_dir=resolve_repo_path(args.output_dir),
    )
    print(f"saved visualizer recording at {rrd_path}")


if __name__ == "__main__":
    main()
