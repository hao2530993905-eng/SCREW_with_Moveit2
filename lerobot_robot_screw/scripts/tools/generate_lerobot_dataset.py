#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/generate_lerobot_dataset.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
from pathlib import Path

from lerobot_robot_screw.dataset import generate_smoke_dataset


REPO_ROOT = Path(__file__).resolve().parents[3]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", type=str, default="local/screw_robot_smoke")
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--use-videos", action="store_true")
    parser.add_argument("--no-overwrite", action="store_true")
    args = parser.parse_args()

    root = generate_smoke_dataset(
        root=resolve_repo_path(args.root),
        repo_id=args.repo_id,
        frames=args.frames,
        fps=args.fps,
        use_videos=args.use_videos,
        overwrite=not args.no_overwrite,
    )
    print(f"created LeRobotDataset at {root}")


if __name__ == "__main__":
    main()
