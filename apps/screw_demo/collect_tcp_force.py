# PROJECT FILE HEADER
# 文件：apps/screw_demo/collect_tcp_force.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse
import csv
import time
from pathlib import Path
from typing import Iterable, List

import numpy as np

from ur_driver import URDriver


REPO_ROOT = Path(__file__).resolve().parents[2]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


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
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


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


def mean_force(robot: URDriver, samples: int, dt: float) -> np.ndarray:
    values: List[List[float]] = []
    for _ in range(samples):
        values.append(robot.get_tcp_force())
        time.sleep(dt)
    return np.mean(np.asarray(values, dtype=float), axis=0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect TCP force/wrench without controlling motion.")
    parser.add_argument("--host", default="192.168.1.5", help="UR robot IP address")
    parser.add_argument(
        "--out-dir",
        default="artifacts/logs/screw_demo",
        help="directory for CSV output",
    )
    parser.add_argument("--name", default=None, help="optional CSV file name without extension")
    parser.add_argument("--duration", type=float, default=20.0, help="collection time in seconds")
    parser.add_argument("--hz", type=float, default=50.0, help="sampling rate")
    parser.add_argument("--bias-samples", type=int, default=50, help="stationary samples used as force bias")
    parser.add_argument("--label", default="", help="free_space, normal_insert, offset_contact, etc.")
    parser.add_argument("--insert-sign", type=float, default=1.0, choices=(-1.0, 1.0))
    args = parser.parse_args()

    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if args.hz <= 0.0:
        raise ValueError("--hz must be positive")

    out_dir = resolve_repo_path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    name = args.name or f"tcp_force_{time.strftime('%Y%m%d_%H%M%S')}"
    out_path = out_dir / f"{name}.csv"

    period = 1.0 / args.hz
    fieldnames = [
        "time",
        "elapsed",
        "label",
        "x",
        "y",
        "z",
        "rx",
        "ry",
        "rz",
        "Fx",
        "Fy",
        "Fz",
        "Tx",
        "Ty",
        "Tz",
        "f_axial",
        "f_lateral",
    ]

    with URDriver(args.host, connect_control=False) as robot, out_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        print(f"Collecting {args.bias_samples} stationary samples as force bias...")
        bias = mean_force(robot, args.bias_samples, period)
        print("Force bias:", bias.tolist())
        print(f"Writing force data to {out_path}")

        start = time.time()
        next_time = start
        while True:
            now = time.time()
            elapsed = now - start
            if elapsed > args.duration:
                break

            pose = np.asarray(robot.get_tcp_pose(), dtype=float)
            wrench = np.asarray(robot.get_tcp_force(), dtype=float) - bias
            f_axial, f_lateral = split_axial_lateral_force(pose, wrench, args.insert_sign)

            writer.writerow(
                {
                    "time": now,
                    "elapsed": elapsed,
                    "label": args.label,
                    "x": pose[0],
                    "y": pose[1],
                    "z": pose[2],
                    "rx": pose[3],
                    "ry": pose[4],
                    "rz": pose[5],
                    "Fx": wrench[0],
                    "Fy": wrench[1],
                    "Fz": wrench[2],
                    "Tx": wrench[3],
                    "Ty": wrench[4],
                    "Tz": wrench[5],
                    "f_axial": f_axial,
                    "f_lateral": f_lateral,
                }
            )
            f.flush()

            next_time += period
            time.sleep(max(0.0, next_time - time.time()))

    print(f"Done: {out_path}")


if __name__ == "__main__":
    main()
