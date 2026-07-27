#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/simulate_and_save_readings.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from lerobot_robot_screw.config_screw_robot import ScrewRobotConfig
from lerobot_robot_screw.dataset import ACTION, OBS_SCREW_STATE, OBS_STATE
from lerobot_robot_screw.keyboard_action import action_from_key
from lerobot_robot_screw.processors import ACTION_NAMES, STATE_NAMES
from lerobot_robot_screw.screw_robot import ScrewRobot


REPO_ROOT = Path(__file__).resolve().parents[3]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def to_list(value: np.ndarray) -> list[float]:
    return [float(item) for item in value.tolist()]


def simulate_readings(output_dir: Path, frames: int, fps: int) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)

    config = ScrewRobotConfig(dry_run=True, fps=fps, use_images=False)
    robot = ScrewRobot(config)
    robot.connect()

    keys = ["right", "up", "u", "left", "down", "j", "d", "a"]
    rows: list[dict] = []
    for frame_index in range(frames):
        key = keys[frame_index % len(keys)]
        phase_id = frame_index % 4
        robot._phase_id = phase_id

        command_action = action_from_key(key, config)
        if 4 <= frame_index < 8:
            command_action[6] = 120.0
        elif 8 <= frame_index < 12:
            command_action[6] = -60.0
        applied_action = robot.send_action({ACTION: command_action})[ACTION]
        observation = robot.get_observation()

        rows.append(
            {
                "frame_index": frame_index,
                "timestamp": frame_index / fps,
                "key": key,
                "phase_id": phase_id,
                "command_action": to_list(command_action),
                "applied_action": to_list(applied_action),
                "tcp_pose": to_list(observation["observation.tcp_pose"]),
                "tcp_force": to_list(observation["observation.tcp_force"]),
                OBS_STATE: to_list(observation[OBS_STATE]),
                OBS_SCREW_STATE: to_list(observation[OBS_SCREW_STATE]),
                "screw_status": observation["screw_status"],
            }
        )

    robot.disconnect()

    jsonl_path = output_dir / "readings.jsonl"
    csv_path = output_dir / "readings.csv"

    with jsonl_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    fieldnames = ["frame_index", "timestamp", "key", "phase_id"]
    fieldnames += [f"command_{name}" for name in ACTION_NAMES]
    fieldnames += [f"applied_{name}" for name in ACTION_NAMES]
    fieldnames += [f"tcp_pose_{i}" for i in range(6)]
    fieldnames += [f"tcp_force_{i}" for i in range(6)]
    fieldnames += [f"state_{name}" for name in STATE_NAMES]
    fieldnames += [
        "screw_target_speed_rpm",
        "screw_measured_speed_rpm",
        "screw_motor_position_rad",
        "screw_motor_velocity_rad_s",
        "screw_current_amp",
        "screw_torque_nm",
        "screw_fault",
    ]

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat_row = {
                "frame_index": row["frame_index"],
                "timestamp": row["timestamp"],
                "key": row["key"],
                "phase_id": row["phase_id"],
            }
            flat_row.update(
                {f"command_{name}": value for name, value in zip(ACTION_NAMES, row["command_action"])}
            )
            flat_row.update(
                {f"applied_{name}": value for name, value in zip(ACTION_NAMES, row["applied_action"])}
            )
            flat_row.update({f"tcp_pose_{i}": value for i, value in enumerate(row["tcp_pose"])})
            flat_row.update({f"tcp_force_{i}": value for i, value in enumerate(row["tcp_force"])})
            flat_row.update({f"state_{name}": value for name, value in zip(STATE_NAMES, row[OBS_STATE])})
            flat_row.update(
                {
                    "screw_target_speed_rpm": row[OBS_SCREW_STATE][0],
                    "screw_measured_speed_rpm": row[OBS_SCREW_STATE][1],
                    "screw_motor_position_rad": row[OBS_SCREW_STATE][2],
                    "screw_motor_velocity_rad_s": row[OBS_SCREW_STATE][3],
                    "screw_current_amp": row[OBS_SCREW_STATE][4],
                    "screw_torque_nm": row[OBS_SCREW_STATE][5],
                    "screw_fault": row[OBS_SCREW_STATE][6],
                }
            )
            writer.writerow(flat_row)

    return jsonl_path, csv_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/datasets/sim_readings"),
    )
    parser.add_argument("--frames", type=int, default=16)
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    jsonl_path, csv_path = simulate_readings(resolve_repo_path(args.output_dir), args.frames, args.fps)
    print(f"saved jsonl: {jsonl_path}")
    print(f"saved csv: {csv_path}")


if __name__ == "__main__":
    main()
