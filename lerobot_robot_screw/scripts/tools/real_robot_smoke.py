#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/tools/real_robot_smoke.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from lerobot.configs.parser import wrap
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config

from lerobot_robot_screw.dataset import ACTION, OBS_SCREW_STATE, OBS_STATE


REPO_ROOT = Path(__file__).resolve().parents[3]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


@dataclass
class RealRobotSmokeConfig:
    robot: RobotConfig
    read_frames: int = 5
    output_path: str = "artifacts/datasets/real_robot_smoke/readings.jsonl"
    send_test_action: bool = False
    action_duration_s: float = 2.0
    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0
    drx_rad: float = 0.0
    dry_rad: float = 0.0
    drz_rad: float = 0.0
    screw_speed_rpm: float = 0.0


def to_jsonable(value):
    if isinstance(value, np.ndarray):
        return value.astype(float).tolist()
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def summarize_observation(frame_index: int, obs: dict) -> dict:
    return {
        "frame_index": frame_index,
        "timestamp": time.time(),
        OBS_STATE: to_jsonable(obs[OBS_STATE]),
        "observation.tcp_pose": to_jsonable(obs["observation.tcp_pose"]),
        "observation.tcp_force": to_jsonable(obs["observation.tcp_force"]),
        OBS_SCREW_STATE: to_jsonable(obs[OBS_SCREW_STATE]),
        "phase_id": to_jsonable(obs["phase_id"]),
        "screw_status": obs.get("screw_status", {}),
    }


@wrap()
def main(cfg: RealRobotSmokeConfig) -> None:
    output_path = resolve_repo_path(cfg.output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    robot = make_robot_from_config(cfg.robot)
    robot.connect()

    try:
        with output_path.open("w", encoding="utf-8") as f:
            for frame_index in range(cfg.read_frames):
                obs = robot.get_observation()
                row = summarize_observation(frame_index, obs)
                f.write(json.dumps(row, ensure_ascii=False) + "\n")
                print(json.dumps(row, ensure_ascii=False))
                time.sleep(1.0 / max(cfg.robot.fps, 1))

            if cfg.send_test_action:
                action = np.asarray(
                    [
                        cfg.dx_m,
                        cfg.dy_m,
                        cfg.dz_m,
                        cfg.drx_rad,
                        cfg.dry_rad,
                        cfg.drz_rad,
                        cfg.screw_speed_rpm,
                    ],
                    dtype=np.float32,
                )
                applied = robot.send_action({ACTION: action})
                print(json.dumps({"sent_action": to_jsonable(action), "applied_action": to_jsonable(applied[ACTION])}))
                started = time.monotonic()
                action_frame = 0
                while time.monotonic() - started < cfg.action_duration_s:
                    if hasattr(robot, "heartbeat_screw_driver"):
                        robot.heartbeat_screw_driver()
                    obs = robot.get_observation()
                    row = summarize_observation(cfg.read_frames + action_frame, obs)
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
                    print(json.dumps(row, ensure_ascii=False))
                    action_frame += 1
                    time.sleep(0.1)
    finally:
        robot.disconnect()

    print(f"saved readings: {output_path}")


if __name__ == "__main__":
    main()
