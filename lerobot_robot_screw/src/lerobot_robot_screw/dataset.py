# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/src/lerobot_robot_screw/dataset.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np

from .config_screw_robot import ScrewRobotConfig
from .keyboard_action import action_from_key
from .processors import ACTION_NAMES, STATE_NAMES
from .screw_robot import ScrewRobot

OBS_STATE = "observation.state"
OBS_IMAGE = "observation.images.front"
OBS_SCREW_STATE = "observation.screw_state"
ACTION = "action"


def dataset_features(height: int = 64, width: int = 64, use_videos: bool = True) -> dict:
    image_dtype = "video" if use_videos else "image"
    return {
        OBS_STATE: {"dtype": "float32", "shape": (13,), "names": STATE_NAMES},
        ACTION: {"dtype": "float32", "shape": (7,), "names": ACTION_NAMES},
        OBS_SCREW_STATE: {
            "dtype": "float32",
            "shape": (7,),
            "names": [
                "target_speed_rpm",
                "measured_speed_rpm",
                "motor_position_rad",
                "motor_velocity_rad_s",
                "current_amp",
                "torque_nm",
                "fault",
            ],
        },
        OBS_IMAGE: {
            "dtype": image_dtype,
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        "phase_id": {"dtype": "float32", "shape": (1,), "names": ["phase_id"]},
        "hole_id": {"dtype": "string", "shape": (1,), "names": ["hole_id"]},
    }


def generate_smoke_dataset(
    root: str | Path,
    repo_id: str = "local/screw_robot_smoke",
    frames: int = 12,
    fps: int = 20,
    use_videos: bool = True,
    overwrite: bool = True,
) -> Path:
    # Importing LeRobotDataset loads a large dependency graph. Keep it lazy so
    # modules that only need feature definitions do not pay that startup cost.
    from lerobot.datasets import LeRobotDataset

    root = Path(root)
    if overwrite and root.exists():
        shutil.rmtree(root)

    config = ScrewRobotConfig(fps=fps, dry_run=True, image_height=64, image_width=64)
    robot = ScrewRobot(config)
    robot.connect()
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=dataset_features(config.image_height, config.image_width, use_videos),
        root=root,
        robot_type=config.type,
        use_videos=use_videos,
        image_writer_processes=0,
        image_writer_threads=0,
    )

    keys = ["right", "up", "u", "left", "down", "j", "d", "a"]
    for frame_index in range(frames):
        phase_id = frame_index % 4
        robot._phase_id = phase_id
        action = action_from_key(keys[frame_index % len(keys)], config)
        applied = robot.send_action({ACTION: action})[ACTION]
        obs = robot.get_observation()
        dataset.add_frame(
            {
                OBS_STATE: obs[OBS_STATE],
                OBS_SCREW_STATE: obs[OBS_SCREW_STATE],
                OBS_IMAGE: obs[OBS_IMAGE],
                ACTION: applied.astype(np.float32),
                "phase_id": np.asarray([phase_id], dtype=np.float32),
                "hole_id": "hole_000",
                "task": "dry-run screw one hole",
            }
        )

    dataset.save_episode()
    dataset.finalize()
    robot.disconnect()
    return root
