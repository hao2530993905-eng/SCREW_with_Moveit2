# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/src/lerobot_robot_screw/keyboard_action.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import numpy as np

from .config_screw_robot import ScrewRobotConfig


KEY_TO_AXIS_SIGN = {
    "left": (0, -1.0),
    "right": (0, 1.0),
    "down": (1, -1.0),
    "up": (1, 1.0),
    "j": (2, -1.0),
    "u": (2, 1.0),
    "a": (5, -1.0),
    "d": (5, 1.0),
}


def action_from_key(key: str, config: ScrewRobotConfig | None = None) -> np.ndarray:
    """Convert one keyboard jog key to a LeRobot 6D delta TCP action."""

    cfg = config or ScrewRobotConfig()
    action = np.zeros(7, dtype=np.float32)
    if key not in KEY_TO_AXIS_SIGN:
        return action

    axis, sign = KEY_TO_AXIS_SIGN[key]
    step = cfg.jog_step_m if axis < 3 else cfg.jog_step_rad
    action[axis] = sign * step
    return action


def action_sequence_from_keys(keys: list[str], config: ScrewRobotConfig | None = None) -> list[np.ndarray]:
    return [action_from_key(key, config) for key in keys]
