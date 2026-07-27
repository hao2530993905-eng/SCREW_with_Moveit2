# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/src/lerobot_robot_screw/__init__.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

"""SCREW robot plugin for LeRobot.

Importing this package registers ``--robot.type=screw_robot`` through
``ScrewRobotConfig``.
"""

from .config_screw_robot import ScrewRobotConfig
from .screw_robot import ScrewRobot

__all__ = ["ScrewRobot", "ScrewRobotConfig"]

