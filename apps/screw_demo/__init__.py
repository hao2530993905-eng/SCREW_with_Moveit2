# PROJECT FILE HEADER
# 文件：apps/screw_demo/__init__.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

"""
PROJECT FILE HEADER
文件：apps/screw_demo/__init__.py
作用：UR5e、电批和力数据实验程序包初始化。
用法：python apps/screw_demo/main.py --help
注意：实验日志统一写入 artifacts/logs/。
END PROJECT FILE HEADER

UR5e and screwdriver experiment application.

Usage:
    python apps/screw_demo/main.py --help
    python apps/screw_demo/teach.py --help

This package is imported by the LeRobot robot adapter when real hardware mode is enabled.
"""
