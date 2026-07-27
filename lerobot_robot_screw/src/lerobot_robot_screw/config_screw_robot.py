# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/src/lerobot_robot_screw/config_screw_robot.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("screw_robot")
@dataclass
class ScrewRobotConfig(RobotConfig):
    """Configuration for the SCREW robot LeRobot wrapper."""

    host: str = "192.168.1.5"
    port: int = 30003
    dry_run: bool = True
    fps: int = 20
    jog_step_m: float = 0.0005
    jog_step_rad: float = 0.01
    max_translation_step_m: float = 0.002
    max_rotation_step_rad: float = 0.05
    max_align_offset_m: float = 0.010
    image_height: int = 64
    image_width: int = 64
    use_images: bool = True
    ur_driver_module: str = "apps.screw_demo.ur_driver"
    ur_driver_class: str = "URDriver"
    ur_connect_control: bool = True
    screw_client_module: str = "apps.screw_demo.screw_client"
    screw_client_factory: str = "make_screw_client"
    screw_host: str = "127.0.0.1"
    screw_port: int = 5055
    screw_timeout_s: float = 2.0
    max_screw_speed_rpm: float = 300.0
