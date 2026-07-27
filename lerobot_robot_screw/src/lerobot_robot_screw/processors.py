# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/src/lerobot_robot_screw/processors.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import numpy as np


STATE_NAMES = [
    "tcp_x_local_m",
    "tcp_y_local_m",
    "tcp_z_local_m",
    "tcp_rx_local_rad",
    "tcp_ry_local_rad",
    "tcp_rz_local_rad",
    "force_x_n",
    "force_y_n",
    "force_z_n",
    "torque_x_nm",
    "torque_y_nm",
    "torque_z_nm",
    "phase_id",
]

ACTION_NAMES = [
    "delta_x_m",
    "delta_y_m",
    "delta_z_m",
    "delta_rx_rad",
    "delta_ry_rad",
    "delta_rz_rad",
    "screw_speed_rpm",
]

SCREW_STATE_NAMES = [
    "target_speed_rpm",
    "measured_speed_rpm",
    "motor_position_rad",
    "motor_velocity_rad_s",
    "current_amp",
    "torque_nm",
    "fault",
]


def as_pose6(value: np.ndarray | list[float] | tuple[float, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {array.shape}")
    return array


def tcp_pose_to_hole_frame(
    tcp_pose: np.ndarray | list[float] | tuple[float, ...],
    hole_origin_pose: np.ndarray | list[float] | tuple[float, ...],
) -> np.ndarray:
    """Return TCP pose error expressed relative to the current hole origin.

    This dry-run processor uses the same 6D pose convention as the existing UR
    driver path: xyz in meters followed by rx/ry/rz in radians.
    """

    return as_pose6(tcp_pose, "tcp_pose") - as_pose6(hole_origin_pose, "hole_origin_pose")


def build_observation_state(
    tcp_pose: np.ndarray | list[float] | tuple[float, ...],
    tcp_force: np.ndarray | list[float] | tuple[float, ...],
    phase_id: int | float,
    hole_origin_pose: np.ndarray | list[float] | tuple[float, ...] | None = None,
) -> np.ndarray:
    """Build LeRobot ``observation.state`` as 13 float32 values.

    Layout: 6D local TCP pose error, 6D force/torque, and one phase id.
    """

    tcp = as_pose6(tcp_pose, "tcp_pose")
    force = as_pose6(tcp_force, "tcp_force")
    if hole_origin_pose is None:
        local_pose = tcp
    else:
        local_pose = tcp_pose_to_hole_frame(tcp, hole_origin_pose)
    return np.concatenate([local_pose, force, np.asarray([phase_id], dtype=np.float32)]).astype(np.float32)


def build_screw_state(status: dict | None) -> np.ndarray:
    """Convert screw-driver status JSON into a compact numeric state."""

    status = status or {}
    return np.asarray(
        [
            float(status.get("target_speed_rpm", 0.0)),
            float(status.get("measured_speed_rpm", 0.0)),
            float(status.get("motor_position_rad", 0.0)),
            float(status.get("motor_velocity_rad_s", 0.0)),
            float(status.get("current_amp", 0.0)),
            float(status.get("torque_nm", 0.0)),
            1.0 if status.get("fault", False) else 0.0,
        ],
        dtype=np.float32,
    )


def synthetic_front_image(height: int, width: int, frame_index: int = 0, phase_id: int = 0) -> np.ndarray:
    """Create a deterministic RGB frame for dataset and visualizer smoke tests."""

    y = np.linspace(0, 255, height, dtype=np.uint16)[:, None]
    x = np.linspace(0, 255, width, dtype=np.uint16)[None, :]
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[..., 0] = ((x + frame_index * 7) % 255).astype(np.uint8)
    image[..., 1] = ((y + phase_id * 35) % 255).astype(np.uint8)
    image[..., 2] = 120
    image[height // 2 - 1 : height // 2 + 1, :, :] = 255
    image[:, width // 2 - 1 : width // 2 + 1, :] = 255
    return image
