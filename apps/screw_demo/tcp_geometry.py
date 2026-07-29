from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def _as_pose(values: Iterable[float], name: str) -> np.ndarray:
    pose = np.asarray(values, dtype=float).reshape(-1)
    if pose.size != 6:
        raise ValueError(f"{name} must contain 6 values, got {pose.size}")
    if not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} contains non-finite values")
    return pose


def _rotation_vector_to_quaternion(rotation: Iterable[float]) -> np.ndarray:
    vector = np.asarray(rotation, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0])
    axis = vector / angle
    scale = math.sin(angle / 2.0)
    return np.asarray([
        axis[0] * scale,
        axis[1] * scale,
        axis[2] * scale,
        math.cos(angle / 2.0),
    ])


def _normalize_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    result = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(result))
    if norm < 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0])
    result = result / norm
    if result[3] < 0.0:
        result = -result
    return result


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def _quaternion_inverse(quaternion: Iterable[float]) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    return np.asarray([-q[0], -q[1], -q[2], q[3]])


def _quaternion_rotate(quaternion: Iterable[float], vector: Iterable[float]) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    v = np.asarray(vector, dtype=float)
    q_vector = q[:3]
    return (
        v
        + 2.0 * q[3] * np.cross(q_vector, v)
        + 2.0 * np.cross(q_vector, np.cross(q_vector, v))
    )


def _quaternion_to_rotation_vector(quaternion: Iterable[float]) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    vector_norm = float(np.linalg.norm(q[:3]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(q[3]))
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return q[:3] / vector_norm * angle


def compose_pose(left: Iterable[float], right: Iterable[float]) -> list[float]:
    """Compose two UR poses: T_result = T_left * T_right."""
    left_pose = _as_pose(left, "left pose")
    right_pose = _as_pose(right, "right pose")
    left_q = _rotation_vector_to_quaternion(left_pose[3:])
    right_q = _rotation_vector_to_quaternion(right_pose[3:])
    result_q = _quaternion_multiply(left_q, right_q)
    result_position = (
        left_pose[:3]
        + _quaternion_rotate(left_q, right_pose[:3])
    )
    return np.concatenate([
        result_position,
        _quaternion_to_rotation_vector(result_q),
    ]).tolist()


def tcp_target_to_ee_target(
    tcp_target: Iterable[float],
    ee_to_tcp: Iterable[float],
) -> list[float]:
    """Return T_base_ee from T_base_tcp and the fixed T_ee_tcp transform."""
    target = _as_pose(tcp_target, "TCP target")
    offset = _as_pose(ee_to_tcp, "end-effector to TCP offset")
    target_q = _rotation_vector_to_quaternion(target[3:])
    offset_q = _rotation_vector_to_quaternion(offset[3:])
    ee_q = _quaternion_multiply(target_q, _quaternion_inverse(offset_q))
    ee_position = target[:3] - _quaternion_rotate(ee_q, offset[:3])
    return np.concatenate([
        ee_position,
        _quaternion_to_rotation_vector(ee_q),
    ]).tolist()
