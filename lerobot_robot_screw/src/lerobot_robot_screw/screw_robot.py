# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/src/lerobot_robot_screw/screw_robot.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.robots.robot import Robot

from .config_screw_robot import ScrewRobotConfig
from .processors import build_observation_state, build_screw_state, synthetic_front_image


class ScrewRobot(Robot):
    """LeRobot wrapper around the SCREW/UR driver.

    ``dry_run=True`` keeps all behavior in memory, so config and robot smoke
    tests never need the physical arm connected.
    """

    config_class = ScrewRobotConfig
    name = "screw_robot"

    def __init__(self, config: ScrewRobotConfig):
        super().__init__(config)
        self.config = config
        self._connected = False
        self._driver: Any | None = None
        self._screw_client: Any | None = None
        self._tcp_pose = np.zeros(6, dtype=np.float32)
        self._tcp_force = np.zeros(6, dtype=np.float32)
        self._screw_status: dict[str, Any] = {}
        self._hole_origin_pose = np.zeros(6, dtype=np.float32)
        self._phase_id = 0
        self._frame_index = 0

    @property
    def observation_features(self) -> dict[str, Any]:
        features: dict[str, Any] = {
            "observation.state": (13,),
            "observation.tcp_pose": (6,),
            "observation.tcp_force": (6,),
            "observation.screw_state": (7,),
            "phase_id": (1,),
        }
        if self.config.use_images:
            features["observation.images.front"] = (
                self.config.image_height,
                self.config.image_width,
                3,
            )
        return features

    @property
    def action_features(self) -> dict[str, Any]:
        return {"action": (7,)}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return
        if not self.config.dry_run:
            module = self._import_driver_module(self.config.ur_driver_module)
            driver_class = getattr(module, self.config.ur_driver_class)
            self._driver = self._make_ur_driver(driver_class)
            if hasattr(self._driver, "connect"):
                self._driver.connect()
        screw_module = self._import_driver_module(self.config.screw_client_module)
        screw_factory = getattr(screw_module, self.config.screw_client_factory)
        self._screw_client = screw_factory(
            host=self.config.screw_host,
            port=self.config.screw_port,
            timeout=self.config.screw_timeout_s,
            dry_run=self.config.dry_run,
        )
        if hasattr(self._screw_client, "connect"):
            self._screw_client.connect()
        self._connected = True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def get_observation(self) -> dict[str, Any]:
        if not self._connected:
            raise RuntimeError("ScrewRobot must be connected before get_observation().")

        if self._driver is not None:
            self._tcp_pose = self._read_driver_pose()
            self._tcp_force = self._read_driver_force()
        if self._screw_client is not None and hasattr(self._screw_client, "status"):
            self._screw_status = self._extract_screw_status(self._screw_client.status())

        observation: dict[str, Any] = {
            "observation.state": build_observation_state(
                self._tcp_pose,
                self._tcp_force,
                self._phase_id,
                self._hole_origin_pose,
            ),
            "observation.tcp_pose": self._tcp_pose.astype(np.float32),
            "observation.tcp_force": self._tcp_force.astype(np.float32),
            "observation.screw_state": build_screw_state(self._screw_status),
            "screw_status": dict(self._screw_status),
            "phase_id": np.asarray([self._phase_id], dtype=np.float32),
        }
        if self.config.use_images:
            observation["observation.images.front"] = synthetic_front_image(
                self.config.image_height,
                self.config.image_width,
                self._frame_index,
                self._phase_id,
            )
        self._frame_index += 1
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self._connected:
            raise RuntimeError("ScrewRobot must be connected before send_action().")
        raw_action = action.get("action", action.get("action.delta_tcp_pose"))
        if raw_action is None:
            raise KeyError("Expected action dict to contain 'action' or 'action.delta_tcp_pose'.")

        delta = np.asarray(raw_action, dtype=np.float32)
        if delta.shape == (6,):
            delta = np.concatenate([delta, np.asarray([0.0], dtype=np.float32)])
        if delta.shape != (7,):
            raise ValueError(f"action must have shape (7,) or legacy shape (6,), got {delta.shape}")
        applied_tcp = self._limit_delta(delta[:6])
        applied_screw_speed = float(
            np.clip(delta[6], -self.config.max_screw_speed_rpm, self.config.max_screw_speed_rpm)
        )
        target_pose = self._tcp_pose + applied_tcp

        has_tcp_motion = bool(np.any(np.abs(applied_tcp) > 1e-9))
        if self._driver is not None and not self.config.dry_run and has_tcp_motion:
            if not self.config.ur_connect_control:
                raise RuntimeError(
                    "TCP action is non-zero but robot was opened with "
                    "ur_connect_control=false. Set all TCP deltas to 0 for screw-only "
                    "commands, or enable --robot.ur_connect_control=true after RTDEControl is stable."
                )
            if hasattr(self._driver, "send_action"):
                self._driver.send_action(target_pose)
            elif hasattr(self._driver, "move_l"):
                self._driver.move_l(target_pose)
            elif hasattr(self._driver, "servo_l"):
                self._driver.servo_l(target_pose)
            else:
                raise AttributeError("Configured UR driver has no send_action/move_l/servo_l method.")

        if self.config.dry_run or has_tcp_motion:
            self._tcp_pose = target_pose.astype(np.float32)
        if self._screw_client is not None:
            response = self._screw_client.set_speed(applied_screw_speed)
            self._screw_status = self._extract_screw_status(response)
        return {"action": np.concatenate([applied_tcp, np.asarray([applied_screw_speed], dtype=np.float32)])}

    def disconnect(self) -> None:
        if self._screw_client is not None:
            if hasattr(self._screw_client, "hold"):
                self._screw_client.hold()
            if hasattr(self._screw_client, "close"):
                self._screw_client.close()
        self._screw_client = None
        if self._driver is not None and hasattr(self._driver, "disconnect"):
            self._driver.disconnect()
        self._driver = None
        self._connected = False

    def heartbeat_screw_driver(self) -> dict[str, Any]:
        if self._screw_client is None or not hasattr(self._screw_client, "heartbeat"):
            return {}
        self._screw_status = self._extract_screw_status(self._screw_client.heartbeat())
        return dict(self._screw_status)

    def _limit_delta(self, delta: np.ndarray) -> np.ndarray:
        limited = delta.astype(np.float32).copy()
        limited[:3] = np.clip(
            limited[:3],
            -self.config.max_translation_step_m,
            self.config.max_translation_step_m,
        )
        limited[3:] = np.clip(
            limited[3:],
            -self.config.max_rotation_step_rad,
            self.config.max_rotation_step_rad,
        )
        if not np.any(np.abs(limited[:3]) > 1e-9):
            return limited
        target_offset = self._tcp_pose[:3] + limited[:3] - self._hole_origin_pose[:3]
        distance = float(np.linalg.norm(target_offset))
        if distance > self.config.max_align_offset_m:
            scale = self.config.max_align_offset_m / distance
            limited[:3] = self._hole_origin_pose[:3] + target_offset * scale - self._tcp_pose[:3]
        return limited

    def _read_driver_pose(self) -> np.ndarray:
        for name in ("get_tcp_pose", "read_tcp_pose", "get_pose"):
            if hasattr(self._driver, name):
                return np.asarray(getattr(self._driver, name)(), dtype=np.float32)
        return self._tcp_pose

    def _read_driver_force(self) -> np.ndarray:
        for name in ("get_tcp_force", "read_tcp_force", "get_force"):
            if hasattr(self._driver, name):
                return np.asarray(getattr(self._driver, name)(), dtype=np.float32)
        return self._tcp_force

    def _import_driver_module(self, module_name: str):
        try:
            return importlib.import_module(module_name)
        except ModuleNotFoundError:
            cwd = str(Path.cwd())
            if cwd not in sys.path:
                sys.path.insert(0, cwd)
            return importlib.import_module(module_name)

    def _make_ur_driver(self, driver_class):
        attempts = (
            lambda: driver_class(
                host=self.config.host,
                port=self.config.port,
                connect_control=self.config.ur_connect_control,
            ),
            lambda: driver_class(host=self.config.host, connect_control=self.config.ur_connect_control),
            lambda: driver_class(host=self.config.host, port=self.config.port),
            lambda: driver_class(self.config.host),
        )
        last_error = None
        for make_driver in attempts:
            try:
                return make_driver()
            except TypeError as exc:
                last_error = exc
        raise last_error

    def _extract_screw_status(self, response: dict[str, Any]) -> dict[str, Any]:
        if "ok" in response and not response.get("ok", False):
            raise RuntimeError(f"Screw-driver command failed: {response}")
        status = response.get("status", response)
        if not isinstance(status, dict):
            raise RuntimeError(f"Screw-driver response has no status dict: {response}")
        return status
