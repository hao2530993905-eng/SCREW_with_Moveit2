# PROJECT FILE HEADER
# 文件：apps/screw_demo/screw_client.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import json
import socket
from typing import Any, Dict, Optional, TextIO


class ScrewClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 5055, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._file: Optional[TextIO] = None

    def connect(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._file = self._sock.makefile("r", encoding="utf-8", newline="\n")

    def close(self) -> None:
        if self._file is not None:
            try:
                self._file.close()
            finally:
                self._file = None
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def __enter__(self) -> "ScrewClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def request(self, command: str, **params: Any) -> Dict[str, Any]:
        payload = {"cmd": command, **params}
        data = (json.dumps(payload) + "\n").encode("utf-8")
        line = self._request_once(data)
        if not line:
            self.close()
            line = self._request_once(data)
        if not line:
            raise RuntimeError("No response from screw server")
        return json.loads(line)

    def _request_once(self, data: bytes) -> str:
        try:
            self.connect()
            assert self._sock is not None
            assert self._file is not None
            self._sock.sendall(data)
            return self._file.readline()
        except (OSError, RuntimeError):
            self.close()
            return ""

    def status(self) -> Dict[str, Any]:
        return self.request("status")

    def forward(self, speed: float = 120.0) -> Dict[str, Any]:
        return self.request("forward", speed=speed)

    def reverse(self, speed: float = 120.0) -> Dict[str, Any]:
        return self.request("reverse", speed=speed)

    def set_speed(self, speed: float) -> Dict[str, Any]:
        return self.request("set_speed", speed=speed)

    def set_torque_stop(self, torque_nm: float) -> Dict[str, Any]:
        return self.request("set_torque_stop", torque_nm=torque_nm)

    def hold(self) -> Dict[str, Any]:
        return self.request("hold")

    def heartbeat(self) -> Dict[str, Any]:
        return self.request("heartbeat")

    def clear_fault(self) -> Dict[str, Any]:
        return self.request("clear_fault")

    def estop(self, active: bool = True) -> Dict[str, Any]:
        return self.request("estop", active=active)


class DryRunScrewClient:
    def __init__(self):
        self.mode = "IDLE"
        self.speed = 0.0

    def connect(self) -> None:
        return None

    def close(self) -> None:
        return None

    def __enter__(self) -> "DryRunScrewClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        return None

    def _response(self, ok: bool = True, message: str = "dry_run") -> Dict[str, Any]:
        return {
            "ok": ok,
            "message": message,
            "status": {
                "mode": self.mode,
                "target_speed_rpm": self.speed,
                "measured_speed_rpm": self.speed,
                "motor_position_rad": 0.0,
                "motor_velocity_rad_s": self.speed * 0.104719755,
                "current_amp": 0.0,
                "torque_nm": 0.0,
                "mos_temperature_c": 0,
                "rotor_temperature_c": 0,
                "emergency_stop": False,
                "fault": False,
                "fault_message": "",
                "tick": 0,
                "feedback_count": 0,
                "feedback_age_ms": 0.0,
            },
        }

    def status(self) -> Dict[str, Any]:
        return self._response(message="status")

    def forward(self, speed: float = 120.0) -> Dict[str, Any]:
        self.mode = "FORWARD"
        self.speed = abs(speed)
        return self._response(message="forward")

    def reverse(self, speed: float = 120.0) -> Dict[str, Any]:
        self.mode = "REVERSE"
        self.speed = -abs(speed)
        return self._response(message="reverse")

    def set_speed(self, speed: float) -> Dict[str, Any]:
        self.speed = speed
        self.mode = "FORWARD" if speed > 0 else "REVERSE" if speed < 0 else "IDLE"
        return self._response(message="set_speed")

    def set_torque_stop(self, torque_nm: float) -> Dict[str, Any]:
        return self._response(message="set_torque_stop")

    def hold(self) -> Dict[str, Any]:
        self.mode = "HOLDING"
        self.speed = 0.0
        return self._response(message="hold")

    def heartbeat(self) -> Dict[str, Any]:
        return self._response(message="heartbeat")

    def clear_fault(self) -> Dict[str, Any]:
        return self._response(message="clear_fault")

    def estop(self, active: bool = True) -> Dict[str, Any]:
        self.mode = "EMERGENCY_STOP" if active else "IDLE"
        self.speed = 0.0
        return self._response(message="estop")


def make_screw_client(
    host: str = "127.0.0.1",
    port: int = 5055,
    timeout: float = 2.0,
    dry_run: bool = False,
) -> Any:
    if dry_run:
        return DryRunScrewClient()
    return ScrewClient(host=host, port=port, timeout=timeout)


def assert_ok(response: Dict[str, Any], action: Optional[str] = None) -> Dict[str, Any]:
    if not response.get("ok", False):
        label = action or response.get("message", "request")
        raise RuntimeError(f"Screw command failed: {label}: {response}")
    return response
