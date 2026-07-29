# PROJECT FILE HEADER
# 文件：apps/screw_demo/ur_driver.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import socket
import threading
import time
from typing import Any, Iterable, List, Optional

import numpy as np

try:
    import rtde_control
    import rtde_receive
except ModuleNotFoundError:
    rtde_control = None
    rtde_receive = None


def _as_6d_list(values: Iterable[float], name: str) -> List[float]:
    result = np.asarray(values, dtype=float).reshape(-1).tolist()
    if len(result) != 6:
        raise ValueError(f"{name} must contain 6 values, got {len(result)}")
    return result


class URDriver:
    """Small UR robot wrapper used by screw_demo.

    The class only depends on rtde_control and rtde_receive. It deliberately
    leaves camera, force sensor, gripper, and data collection code outside.
    """

    def __init__(
        self,
        host: str = "192.168.1.5",
        connect_control: bool = True,
        connect_timeout_s: float = 10.0,
    ):
        self.host = host
        self.connect_control = connect_control
        self.connect_timeout_s = connect_timeout_s
        self.rtde_c: Optional[Any] = None
        self.rtde_r: Optional[Any] = None
        self.connected = False

    def _check_tcp_port(self, port: int, purpose: str) -> None:
        if self.connect_timeout_s <= 0.0:
            raise ValueError("UR connect timeout must be positive")
        try:
            with socket.create_connection(
                (self.host, port),
                timeout=self.connect_timeout_s,
            ):
                return
        except socket.timeout as exc:
            raise RuntimeError(
                f"UR {purpose} port {self.host}:{port} did not respond within "
                f"{self.connect_timeout_s:.1f} s; check the robot IP, Ethernet cable, "
                "computer subnet and PolyScope network settings"
            ) from exc
        except OSError as exc:
            raise RuntimeError(
                f"Cannot reach UR {purpose} port {self.host}:{port}: {exc}; "
                "check the robot IP, Ethernet cable, computer subnet and PolyScope network settings"
            ) from exc

    def connect(self) -> bool:
        if rtde_control is None or rtde_receive is None:
            raise RuntimeError("rtde_control/rtde_receive is not installed")
        if self.connected:
            return True

        print(
            f"[UR连接 1/3] 检查 RTDE 端口 {self.host}:30004 "
            f"（超时 {self.connect_timeout_s:.1f} 秒）..."
        )
        self._check_tcp_port(30004, "RTDE")
        if self.connect_control:
            print(
                f"[UR连接 2/3] 检查脚本端口 {self.host}:30002 "
                f"（超时 {self.connect_timeout_s:.1f} 秒）..."
            )
            self._check_tcp_port(30002, "script")
        else:
            print("[UR连接 2/3] 只读模式，不检查控制脚本端口。")

        print("[UR连接 3/3] TCP 端口可达，正在初始化 ur_rtde 接口...")
        try:
            if self.connect_control:
                self.rtde_c = rtde_control.RTDEControlInterface(self.host)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.host)
        except Exception as exc:
            self.close()
            raise RuntimeError(
                f"UR TCP ports are reachable, but ur_rtde initialization failed for "
                f"{self.host}: {exc}. Check PolyScope mode, robot program state and "
                "ur-rtde/PolyScope compatibility."
            ) from exc
        self.connected = True
        print("[UR连接完成] RTDE control/receive interfaces are ready.")
        return True

    def _require_connected(self) -> None:
        if not self.connected or self.rtde_r is None:
            raise RuntimeError("URDriver is not connected. Call connect() first.")

    def _require_control(self) -> None:
        if self.rtde_c is None:
            raise RuntimeError("URDriver was opened without RTDE control access.")

    def ensure_control_script_ready(self, timeout_s: float = 2.0) -> None:
        """Ensure the ur_rtde control script is running before a servoL phase."""
        self._require_connected()
        self._require_control()
        if not self.rtde_c.isConnected():
            if not self.rtde_c.reconnect():
                raise RuntimeError("Failed to reconnect the UR RTDE control interface")
        if self.rtde_c.isProgramRunning():
            return
        print("[UR控制] moveJ 后控制脚本已停止，正在重新上传 RTDE 控制脚本...")
        if not self.rtde_c.reuploadScript():
            raise RuntimeError("Failed to reupload the UR RTDE control script")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.rtde_c.isProgramRunning():
                print("[UR控制] RTDE 控制脚本已恢复，可以进入 servoL。")
                return
            time.sleep(0.02)
        raise RuntimeError("UR RTDE control script did not start after reupload")

    def get_tcp_pose(self) -> List[float]:
        self._require_connected()
        return self.rtde_r.getActualTCPPose()

    def get_tcp_offset(self) -> List[float]:
        """Return the controller's currently active flange-to-TCP transform."""
        self._require_connected()
        for interface in (self.rtde_r, self.rtde_c):
            getter = getattr(interface, "getTCPOffset", None)
            if callable(getter):
                return _as_6d_list(getter(), "active TCP offset")
        raise RuntimeError(
            "Installed ur_rtde exposes getTCPOffset() on neither the "
            "receive nor control interface"
        )

    def get_tcp_speed(self) -> List[float]:
        self._require_connected()
        return self.rtde_r.getActualTCPSpeed()

    def get_tcp_force(self) -> List[float]:
        """Return the actual TCP wrench [Fx, Fy, Fz, Tx, Ty, Tz]."""
        self._require_connected()
        if not hasattr(self.rtde_r, "getActualTCPForce"):
            raise RuntimeError("rtde_receive does not expose getActualTCPForce()")
        return self.rtde_r.getActualTCPForce()

    def get_joint_positions(self) -> List[float]:
        self._require_connected()
        return self.rtde_r.getActualQ()

    def _wait_for_joint_target(
        self,
        target: Iterable[float],
        timeout: float,
        tolerance: float,
        poll_interval: float = 0.02,
    ) -> bool:
        target_q = np.asarray(_as_6d_list(target, "target"), dtype=float)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            actual_q = np.asarray(self.get_joint_positions(), dtype=float)
            if np.max(np.abs(actual_q - target_q)) <= tolerance:
                return True
            time.sleep(poll_interval)
        return False

    def _wait_for_tcp_target(
        self,
        target: Iterable[float],
        timeout: float,
        position_tolerance: float,
        rotation_tolerance: float,
        poll_interval: float = 0.02,
        cancel_event: Optional[threading.Event] = None,
    ) -> bool:
        target_pose = np.asarray(_as_6d_list(target, "target"), dtype=float)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                return False
            actual_pose = np.asarray(self.get_tcp_pose(), dtype=float)
            position_error = np.linalg.norm(actual_pose[:3] - target_pose[:3])
            rotation_error = np.linalg.norm(actual_pose[3:] - target_pose[3:])
            if position_error <= position_tolerance and rotation_error <= rotation_tolerance:
                return True
            time.sleep(poll_interval)
        return False

    def move_j(
        self,
        joints: Iterable[float],
        speed: float = 0.5,
        acceleration: float = 0.5,
        timeout: float = 10.0,
        joint_tolerance: float = 0.001,
    ) -> bool:
        """Move in joint space and return only after success or timeout."""
        self._require_connected()
        self._require_control()
        target = _as_6d_list(joints, "joints")
        # Use a synchronous moveJ here.  With the asynchronous ur_rtde mode,
        # the joint feedback can enter tolerance slightly before the controller
        # has finalized the async operation.  A servoL sent immediately after
        # that is rejected, which then triggers coordinated cleanup and HOLDs
        # the screwdriver almost as soon as it starts.
        command_ok = self.rtde_c.moveJ(target, speed, acceleration, False)
        if not command_ok:
            return False
        reached = self._wait_for_joint_target(target, timeout, joint_tolerance)
        if not reached:
            self.stop_joint_motion()
        return reached

    def move_l(
        self,
        pose: Iterable[float],
        speed: float = 0.03,
        acceleration: float = 0.1,
        timeout: float = 10.0,
        position_tolerance: float = 0.001,
        rotation_tolerance: float = 0.01,
        cancel_event: Optional[threading.Event] = None,
    ) -> bool:
        """Move TCP linearly and return only after success or timeout."""
        self._require_connected()
        self._require_control()
        target = _as_6d_list(pose, "pose")
        command_ok = self.rtde_c.moveL(target, speed, acceleration, True)
        if not command_ok:
            return False
        reached = self._wait_for_tcp_target(
            target,
            timeout,
            position_tolerance,
            rotation_tolerance,
            cancel_event=cancel_event,
        )
        if not reached:
            self.stop_linear_motion()
        return reached

    def servo_l(
        self,
        pose: Iterable[float],
        dt: float = 0.02,
        speed: float = 0.0,
        acceleration: float = 0.0,
        lookahead_time: float = 0.1,
        gain: float = 300.0,
    ) -> bool:
        self._require_connected()
        self._require_control()
        target = _as_6d_list(pose, "pose")
        return self.rtde_c.servoL(
            target,
            speed,
            acceleration,
            dt,
            lookahead_time,
            gain,
        )

    def follow_servo_path(
        self,
        path: Iterable[Iterable[float]],
        dt: float = 0.02,
        lookahead_time: float = 0.1,
        gain: float = 300.0,
        final_timeout: float = 2.0,
        position_tolerance: float = 0.001,
        rotation_tolerance: float = 0.01,
    ) -> bool:
        self._require_connected()
        self._require_control()
        last_pose = None
        try:
            for pose in path:
                last_pose = _as_6d_list(pose, "pose")
                start_time = self.rtde_c.initPeriod()
                command_ok = self.servo_l(
                    last_pose,
                    dt=dt,
                    lookahead_time=lookahead_time,
                    gain=gain,
                )
                if not command_ok:
                    return False
                self.rtde_c.waitPeriod(start_time)
        finally:
            self.stop_servo()
        if last_pose is None:
            return False
        return self._wait_for_tcp_target(
            last_pose,
            final_timeout,
            position_tolerance,
            rotation_tolerance,
        )

    def stop_servo(self) -> None:
        if self.rtde_c is not None:
            self.rtde_c.servoStop()

    def stop_linear_motion(self, acceleration: float = 0.5) -> None:
        if self.rtde_c is not None:
            try:
                self.rtde_c.stopL(acceleration)
            except Exception:
                self.stop_servo()

    def stop_joint_motion(self, acceleration: float = 1.5) -> None:
        if self.rtde_c is not None:
            try:
                self.rtde_c.stopJ(acceleration)
            except Exception:
                self.stop_servo()

    def close(self) -> None:
        if self.rtde_c is not None:
            try:
                self.rtde_c.servoStop()
            except Exception:
                pass
            try:
                self.rtde_c.stopScript()
            except Exception:
                pass
            try:
                self.rtde_c.disconnect()
            except Exception:
                pass
            self.rtde_c = None
        if self.rtde_r is not None:
            try:
                disconnect = getattr(self.rtde_r, "disconnect", None)
                if callable(disconnect):
                    disconnect()
            except Exception:
                pass
            self.rtde_r = None
        self.connected = False

    def __enter__(self) -> "URDriver":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def make_linear_path(
    start_pose: Iterable[float],
    end_pose: Iterable[float],
    steps: int = 100,
) -> List[List[float]]:
    start = np.asarray(_as_6d_list(start_pose, "start_pose"), dtype=float)
    end = np.asarray(_as_6d_list(end_pose, "end_pose"), dtype=float)
    if steps < 2:
        raise ValueError("steps must be at least 2")
    return [((1.0 - a) * start + a * end).tolist() for a in np.linspace(0, 1, steps)]


def sleep_after_motion(seconds: float = 0.5) -> None:
    time.sleep(seconds)
