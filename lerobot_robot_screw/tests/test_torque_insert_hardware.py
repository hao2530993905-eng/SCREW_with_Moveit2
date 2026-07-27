#!/usr/bin/env python3
"""Real screwdriver torque test, with optional UR3 moveL insertion."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import importlib.util
import json
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
RECORD_SCRIPT = REPO_ROOT / "lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py"
LOG_ROOT = REPO_ROOT / "artifacts" / "电批力矩测试log"


def load_record_module():
    spec = importlib.util.spec_from_file_location("torque_hardware_record_module", RECORD_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {RECORD_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Test coordinated UR3 insertion and screwdriver torque control. "
            "UR motion, adaptive insert speed, and dynamic screwdriver slowdown are always enabled."
        )
    )
    parser.add_argument("--robot-host", default="192.168.1.5")
    parser.add_argument("--robot-connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--screw-host", default="127.0.0.1")
    parser.add_argument("--screw-port", type=int, default=5055)
    parser.add_argument("--screw-timeout-s", type=float, default=2.0)
    parser.add_argument("--insert-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--max-insert-depth-mm", type=float, default=1.0)
    parser.add_argument("--insert-speed-mm-s", type=float, default=0.5)
    parser.add_argument("--linear-acc-mm-s2", type=float, default=20.0)
    parser.add_argument("--stop-linear-acc-mm-s2", type=float, default=2000.0)
    parser.add_argument(
        "--screw-hold-delay-s",
        type=float,
        default=-1.0,
        help="negative: wait for actual UR TCP speed to approach zero",
    )
    parser.add_argument("--stop-sync-speed-threshold-mm-s", type=float, default=0.5)
    parser.add_argument("--motion-timeout-s", type=float, default=10.0)
    parser.add_argument("--screw-speed-rpm", type=float, default=10.0)
    parser.add_argument("--dynamic-speed-servo-dt", type=float, default=0.008)
    parser.add_argument("--dynamic-speed-min-valid-rpm", type=float, default=30.0)
    parser.add_argument("--dynamic-speed-stall-rpm", type=float, default=20.0)
    parser.add_argument("--dynamic-speed-min-ratio", type=float, default=0.10)
    parser.add_argument("--dynamic-speed-filter-alpha", type=float, default=0.20)
    parser.add_argument("--dynamic-screw-speed-command-interval-s", type=float, default=0.05)
    parser.add_argument("--dynamic-screw-speed-slew-rpm-s", type=float, default=3000.0)
    parser.add_argument("--dynamic-screw-speed-min-command-delta-rpm", type=float, default=20.0)
    parser.add_argument("--dynamic-screw-full-crawl-ratio", type=float, default=0.70)
    parser.add_argument(
        "--rotation-only-speed-rpm",
        type=float,
        default=80.0,
        help="stop UR translation once commanded screwdriver speed falls to this value",
    )
    parser.add_argument("--dynamic-screw-speed-prediction-s", type=float, default=0.10)
    parser.add_argument("--dynamic-screw-prediction-min-rise-nm", type=float, default=0.15)
    parser.add_argument("--torque-stop-nm", type=float, required=True)
    parser.add_argument("--torque-poll-s", type=float, default=0.005)
    parser.add_argument("--backend-torque-guard-ratio", type=float, default=2.0)
    parser.add_argument("--torque-filter-window", type=int, default=5)
    parser.add_argument("--torque-confirm-samples", type=int, default=3)
    parser.add_argument("--torque-min-active-s", type=float, default=0.2)
    parser.add_argument("--torque-slowdown-ratio", type=float, default=0.65)
    parser.add_argument("--torque-crawl-rpm", type=float, default=60.0)
    args = parser.parse_args()

    if args.max_insert_depth_mm <= 0.0:
        parser.error("--max-insert-depth-mm must be positive")
    if args.robot_connect_timeout_s <= 0.0:
        parser.error("--robot-connect-timeout-s must be positive")
    if args.insert_speed_mm_s <= 0.0:
        parser.error("--insert-speed-mm-s must be positive")
    if args.screw_speed_rpm == 0.0:
        parser.error("--screw-speed-rpm must be non-zero")
    if args.torque_stop_nm <= 0.0:
        parser.error("--torque-stop-nm must be positive")
    if args.torque_poll_s <= 0.0:
        parser.error("--torque-poll-s must be positive")
    if args.torque_filter_window <= 0 or args.torque_confirm_samples <= 0:
        parser.error("filter window and confirmation sample count must be positive")
    if args.torque_min_active_s < 0.0:
        parser.error("--torque-min-active-s must be non-negative")
    if not 0.0 < args.torque_slowdown_ratio < 1.0:
        parser.error("--torque-slowdown-ratio must be between 0 and 1")
    if args.torque_crawl_rpm <= 0.0:
        parser.error("--torque-crawl-rpm must be positive")
    if args.backend_torque_guard_ratio <= 1.0:
        parser.error("--backend-torque-guard-ratio must be greater than 1")
    if args.stop_sync_speed_threshold_mm_s <= 0.0:
        parser.error("--stop-sync-speed-threshold-mm-s must be positive")
    if args.dynamic_speed_servo_dt <= 0.0:
        parser.error("--dynamic-speed-servo-dt must be positive")
    if not 0.0 <= args.dynamic_speed_min_ratio <= 1.0:
        parser.error("--dynamic-speed-min-ratio must be between 0 and 1")
    if not 0.0 < args.dynamic_speed_filter_alpha <= 1.0:
        parser.error("--dynamic-speed-filter-alpha must be between 0 and 1")
    if args.dynamic_screw_speed_command_interval_s <= 0.0:
        parser.error("--dynamic-screw-speed-command-interval-s must be positive")
    if args.dynamic_screw_speed_slew_rpm_s <= 0.0:
        parser.error("--dynamic-screw-speed-slew-rpm-s must be positive")
    if args.rotation_only_speed_rpm <= 0.0:
        parser.error("--rotation-only-speed-rpm must be positive")
    if not args.torque_slowdown_ratio < args.dynamic_screw_full_crawl_ratio < 1.0:
        parser.error(
            "--dynamic-screw-full-crawl-ratio must be above slowdown ratio and below 1"
        )
    if args.dynamic_screw_prediction_min_rise_nm < 0.0:
        parser.error("--dynamic-screw-prediction-min-rise-nm must be non-negative")
    return args


class TeeStream:
    def __init__(self, *streams):
        self.streams = streams
        self.lock = threading.Lock()

    def write(self, data: str) -> int:
        with self.lock:
            for stream in self.streams:
                stream.write(data)
                stream.flush()
        return len(data)

    def flush(self) -> None:
        with self.lock:
            for stream in self.streams:
                stream.flush()


class TorqueRunLogger:
    FIELDS = [
        "elapsed_s",
        "wall_time",
        "event",
        "mode",
        "target_speed_rpm",
        "measured_speed_rpm",
        "motor_position_rad",
        "motor_velocity_rad_s",
        "current_amp",
        "torque_nm",
        "abs_torque_nm",
        "mos_temperature_c",
        "rotor_temperature_c",
        "fault",
        "fault_message",
        "emergency_stop",
        "tick",
        "feedback_count",
        "feedback_age_ms",
        "response_json",
    ]

    def __init__(self, move_l: bool):
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        mode = "moveL" if move_l else "screw_only"
        stem = f"torque_test_{mode}_{timestamp}"
        self.csv_path = LOG_ROOT / f"{stem}.csv"
        self.text_path = LOG_ROOT / f"{stem}.log"
        self.started = time.monotonic()
        self.max_abs_torque_nm = 0.0
        self.max_torque_sample_nm = 0.0
        self.sample_count = 0
        self.stop_reason = "尚未确定"
        self.rows: list[dict[str, Any]] = []
        self.lock = threading.Lock()
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.csv_file.flush()

    @staticmethod
    def extract_status(response: dict[str, Any]) -> dict[str, Any]:
        status = response.get("status", response)
        return status if isinstance(status, dict) else {}

    def record(self, response: dict[str, Any], event: str) -> None:
        status = self.extract_status(response)
        try:
            torque_nm = float(status.get("torque_nm", 0.0))
        except (TypeError, ValueError):
            torque_nm = 0.0
        abs_torque_nm = abs(torque_nm)
        with self.lock:
            self.sample_count += 1
            if abs_torque_nm > self.max_abs_torque_nm:
                self.max_abs_torque_nm = abs_torque_nm
                self.max_torque_sample_nm = torque_nm
            row = {field: status.get(field, "") for field in self.FIELDS}
            row.update(
                {
                    "elapsed_s": f"{time.monotonic() - self.started:.6f}",
                    "wall_time": datetime.now().isoformat(timespec="milliseconds"),
                    "event": event,
                    "torque_nm": torque_nm,
                    "abs_torque_nm": abs_torque_nm,
                    "response_json": json.dumps(response, ensure_ascii=False, sort_keys=True),
                }
            )
            self.writer.writerow(row)
            self.csv_file.flush()
            self.rows.append(dict(row))

    def close(self) -> None:
        with self.lock:
            if not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()

    def set_stop_reason(self, reason: str) -> None:
        with self.lock:
            self.stop_reason = reason

    def snapshot_rows(self) -> list[dict[str, Any]]:
        with self.lock:
            return [dict(row) for row in self.rows]


def print_test_parameters(cli: argparse.Namespace) -> None:
    """Print every effective CLI parameter at the beginning of the text log."""
    print("")
    print("===== 本次测试全部参数 =====")
    for name, value in sorted(vars(cli).items()):
        if isinstance(value, Path):
            value = str(value)
        print(f"{name}={value}")
    print("===== 参数打印结束 =====")
    print("")


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def print_all_torque_samples(torque_logger: TorqueRunLogger) -> None:
    """Print the complete in-memory torque timeline into the terminal log."""
    rows = torque_logger.snapshot_rows()
    print("")
    print("===== 全过程力矩明细 =====")
    print(
        "序号 | elapsed_s | wall_time | event | mode | target_rpm | measured_rpm | "
        "torque_nm | abs_torque_nm | feedback_count | feedback_age_ms | fault | fault_message"
    )
    for index, row in enumerate(rows, start=1):
        print(
            f"{index:04d} | "
            f"{_as_float(row.get('elapsed_s')):10.6f} | "
            f"{row.get('wall_time', '')} | "
            f"{row.get('event', '')} | "
            f"{row.get('mode', '')} | "
            f"{_as_float(row.get('target_speed_rpm')):10.3f} | "
            f"{_as_float(row.get('measured_speed_rpm')):12.3f} | "
            f"{_as_float(row.get('torque_nm')):10.6f} | "
            f"{_as_float(row.get('abs_torque_nm')):13.6f} | "
            f"{row.get('feedback_count', '')} | "
            f"{_as_float(row.get('feedback_age_ms')):12.3f} | "
            f"{row.get('fault', '')} | "
            f"{row.get('fault_message', '')}"
        )

    torques = [_as_float(row.get("torque_nm")) for row in rows]
    if torques:
        mean_torque = sum(torques) / len(torques)
        rms_torque = math.sqrt(sum(value * value for value in torques) / len(torques))
        print("----- 全过程力矩统计 -----")
        print(f"样本总数：{len(torques)}")
        print(f"最小有符号力矩：{min(torques):.6f} Nm")
        print(f"最大有符号力矩：{max(torques):.6f} Nm")
        print(f"平均有符号力矩：{mean_torque:.6f} Nm")
        print(f"力矩 RMS：{rms_torque:.6f} Nm")
        print(f"最大绝对力矩：{max(abs(value) for value in torques):.6f} Nm")
    else:
        print("本次测试没有收到任何力矩样本。")
    print("===== 全过程力矩明细结束 =====")


class LoggedScrewClient:
    def __init__(self, client, logger: TorqueRunLogger):
        self.client = client
        self.logger = logger

    def __getattr__(self, name: str):
        return getattr(self.client, name)

    def connect(self) -> None:
        self.client.connect()

    def close(self) -> None:
        self.client.close()

    def _call_and_log(self, event: str, method, *args, **kwargs):
        response = method(*args, **kwargs)
        if isinstance(response, dict):
            self.logger.record(response, event)
        return response

    def status(self):
        return self._call_and_log("status", self.client.status)

    def set_speed(self, speed: float):
        return self._call_and_log("set_speed", self.client.set_speed, speed)

    def set_torque_stop(self, torque_nm: float):
        return self._call_and_log(
            "set_torque_stop", self.client.set_torque_stop, torque_nm
        )

    def heartbeat(self):
        return self._call_and_log("heartbeat", self.client.heartbeat)

    def hold(self):
        return self._call_and_log("hold", self.client.hold)


def confirm_start(move_l: bool) -> bool:
    if move_l:
        message = "输入 START 才会启动真实 UR3 moveL 和电批："
    else:
        message = "输入 START 才会启动真实电批（不会连接或移动 UR3）："
    return input(message).strip() == "START"


def read_robot_safety_reason(robot) -> str | None:
    """Return a Chinese reason when UR reports a safety-related stop."""
    receiver = getattr(robot, "rtde_r", None)
    if receiver is None:
        return None
    try:
        if hasattr(receiver, "isEmergencyStopped") and receiver.isEmergencyStopped():
            return "机械臂触发急停（Emergency Stop）"
    except Exception:
        pass
    try:
        if hasattr(receiver, "isProtectiveStopped") and receiver.isProtectiveStopped():
            return "机械臂触发保护性停止（Protective Stop）"
    except Exception:
        pass
    try:
        if hasattr(receiver, "getSafetyStatusBits"):
            bits = int(receiver.getSafetyStatusBits())
            if bits & (1 << 4):
                return "机械臂触发安全防护停止（Safeguard Stop）"
            if bits & ((1 << 5) | (1 << 6) | (1 << 7)):
                return "机械臂触发急停（Emergency Stop）"
            if bits & (1 << 2):
                return "机械臂触发保护性停止（Protective Stop）"
            if bits & (1 << 10):
                return "机械臂因安全系统停止"
            if bits & (1 << 9):
                return "机械臂安全系统报告故障"
            if bits & (1 << 8):
                return "机械臂安全限制被违反"
    except Exception:
        pass
    return None


def exception_chain_text(exc: BaseException) -> str:
    messages: list[str] = []
    current: BaseException | None = exc
    seen: set[int] = set()
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        messages.append(str(current))
        current = current.__cause__ or current.__context__
    return " | ".join(messages)


def classify_exception_reason(exc: BaseException, robot, cli) -> str:
    safety_reason = read_robot_safety_reason(robot) if robot is not None else None
    if safety_reason is not None:
        return safety_reason

    message = exception_chain_text(exc)
    lowered = message.lower()
    if "maximum insertion travel reached before torque threshold" in lowered:
        return (
            "机械臂已到达最大移动距离 "
            f"{cli.max_insert_depth_mm:.3f} mm，但电批力矩尚未达到 "
            f"{cli.torque_stop_nm:.6f} Nm"
        )
    if "insertion movel stopped before torque threshold" in lowered:
        return "机械臂 moveL 在达到力矩阈值前停止（可能为运动超时、命令失败或外部停止）"
    if "torque-controlled movel failed" in lowered:
        return "机械臂扭矩控制 moveL 执行失败"
    if "can motor feedback timeout" in lowered:
        return "电批 CAN 电机反馈超时"
    if "emergency stop" in lowered and "screw" in lowered:
        return "电批软件急停已激活"
    if "screw-driver emergency stop" in lowered:
        return "电批软件急停已激活"
    if "screw-driver fault" in lowered:
        return f"电批报告故障：{message}"
    if "no response from screw server" in lowered:
        return "电批服务无响应或 Socket 通信中断"
    if "screwdriver hold failed" in lowered or "screw command failed: hold" in lowered:
        return "停止时电批 hold 命令失败"
    if "cannot reach ur" in lowered or "could not connect to" in lowered:
        return "机械臂网络或 RTDE 端口连接失败"
    if "ur tcp ports are reachable" in lowered and "initialization failed" in lowered:
        return "机械臂端口可达，但 ur_rtde 初始化失败"
    if "robot connection failed" in lowered:
        return "机械臂连接失败"
    if "heartbeat" in lowered:
        return "电批心跳失败，电批已停止"
    return f"发生未分类异常：{message}"


def run_with_robot(
    cli,
    record,
    robot,
    screw,
    screw_lock: threading.Lock,
    torque_logger: TorqueRunLogger,
) -> bool:
    print("运行模式：UR3 动态直线进给和电批动态力矩控制（固定启用）。")
    print("注意：只有看到 START 输入提示后，输入 START 才有效。")

    current_pose = np.asarray(robot.get_tcp_pose(), dtype=float)
    target = record.offset_in_tcp_frame(
        current_pose,
        [0.0, 0.0, cli.insert_sign * cli.max_insert_depth_mm * 0.001],
    )
    print("当前 TCP [mm, rad]：", record.format_pose_mm(current_pose))
    print("测试目标 [mm, rad]：", record.format_pose_mm(target))
    print(f"最大前进：{cli.max_insert_depth_mm:.3f} mm")
    print(f"前进速度：{cli.insert_speed_mm_s:.3f} mm/s")
    print(f"电批速度：{cli.screw_speed_rpm:.3f} rpm")
    print(f"拧紧阈值：{cli.torque_stop_nm:.6f} Nm")
    if not confirm_start(True):
        print("未输入 START，测试取消。")
        torque_logger.set_stop_reason("用户未输入 START，测试在启动前取消")
        return False

    with screw_lock:
        response = screw.set_torque_stop(
            cli.torque_stop_nm * cli.backend_torque_guard_ratio
        )
    record.assert_ok(response, "set_torque_stop")
    with screw_lock:
        response = screw.set_speed(cli.screw_speed_rpm)
    record.healthy_screw_status(response, "set_speed")
    heartbeat = record.ScrewHeartbeat(screw, screw_lock)
    heartbeat.start()
    try:
        controller_args = SimpleNamespace(
            torque_stop_nm=cli.torque_stop_nm,
            torque_poll_s=cli.torque_poll_s,
            torque_filter_window=cli.torque_filter_window,
            torque_confirm_samples=cli.torque_confirm_samples,
            torque_min_active_s=cli.torque_min_active_s,
            torque_slowdown_ratio=cli.torque_slowdown_ratio,
            torque_crawl_rpm=cli.torque_crawl_rpm,
            screw_speed_rpm=cli.screw_speed_rpm,
            dynamic_speed_servo_dt=cli.dynamic_speed_servo_dt,
            dynamic_speed_min_valid_rpm=cli.dynamic_speed_min_valid_rpm,
            dynamic_speed_stall_rpm=cli.dynamic_speed_stall_rpm,
            dynamic_speed_min_ratio=cli.dynamic_speed_min_ratio,
            dynamic_speed_filter_alpha=cli.dynamic_speed_filter_alpha,
            dynamic_screw_speed_command_interval_s=cli.dynamic_screw_speed_command_interval_s,
            dynamic_screw_speed_slew_rpm_s=cli.dynamic_screw_speed_slew_rpm_s,
            dynamic_screw_speed_min_command_delta_rpm=cli.dynamic_screw_speed_min_command_delta_rpm,
            dynamic_screw_full_crawl_ratio=cli.dynamic_screw_full_crawl_ratio,
            rotation_only_speed_rpm=cli.rotation_only_speed_rpm,
            dynamic_screw_speed_prediction_s=cli.dynamic_screw_speed_prediction_s,
            dynamic_screw_prediction_min_rise_nm=cli.dynamic_screw_prediction_min_rise_nm,
            motion_timeout=cli.motion_timeout_s,
            insert_speed=cli.insert_speed_mm_s * 0.001,
            linear_acc=cli.linear_acc_mm_s2 * 0.001,
            stop_linear_acc=cli.stop_linear_acc_mm_s2 * 0.001,
            screw_hold_delay_s=cli.screw_hold_delay_s,
            stop_sync_speed_threshold=cli.stop_sync_speed_threshold_mm_s * 0.001,
        )
        controller = record.TorqueControlledInsert(robot, screw, screw_lock, controller_args)
        controller.run(target)
        print("测试通过：连续滤波扭矩达到阈值，机械臂与电批已协调停止。")
        torque_logger.set_stop_reason(
            "电批力矩达到设定阈值 "
            f"{cli.torque_stop_nm:.6f} Nm，已同时停止机械臂和电批"
        )
        return True
    finally:
        heartbeat.stop()


def run(cli, torque_logger: TorqueRunLogger) -> int:
    record = load_record_module()
    raw_screw = record.make_screw_client(
        host=cli.screw_host,
        port=cli.screw_port,
        timeout=cli.screw_timeout_s,
        dry_run=False,
    )
    screw = LoggedScrewClient(raw_screw, torque_logger)
    screw_lock = threading.Lock()
    robot = None
    try:
        print(f"力矩 CSV：{torque_logger.csv_path}")
        robot = record.URDriver(
            cli.robot_host,
            connect_timeout_s=cli.robot_connect_timeout_s,
        )
        print("正在连接 UR3 和电批服务...")
        if not robot.connect():
            raise RuntimeError("UR3 connection failed")

        screw.connect()
        record.preflight_screwdriver(screw, screw_lock, require_fresh_feedback=True)
        # Robot was connected above so the preflight can stop before any prompt or motion.
        return 0 if run_with_robot(
            cli, record, robot, screw, screw_lock, torque_logger
        ) else 2
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，立即停止。")
        torque_logger.set_stop_reason("用户按下 Ctrl+C，测试被人工中止")
        return 130
    except Exception as exc:
        torque_logger.set_stop_reason(classify_exception_reason(exc, robot, cli))
        print(f"测试失败：{exc}", file=sys.stderr)
        return 1
    finally:
        safety_reason = read_robot_safety_reason(robot) if robot is not None else None
        if safety_reason is not None:
            torque_logger.set_stop_reason(safety_reason)
        if robot is not None:
            try:
                robot.stop_linear_motion(0.5)
            except Exception:
                pass
        try:
            with screw_lock:
                screw.hold()
        except Exception as exc:
            print(f"电批 hold 失败：{exc}", file=sys.stderr)
            if torque_logger.stop_reason == "尚未确定":
                torque_logger.set_stop_reason("停止时电批 hold 命令失败")
        screw.close()
        if robot is not None:
            robot.close()


def main() -> int:
    cli = parse_args()
    torque_logger = TorqueRunLogger(True)
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    text_file = torque_logger.text_path.open("w", encoding="utf-8")
    sys.stdout = TeeStream(original_stdout, text_file)
    sys.stderr = TeeStream(original_stderr, text_file)
    try:
        print(f"终端日志：{torque_logger.text_path}")
        print("测试模式：UR3 + 电批协调力控（固定启用）")
        print_test_parameters(cli)
        return run(cli, torque_logger)
    finally:
        print("")
        print("===== 本次电批力矩测试结束 =====")
        print(f"停止原因：{torque_logger.stop_reason}")
        print(f"收到状态样本：{torque_logger.sample_count}")
        print(
            "收到的最大绝对力矩："
            f"{torque_logger.max_abs_torque_nm:.6f} Nm "
            f"（对应原始有符号值 {torque_logger.max_torque_sample_nm:.6f} Nm）"
        )
        print_all_torque_samples(torque_logger)
        print(f"力矩 CSV：{torque_logger.csv_path}")
        print(f"终端日志：{torque_logger.text_path}")
        torque_logger.close()
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        text_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
