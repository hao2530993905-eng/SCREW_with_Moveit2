#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
from collections import deque
import datetime as dt
import importlib.util
import json
import math
import shutil
import statistics
import sys
import threading
import traceback
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_SRC_DIR = REPO_ROOT / "lerobot_robot_screw" / "src"
SCREW_DEMO_DIR = REPO_ROOT / "apps" / "screw_demo"
for path in reversed((PACKAGE_SRC_DIR, REPO_ROOT, SCREW_DEMO_DIR)):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import numpy as np


def _load_lightweight_processors():
    path = PACKAGE_SRC_DIR / "lerobot_robot_screw" / "processors.py"
    spec = importlib.util.spec_from_file_location("screw_record_processors", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load lightweight processors: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_processors = _load_lightweight_processors()
build_observation_state = _processors.build_observation_state
build_screw_state = _processors.build_screw_state
synthetic_front_image = _processors.synthetic_front_image

OBS_STATE = "observation.state"
OBS_IMAGE = "observation.images.front"
OBS_SCREW_STATE = "observation.screw_state"
ACTION = "action"


def dataset_features(height: int = 64, width: int = 64, use_videos: bool = True) -> dict:
    image_dtype = "video" if use_videos else "image"
    return {
        OBS_STATE: {"dtype": "float32", "shape": (13,), "names": _processors.STATE_NAMES},
        ACTION: {"dtype": "float32", "shape": (7,), "names": _processors.ACTION_NAMES},
        OBS_SCREW_STATE: {
            "dtype": "float32",
            "shape": (7,),
            "names": [
                "target_speed_rpm",
                "measured_speed_rpm",
                "motor_position_rad",
                "motor_velocity_rad_s",
                "current_amp",
                "torque_nm",
                "fault",
            ],
        },
        OBS_IMAGE: {
            "dtype": image_dtype,
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        "phase_id": {"dtype": "float32", "shape": (1,), "names": ["phase_id"]},
        "hole_id": {"dtype": "string", "shape": (1,), "names": ["hole_id"]},
    }

from main import (  # noqa: E402
    DEMO_APPROACH_POSE,
    DryRunRobot,
    DemoState,
    format_values,
    manual_align,
    move_joint_to_pose,
    move_linear,
    offset_in_tcp_frame,
    read_key,
    resolve_approach_pose,
    rotation_vector_to_matrix,
    validate_args,
    wait_for_enter_or_abort,
)
from screw_client import assert_ok, make_screw_client  # noqa: E402
from ur_driver import URDriver  # noqa: E402


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


PHASE_MANUAL_ALIGN = 1
PHASE_INSERT = 2
PHASE_POST_INSERT = 3

MM_TO_M = 0.001

_LEROBOT_DATASET_CLASS = None
_LEROBOT_DATASET_ERROR: BaseException | None = None
_LEROBOT_DATASET_READY = threading.Event()
_LEROBOT_DATASET_THREAD: threading.Thread | None = None
_LEROBOT_DATASET_LOCK = threading.Lock()
FULL_AUTO_ACTIVE = False


def healthy_screw_status(response: dict[str, Any], action: str = "status") -> dict[str, Any]:
    """Validate a server response and return a healthy screwdriver status dict."""
    if not isinstance(response, dict):
        raise RuntimeError(f"Screw-driver {action} response is not a dict: {response!r}")
    assert_ok(response, action)
    status = response.get("status", response)
    if not isinstance(status, dict):
        raise RuntimeError(f"Screw-driver {action} response has no status dict: {response}")
    if bool(status.get("emergency_stop", False)):
        raise RuntimeError("Screw-driver emergency stop is active")
    if bool(status.get("fault", False)):
        message = status.get("fault_message") or "unknown screwdriver fault"
        raise RuntimeError(f"Screw-driver fault: {message}")
    try:
        torque_nm = float(status.get("torque_nm", 0.0))
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"Invalid screwdriver torque value: {status.get('torque_nm')!r}") from exc
    if not math.isfinite(torque_nm):
        raise RuntimeError(f"Non-finite screwdriver torque value: {torque_nm!r}")
    return status


def preflight_screwdriver(
    screw_client,
    screw_lock: threading.Lock | None = None,
    require_fresh_feedback: bool = False,
) -> dict[str, Any]:
    """Fail before robot motion when the screwdriver service is unhealthy."""
    if screw_lock is None:
        response = screw_client.status()
    else:
        with screw_lock:
            response = screw_client.status()
    status = healthy_screw_status(response, "preflight status")
    if require_fresh_feedback:
        missing = [name for name in ("feedback_count", "feedback_age_ms") if name not in status]
        if missing:
            raise RuntimeError(
                "Screw-driver server is missing CAN freshness fields "
                f"{missing}; rebuild and restart apps/screw_driver/build-openarm/screw_server"
            )
        if int(status.get("feedback_count", 0)) <= 0:
            raise RuntimeError("Screw-driver has not received a valid CAN motor feedback frame")
    print(
        "Screw-driver preflight OK: "
        f"mode={status.get('mode', 'UNKNOWN')}, "
        f"torque={float(status.get('torque_nm', 0.0)):.4f} Nm, "
        f"measured_speed={float(status.get('measured_speed_rpm', 0.0)):.1f} rpm"
    )
    return status


def _load_lerobot_dataset() -> None:
    global _LEROBOT_DATASET_CLASS, _LEROBOT_DATASET_ERROR
    started = time.perf_counter()
    try:
        from lerobot.datasets import LeRobotDataset

        _LEROBOT_DATASET_CLASS = LeRobotDataset
        print(f"[后台] LeRobotDataset 预加载完成: {time.perf_counter() - started:.3f} 秒")
    except BaseException as exc:
        _LEROBOT_DATASET_ERROR = exc
        print(f"[后台] LeRobotDataset 预加载失败: {type(exc).__name__}: {exc}")
    finally:
        _LEROBOT_DATASET_READY.set()


def start_lerobot_dataset_preload() -> threading.Thread:
    global _LEROBOT_DATASET_THREAD
    with _LEROBOT_DATASET_LOCK:
        if _LEROBOT_DATASET_THREAD is None:
            print("[后台] 开始预加载 LeRobotDataset...")
            _LEROBOT_DATASET_THREAD = threading.Thread(
                target=_load_lerobot_dataset,
                name="lerobot-dataset-preload",
                daemon=True,
            )
            _LEROBOT_DATASET_THREAD.start()
        return _LEROBOT_DATASET_THREAD


def get_lerobot_dataset_class():
    start_lerobot_dataset_preload()
    _LEROBOT_DATASET_READY.wait()
    if _LEROBOT_DATASET_ERROR is not None:
        raise RuntimeError("LeRobotDataset import failed") from _LEROBOT_DATASET_ERROR
    if _LEROBOT_DATASET_CLASS is None:
        raise RuntimeError("LeRobotDataset import completed without a class")
    return _LEROBOT_DATASET_CLASS


def pose_xyz_mm_to_m(pose: list[float] | tuple[float, ...] | np.ndarray | None):
    if pose is None:
        return None
    converted = np.asarray(pose, dtype=float).copy()
    if converted.shape != (6,):
        raise ValueError(f"--approach-pose must contain 6 values, got {converted.shape}")
    converted[:3] *= MM_TO_M
    return converted.tolist()


def pose_with_xyz_mm(pose: list[float] | tuple[float, ...] | np.ndarray) -> np.ndarray:
    converted = np.asarray(pose, dtype=float).copy()
    converted[:3] /= MM_TO_M
    return converted


def format_pose_mm(pose: list[float] | tuple[float, ...] | np.ndarray) -> str:
    values = pose_with_xyz_mm(pose)
    return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def convert_cli_mm_to_robot_m(args):
    args.approach_pose = pose_xyz_mm_to_m(args.approach_pose)
    for name in (
        "jog_step",
        "max_align_offset",
        "max_insert_depth",
        "jog_speed",
        "insert_speed",
        "linear_acc",
        "stop_linear_acc",
        "stop_sync_speed_threshold",
        "warn_distance",
        "ik_position_error",
    ):
        setattr(args, name, getattr(args, name) * MM_TO_M)
    return args


class InsertDatasetRecorder:
    def __init__(self, robot, screw_client, screw_lock: threading.Lock, args):
        self.robot = robot
        self.screw_client = screw_client
        self.screw_lock = screw_lock
        self.args = args
        self.period = 1.0 / args.fps
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.error: BaseException | None = None
        self.frame_count = 0
        self.phase_id = PHASE_MANUAL_ALIGN
        self.screw_speed_rpm = 0.0
        self.previous_pose: np.ndarray | None = None
        self.frames: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self.raw_output_path: Path | None = args.raw_output_path
        self.raw_file = None

        if args.overwrite and args.dataset_root.exists():
            shutil.rmtree(args.dataset_root)
        if self.raw_output_path is not None:
            self.raw_output_path.parent.mkdir(parents=True, exist_ok=True)
            self.raw_file = self.raw_output_path.open("w", encoding="utf-8")

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        if not self.ready_event.wait(timeout=2.0):
            raise RuntimeError("LeRobotDataset recorder did not start within 2 seconds")

    def stop(self, save: bool = True) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=5.0)
        if save and self.frame_count > 0:
            self._save_lerobot_dataset()
        if self.raw_file is not None:
            self.raw_file.close()
            self.raw_file = None
        if self.error is not None:
            raise RuntimeError("LeRobotDataset recorder failed") from self.error

    def _save_lerobot_dataset(self) -> None:
        """Materialize buffered frames only after robot motion has finished."""
        lerobot_dataset_class = get_lerobot_dataset_class()
        dataset = lerobot_dataset_class.create(
            repo_id=self.args.repo_id,
            fps=self.args.fps,
            features=dataset_features(
                self.args.image_height,
                self.args.image_width,
                self.args.use_videos,
            ),
            root=self.args.dataset_root,
            robot_type="screw_robot",
            use_videos=self.args.use_videos,
            image_writer_processes=0,
            image_writer_threads=0,
        )
        for frame in self.frames:
            dataset.add_frame(frame)
        dataset.save_episode()
        dataset.finalize()

    def set_phase(self, phase_id: int) -> None:
        with self._lock:
            self.phase_id = phase_id

    def set_screw_speed(self, speed_rpm: float) -> None:
        with self._lock:
            self.screw_speed_rpm = speed_rpm

    def _snapshot_command_state(self) -> tuple[int, float]:
        with self._lock:
            return self.phase_id, self.screw_speed_rpm

    def _run(self) -> None:
        next_sample = time.monotonic()
        self.ready_event.set()
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now < next_sample:
                    time.sleep(min(next_sample - now, 0.002))
                    continue
                next_sample += self.period
                self._record_frame()
        except BaseException as exc:
            self.error = exc
            traceback.print_exc()
            self.stop_event.set()

    def _record_frame(self) -> None:
        phase_id, screw_speed_rpm = self._snapshot_command_state()
        if screw_speed_rpm != 0.0 and hasattr(self.screw_client, "heartbeat"):
            with self.screw_lock:
                self.screw_client.heartbeat()

        tcp_pose = np.asarray(self.robot.get_tcp_pose(), dtype=np.float32)
        tcp_force = np.asarray(self.robot.get_tcp_force(), dtype=np.float32)
        with self.screw_lock:
            response = self.screw_client.status()
        screw_status = self._extract_screw_status(response)
        screw_state = build_screw_state(screw_status)

        if self.previous_pose is None:
            tcp_delta = np.zeros(6, dtype=np.float32)
        else:
            tcp_delta = (tcp_pose - self.previous_pose).astype(np.float32)
        self.previous_pose = tcp_pose.copy()

        action = np.concatenate(
            [tcp_delta, np.asarray([screw_speed_rpm], dtype=np.float32)]
        ).astype(np.float32)
        image = synthetic_front_image(
            self.args.image_height,
            self.args.image_width,
            self.frame_count,
            phase_id,
        )

        observation_state = build_observation_state(
            tcp_pose,
            tcp_force,
            phase_id=phase_id,
            hole_origin_pose=self.args.hole_origin_pose,
        )
        frame = {
            OBS_STATE: observation_state,
            OBS_SCREW_STATE: screw_state,
            OBS_IMAGE: image,
            ACTION: action,
            "phase_id": np.asarray([phase_id], dtype=np.float32),
            "hole_id": self.args.hole_id,
            "task": self.args.task,
        }

        if self.raw_file is not None:
            tcp_pose_mm = pose_with_xyz_mm(tcp_pose)
            tcp_delta_mm = pose_with_xyz_mm(tcp_delta)
            self.raw_file.write(
                json.dumps(
                    {
                        "frame_index": self.frame_count,
                        "timestamp": time.time(),
                        OBS_STATE: observation_state.astype(float).tolist(),
                        OBS_SCREW_STATE: screw_state.astype(float).tolist(),
                        ACTION: action.astype(float).tolist(),
                        "phase_id": phase_id,
                        "hole_id": self.args.hole_id,
                        "tcp_pose": tcp_pose.astype(float).tolist(),
                        "tcp_pose_mm": tcp_pose_mm.astype(float).tolist(),
                        "tcp_delta": tcp_delta.astype(float).tolist(),
                        "tcp_delta_mm": tcp_delta_mm.astype(float).tolist(),
                        "tcp_force": tcp_force.astype(float).tolist(),
                        "screw_status": screw_status,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            self.raw_file.flush()

        self.frames.append(frame)
        self.frame_count += 1

    @staticmethod
    def _extract_screw_status(response: dict[str, Any]) -> dict[str, Any]:
        if "ok" in response and not response.get("ok", False):
            raise RuntimeError(f"Screw-driver command failed: {response}")
        status = response.get("status", response)
        if not isinstance(status, dict):
            raise RuntimeError(f"Screw-driver response has no status dict: {response}")
        return status


class ScrewHeartbeat:
    def __init__(self, screw_client, screw_lock: threading.Lock, period_s: float = 0.1):
        self.screw_client = screw_client
        self.screw_lock = screw_lock
        self.period_s = period_s
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.error: BaseException | None = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.error is not None:
            raise RuntimeError("Screw heartbeat failed") from self.error

    def _run(self) -> None:
        try:
            while not self.stop_event.is_set():
                with self.screw_lock:
                    response = self.screw_client.heartbeat()
                healthy_screw_status(response, "heartbeat")
                time.sleep(self.period_s)
        except BaseException as exc:
            self.error = exc
            self.stop_event.set()


class TorqueControlledInsert:
    """Run bounded moveL and stop both devices at the torque threshold."""

    def __init__(self, robot, screw_client, screw_lock: threading.Lock, args):
        self.robot = robot
        self.screw_client = screw_client
        self.screw_lock = screw_lock
        self.args = args
        self.cancel_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.move_result: bool | None = None
        self.move_error: BaseException | None = None
        self.torque_reached = False
        self.last_torque_nm = 0.0
        self.filtered_torque_nm = 0.0
        self.peak_torque_nm = 0.0
        self.confirmed_samples = 0
        self.trigger_source = "not_triggered"
        self.slowdown_sent = False
        self.measured_screw_speed_rpm = 0.0
        self.screw_speed_baseline_rpm = 0.0
        self.adaptive_insert_speed_m_s = 0.0
        self._speed_lock = threading.Lock()
        self.commanded_screw_speed_rpm = float(self.args.screw_speed_rpm)
        self.last_screw_speed_command_at = 0.0
        self.last_speed_control_torque_nm = 0.0
        self.last_speed_control_torque_at: float | None = None
        self.speed_control_peak_torque_nm = 0.0
        self.rotation_only_phase = False
        self._torque_trend_samples: deque[tuple[float, float]] = deque(maxlen=30)
        self.stop_reason = "not_started"
        self.started_at: float | None = None
        self.threshold_reached_at: float | None = None
        self.robot_stop_sent_at: float | None = None
        self.robot_stop_returned_at: float | None = None
        self.screw_hold_sent_at: float | None = None
        self.planned_screw_hold_delay_s = 0.0
        self.sync_tcp_speed_m_s = 0.0
        self._torque_samples: deque[float] = deque(maxlen=self.args.torque_filter_window)
        self._stop_lock = threading.Lock()
        self._stop_done = threading.Event()
        self._stop_started = False
        self._stop_error: BaseException | None = None

    def run(self, target: np.ndarray) -> bool:
        self.started_at = time.perf_counter()
        self.stop_reason = "monitoring"
        self.worker = threading.Thread(
            target=self._run_move,
            args=(target,),
            name="torque-controlled-insert",
            daemon=True,
        )
        self.worker.start()
        try:
            while self.worker.is_alive():
                status = self._read_status()
                measured_speed_rpm = abs(float(status.get("measured_speed_rpm", 0.0)))
                with self._speed_lock:
                    self.measured_screw_speed_rpm = measured_speed_rpm
                    if measured_speed_rpm >= self.args.dynamic_speed_min_valid_rpm:
                        self.screw_speed_baseline_rpm = max(
                            self.screw_speed_baseline_rpm, measured_speed_rpm
                        )
                self.last_torque_nm = float(status.get("torque_nm", 0.0))
                absolute_torque = abs(self.last_torque_nm)
                self.peak_torque_nm = max(self.peak_torque_nm, absolute_torque)
                self._torque_samples.append(absolute_torque)
                self.filtered_torque_nm = float(statistics.median(self._torque_samples))
                active_s = time.perf_counter() - self.started_at
                threshold_enabled = (
                    self.args.torque_stop_nm > 0.0
                    and active_s >= self.args.torque_min_active_s
                )
                # The raw threshold is a hard safety limit and must not wait for
                # the median window/confirmation counter.  On a fast torque ramp,
                # a 5-sample median followed by 3 confirmations adds roughly
                # (window//2 + confirmations-1) * poll_period of avoidable delay.
                # Keep the filtered path for rejecting isolated sub-threshold
                # noise, but stop on the first fresh sample at/above the limit.
                raw_limit_reached = (
                    threshold_enabled and absolute_torque >= self.args.torque_stop_nm
                )
                filtered_candidate = (
                    threshold_enabled
                    and len(self._torque_samples) >= self.args.torque_filter_window
                    and self.filtered_torque_nm >= self.args.torque_stop_nm
                )
                slowdown_threshold = self.args.torque_stop_nm * self.args.torque_slowdown_ratio
                now = time.perf_counter()
                control_torque_nm = self.filtered_torque_nm
                with self._speed_lock:
                    commanded_abs_rpm = abs(self.commanded_screw_speed_rpm)
                rotation_only_speed_rpm = min(
                    abs(self.args.screw_speed_rpm),
                    self.args.rotation_only_speed_rpm,
                )
                if (
                    threshold_enabled
                    and not self.rotation_only_phase
                    and self.slowdown_sent
                    and commanded_abs_rpm <= rotation_only_speed_rpm
                ):
                    with self._speed_lock:
                        self.rotation_only_phase = True
                    print(
                        "Rotation-only tightening: "
                        f"screw command={commanded_abs_rpm:.1f} rpm is in final "
                        f"crawl stage; UR axial feed stopped at "
                        f"filtered_torque={control_torque_nm:.4f} Nm"
                    )
                self._torque_trend_samples.append((now, control_torque_nm))
                torque_slope_nm_s = 0.0
                trend = list(self._torque_trend_samples)
                if len(trend) >= 5 and trend[-1][0] - trend[0][0] >= 0.06:
                    mean_t = sum(item[0] for item in trend) / len(trend)
                    mean_q = sum(item[1] for item in trend) / len(trend)
                    denominator = sum((item[0] - mean_t) ** 2 for item in trend)
                    if denominator > 1e-12:
                        fitted_slope = sum(
                            (item[0] - mean_t) * (item[1] - mean_q)
                            for item in trend
                        ) / denominator
                        net_rise = trend[-1][1] - trend[0][1]
                        if net_rise >= self.args.dynamic_screw_prediction_min_rise_nm:
                            torque_slope_nm_s = max(0.0, fitted_slope)
                self.last_speed_control_torque_nm = control_torque_nm
                self.last_speed_control_torque_at = now
                prediction_floor_nm = slowdown_threshold * 0.5
                predicted_torque_nm = control_torque_nm
                if control_torque_nm >= prediction_floor_nm:
                    predicted_torque_nm += (
                        torque_slope_nm_s * self.args.dynamic_screw_speed_prediction_s
                    )
                self.speed_control_peak_torque_nm = max(
                    self.speed_control_peak_torque_nm,
                    control_torque_nm,
                )
                predicted_torque_nm = max(
                    self.speed_control_peak_torque_nm, predicted_torque_nm
                )
                if (
                    threshold_enabled
                    and predicted_torque_nm >= slowdown_threshold
                    and absolute_torque < self.args.torque_stop_nm
                    and status.get("mode") not in {"HOLDING", "FAULT", "EMERGENCY_STOP"}
                ):
                    with self._speed_lock:
                        measured_high_rpm = self.screw_speed_baseline_rpm
                    high_rpm = min(
                        abs(self.args.screw_speed_rpm),
                        measured_high_rpm
                        if measured_high_rpm >= self.args.dynamic_speed_min_valid_rpm
                        else abs(self.args.screw_speed_rpm),
                    )
                    crawl_abs_rpm = min(high_rpm, self.args.torque_crawl_rpm)
                    full_crawl_torque_nm = (
                        self.args.torque_stop_nm
                        * self.args.dynamic_screw_full_crawl_ratio
                    )
                    normalized = (
                        predicted_torque_nm - slowdown_threshold
                    ) / max(full_crawl_torque_nm - slowdown_threshold, 1e-6)
                    normalized = min(max(normalized, 0.0), 1.0)
                    desired_abs_rpm = high_rpm + (
                        crawl_abs_rpm - high_rpm
                    ) * normalized

                    since_command_s = now - self.last_screw_speed_command_at
                    current_abs_rpm = min(
                        abs(self.commanded_screw_speed_rpm), high_rpm
                    )
                    max_drop = self.args.dynamic_screw_speed_slew_rpm_s * since_command_s
                    if normalized >= 1.0:
                        next_abs_rpm = desired_abs_rpm
                    else:
                        next_abs_rpm = min(
                            current_abs_rpm,
                            max(desired_abs_rpm, current_abs_rpm - max_drop),
                        )
                    should_send = (
                        since_command_s >= self.args.dynamic_screw_speed_command_interval_s
                        and current_abs_rpm - next_abs_rpm
                        >= self.args.dynamic_screw_speed_min_command_delta_rpm
                    )
                    if should_send:
                        next_rpm = math.copysign(next_abs_rpm, self.args.screw_speed_rpm)
                        with self.screw_lock:
                            response = self.screw_client.set_speed(next_rpm)
                        assert_ok(response, "dynamic torque slowdown")
                        with self._speed_lock:
                            self.commanded_screw_speed_rpm = next_rpm
                        self.last_screw_speed_command_at = now
                        self.slowdown_sent = True
                        print(
                            "Dynamic screwdriver speed: "
                            f"torque={control_torque_nm:.4f}, "
                            f"predicted={predicted_torque_nm:.4f}/"
                            f"{self.args.torque_stop_nm:.4f} Nm, "
                            f"effective_high={high_rpm:.1f} rpm, "
                            f"command={next_rpm:.1f} rpm"
                        )
                if raw_limit_reached:
                    self.confirmed_samples = 1
                    self.trigger_source = "raw_hard_limit"
                elif filtered_candidate:
                    self.confirmed_samples += 1
                    self.trigger_source = "filtered_confirmation"
                else:
                    self.confirmed_samples = 0
                    self.trigger_source = "not_triggered"

                if raw_limit_reached or (
                    filtered_candidate
                    and self.confirmed_samples >= self.args.torque_confirm_samples
                ):
                    self.torque_reached = True
                    self.stop_reason = "torque_confirmed"
                    self.threshold_reached_at = time.perf_counter()
                    self.cancel_event.set()
                    print(
                        "Torque threshold confirmed: "
                        f"raw={self.last_torque_nm:.4f} Nm, "
                        f"filtered={self.filtered_torque_nm:.4f} Nm, "
                        f"limit={self.args.torque_stop_nm:.4f} Nm, "
                        f"samples={self.confirmed_samples}, "
                        f"trigger={self.trigger_source}"
                    )
                    self._stop_both()
                    break
                time.sleep(self.args.torque_poll_s)
        except BaseException as exc:
            self.stop_reason = f"error:{type(exc).__name__}"
            self.cancel_event.set()
            try:
                self._stop_both()
            except BaseException as stop_exc:
                raise RuntimeError(
                    f"Insertion failed ({exc}) and coordinated stop also failed ({stop_exc})"
                ) from exc
            raise
        finally:
            if self.worker is not None:
                self.worker.join(timeout=max(2.0, self.args.motion_timeout + 1.0))
                if self.worker.is_alive():
                    self._stop_both()
                    raise RuntimeError("Torque-controlled moveL thread did not stop")

        if self.move_error is not None:
            self.stop_reason = "move_error"
            self._stop_both()
            raise RuntimeError("Torque-controlled moveL failed") from self.move_error
        if self.torque_reached:
            self._print_stop_summary()
            return True
        if self.move_result:
            self.stop_reason = "maximum_travel_reached"
            self.cancel_event.set()
            self._stop_both()
            raise RuntimeError(
                "Maximum insertion travel reached before torque threshold; "
                "the screwdriver was not considered tight"
            )
        self.stop_reason = "motion_stopped_without_torque"
        self.cancel_event.set()
        self._stop_both()
        raise RuntimeError(
            "Insertion moveL stopped before torque threshold; "
            "the screwdriver was not considered tight"
        )

    def _run_move(self, target: np.ndarray) -> None:
        try:
            if not hasattr(self.robot, "servo_l"):
                raise RuntimeError(
                    "Adaptive insertion is permanently enabled, but robot has no servo_l interface"
                )
            self.move_result = self._run_adaptive_servo_insert(target)
        except BaseException as exc:
            self.move_error = exc

    def _run_adaptive_servo_insert(self, target: np.ndarray) -> bool:
        start = np.asarray(self.robot.get_tcp_pose(), dtype=float)
        target = np.asarray(target, dtype=float)
        translation = target[:3] - start[:3]
        distance = float(np.linalg.norm(translation))
        if distance <= 1e-9:
            return True
        started = time.perf_counter()
        last_reported = started
        progress = 0.0
        filtered_ratio = 0.0
        rotation_only_hold_pose: np.ndarray | None = None
        print("\n[TORQUE_CONTROLLED_INSERT_ADAPTIVE_SERVOL]")
        print("Target pose:", format_values(target))
        try:
            while progress < distance and not self.cancel_event.is_set():
                if time.perf_counter() - started > self.args.motion_timeout:
                    return False
                with self._speed_lock:
                    measured_rpm = self.measured_screw_speed_rpm
                    baseline_rpm = self.screw_speed_baseline_rpm
                    commanded_rpm = abs(self.commanded_screw_speed_rpm)
                    rotation_only = self.rotation_only_phase
                if rotation_only:
                    if rotation_only_hold_pose is None:
                        # Do not call stopL here. stopL terminates the active
                        # servoL session, which makes the motion worker return
                        # early and the coordinated cleanup then sends HOLD to
                        # the screwdriver.  Lock the actual TCP pose by keeping
                        # the servoL stream alive at this fixed target instead.
                        rotation_only_hold_pose = np.asarray(
                            self.robot.get_tcp_pose(), dtype=float
                        )
                        print(
                            "Adaptive insert: UR position locked; screwdriver continues rotating"
                        )
                    requested_ratio = 0.0
                    filtered_ratio = 0.0
                elif baseline_rpm < self.args.dynamic_speed_min_valid_rpm:
                    requested_ratio = 0.0
                elif measured_rpm <= self.args.dynamic_speed_stall_rpm:
                    requested_ratio = 0.0
                else:
                    measured_ratio = min(1.0, measured_rpm / baseline_rpm)
                    commanded_ratio = min(
                        1.0, commanded_rpm / max(baseline_rpm, 1e-6)
                    )
                    requested_ratio = min(measured_ratio, commanded_ratio)
                    requested_ratio = max(
                        self.args.dynamic_speed_min_ratio, requested_ratio
                    )
                alpha = self.args.dynamic_speed_filter_alpha
                if not rotation_only:
                    filtered_ratio += alpha * (requested_ratio - filtered_ratio)
                adaptive_speed = self.args.insert_speed * filtered_ratio
                self.adaptive_insert_speed_m_s = adaptive_speed
                now = time.perf_counter()
                if now - last_reported >= 0.25:
                    print(
                        "Adaptive insert: "
                        f"measured={measured_rpm:.1f} rpm, "
                        f"baseline={baseline_rpm:.1f} rpm, "
                        f"ratio={filtered_ratio:.3f}, "
                        f"UR_speed={adaptive_speed * 1000.0:.3f} mm/s"
                    )
                    last_reported = now
                if rotation_only_hold_pose is None:
                    progress = min(
                        distance,
                        progress + adaptive_speed * self.args.dynamic_speed_servo_dt,
                    )
                    fraction = progress / distance
                    command = start + fraction * (target - start)
                    command[3:] = target[3:]
                else:
                    command = rotation_only_hold_pose.copy()
                if self.cancel_event.is_set():
                    break
                if not self.robot.servo_l(
                    command,
                    dt=self.args.dynamic_speed_servo_dt,
                    lookahead_time=0.05,
                    gain=300.0,
                ):
                    raise RuntimeError(
                        "UR servoL command was rejected after the approach move; "
                        "the screwdriver will be held for safety"
                    )
                time.sleep(self.args.dynamic_speed_servo_dt)
            return progress >= distance
        finally:
            # Coordinated stop owns the robot stop once torque cancellation has
            # started; avoid issuing a competing servoStop after stopL.
            if not self.cancel_event.is_set():
                self.robot.stop_servo()

    def _read_status(self) -> dict[str, Any]:
        with self.screw_lock:
            response = self.screw_client.status()
        return healthy_screw_status(response, "status")

    def _stop_both(self) -> None:
        """Stop UR motion first, then hold the screwdriver immediately afterwards.

        ur_rtde stopL may block until the commanded deceleration has started or
        completed.  Starting the screwdriver hold thread first made the electric
        driver visibly stop before the UR.  The explicit ordering prevents that
        unsafe lead while preserving the single-call coordinated-stop contract.
        """
        with self._stop_lock:
            if self._stop_started:
                first_caller = False
            else:
                self._stop_started = True
                first_caller = True
        if not first_caller:
            if not self._stop_done.wait(timeout=2.5):
                raise RuntimeError("Timed out waiting for coordinated stop")
            if self._stop_error is not None:
                raise RuntimeError("Coordinated stop failed") from self._stop_error
            return

        configured_delay_s = getattr(self.args, "screw_hold_delay_s", -1.0)
        feedback_synchronized = configured_delay_s < 0.0
        self.planned_screw_hold_delay_s = (
            -1.0 if feedback_synchronized else min(max(configured_delay_s, 0.0), 0.2)
        )

        errors: list[BaseException] = []
        stop_started = threading.Event()

        def timed_screwdriver_hold() -> None:
            try:
                stop_started.wait(timeout=1.0)
                if feedback_synchronized:
                    monitor_deadline = time.perf_counter() + 0.25
                    low_speed_samples = 0
                    while time.perf_counter() < monitor_deadline:
                        try:
                            actual_tcp_speed = np.asarray(self.robot.get_tcp_speed(), dtype=float)
                            self.sync_tcp_speed_m_s = float(
                                np.linalg.norm(actual_tcp_speed[:3])
                            )
                            if self.sync_tcp_speed_m_s <= self.args.stop_sync_speed_threshold:
                                low_speed_samples += 1
                                if low_speed_samples >= 3:
                                    break
                            else:
                                low_speed_samples = 0
                        except Exception:
                            # Never skip the screwdriver safety stop because UR
                            # speed feedback became temporarily unavailable.
                            break
                        time.sleep(0.001)
                else:
                    deadline = self.robot_stop_sent_at + self.planned_screw_hold_delay_s
                    remaining_s = deadline - time.perf_counter()
                    if remaining_s > 0.0:
                        time.sleep(remaining_s)
                self.screw_hold_sent_at = time.perf_counter()
                if feedback_synchronized:
                    print(
                        "Coordinated stop: UR actual TCP speed reached "
                        f"{self.sync_tcp_speed_m_s * 1000.0:.3f} mm/s; sending screwdriver hold"
                    )
                else:
                    print(
                        "Coordinated stop: sending screwdriver hold after fixed delay "
                        f"({self.planned_screw_hold_delay_s * 1000.0:.3f} ms)"
                    )
                with self.screw_lock:
                    response = self.screw_client.hold()
                assert_ok(response, "hold")
            except BaseException as exc:
                errors.append(exc)

        hold_thread = threading.Thread(
            target=timed_screwdriver_hold,
            name="timed-screwdriver-hold",
        )
        hold_thread.start()
        try:
            self.robot_stop_sent_at = time.perf_counter()
            stop_started.set()
            print("Coordinated stop: sending UR stopL and starting sync timer")
            self.robot.stop_linear_motion(self.args.stop_linear_acc)
            self.robot_stop_returned_at = time.perf_counter()
        except BaseException as exc:
            errors.append(exc)
        finally:
            hold_thread.join(timeout=1.0)
            if hold_thread.is_alive():
                errors.append(RuntimeError("Timed screwdriver hold did not complete"))
            self._stop_error = errors[0] if errors else None
            self._stop_done.set()
        if self._stop_error is not None:
            raise RuntimeError("Coordinated robot/screwdriver stop failed") from self._stop_error

    def _print_stop_summary(self) -> None:
        delta_ms = None
        stopl_duration_ms = None
        if self.robot_stop_sent_at is not None and self.screw_hold_sent_at is not None:
            delta_ms = (self.screw_hold_sent_at - self.robot_stop_sent_at) * 1000.0
        if self.robot_stop_sent_at is not None and self.robot_stop_returned_at is not None:
            stopl_duration_ms = (
                self.robot_stop_returned_at - self.robot_stop_sent_at
            ) * 1000.0
        print(
            "Coordinated stop summary: "
            f"reason={self.stop_reason}, peak_torque={self.peak_torque_nm:.4f} Nm, "
            f"filtered_torque={self.filtered_torque_nm:.4f} Nm, "
            f"trigger={self.trigger_source}, "
            f"hold_after_stopL_start={delta_ms:.3f} ms, "
            f"stopL_call_duration={stopl_duration_ms:.3f} ms, "
            f"tcp_speed_at_hold={self.sync_tcp_speed_m_s * 1000.0:.3f} mm/s, "
            f"sync_mode={'feedback' if self.planned_screw_hold_delay_s < 0.0 else 'fixed_delay'}"
            if delta_ms is not None and stopl_duration_ms is not None else
            "Coordinated stop summary: stop command timestamps unavailable"
        )


def parse_args(argv: list[str] | None = None):
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(
        description="Run the main manual-align/insert flow and save one standard LeRobotDataset episode."
    )
    parser.add_argument("--robot-host", default="192.168.1.5")
    parser.add_argument("--enable-robot", action="store_true")
    parser.add_argument(
        "--approach-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        default=None,
        help="approach TCP pose; XYZ in millimeters, RX/RY/RZ in radians",
    )
    parser.add_argument("--use-hardcoded-pose", action="store_true")
    parser.add_argument("--jog-step", type=float, default=0.5, help="manual jog step in millimeters")
    parser.add_argument(
        "--max-align-offset",
        type=float,
        default=10.0,
        help="maximum manual XY offset radius in millimeters",
    )
    parser.add_argument(
        "--max-insert-depth",
        type=float,
        default=20.0,
        help="maximum safety travel in millimeters; normal stop is torque-based",
    )
    parser.add_argument("--insert-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--joint-speed", type=float, default=0.15)
    parser.add_argument("--joint-acc", type=float, default=0.15)
    parser.add_argument("--jog-speed", type=float, default=5.0, help="manual jog moveL speed in mm/s")
    parser.add_argument("--insert-speed", type=float, default=3.0, help="insert moveL speed in mm/s")
    parser.add_argument("--linear-acc", type=float, default=30.0, help="moveL acceleration in mm/s^2")
    parser.add_argument("--motion-timeout", type=float, default=20.0)
    parser.add_argument(
        "--warn-distance",
        type=float,
        default=50.0,
        help="ask for extra confirmation when TCP target is farther than this many millimeters",
    )
    parser.add_argument("--ik-position-error", type=float, default=0.1, help="IK position error in millimeters")
    parser.add_argument("--ik-orientation-error", type=float, default=1e-3)

    # Compatibility fields required by screw_demo.main.validate_args().
    parser.set_defaults(record_force=False)
    parser.add_argument("--force-log-hz", type=float, default=50.0)
    parser.add_argument("--force-log-start-timeout", type=float, default=2.0)
    parser.add_argument("--force-log-tail", type=float, default=0.5)
    parser.add_argument("--force-log-success-tail", type=float, default=0.0)
    parser.add_argument("--force-bias-samples", type=int, default=0)
    parser.add_argument("--force-bias-dt", type=float, default=0.02)

    parser.add_argument("--screw-host", default="127.0.0.1")
    parser.add_argument("--screw-port", type=int, default=5055)
    parser.add_argument("--screw-timeout-s", type=float, default=2.0)
    parser.add_argument("--screw-speed-rpm", type=float, default=60.0)
    parser.add_argument(
        "--torque-stop-nm",
        type=float,
        default=0.0,
        help="stop threshold from screwdriver torque feedback; required in real mode",
    )
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
    parser.add_argument(
        "--torque-poll-s",
        type=float,
        default=0.005,
        help="screwdriver status polling interval during insertion",
    )
    parser.add_argument(
        "--backend-torque-guard-ratio",
        type=float,
        default=2.0,
        help="moveL-only emergency screwdriver limit relative to coordinated stop threshold",
    )
    parser.add_argument(
        "--torque-filter-window",
        type=int,
        default=5,
        help="median-filter window for absolute torque samples",
    )
    parser.add_argument(
        "--torque-confirm-samples",
        type=int,
        default=3,
        help="consecutive filtered samples required before declaring the screw tight",
    )
    parser.add_argument(
        "--torque-min-active-s",
        type=float,
        default=0.2,
        help="ignore torque threshold candidates during this startup interval",
    )
    parser.add_argument(
        "--torque-slowdown-ratio",
        type=float,
        default=0.65,
        help="reduce screwdriver speed at this fraction of the hard torque limit",
    )
    parser.add_argument(
        "--torque-crawl-rpm",
        type=float,
        default=60.0,
        help="screwdriver speed after the torque approach threshold",
    )
    parser.add_argument(
        "--stop-linear-acc",
        type=float,
        default=2000.0,
        help="UR stopL deceleration in mm/s^2 used at the torque threshold",
    )
    parser.add_argument(
        "--screw-hold-delay-s",
        type=float,
        default=-1.0,
        help="fixed delay after stopL; negative waits for actual UR TCP speed to approach zero",
    )
    parser.add_argument(
        "--stop-sync-speed-threshold",
        type=float,
        default=0.5,
        help="UR TCP speed threshold in mm/s for feedback-synchronized screwdriver hold",
    )
    parser.add_argument("--post-insert-record-s", type=float, default=1.0)

    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
help="dataset output directory; defaults to a timestamped directory under artifacts/datasets",
    )
    parser.add_argument(
        "--repo-id",
        default=None,
        help="LeRobot repo id; defaults to local/real_screw_insert_<timestamp>",
    )
    parser.add_argument("--raw-output-path", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--use-videos", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--image-height", type=int, default=64)
    parser.add_argument("--image-width", type=int, default=64)
    parser.add_argument("--hole-id", default="hole_000")
    parser.add_argument("--task", default="manual align then insert screw with screwdriver")
    args = parser.parse_args(argv)

    if not 0.0 < args.torque_slowdown_ratio < 1.0:
        parser.error("--torque-slowdown-ratio must be between 0 and 1")
    if args.torque_crawl_rpm <= 0.0:
        parser.error("--torque-crawl-rpm must be positive")
    if args.backend_torque_guard_ratio <= 1.0:
        parser.error("--backend-torque-guard-ratio must be greater than 1")
    if args.stop_sync_speed_threshold <= 0.0:
        parser.error("--stop-sync-speed-threshold must be positive")
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

    if args.dataset_root is None:
        args.dataset_root = REPO_ROOT / "artifacts" / "datasets" / f"real_insert_dataset_{timestamp}"
    else:
        args.dataset_root = resolve_repo_path(args.dataset_root)
    if args.repo_id is None:
        args.repo_id = f"local/real_screw_insert_{timestamp}"
    args.hole_origin_pose = None
    if args.raw_output_path is None:
        args.raw_output_path = args.dataset_root.parent / f"{args.dataset_root.name}_raw.jsonl"
    else:
        args.raw_output_path = resolve_repo_path(args.raw_output_path)
    if args.force_bias_dt is None:
        args.force_bias_dt = 1.0 / args.force_log_hz
    return convert_cli_mm_to_robot_m(args)


def validate_output_paths(args: argparse.Namespace) -> None:
    if args.dataset_root.exists() and not args.overwrite:
        raise FileExistsError(
            f"Dataset root already exists: {args.dataset_root}. "
            "Use --overwrite to replace it, or pass a new --dataset-root."
        )
    args.dataset_root.parent.mkdir(parents=True, exist_ok=True)
    if args.raw_output_path is not None:
        args.raw_output_path.parent.mkdir(parents=True, exist_ok=True)


def run(args, robot=None, screw_client=None) -> DemoState:
    validate_args(args)
    approach_pose = resolve_approach_pose(args)
    validate_output_paths(args)
    if robot is None:
        robot = URDriver(args.robot_host) if args.enable_robot else DryRunRobot()
    if screw_client is None:
        screw_client = make_screw_client(
            host=args.screw_host,
            port=args.screw_port,
            timeout=args.screw_timeout_s,
            dry_run=not args.enable_robot,
        )
    screw_lock = threading.Lock()
    recorder: InsertDatasetRecorder | None = None

    try:
        robot_is_connected = getattr(robot, "connected", False)
        if args.enable_robot and robot_is_connected:
            print("Using preconnected robot...")
        elif args.enable_robot:
            print("Connecting to robot...")
        else:
            print("Starting dry-run robot...")
        if not robot_is_connected:
            if not robot.connect():
                raise RuntimeError("Robot connection failed")
        screw_client.connect()
        preflight_screwdriver(
            screw_client,
            screw_lock,
            require_fresh_feedback=args.enable_robot,
        )

        # In full-auto mode there must be no dependency initialization pause
        # after the tool reaches the approach pose. Fully prepare the dataset
        # object before issuing MOVEJ, but do not start recording until arrival.
        if FULL_AUTO_ACTIVE:
            prepare_started = time.perf_counter()
            print("[FULL_AUTO] 正在提前准备 LeRobotDataset 记录器...")
            recorder = InsertDatasetRecorder(robot, screw_client, screw_lock, args)
            print(
                "[FULL_AUTO] LeRobotDataset 记录器已准备完成: "
                f"{time.perf_counter() - prepare_started:.3f} 秒"
            )

        print("\n[INIT]")
        print("This script records a LeRobotDataset episode from manual align through insert.")
        print("Current pose [mm, rad]:", format_pose_mm(robot.get_tcp_pose()))
        print("Current joints:", format_values(robot.get_joint_positions()))
        print("Configured approach pose [mm, rad]:", format_pose_mm(approach_pose))
        if not wait_for_enter_or_abort("Confirm the workspace is clear."):
            return DemoState.ABORTED

        if not move_joint_to_pose(robot, approach_pose, args, "MOVE_APPROACH_MOVEJ"):
            return DemoState.ABORTED
        print("Approach pose reached by joint-space motion.")
        if not wait_for_enter_or_abort("Confirm the tool is near the intended hole."):
            return DemoState.ABORTED

        if recorder is None:
            recorder = InsertDatasetRecorder(robot, screw_client, screw_lock, args)
        recorder.set_phase(PHASE_MANUAL_ALIGN)
        recorder.start()
        print(f"LeRobotDataset recording started at manual align: {args.dataset_root}")

        if not manual_align(robot, approach_pose, args):
            return DemoState.ABORTED

        aligned_pose = np.asarray(robot.get_tcp_pose(), dtype=float)
        args.hole_origin_pose = aligned_pose.astype(np.float32)
        local_insert = [0.0, 0.0, args.insert_sign * args.max_insert_depth]
        insert_target = offset_in_tcp_frame(aligned_pose, local_insert)
        axis = rotation_vector_to_matrix(aligned_pose[3:])[:, 2] * args.insert_sign

        print("\n[INSERT_WITH_SCREWDRIVER]")
        print(f"Maximum insertion travel: {args.max_insert_depth * 1000.0:.1f} mm")
        print(f"Torque stop threshold: {args.torque_stop_nm:.4f} Nm")
        print("Insertion direction in robot base frame:", format_values(axis))
        print("Insert target pose [mm, rad]:", format_pose_mm(insert_target))
        print(f"Screwdriver target speed: {args.screw_speed_rpm:.1f} rpm")
        print("Press P to start screwdriver and execute insert moveL, or Q to abort.")
        while True:
            key = read_key()
            if key == "q":
                return DemoState.ABORTED
            if key == "p":
                break

        insert_error: BaseException | None = None
        heartbeat: ScrewHeartbeat | None = None
        recorder.set_phase(PHASE_INSERT)
        recorder.set_screw_speed(args.screw_speed_rpm)
        try:
            prepare_control = getattr(robot, "ensure_control_script_ready", None)
            if callable(prepare_control):
                # The asynchronous approach moveJ may finish by exiting the
                # uploaded ur_rtde script. Restore it before starting the
                # screwdriver so a failed first servoL cannot immediately
                # trigger cleanup and HOLD the screwdriver.
                prepare_control()
            with screw_lock:
                response = screw_client.set_torque_stop(
                    args.torque_stop_nm * args.backend_torque_guard_ratio
                )
            assert_ok(response, "set_torque_stop")
            with screw_lock:
                response = screw_client.set_speed(args.screw_speed_rpm)
            healthy_screw_status(response, "set_speed")
            heartbeat = ScrewHeartbeat(screw_client, screw_lock)
            heartbeat.start()
            TorqueControlledInsert(
                robot,
                screw_client,
                screw_lock,
                args,
            ).run(insert_target)
        except BaseException as exc:
            insert_error = exc
        finally:
            recorder.set_phase(PHASE_POST_INSERT)
            recorder.set_screw_speed(0.0)
            try:
                if heartbeat is not None:
                    heartbeat.stop()
                with screw_lock:
                    response = screw_client.hold()
                assert_ok(response, "hold")
            finally:
                if args.post_insert_record_s > 0.0:
                    time.sleep(args.post_insert_record_s)

        if insert_error is not None:
            raise insert_error

        print("\n[DONE]")
        print(f"Recorded {recorder.frame_count} frame(s).")
        print("Final pose [mm, rad]:", format_pose_mm(robot.get_tcp_pose()))
        print("Final joints:", format_values(robot.get_joint_positions()))
        return DemoState.DONE

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping motion.")
        robot.stop_linear_motion()
        robot.stop_joint_motion()
        return DemoState.ABORTED
    except Exception as exc:
        print(f"\n[ERROR] {exc}")
        robot.stop_linear_motion()
        robot.stop_joint_motion()
        return DemoState.ERROR
    finally:
        if recorder is not None:
            try:
                recorder.stop(save=recorder.frame_count > 0)
                print(f"Saved LeRobotDataset: {args.dataset_root}")
            except Exception as exc:
                print(f"Recorder stop/save failed: {exc}")
        try:
            with screw_lock:
                screw_client.hold()
        except Exception:
            pass
        if hasattr(screw_client, "close"):
            screw_client.close()
        robot.close()


def main() -> None:
    result = run(parse_args())
    if result not in (DemoState.DONE, DemoState.ABORTED):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
