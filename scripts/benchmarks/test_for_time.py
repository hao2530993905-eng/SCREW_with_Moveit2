#!/usr/bin/env python3
# PROJECT FILE HEADER
# 文件：scripts/benchmarks/test_for_time.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

"""Measure startup stages of the camera-guided recording entry point."""

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TARGET = PROJECT_ROOT / "lerobot_robot_screw" / "scripts" / "recording" / "record_camera_insert_dataset.py"
LOG_PATH = PROJECT_ROOT / "artifacts" / "logs" / "benchmarks" / f"test_for_time_{datetime.now():%Y%m%d_%H%M%S}.log"


def load_target():
    spec = importlib.util.spec_from_file_location("record_camera_insert_timed", TARGET)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载脚本: {TARGET}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_line(log, message: str) -> None:
    print(message, flush=True)
    log.write(message + "\n")
    log.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="测试相机脚本到机械臂连接完成各阶段耗时")
    parser.add_argument("--camera-ip", default="192.168.1.66")
    parser.add_argument("--camera-python", default=sys.executable)
    parser.add_argument(
        "--no-save-ply",
        action="store_true",
        help="将参数传递给 ring.py，不保存和重新读取 PLY",
    )
    parser.add_argument(
        "--only-outer",
        action="store_true",
        help="将参数传递给 ring.py，只拟合外圈",
    )
    parser.add_argument("--record-python", default=sys.executable)
    parser.add_argument("--robot-host", default="192.168.1.5")
    parser.add_argument("--screw-host", default="127.0.0.1")
    parser.add_argument("--screw-port", type=int, default=5055)
    args = parser.parse_args()

    target = load_target()
    started = time.perf_counter()
    marks: dict[str, float] = {"脚本启动": started}

    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LOG_PATH.open("w", encoding="utf-8") as log:
        write_line(log, "===== 启动耗时测试开始 =====")
        write_line(log, f"测试脚本: {TARGET}")
        write_line(log, f"日志文件: {LOG_PATH}")
        write_line(log, f"开始时间: {datetime.now():%Y-%m-%d %H:%M:%S}")

        camera_command = [args.camera_python, str(target.CAMERA_SCRIPT), args.camera_ip]
        if args.no_save_ply:
            camera_command.append("--no-save-ply")
        if args.only_outer:
            camera_command.append("--only-outer")
        camera_start = time.perf_counter()
        marks["相机进程启动"] = camera_start
        write_line(log, "[阶段1] 正在启动相机并执行拍摄与点云处理...")
        try:
            completed = subprocess.run(
                camera_command,
                cwd=str(target.CAMERA_DIR),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            if completed.returncode != 0:
                raise RuntimeError(f"Camera script failed with exit code {completed.returncode}")
            camera_result = target.parse_camera_result(completed.stdout)
        except Exception as exc:
            elapsed = time.perf_counter() - started
            write_line(log, f"相机阶段失败: {type(exc).__name__}: {exc}")
            write_line(log, f"失败前总耗时: {elapsed:.3f} 秒")
            return 1
        camera_done = time.perf_counter()
        marks["相机结果返回"] = camera_done
        write_line(log, f"[阶段1完成] 相机拍摄和点云处理耗时: {camera_done - camera_start:.3f} 秒")

        calculation_start = camera_done
        write_line(log, "[阶段2] 正在计算目标点和目标姿态...")
        camera_center = target.select_ring_center(camera_result, "outer")
        camera_normal = target.outer_plane_normal(camera_result)
        calibration = target.load_calibration(None)
        base_pose = list(target.DEFAULT_BASE_APPROACH_POSE_MM_RAD)
        approach_pose = target.camera_center_to_approach_pose(
            camera_center, camera_normal, base_pose, calibration
        )
        calculation_done = time.perf_counter()
        marks["目标位姿计算完成"] = calculation_done
        write_line(log, f"[阶段2完成] 目标位姿计算耗时: {calculation_done - calculation_start:.3f} 秒")
        write_line(log, f"外圈圆心: {camera_center}")
        write_line(log, f"目标位姿: {approach_pose}")

        record_command = [
            args.record_python,
            "-u",
            str(target.RECORD_SCRIPT),
            "--enable-robot",
            "--robot-host",
            args.robot_host,
            "--approach-pose",
            *[f"{value:.12g}" for value in approach_pose],
            "--screw-host",
            args.screw_host,
            "--screw-port",
            str(args.screw_port),
            "--insert-depth",
            "40",
            "--insert-speed",
            "3",
            "--insert-sign",
            "1",
            "--screw-speed-rpm",
            "60",
            "--fps",
            "20",
        ]
        record_start = time.perf_counter()
        marks["机械臂启动命令执行"] = record_start
        write_line(log, "[阶段3] 相机处理完成，正在启动机械臂连接...")
        write_line(log, "记录脚本命令: " + " ".join(record_command))

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        process = subprocess.Popen(
            record_command,
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
        robot_ready = None
        try:
            assert process.stdout is not None
            for line in process.stdout:
                line = line.rstrip()
                if line:
                    write_line(log, "[记录脚本] " + line)
                if "Current joints:" in line:
                    robot_ready = time.perf_counter()
                    marks["机械臂启动完成"] = robot_ready
                    write_line(log, f"[阶段3完成] 机械臂连接并完成初始化耗时: {robot_ready - record_start:.3f} 秒")
                    write_line(log, "已在机械臂开始运动前停止测试，不执行移动和数据采集。")
                    process.send_signal(__import__("signal").SIGINT)
                    break
            if robot_ready is None and process.poll() is None:
                process.terminate()
        finally:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

        finished = time.perf_counter()
        marks["测试结束"] = finished
        write_line(log, "")
        write_line(log, "===== 耗时汇总 =====")
        write_line(log, f"启动脚本 -> 相机启动: {marks['相机进程启动'] - marks['脚本启动']:.3f} 秒")
        write_line(log, f"相机启动 -> 相机结果返回: {marks['相机结果返回'] - marks['相机进程启动']:.3f} 秒")
        write_line(log, f"相机结果返回 -> 目标位姿计算完成: {marks['目标位姿计算完成'] - marks['相机结果返回']:.3f} 秒")
        if robot_ready is not None:
            write_line(log, f"目标位姿计算完成 -> 机械臂启动完成: {robot_ready - marks['目标位姿计算完成']:.3f} 秒")
            write_line(log, f"相机结果返回 -> 机械臂启动完成: {robot_ready - marks['相机结果返回']:.3f} 秒")
        write_line(log, f"脚本启动 -> 机械臂启动完成: {(robot_ready or finished) - started:.3f} 秒")
        write_line(log, f"实际测试总耗时: {finished - started:.3f} 秒")
        write_line(log, "===== 启动耗时测试结束 =====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
