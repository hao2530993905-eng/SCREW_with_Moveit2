#!/usr/bin/env python3
# PROJECT FILE HEADER
# 文件：scripts/benchmarks/test_for_time_parallel.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

"""Measure camera/robot parallel startup time of the camera wrapper."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TARGET = PROJECT_ROOT / "lerobot_robot_screw" / "scripts" / "recording" / "record_camera_insert_dataset_parallel.py"
LOG_PATH = PROJECT_ROOT / "artifacts" / "logs" / "benchmarks" / f"test_for_time_parallel_{datetime.now():%Y%m%d_%H%M%S}.log"


def write_line(log, message: str) -> None:
    print(message, flush=True)
    log.write(message + "\n")
    log.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="测试相机和机械臂并行启动耗时")
    parser.add_argument("--camera-ip", default="192.168.1.66")
    parser.add_argument("--camera-python", default=sys.executable)
    parser.add_argument("--record-python", default=sys.executable)
    parser.add_argument("--robot-host", default="192.168.1.5")
    parser.add_argument("--screw-host", default="127.0.0.1")
    parser.add_argument("--screw-port", type=int, default=5055)
    args = parser.parse_args()

    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    command = [
        args.camera_python,
        "-u",
        str(TARGET),
        "--camera-ip",
        args.camera_ip,
        "--target-ring",
        "outer",
        "--",
        "--enable-robot",
        "--robot-host",
        args.robot_host,
        "--insert-depth",
        "40",
        "--insert-speed",
        "3",
        "--insert-sign",
        "1",
        "--screw-host",
        args.screw_host,
        "--screw-port",
        str(args.screw_port),
        "--screw-speed-rpm",
        "60",
        "--fps",
        "20",
    ]

    start = time.perf_counter()
    marks: dict[str, float] = {"脚本启动": start}
    process = None

    with LOG_PATH.open("w", encoding="utf-8") as log:
        write_line(log, "===== 并行启动耗时测试开始 =====")
        write_line(log, f"测试脚本: {TARGET}")
        write_line(log, f"日志文件: {LOG_PATH}")
        write_line(log, f"开始时间: {datetime.now():%Y-%m-%d %H:%M:%S}")
        write_line(log, "[阶段1] 正在同时启动相机和机械臂连接...")

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        process = subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        robot_ready = None
        camera_done = None
        robot_start = None
        record_start = None
        try:
            assert process.stdout is not None
            for raw_line in process.stdout:
                line = raw_line.rstrip()
                now = time.perf_counter()

                if "正在启动相机" in line and "相机启动" not in marks:
                    marks["相机启动"] = now
                    write_line(log, "[相机] 已启动相机任务")
                elif "正在与相机并行启动机械臂和电批连接" in line:
                    robot_start = now
                    marks["机械臂连接启动"] = now
                    write_line(log, "[机械臂] 已启动机械臂和电批连接任务")
                elif "机械臂和电批连接已准备就绪" in line:
                    robot_ready = now
                    marks["机械臂连接完成"] = now
                    write_line(log, "[机械臂] 机械臂和电批连接已完成")
                elif "Generated approach pose" in line:
                    camera_done = now
                    marks["相机处理完成"] = now
                    write_line(log, "[相机] 拍摄、点云处理和目标位姿计算已完成")
                elif "正在当前进程中复用已连接的机械臂和电批" in line:
                    record_start = now
                    marks["记录流程启动"] = now
                    write_line(log, "[阶段2] 开始复用已连接设备执行记录流程")
                elif "Current joints:" in line:
                    marks["机械臂初始化完成"] = now
                    write_line(log, "[机械臂] 已读取当前关节状态，初始化完成")
                    write_line(log, "已在机械臂开始运动前停止测试，不执行移动和数据采集。")
                    process.send_signal(signal.SIGINT)
                    break

                if "Traceback" in line or "Error" in line or "错误" in line:
                    write_line(log, "[错误] " + line)

            if process.poll() is None:
                process.terminate()
        finally:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

        finished = time.perf_counter()
        write_line(log, "")
        write_line(log, "===== 并行耗时汇总 =====")

        if "相机启动" in marks and camera_done is not None:
            write_line(log, f"相机启动 -> 相机拍摄和计算完成: {camera_done - marks['相机启动']:.3f} 秒")
        if robot_start is not None and robot_ready is not None:
            write_line(log, f"机械臂连接启动 -> 机械臂连接完成: {robot_ready - robot_start:.3f} 秒")
        if camera_done is not None and robot_ready is not None:
            write_line(log, f"相机处理完成 -> 机械臂连接完成: {robot_ready - camera_done:.3f} 秒")
        if record_start is not None and "机械臂初始化完成" in marks:
            write_line(log, f"复用连接 -> 机械臂初始化完成: {marks['机械臂初始化完成'] - record_start:.3f} 秒")
        if "机械臂初始化完成" in marks:
            write_line(log, f"脚本启动 -> 机械臂初始化完成: {marks['机械臂初始化完成'] - start:.3f} 秒")
        write_line(log, f"并行流程实际总耗时: {finished - start:.3f} 秒")
        write_line(log, "===== 并行启动耗时测试结束 =====")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
