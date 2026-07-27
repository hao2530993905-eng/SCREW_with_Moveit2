# PROJECT FILE HEADER
# 文件：apps/screw_demo/tests/test_force_sensor.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse
import csv
import math
import time
from pathlib import Path
from typing import Iterable, List

import numpy as np

import _path_setup  # noqa: F401
from ur_driver import URDriver


DEFAULT_LOG_DIR = Path(__file__).resolve().parents[3] / "artifacts" / "logs" / "screw_demo"


def read_mean_wrench(robot: URDriver, samples: int, period: float) -> np.ndarray:
    values: List[List[float]] = []
    for _ in range(samples):
        values.append(robot.get_tcp_force())
        time.sleep(period)
    return np.mean(np.asarray(values, dtype=float), axis=0)


def force_norm(wrench: Iterable[float]) -> float:
    force = np.asarray(wrench, dtype=float)[:3]
    return float(np.linalg.norm(force))


def torque_norm(wrench: Iterable[float]) -> float:
    torque = np.asarray(wrench, dtype=float)[3:]
    return float(np.linalg.norm(torque))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test UR TCP force/wrench reading through rtde_receive.getActualTCPForce()."
    )
    parser.add_argument("--host", default="192.168.1.5", help="UR robot IP address")
    parser.add_argument("--duration", type=float, default=10.0, help="collection time in seconds")
    parser.add_argument("--hz", type=float, default=50.0, help="sampling rate")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="directory for CSV output")
    parser.add_argument("--name", default=None, help="optional CSV file name without extension")
    parser.add_argument(
        "--bias-samples",
        type=int,
        default=50,
        help="stationary samples used as zero bias; set 0 to record raw values only",
    )
    parser.add_argument("--print-every", type=float, default=0.5, help="status print interval in seconds")
    args = parser.parse_args()

    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if args.hz <= 0.0:
        raise ValueError("--hz must be positive")
    if args.bias_samples < 0:
        raise ValueError("--bias-samples must be non-negative")
    if args.print_every <= 0.0:
        raise ValueError("--print-every must be positive")

    args.log_dir.mkdir(parents=True, exist_ok=True)
    name = args.name or f"force_sensor_test_{time.strftime('%Y%m%d_%H%M%S')}"
    out_path = args.log_dir / f"{name}.csv"
    period = 1.0 / args.hz

    fieldnames = [
        "time",
        "elapsed",
        "raw_Fx",
        "raw_Fy",
        "raw_Fz",
        "raw_Tx",
        "raw_Ty",
        "raw_Tz",
        "Fx",
        "Fy",
        "Fz",
        "Tx",
        "Ty",
        "Tz",
        "force_norm",
        "torque_norm",
    ]

    with URDriver(args.host, connect_control=False) as robot:
        print(f"Connected to UR robot at {args.host}")
        print("Reading TCP force/wrench through RTDE getActualTCPForce()")

        if args.bias_samples > 0:
            print(f"Keep the tool stationary; collecting {args.bias_samples} bias samples...")
            bias = read_mean_wrench(robot, args.bias_samples, period)
        else:
            bias = np.zeros(6, dtype=float)
        print("Bias [Fx, Fy, Fz, Tx, Ty, Tz]:", bias.tolist())
        print(f"Writing force test data to {out_path}")

        start = time.time()
        next_sample = start
        next_print = start
        sample_count = 0

        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            f.flush()

            while True:
                now = time.time()
                elapsed = now - start
                if elapsed > args.duration:
                    break

                raw_wrench = np.asarray(robot.get_tcp_force(), dtype=float)
                wrench = raw_wrench - bias
                f_norm = force_norm(wrench)
                t_norm = torque_norm(wrench)

                writer.writerow(
                    {
                        "time": now,
                        "elapsed": elapsed,
                        "raw_Fx": raw_wrench[0],
                        "raw_Fy": raw_wrench[1],
                        "raw_Fz": raw_wrench[2],
                        "raw_Tx": raw_wrench[3],
                        "raw_Ty": raw_wrench[4],
                        "raw_Tz": raw_wrench[5],
                        "Fx": wrench[0],
                        "Fy": wrench[1],
                        "Fz": wrench[2],
                        "Tx": wrench[3],
                        "Ty": wrench[4],
                        "Tz": wrench[5],
                        "force_norm": f_norm,
                        "torque_norm": t_norm,
                    }
                )
                sample_count += 1

                if now >= next_print:
                    print(
                        f"t={elapsed:6.2f}s "
                        f"F=({wrench[0]:8.3f}, {wrench[1]:8.3f}, {wrench[2]:8.3f}) N "
                        f"|F|={f_norm:8.3f} N "
                        f"T=({wrench[3]:8.4f}, {wrench[4]:8.4f}, {wrench[5]:8.4f}) N*m"
                    )
                    next_print += args.print_every
                    f.flush()

                next_sample += period
                sleep_time = next_sample - time.time()
                if math.isfinite(sleep_time) and sleep_time > 0.0:
                    time.sleep(sleep_time)

    print(f"Done. Saved {sample_count} samples to {out_path}")


if __name__ == "__main__":
    main()
