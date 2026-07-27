# PROJECT FILE HEADER
# 文件：apps/screw_demo/tests/test_servo_line.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse

import numpy as np

import _path_setup  # noqa: F401
from ur_driver import URDriver, make_linear_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Follow a very short TCP line with servoL.")
    parser.add_argument("--host", default="192.168.1.5", help="UR robot IP address")
    parser.add_argument("--dx", type=float, default=0.003, help="X offset in meters")
    parser.add_argument("--steps", type=int, default=100, help="Number of servo points")
    parser.add_argument("--dt", type=float, default=0.02, help="Servo period in seconds")
    args = parser.parse_args()

    with URDriver(args.host) as ur:
        start = np.asarray(ur.get_tcp_pose(), dtype=float)
        end = start.copy()
        end[0] += args.dx

        print("Start pose:", start.tolist())
        print("End pose:", end.tolist())
        print("Following servoL path...")
        path = make_linear_path(start, end, steps=args.steps)
        if not ur.follow_servo_path(path, dt=args.dt):
            raise RuntimeError("Failed to finish servoL path before timeout")
        print("Done.")


if __name__ == "__main__":
    main()
