# PROJECT FILE HEADER
# 文件：apps/screw_demo/tests/test_small_move.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse

import _path_setup  # noqa: F401
from ur_driver import URDriver


def main() -> None:
    parser = argparse.ArgumentParser(description="Move UR TCP up a tiny distance and return.")
    parser.add_argument("--host", default="192.168.1.5", help="UR robot IP address")
    parser.add_argument("--dz", type=float, default=0.001, help="Z offset in meters")
    parser.add_argument("--speed", type=float, default=0.01, help="moveL speed in m/s")
    parser.add_argument("--acc", type=float, default=0.03, help="moveL acceleration in m/s^2")
    parser.add_argument("--timeout", type=float, default=10.0, help="Motion timeout in seconds")
    args = parser.parse_args()

    with URDriver(args.host) as ur:
        start = ur.get_tcp_pose()
        target = start.copy()
        target[2] += args.dz

        print("Start pose:", start)
        print("Target pose:", target)
        print("Moving to target...")
        if not ur.move_l(target, speed=args.speed, acceleration=args.acc, timeout=args.timeout):
            raise RuntimeError("Failed to reach target pose before timeout")
        print("Moving back...")
        if not ur.move_l(start, speed=args.speed, acceleration=args.acc, timeout=args.timeout):
            raise RuntimeError("Failed to return to start pose before timeout")
        print("Done.")


if __name__ == "__main__":
    main()
