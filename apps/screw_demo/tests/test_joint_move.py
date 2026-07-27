# PROJECT FILE HEADER
# 文件：apps/screw_demo/tests/test_joint_move.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse

import _path_setup  # noqa: F401
from ur_driver import URDriver


def main() -> None:
    parser = argparse.ArgumentParser(description="Move one UR joint by a tiny angle and return.")
    parser.add_argument("--host", default="192.168.1.5", help="UR robot IP address")
    parser.add_argument("--joint-index", type=int, default=5, help="Joint index, 0 to 5")
    parser.add_argument("--dq", type=float, default=0.02, help="Joint offset in radians")
    parser.add_argument("--speed", type=float, default=0.2, help="moveJ speed in rad/s")
    parser.add_argument("--acc", type=float, default=0.2, help="moveJ acceleration in rad/s^2")
    parser.add_argument("--timeout", type=float, default=10.0, help="Motion timeout in seconds")
    args = parser.parse_args()

    if not 0 <= args.joint_index <= 5:
        raise ValueError("--joint-index must be between 0 and 5")

    with URDriver(args.host) as ur:
        start = ur.get_joint_positions()
        target = start.copy()
        target[args.joint_index] += args.dq

        print("Start joints:", start)
        print("Target joints:", target)
        print(f"Moving joint {args.joint_index}...")
        if not ur.move_j(target, speed=args.speed, acceleration=args.acc, timeout=args.timeout):
            raise RuntimeError("Failed to reach target joint position before timeout")

        print("Moving back...")
        if not ur.move_j(start, speed=args.speed, acceleration=args.acc, timeout=args.timeout):
            raise RuntimeError("Failed to return to start joint position before timeout")

        print("Done.")


if __name__ == "__main__":
    main()
