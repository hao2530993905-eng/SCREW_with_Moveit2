# PROJECT FILE HEADER
# 文件：apps/screw_demo/tests/test_read_state.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse

import _path_setup  # noqa: F401
from ur_driver import URDriver


def main() -> None:
    parser = argparse.ArgumentParser(description="Read UR robot state without moving.")
    parser.add_argument("--host", default="192.168.1.5", help="UR robot IP address")
    args = parser.parse_args()

    with URDriver(args.host, connect_control=False) as ur:
        print("TCP pose [m, rad]:")
        print(ur.get_tcp_pose())
        print("Joint positions [rad]:")
        print(ur.get_joint_positions())
        print("TCP speed [m/s, rad/s]:")
        print(ur.get_tcp_speed())
        print("TCP force/wrench [N, N*m]:")
        print(ur.get_tcp_force())


if __name__ == "__main__":
    main()
