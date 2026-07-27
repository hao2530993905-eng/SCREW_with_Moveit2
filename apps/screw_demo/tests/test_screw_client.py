# PROJECT FILE HEADER
# 文件：apps/screw_demo/tests/test_screw_client.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse

import _path_setup  # noqa: F401
from screw_client import assert_ok, make_screw_client


def main() -> None:
    parser = argparse.ArgumentParser(description="Test the screw-driver JSON socket client.")
    parser.add_argument("--host", default="127.0.0.1", help="screw_server host")
    parser.add_argument("--port", type=int, default=5055, help="screw_server port")
    parser.add_argument("--dry-run", action="store_true", help="do not connect to C++ server")
    args = parser.parse_args()

    with make_screw_client(args.host, args.port, dry_run=args.dry_run) as client:
        print(assert_ok(client.status(), "status"))
        print(assert_ok(client.forward(120), "forward"))
        print(assert_ok(client.heartbeat(), "heartbeat"))
        print(assert_ok(client.status(), "status"))
        print(assert_ok(client.reverse(80), "reverse"))
        print(assert_ok(client.hold(), "hold"))


if __name__ == "__main__":
    main()
