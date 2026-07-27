# PROJECT FILE HEADER
# 文件：apps/screw_demo/teach.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from ur_driver import URDriver


REPO_ROOT = Path(__file__).resolve().parents[2]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def pose_with_xyz_mm(pose):
    converted = np.asarray(pose, dtype=float).copy()
    converted[:3] *= 1000.0
    return converted


def read_key() -> str:
    if sys.platform.startswith("win"):
        import msvcrt

        return msvcrt.getwch().lower()
    import tty
    import termios

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class DryRunRobot:
    def __init__(self):
        self.pose = np.array([0.0, 0.0, 0.30, 0.0, 3.14, 0.0], dtype=float)

    def connect(self):
        return True

    def get_tcp_pose(self):
        return self.pose.tolist()

    def get_tcp_force(self):
        return [0.0] * 6

    def move_l(self, pose, **kwargs):
        self.pose = np.asarray(pose, dtype=float)
        return True

    def close(self):
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Keyboard teaching script for TCP waypoints.")
    parser.add_argument("--host", default="192.168.1.5")
    parser.add_argument("--enable-robot", action="store_true", help="actually connect to UR robot")
    parser.add_argument(
        "--out",
        default="artifacts/logs/screw_demo/teach_waypoints.json",
    )
    parser.add_argument("--step", type=float, default=0.001, help="linear jog step in meters")
    parser.add_argument("--speed", type=float, default=0.01)
    parser.add_argument("--acc", type=float, default=0.03)
    args = parser.parse_args()

    robot = URDriver(args.host) if args.enable_robot else DryRunRobot()
    robot.connect()
    waypoints = []
    out_path = resolve_repo_path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    key_to_delta = {
        "1": np.array([args.step, 0, 0, 0, 0, 0], dtype=float),
        "2": np.array([-args.step, 0, 0, 0, 0, 0], dtype=float),
        "3": np.array([0, args.step, 0, 0, 0, 0], dtype=float),
        "4": np.array([0, -args.step, 0, 0, 0, 0], dtype=float),
        "5": np.array([0, 0, args.step, 0, 0, 0], dtype=float),
        "6": np.array([0, 0, -args.step, 0, 0, 0], dtype=float),
    }

    print("Keys: 1/2 X +/-; 3/4 Y +/-; 5/6 Z +/-; s save waypoint; q quit")
    print("Displayed and saved waypoint pose uses [mm, mm, mm, rad, rad, rad].")
    try:
        while True:
            pose = np.asarray(robot.get_tcp_pose(), dtype=float)
            output_pose = pose_with_xyz_mm(pose)
            print("pose [mm, rad]:", output_pose.tolist())
            key = read_key()
            if key == "q":
                break
            if key == "s":
                waypoints.append(output_pose.tolist())
                out_path.write_text(json.dumps(waypoints, indent=2), encoding="utf-8")
                print(f"saved waypoint {len(waypoints)} -> {out_path}")
                continue
            if key in key_to_delta:
                target = pose + key_to_delta[key]
                if not robot.move_l(target, speed=args.speed, acceleration=args.acc, timeout=5.0):
                    print("move failed or timed out")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
