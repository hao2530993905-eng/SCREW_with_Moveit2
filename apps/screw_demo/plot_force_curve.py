# PROJECT FILE HEADER
# 文件：apps/screw_demo/plot_force_curve.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def read_column(rows, name):
    return [float(row[name]) for row in rows]


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot TCP force curves from collect_tcp_force.py CSV.")
    parser.add_argument("csv_path", help="CSV file recorded by collect_tcp_force.py")
    parser.add_argument("--out", default=None, help="output PNG path")
    parser.add_argument("--show", action="store_true", help="show interactive window")
    args = parser.parse_args()

    csv_path = resolve_repo_path(args.csv_path)
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"no rows found in {csv_path}")

    t = read_column(rows, "elapsed")
    fx = read_column(rows, "Fx")
    fy = read_column(rows, "Fy")
    fz = read_column(rows, "Fz")
    tx = read_column(rows, "Tx")
    ty = read_column(rows, "Ty")
    tz = read_column(rows, "Tz")
    f_axial = read_column(rows, "f_axial")
    f_lateral = read_column(rows, "f_lateral")

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(t, fx, label="Fx")
    axes[0].plot(t, fy, label="Fy")
    axes[0].plot(t, fz, label="Fz")
    axes[0].set_ylabel("Force (N)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, f_axial, label="f_axial")
    axes[1].plot(t, f_lateral, label="f_lateral")
    axes[1].set_ylabel("Contact feature (N)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, tx, label="Tx")
    axes[2].plot(t, ty, label="Ty")
    axes[2].plot(t, tz, label="Tz")
    axes[2].set_ylabel("Torque (N*m)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)

    label = rows[0].get("label", "")
    title = csv_path.name if not label else f"{csv_path.name} ({label})"
    fig.suptitle(title)
    fig.tight_layout()

    out_path = resolve_repo_path(args.out) if args.out else csv_path.with_suffix(".png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    print(f"Saved plot to {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
