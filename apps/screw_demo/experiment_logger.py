# PROJECT FILE HEADER
# 文件：apps/screw_demo/experiment_logger.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

import csv
import json
import time
from pathlib import Path
from typing import Any, Dict, Optional


REPO_ROOT = Path(__file__).resolve().parents[2]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


class ExperimentLogger:
    def __init__(
        self,
        log_dir: str = "artifacts/logs/screw_demo",
        name: Optional[str] = None,
    ):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_dir = resolve_repo_path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.name = name or f"experiment_{timestamp}"
        self.jsonl_path = self.log_dir / f"{self.name}.jsonl"
        self.csv_path = self.log_dir / f"{self.name}.csv"
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._csv = csv.DictWriter(
            self._csv_file,
            fieldnames=["time", "state", "event", "tcp_x", "tcp_y", "tcp_z", "screw_mode", "ok"],
        )
        self._csv.writeheader()

    def log(
        self,
        state: str,
        event: str,
        robot_pose: Optional[Any] = None,
        screw_status: Optional[Dict[str, Any]] = None,
        ok: bool = True,
        **extra: Any,
    ) -> None:
        now = time.time()
        pose = list(robot_pose) if robot_pose is not None else []
        status = screw_status or {}
        record = {
            "time": now,
            "state": state,
            "event": event,
            "robot_pose": pose,
            "screw_status": status,
            "ok": ok,
            **extra,
        }
        with self.jsonl_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._csv.writerow(
            {
                "time": now,
                "state": state,
                "event": event,
                "tcp_x": pose[0] if len(pose) >= 3 else "",
                "tcp_y": pose[1] if len(pose) >= 3 else "",
                "tcp_z": pose[2] if len(pose) >= 3 else "",
                "screw_mode": status.get("mode", ""),
                "ok": ok,
            }
        )
        self._csv_file.flush()

    def close(self) -> None:
        self._csv_file.close()

    def __enter__(self) -> "ExperimentLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
