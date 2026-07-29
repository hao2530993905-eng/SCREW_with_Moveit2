#!/usr/bin/env python3
"""Run the camera-to-insert workflow with the original direct RTDE moveJ."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PARALLEL_ENTRY = (
    REPO_ROOT
    / "lerobot_robot_screw"
    / "scripts"
    / "recording"
    / "record_camera_insert_dataset_parallel.py"
)


def main() -> None:
    arguments = list(sys.argv[1:])
    if "--start-moveit-stack" in arguments:
        raise SystemExit(
            "This entry always uses original RTDE moveJ; remove --start-moveit-stack."
        )

    try:
        separator = arguments.index("--")
    except ValueError:
        arguments.append("--")
        separator = len(arguments) - 1
    record_arguments = arguments[separator + 1:]
    if "--motion-backend" not in record_arguments:
        arguments.extend(["--motion-backend", "rtde"])

    sys.argv = [str(PARALLEL_ENTRY), *arguments]
    runpy.run_path(str(PARALLEL_ENTRY), run_name="__main__")


if __name__ == "__main__":
    main()
