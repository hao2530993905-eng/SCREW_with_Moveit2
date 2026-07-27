import sys
from dataclasses import dataclass

from lerobot.configs.parser import wrap
from lerobot.robots.config import RobotConfig


@dataclass
class CliConfig:
    robot: RobotConfig


def test_lerobot_cli_discovers_screw_robot_and_overrides(monkeypatch):
    @wrap()
    def parse(cfg: CliConfig) -> CliConfig:
        return cfg

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "prog",
            "--robot.discover_packages_path=lerobot_robot_screw",
            "--robot.type=screw_robot",
            "--robot.host=127.0.0.1",
            "--robot.fps=31",
            "--robot.dry_run=true",
        ],
    )

    cfg = parse()

    assert cfg.robot.type == "screw_robot"
    assert cfg.robot.host == "127.0.0.1"
    assert cfg.robot.fps == 31
    assert cfg.robot.dry_run is True
