from __future__ import annotations

import numpy as np

from lerobot_robot_screw.config_screw_robot import ScrewRobotConfig
from lerobot_robot_screw.screw_robot import ScrewRobot


def test_dry_run_robot_connects_observes_and_clamps_actions_without_hardware():
    config = ScrewRobotConfig(
        dry_run=True,
        max_translation_step_m=0.001,
        max_rotation_step_rad=0.02,
        max_screw_speed_rpm=120.0,
    )
    robot = ScrewRobot(config)

    robot.connect()
    obs = robot.get_observation()
    applied = robot.send_action({"action": np.asarray([1, -1, 0.5, 1, -1, 0.5, 500], dtype=np.float32)})
    next_obs = robot.get_observation()

    assert robot.is_connected
    assert obs["observation.state"].shape == (13,)
    assert obs["observation.screw_state"].shape == (7,)
    assert obs["observation.images.front"].shape == (64, 64, 3)
    assert np.all(np.abs(applied["action"][:3]) <= 0.001)
    assert np.all(np.abs(applied["action"][3:6]) <= 0.02)
    assert applied["action"][6] == 120.0
    assert next_obs["screw_status"]["target_speed_rpm"] == 120.0

    robot.disconnect()
    assert not robot.is_connected
