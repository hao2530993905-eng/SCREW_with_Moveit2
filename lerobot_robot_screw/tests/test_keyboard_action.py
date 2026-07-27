from __future__ import annotations

import numpy as np

from lerobot_robot_screw.config_screw_robot import ScrewRobotConfig
from lerobot_robot_screw.keyboard_action import action_from_key


def test_keyboard_jog_generates_recordable_7d_actions():
    config = ScrewRobotConfig(jog_step_m=0.003, jog_step_rad=0.04)

    assert np.allclose(action_from_key("right", config), [0.003, 0, 0, 0, 0, 0, 0])
    assert np.allclose(action_from_key("u", config), [0, 0, 0.003, 0, 0, 0, 0])
    assert np.allclose(action_from_key("d", config), [0, 0, 0, 0, 0, 0.04, 0])
    assert np.allclose(action_from_key("noop", config), np.zeros(7, dtype=np.float32))
