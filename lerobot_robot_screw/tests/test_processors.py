from __future__ import annotations

import numpy as np

from lerobot_robot_screw.processors import build_observation_state, tcp_pose_to_hole_frame


def test_processors_build_local_state_vector():
    tcp = np.asarray([1, 2, 3, 0.1, 0.2, 0.3], dtype=np.float32)
    hole = np.asarray([0.5, 1, 2, 0, 0.1, 0.2], dtype=np.float32)
    force = np.asarray([0, 0, 5, 0, 0, 0], dtype=np.float32)

    local = tcp_pose_to_hole_frame(tcp, hole)
    state = build_observation_state(tcp, force, phase_id=3, hole_origin_pose=hole)

    assert np.allclose(local, [0.5, 1, 1, 0.1, 0.1, 0.1])
    assert state.shape == (13,)
    assert np.allclose(state[:6], local)
    assert np.allclose(state[6:12], force)
    assert state[12] == 3

