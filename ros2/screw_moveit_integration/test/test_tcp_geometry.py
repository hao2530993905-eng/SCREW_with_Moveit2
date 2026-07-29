import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "screw_moveit_integration"
    / "tcp_geometry.py"
)
SPEC = importlib.util.spec_from_file_location("tcp_geometry_under_test", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"Could not load {MODULE_PATH}")
TCP_GEOMETRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TCP_GEOMETRY)

compose_pose = TCP_GEOMETRY.compose_pose
tcp_target_to_ee_target = TCP_GEOMETRY.tcp_target_to_ee_target


class TcpGeometryTest(unittest.TestCase):
    def test_tcp_offset_is_removed_before_ik(self):
        target = [0.4, 0.1, 0.3, 0.0, 0.0, 0.0]
        offset = [0.0, 0.0, 0.137, 0.0, 0.0, 0.0]

        ee_target = tcp_target_to_ee_target(target, offset)

        np.testing.assert_allclose(
            ee_target,
            [0.4, 0.1, 0.163, 0.0, 0.0, 0.0],
            atol=1e-9,
        )

    def test_offset_follows_target_orientation(self):
        target = [0.4, 0.1, 0.3, 0.0, math.pi / 2.0, 0.0]
        offset = [0.0, 0.0, 0.137, 0.0, 0.0, 0.0]

        ee_target = tcp_target_to_ee_target(target, offset)

        np.testing.assert_allclose(
            ee_target[:3],
            [0.263, 0.1, 0.3],
            atol=1e-9,
        )
        np.testing.assert_allclose(
            ee_target[3:],
            target[3:],
            atol=1e-9,
        )

    def test_full_six_dimensional_offset_round_trip(self):
        target = [0.31, -0.18, 0.42, 0.2, -0.3, 1.0]
        offset = [0.012, -0.006, 0.237, 0.0, 0.15, -0.02]

        ee_target = tcp_target_to_ee_target(target, offset)
        reconstructed = compose_pose(ee_target, offset)

        np.testing.assert_allclose(reconstructed, target, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
