from __future__ import annotations

from lerobot.datasets import LeRobotDataset

from lerobot_robot_screw.dataset import ACTION, OBS_IMAGE, OBS_SCREW_STATE, OBS_STATE, generate_smoke_dataset


def test_standard_lerobot_dataset_contains_state_action_image_and_custom_fields(tmp_path):
    root = tmp_path / "screw_dataset"
    generate_smoke_dataset(root=root, repo_id="local/screw_robot_pytest", frames=6, use_videos=False)

    dataset = LeRobotDataset("local/screw_robot_pytest", root=root, tolerance_s=1e-4)
    sample = dataset[2]

    assert len(dataset) == 6
    assert sample[OBS_STATE].shape[-1] == 13
    assert sample[OBS_SCREW_STATE].shape[-1] == 7
    assert sample[ACTION].shape[-1] == 7
    assert 3 in sample[OBS_IMAGE].shape
    assert "timestamp" in sample
    assert "phase_id" in sample
    assert sample["hole_id"] == "hole_000"
