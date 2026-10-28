from pathlib import Path

import numpy as np
import pytest

from vive_tracker_teleop.calibration import (
    horizontal_world_to_control,
    write_world_to_control_rotation,
)
from vive_tracker_teleop.config import load_vive_config


def test_horizontal_calibration_recovers_expected_openvr_axes():
    rotation = horizontal_world_to_control(
        np.array([0.01, 0.02, -0.30]),
        np.array([-0.25, -0.01, 0.01]),
    )

    np.testing.assert_allclose(
        rotation,
        [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        atol=0.04,
    )
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_horizontal_calibration_rejects_short_or_inconsistent_motion():
    with pytest.raises(ValueError, match="forward movement"):
        horizontal_world_to_control(np.zeros(3), np.array([-0.2, 0.0, 0.0]))
    with pytest.raises(ValueError, match="inconsistent"):
        horizontal_world_to_control(
            np.array([0.0, 0.0, -0.2]),
            np.array([0.2, 0.0, 0.0]),
        )


def test_rotation_writer_preserves_the_rest_of_the_vive_config(tmp_path: Path):
    source = Path("config/modes/vive.yaml")
    destination = tmp_path / "vive.yaml"
    destination.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    expected = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    )

    write_world_to_control_rotation(destination, expected)
    loaded = load_vive_config(destination)

    np.testing.assert_allclose(loaded.world_to_control_rotation, expected)
    text = destination.read_text(encoding="utf-8")
    assert "LHR-306ED0EE" in text
    assert "Fixed transform from each physical Tracker" in text
