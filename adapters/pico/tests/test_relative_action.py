from pathlib import Path

import numpy as np
import pytest

from pico_bimanual_franka_teleop.relative_action import (
    SolvedRelativeAction,
    load_preset_actions,
    sample_solved_action,
)


def test_preset_config_allows_future_unconfigured_slots(tmp_path):
    config = tmp_path / "presets.yaml"
    config.write_text(
        """slots:
  q: {label: Left test, side: left, action: test, speed_scale: 0.5}
  w: null
  e: null
""",
        encoding="utf-8",
    )
    presets = load_preset_actions(config, tmp_path / "data")

    assert presets["q"].path == Path(tmp_path / "data/arm_ui/actions/left__test.yaml")
    assert presets["q"].speed_scale == 0.5
    assert presets["w"] is None and presets["e"] is None


def test_solved_action_interpolates_at_scaled_time():
    solution = SolvedRelativeAction.from_dict(
        {
            "name": "test",
            "side": "left",
            "time_sec": [0.0, 1.0, 2.0],
            "positions": [[0.0] * 7, [1.0] * 7, [2.0] * 7],
            "start_q": [0.0] * 14,
            "speed_scale": 0.5,
        }
    )

    np.testing.assert_allclose(sample_solved_action(solution, 1.0), [0.5] * 7)
    assert solution.duration_sec == pytest.approx(4.0)
