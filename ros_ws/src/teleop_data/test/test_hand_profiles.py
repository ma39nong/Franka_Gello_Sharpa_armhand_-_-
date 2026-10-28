from pathlib import Path

import pytest

from teleop_data.config import load_config
from teleop_data.hand_profiles import create_hand_data_profile


def test_checked_in_recording_config_is_mixed_g20_o30i():
    config = load_config(
        Path(__file__).resolve().parents[1] / "config" / "recording.yaml"
    )
    assert config.hand_profiles["left"].model == "g20"
    assert config.hand_profiles["left"].width == 20
    assert config.hand_profiles["right"].model == "o30i"
    assert config.hand_profiles["right"].width == 20
    assert config.hand_profiles["right"].joint_names[0] == "thumb_cmc_roll"
    assert config.replay_hand_preposition_speed == {
        "left": 100.0,
        "right": 1.0,
    }


def test_o30i_data_profile_is_right_only_and_uses_urdf_radians():
    profile = create_hand_data_profile("o30i", side="right")
    assert profile.lower_bounds[4] == pytest.approx(-0.4)
    assert profile.upper_bounds[4] == pytest.approx(0.03711)
    with pytest.raises(ValueError, match="right only"):
        create_hand_data_profile("o30i", side="left")
