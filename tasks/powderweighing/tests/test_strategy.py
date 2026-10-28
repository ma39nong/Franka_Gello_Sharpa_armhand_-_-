import json
import numpy as np
import pytest

from tasks.powderweighing.strategy import (
    HardcodedPitchRetargeter,
    PitchPhase,
    PitchTrigger,
    TriggerConfig,
    install_on_manus_pipeline,
    load_config,
)


def trigger_config():
    return TriggerConfig(
        ready_enter_m=0.045,
        ready_exit_m=0.055,
        pitch_enter_m=0.018,
        pitch_exit_m=0.030,
        dwell_seconds=0.15,
        selection_margin_m=0.005,
    )


def test_trigger_dwell_pitch_and_hysteresis():
    trigger = PitchTrigger(trigger_config())
    assert trigger.update(0.00, 0.040, 0.080) is PitchPhase.FREE
    assert trigger.update(0.10, 0.040, 0.080) is PitchPhase.FREE
    assert trigger.update(0.16, 0.040, 0.080) is PitchPhase.PITCH_READY
    assert trigger.update(0.17, 0.017, 0.080) is PitchPhase.PITCH
    assert trigger.update(0.18, 0.025, 0.080) is PitchPhase.PITCH
    assert trigger.update(0.19, 0.031, 0.080) is PitchPhase.PITCH_READY
    assert trigger.update(0.20, 0.056, 0.080) is PitchPhase.FREE


def test_trigger_rejects_ambiguous_finger():
    trigger = PitchTrigger(trigger_config())
    trigger.update(0.00, 0.040, 0.043)
    assert trigger.update(0.20, 0.040, 0.043) is PitchPhase.FREE


def test_uncalibrated_template_is_rejected(tmp_path):
    path = tmp_path / "poses.json"
    path.write_text(json.dumps({
        "format": "powderweighing-o30i-hardcoded-poses",
        "version": 1,
        "side": "right",
        "joint_names": [f"j{i}" for i in range(20)],
        "trigger": trigger_config().__dict__,
        "max_joint_speed_rad_s": 3.0,
        "poses_ticks": {"pitch_ready": None, "pitch": None},
    }))
    with pytest.raises(ValueError, match="not calibrated"):
        load_config(path)


class FakeBase:
    joint_names = [f"j{i}" for i in reversed(range(20))]
    lower = np.zeros(20)
    upper = np.ones(20)
    last_qpos = np.zeros(20)

    def retarget(self, _frame):
        return np.full(20, 0.25), {"success": True}

    def reset(self):
        self.last_qpos = np.zeros(20)

    def close(self):
        return None


def test_tick_poses_convert_using_unmodified_zero_to_255_map(tmp_path):
    path = tmp_path / "poses.json"
    path.write_text(json.dumps({
        "format": "powderweighing-o30i-hardcoded-poses",
        "version": 1,
        "side": "right",
        "joint_names": [f"j{i}" for i in range(20)],
        "trigger": trigger_config().__dict__,
        "max_joint_speed_rad_s": 3.0,
        "poses_ticks": {
            "pitch_ready": list(range(20)),
            "pitch": [255] * 20,
        },
    }))
    wrapper = HardcodedPitchRetargeter(FakeBase(), load_config(path))
    expected_ticks = np.array(list(reversed(range(20))))
    assert np.array_equal(wrapper.pitch_ready_ticks, expected_ticks)
    assert np.allclose(wrapper.pitch_ready, expected_ticks / 255.0)
    assert np.array_equal(wrapper.pitch, np.ones(20))


def test_install_wraps_only_right_sharpa_o30i(tmp_path):
    path = tmp_path / "poses.json"
    path.write_text(json.dumps({
        "format": "powderweighing-o30i-hardcoded-poses",
        "version": 1,
        "side": "right",
        "joint_names": [f"j{i}" for i in range(20)],
        "trigger": trigger_config().__dict__,
        "max_joint_speed_rad_s": 3.0,
        "poses_ticks": {
            "pitch_ready": list(range(20)),
            "pitch": [255] * 20,
        },
    }))
    left = object()
    pipeline = type("FakePipeline", (), {})()
    pipeline.hands = {"left": "o30i", "right": "o30i"}
    pipeline.methods = {"left": "sharpa", "right": "sharpa"}
    pipeline.retargeters = {"left": left, "right": FakeBase()}

    wrapped = install_on_manus_pipeline(pipeline, load_config(path))

    assert pipeline.retargeters["left"] is left
    assert pipeline.retargeters["right"] is wrapped
    assert isinstance(wrapped, HardcodedPitchRetargeter)
