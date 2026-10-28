from pathlib import Path
from xml.etree import ElementTree

import mujoco
import numpy as np
import pytest
import yaml

import adapters.wuji.pipeline as pipeline_module
from adapters.wuji.pipeline import _config_path, _device_permutation
from adapters.wuji.wuji_retargeting import Retargeter


ROOT = Path(__file__).resolve().parent
HAND2_DEVICE_PERMUTATION = [
    16, 17, 18, 19,
    0, 1, 2, 3,
    4, 5, 6, 7,
    12, 13, 14, 15,
    8, 9, 10, 11,
]


def test_hand2_configs_resolve_twenty_joint_models():
    for side, prefix in (("left", "l_"), ("right", "r_")):
        config_path = ROOT / "config" / f"retarget_manus_wuji_hand_2_{side}.yaml"
        optimizer = yaml.safe_load(config_path.read_text())["optimizer"]
        urdf_path = (config_path.parent / optimizer["urdf_path"]).resolve()
        mjcf_path = (config_path.parent / optimizer["mjcf_path"]).resolve()
        assert urdf_path.is_file()
        assert mjcf_path.is_file()
        assert "models/hand2_beta" in urdf_path.as_posix()

        urdf = ElementTree.parse(urdf_path).getroot()
        urdf_joints = {
            joint.attrib["name"]
            for joint in urdf.findall("joint")
            if joint.attrib.get("type") in {"revolute", "continuous"}
        }
        mjcf_joints = {
            joint.attrib["name"]
            for joint in ElementTree.parse(mjcf_path).getroot().findall(".//joint")
            if joint.attrib.get("name")
        }
        assert len(urdf_joints) == 20
        assert urdf_joints == mjcf_joints
        assert all(name.startswith(prefix) for name in urdf_joints)

        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        assert model.nu == 20
        assert model.njnt == 20


def test_real_hand2_entry_uses_beta_model_and_device_joint_order():
    for side in ("left", "right"):
        config_path = _config_path(side, "wuji_hand_2")
        optimizer = yaml.safe_load(config_path.read_text())["optimizer"]
        assert "models/hand2_beta" in optimizer["urdf_path"]
        assert "models/hand2_beta" in optimizer["mjcf_path"]

        retargeter = Retargeter.from_yaml(str(config_path), side)
        permutation = _device_permutation(retargeter, config_path)
        assert permutation.tolist() == HAND2_DEVICE_PERMUTATION


def test_right_pinch_corrections_are_local_to_that_profile_and_targets():
    left_path = _config_path("left", "wuji_hand_2")
    right_path = _config_path("right", "wuji_hand_2")
    left = Retargeter.from_yaml(str(left_path), "left").optimizer
    right = Retargeter.from_yaml(str(right_path), "right").optimizer

    np.testing.assert_array_equal(left.pinch_tip_scaling, np.ones(5))
    assert left.index_pinch_thumb_shift_cm == 0.0
    np.testing.assert_array_equal(
        right.pinch_tip_scaling,
        np.array([1.0, 0.81, 0.90, 1.0, 1.0]),
    )
    assert right.index_pinch_thumb_shift_cm == pytest.approx(0.40)

    keypoints = np.zeros((21, 3), dtype=np.float64)
    keypoints[[4, 8, 12, 16, 20], 0] = np.arange(1.0, 6.0)
    original = right._compute_tip_vectors(keypoints, right.scaling)
    corrected = original * right.pinch_tip_scaling[:, None]
    np.testing.assert_allclose(corrected[1], original[1] * 0.81)
    np.testing.assert_allclose(corrected[2], original[2] * 0.90)
    np.testing.assert_array_equal(corrected[[0, 3, 4]], original[[0, 3, 4]])


def test_right_index_pinch_shifts_thumb_toward_middle_finger_only():
    right = Retargeter.from_yaml(
        str(_config_path("right", "wuji_hand_2")), "right"
    ).optimizer
    keypoints = np.zeros((21, 3), dtype=np.float64)
    keypoints[8] = [0.02, 0.01, 0.08]
    keypoints[12] = [0.02, -0.03, 0.08]
    target = np.zeros((5, 3), dtype=np.float64)

    right._apply_index_pinch_thumb_shift(
        target, keypoints, np.array([0.7, 0.7, 0.0, 0.0, 0.0])
    )
    np.testing.assert_allclose(target[0], [0.0, -0.40, 0.0])
    np.testing.assert_array_equal(target[1:], np.zeros((4, 3)))

    middle_pinch_target = np.zeros((5, 3), dtype=np.float64)
    right._apply_index_pinch_thumb_shift(
        middle_pinch_target,
        keypoints,
        np.array([0.7, 0.0, 0.7, 0.0, 0.0]),
    )
    np.testing.assert_array_equal(middle_pinch_target, np.zeros((5, 3)))


@pytest.mark.parametrize("side", ("left", "right"))
def test_real_hand2_tick_sends_reordered_command(monkeypatch, side):
    config_path = _config_path(side, "wuji_hand_2")
    real_retargeter = Retargeter.from_yaml(str(config_path), side)
    source_qpos = np.arange(20, dtype=np.float64)
    sent = []
    resets = []

    class FakeRetargeter:
        optimizer = real_retargeter.optimizer

        def retarget(self, _landmarks):
            return source_qpos.copy()

        def reset(self):
            resets.append(side)

    class FakeBridge:
        def __init__(self, _library):
            pass

        def connect(self, _calibrations):
            pass

        def read(self, _side, _timeout):
            return object()

        def close(self):
            pass

    class FakeBackend:
        def __init__(self, **_kwargs):
            pass

        def send(self, command):
            sent.append(np.asarray(command).copy())

        def close(self):
            pass

    monkeypatch.setattr(
        pipeline_module.Retargeter,
        "from_yaml",
        lambda _path, _side: FakeRetargeter(),
    )
    monkeypatch.setattr(pipeline_module, "ManusBridge", FakeBridge)
    monkeypatch.setattr(
        pipeline_module, "canonical_landmarks", lambda _frame: object()
    )
    monkeypatch.setattr(pipeline_module, "WujiHand2Backend", FakeBackend)

    pipeline = pipeline_module.WujiHandPipeline(
        sides=(side,),
        addresses={side: "test-address"},
    )
    try:
        pipeline.tick(now=1.0, active={side: True})
        pipeline.tick(now=1.1, active={side: False})
    finally:
        pipeline.close()

    assert len(sent) == 1
    assert resets == [side]
    snapshot = pipeline.command_snapshot(side)
    assert snapshot is not None
    assert snapshot["valid"] is True
    assert snapshot["positions"] == tuple(sent[0].tolist())
    np.testing.assert_array_equal(
        sent[0], source_qpos[HAND2_DEVICE_PERMUTATION]
    )


def test_hand2_home_pose_starts_from_measured_feedback():
    pipeline = object.__new__(pipeline_module.WujiHandPipeline)
    pipeline.sides = ("left",)
    pipeline.models = {"left": "wuji_hand_2"}
    pipeline.joint_limits = {"left": tuple((-2.0, 2.0) for _ in range(20))}
    pipeline.open_until = {"left": 10.0}
    pipeline.pose_moves = {"left": None}
    pipeline.last_feedback = {"left": None}
    pipeline.backends = {
        "left": type(
            "Backend",
            (),
            {
                "read_position": lambda _self: np.full(20, 0.1),
                "last_command_position": None,
            },
        )()
    }

    pipeline.request_pose({"left": tuple([0.5] * 20)}, max_speed=0.5)

    move = pipeline.pose_moves["left"]
    np.testing.assert_allclose(move["start"], 0.1)
    np.testing.assert_allclose(move["target"], 0.5)
    assert move["duration"] == pytest.approx(1.5)
    assert pipeline.open_until["left"] == 0.0
