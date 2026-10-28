from pathlib import Path

import numpy as np
import pytest

from manus_teleop.o30i_retarget import O30IRetargeter

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF = (
    REPO_ROOT
    / "assets"
    / "linkerhand_o30i"
    / "right"
    / "linkerhand_o30i_right.urdf"
)


def test_o30i_retargeter_emits_twenty_urdf_radian_joints():
    retargeter = O30IRetargeter(URDF, "right")
    landmarks = retargeter.robot_landmarks(np.zeros(20))
    qpos, stats = retargeter.retarget(landmarks)
    assert len(retargeter.joint_names) == 20
    assert qpos.shape == (20,)
    assert np.isfinite(qpos).all()
    assert np.all(qpos >= retargeter.lower)
    assert np.all(qpos <= retargeter.upper)
    assert stats["loss"] >= 0.0


def test_o30i_retargeter_rejects_wrong_side_and_bad_landmarks():
    try:
        O30IRetargeter(URDF, "left")
    except ValueError as error:
        assert "right only" in str(error)
    else:
        raise AssertionError("left O30i unexpectedly accepted")
    retargeter = O30IRetargeter(URDF, "right")
    try:
        retargeter.retarget(np.zeros((20, 3)))
    except ValueError as error:
        assert "shape" in str(error)
    else:
        raise AssertionError("bad landmarks unexpectedly accepted")


def test_o30i_length_normalization_preserves_each_segment_direction():
    retargeter = O30IRetargeter(URDF, "right")
    robot = retargeter.robot_landmarks(np.zeros(20))
    shortened = robot.copy()
    chains = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "pinky": (17, 18, 19, 20),
    }
    for chain in chains.values():
        cursor = shortened[chain[0]].copy()
        segments = [
            robot[chain[index + 1]] - robot[chain[index]]
            for index in range(3)
        ]
        for index, segment in enumerate(segments):
            cursor = cursor + 0.6 * segment
            shortened[chain[index + 1]] = cursor

    normalized = retargeter.target_positions(shortened)

    for chain in chains.values():
        for index in range(3):
            expected = robot[chain[index + 1]] - robot[chain[index]]
            actual = normalized[chain[index + 1]] - normalized[chain[index]]
            expected /= np.linalg.norm(expected)
            actual /= np.linalg.norm(actual)
            assert np.allclose(actual, expected, atol=1e-8)


def test_o30i_ordinary_finger_targets_follow_anatomical_joint_frames():
    retargeter = O30IRetargeter(URDF, "right")
    for finger in ("index", "middle", "ring", "pinky"):
        expected = [
            retargeter.model.getFrameId(f"{finger}_proximal"),
            retargeter.model.getFrameId(f"{finger}_middle"),
            retargeter.model.getFrameId(f"{finger}_distal"),
        ]
        assert retargeter._frames[finger][:-1] == expected


def test_o30i_curl_keeps_the_mcp_knuckle_flexed():
    # The regression this locks: pure position matching under-flexed the MCP
    # against the palm by 15-19 degrees at full curl (2026-07-29 session),
    # dumping curl into the distal joints - the operator felt it as a weak
    # grasp. The segment-direction terms must preserve the distribution.
    retargeter = O30IRetargeter(
        URDF, "right", smooth_weight=0.0, filter_alpha=1.0, max_iterations=200
    )
    desired = np.zeros(20)
    for finger in ("index", "middle", "ring", "pinky"):
        desired[retargeter.joint_names.index(f"{finger}_mcp_pitch")] = 1.2
        desired[retargeter.joint_names.index(f"{finger}_pip")] = 1.0
        desired[retargeter.joint_names.index(f"{finger}_dip")] = 0.4
    landmarks = retargeter.robot_landmarks(desired)
    solved, _ = retargeter.retarget(landmarks)
    for finger in ("index", "middle", "ring", "pinky"):
        mcp = solved[retargeter.joint_names.index(f"{finger}_mcp_pitch")]
        assert abs(mcp - 1.2) < 0.15, f"{finger} mcp {mcp:.2f}"


def test_o30i_pinch_closes_the_thumb_index_gap():
    # Thumb and index tips touching in the input must nearly touch on the
    # robot. The pose below is the measured reachability oracle: the URDF can
    # close this gap to zero, so a large residual is a solver failure.
    retargeter = O30IRetargeter(URDF, "right", filter_alpha=1.0)
    pinch = np.zeros(20)
    for name, value in (
        ("thumb_cmc_yaw", 1.61),
        ("thumb_mcp", 0.26),
        ("thumb_ip", 1.57),
        ("index_mcp_pitch", 1.26),
        ("index_pip", 0.90),
        ("index_dip", 0.38),
    ):
        pinch[retargeter.joint_names.index(name)] = value
    landmarks = retargeter.robot_landmarks(pinch)
    for _ in range(3):  # warm-started convergence, as in a live stream
        qpos, stats = retargeter.retarget(landmarks)
    assert stats["pinch_activation"] > 0.5
    achieved = retargeter.robot_landmarks(qpos)
    gap = np.linalg.norm(achieved[4] - achieved[8])
    assert gap < 0.008, f"thumb-index gap {1000 * gap:.1f} mm"


def test_o30i_endpoint_calibration_reaches_open_and_curled_limits():
    oracle = O30IRetargeter(URDF, "right", filter_alpha=1.0)
    opened = oracle.robot_landmarks(np.zeros(20))
    desired = np.zeros(20)
    for name in ("pinky_mcp_pitch", "pinky_pip", "pinky_dip"):
        desired[oracle.joint_names.index(name)] = oracle.upper[
            oracle.joint_names.index(name)
        ]
    curled = oracle.robot_landmarks(desired)

    from pico_bimanual_franka_teleop.hand_retarget import (
        CANONICAL_FINGERS,
        chain_bend_angle,
    )

    open_bend = chain_bend_angle(opened[list(CANONICAL_FINGERS["pinky"])])
    curl_bend = chain_bend_angle(curled[list(CANONICAL_FINGERS["pinky"])])
    retargeter = O30IRetargeter(
        URDF,
        "right",
        filter_alpha=1.0,
        finger_open_ranges={"pinky": (open_bend + 0.01, open_bend + 0.1)},
        finger_curl_ranges={"pinky": (curl_bend - 0.1, curl_bend - 0.01)},
    )
    open_qpos, _ = retargeter.retarget(opened)
    curled_qpos, _ = retargeter.retarget(curled)
    for name in ("pinky_mcp_pitch", "pinky_pip", "pinky_dip"):
        index = retargeter.joint_names.index(name)
        assert open_qpos[index] == pytest.approx(retargeter.lower[index])
        assert curled_qpos[index] == pytest.approx(retargeter.upper[index])


def test_o30i_middle_pinch_anchor_has_bounded_activation_and_exact_endpoint():
    anchor = {
        "thumb_cmc_yaw": 1.2,
        "middle_mcp_pitch": 1.1,
        "middle_pip": 0.8,
    }
    retargeter = O30IRetargeter(
        URDF,
        "right",
        filter_alpha=1.0,
        contact_deadzone=0.01,
        middle_pinch_start=0.04,
        middle_pinch_activation_step=0.2,
        middle_pinch_anchor=anchor,
    )
    pinch = retargeter.robot_landmarks(np.zeros(20))
    pinch[4] = pinch[12]
    for expected in (0.2, 0.4, 0.6, 0.8, 1.0):
        qpos, _ = retargeter.retarget(pinch)
        assert retargeter._middle_pinch_activation == pytest.approx(expected)
    for name, value in anchor.items():
        assert qpos[retargeter.joint_names.index(name)] == pytest.approx(value)
    retargeter.reset()
    assert retargeter._middle_pinch_activation == 0.0


def test_o30i_index_middle_anchor_has_bounded_activation_and_zero_gap():
    anchor = {
        "index_mcp_roll": -0.19600,
        "index_mcp_pitch": 0.04298,
        "index_pip": 0.01072,
        "index_dip": 0.00182,
        "middle_mcp_roll": -0.38000,
        "middle_mcp_pitch": 0.0,
        "middle_pip": 0.06494,
        "middle_dip": 0.04417,
    }
    retargeter = O30IRetargeter(
        URDF,
        "right",
        filter_alpha=1.0,
        index_middle_pinch_anchor=anchor,
        index_middle_contact_distance=0.024,
        index_middle_start_distance=0.040,
        index_middle_activation_step=0.25,
    )
    landmarks = np.zeros((21, 3))
    starts = {
        "thumb": np.array([0.0, -0.10, 0.0]),
        "index": np.array([0.0, 0.00, 0.0]),
        "middle": np.array([0.0, 0.02, 0.0]),
        "ring": np.array([0.0, 0.05, 0.0]),
        "pinky": np.array([0.0, 0.08, 0.0]),
    }
    chains = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "pinky": (17, 18, 19, 20),
    }
    for finger, chain in chains.items():
        for step, landmark in enumerate(chain):
            landmarks[landmark] = starts[finger] + [0.03 * step, 0.0, 0.0]
    for expected in (0.25, 0.5, 0.75, 1.0):
        qpos = np.zeros(20)
        requested = retargeter._apply_index_middle_pinch_anchor(qpos, landmarks)
        assert requested == pytest.approx(1.0)
        assert retargeter._index_middle_activation == pytest.approx(expected)
    for name, value in anchor.items():
        assert qpos[retargeter.joint_names.index(name)] == pytest.approx(value)
    achieved = retargeter.robot_landmarks(qpos)
    assert np.linalg.norm(achieved[8] - achieved[12]) < 1e-4
    retargeter.reset()
    assert retargeter._index_middle_activation == 0.0


def test_o30i_retargeting_assigns_distal_motion_to_dip():
    retargeter = O30IRetargeter(
        URDF,
        "right",
        smooth_weight=0.0,
        filter_alpha=1.0,
        max_iterations=200,
    )
    desired = np.zeros(20)
    dip_index = retargeter.joint_names.index("index_dip")
    for angle in np.linspace(0.0, 0.8, 41):
        desired[dip_index] = angle
        landmarks = retargeter.robot_landmarks(desired)
        solved, stats = retargeter.retarget(landmarks)

    assert stats["success"]
    assert solved[dip_index] == pytest.approx(0.8, abs=1e-5)

