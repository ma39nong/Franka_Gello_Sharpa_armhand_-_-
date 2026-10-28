import json
import sys
from pathlib import Path

import numpy as np
import pytest

from pico_bimanual_franka_teleop import hand_landmarks as hl
from pico_bimanual_franka_teleop.hand_retarget import (
    CANONICAL_FINGERS,
    L20Retargeter,
    chain_bend_angle,
    index_middle_pinch_request,
)
from pico_bimanual_franka_teleop.hand_stream import (
    HandQposPacket,
    build_hand_packet,
    encode_hand_packet,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
ASSETS = REPO_ROOT / "assets"


def urdf_for(side: str) -> Path:
    if side == "left":
        return (
            ASSETS
            / "linkerhand_l20_v101"
            / "linkerhand_L20_V10.1_left.urdf"
            / "linkerhand_L20v10.1_left.urdf"
        )
    return ASSETS / "linkerhand_l20" / side / f"linkerhand_l20_{side}.urdf"


from hand_fixtures import synthetic_skeleton  # noqa: E402


def test_canonical_mapping_uses_proximal_not_metacarpal():
    # The MCP knuckle must be *_proximal. Choosing *_metacarpal is the 3.6x
    # scale trap described in hand_landmarks.
    assert hl.CANONICAL_FROM_OPENXR[5] == 7, "index base must be index_proximal"
    assert hl.CANONICAL_FROM_OPENXR[9] == 12, "middle base must be middle_proximal"
    assert hl.CANONICAL_FROM_OPENXR[13] == 17, "ring base must be ring_proximal"
    assert hl.CANONICAL_FROM_OPENXR[17] == 22, "little base must be little_proximal"
    metacarpals = {6, 11, 16, 21}
    assert not metacarpals.intersection(hl.CANONICAL_FROM_OPENXR)


def test_canonical_mapping_covers_wrist_and_thumb():
    assert hl.CANONICAL_FROM_OPENXR[0] == hl.OPENXR_WRIST
    # The OpenXR thumb has no intermediate joint, so its four joints map
    # one-to-one onto the canonical thumb landmarks.
    assert hl.CANONICAL_FROM_OPENXR[1:5] == (2, 3, 4, 5)
    assert len(hl.CANONICAL_FROM_OPENXR) == hl.CANONICAL_LANDMARK_COUNT


def test_chain_bend_angle_is_length_and_pose_invariant():
    straight = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    )
    bent = np.array(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 3.0, 0.0], [0.0, 3.0, 0.0]]
    )
    assert chain_bend_angle(straight) == pytest.approx(0.0)
    assert chain_bend_angle(bent) == pytest.approx(np.pi)
    assert chain_bend_angle(4.2 * bent + np.array([5.0, -2.0, 9.0])) == pytest.approx(
        np.pi
    )


def test_index_middle_pinch_request_rejects_fists_and_thumb_curl():
    points = np.zeros((21, 3), dtype=float)
    chains = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "pinky": (17, 18, 19, 20),
    }
    starts = {
        "thumb": np.array([0.0, -0.10, 0.0]),
        "index": np.array([0.0, 0.00, 0.0]),
        "middle": np.array([0.0, 0.02, 0.0]),
        "ring": np.array([0.0, 0.05, 0.0]),
        "pinky": np.array([0.0, 0.08, 0.0]),
    }
    for finger, chain in chains.items():
        for step, landmark in enumerate(chain):
            points[landmark] = starts[finger] + np.array([0.03 * step, 0.0, 0.0])
    request = index_middle_pinch_request(
        points, contact_distance=0.025, start_distance=0.04
    )
    assert request == pytest.approx(1.0)

    fist = points.copy()
    fist[[6, 7, 8]] = ([0.03, 0.0, 0.0], [0.03, 0.03, 0.0], [0.09, 0.0, 0.0])
    assert index_middle_pinch_request(
        fist, contact_distance=0.025, start_distance=0.04
    ) == pytest.approx(0.0)

    thumb_curled = points.copy()
    thumb_curled[[2, 3, 4]] = (
        [0.03, -0.10, 0.0],
        [0.03, -0.07, 0.0],
        [0.09, -0.10, 0.0],
    )
    assert index_middle_pinch_request(
        thumb_curled, contact_distance=0.025, start_distance=0.04
    ) == pytest.approx(0.0)


def test_validate_skeleton_rejects_bad_input():
    with pytest.raises(ValueError):
        hl.validate_skeleton(np.zeros((21, 3)))
    with pytest.raises(ValueError):
        hl.validate_skeleton(np.zeros((26, 7)))  # all joints at the origin
    bad = synthetic_skeleton()
    bad[7, 0] = np.nan
    with pytest.raises(ValueError):
        hl.validate_skeleton(bad)
    # The binding zero-fills a whole row for a joint it has no data for, which
    # is what leaves the quaternion at zero.
    missing = synthetic_skeleton()
    missing[12, :] = 0.0
    with pytest.raises(ValueError):
        hl.validate_skeleton(missing)


def test_validate_skeleton_accepts_a_joint_at_the_frame_origin():
    # A wrist legitimately sits at the origin in a local frame, so absence must
    # be judged from the quaternion rather than the position.
    skeleton = synthetic_skeleton()
    assert np.allclose(skeleton[hl.OPENXR_WRIST, :3], 0.0)
    hl.validate_skeleton(skeleton)


def test_palm_scale_is_positive_and_translation_invariant():
    marks = hl.to_canonical_landmarks(synthetic_skeleton())
    scale = hl.palm_scale(marks)
    assert scale > 0.0
    moved = marks + np.array([1.5, -2.0, 3.0])
    assert hl.palm_scale(moved) == pytest.approx(scale, rel=1e-12)


def test_chirality_separates_sides_and_ignores_origin():
    right = hl.to_canonical_landmarks(synthetic_skeleton(mirror=False))
    left = hl.to_canonical_landmarks(synthetic_skeleton(mirror=True))
    assert np.sign(hl.chirality(right)) == hl.expected_chirality_sign("right")
    assert np.sign(hl.chirality(left)) == hl.expected_chirality_sign("left")
    shifted = right + np.array([4.0, -1.0, 2.5])
    assert hl.chirality(shifted) == pytest.approx(hl.chirality(right), rel=1e-9)


@pytest.mark.parametrize("side", ["left", "right"])
def test_retargeter_loads_and_reports_21_joints(side):
    with L20Retargeter(urdf_for(side), side) as retargeter:
        # The packet remains 21 joints for compatibility, while Pinocchio solves
        # the 16 physical actuators and applies the five URDF mimic constraints.
        assert retargeter.dof == 21
        assert retargeter.model.nq == 16
        assert np.all(retargeter.lower <= retargeter.upper)
        # A proper rotation, not a reflection.
        assert float(np.linalg.det(retargeter.robot_frame)) == pytest.approx(1.0, abs=1e-9)
        assert retargeter.robot_scale > 0.0


@pytest.mark.parametrize("side", ["left", "right"])
def test_robot_own_landmarks_are_an_exact_fixed_point(side):
    # Feeding the robot its own landmarks must reproduce them exactly and solve
    # to zero. This is what pins the human/robot correspondence: it fails if the
    # scale reference or the centroid origin is inconsistent between the two.
    with L20Retargeter(urdf_for(side), side) as retargeter:
        own = retargeter.robot_landmarks()
        targets = retargeter.target_positions(own)
        expected = np.stack([own[t.landmark_index] for t in retargeter.targets])
        assert np.allclose(targets, expected, atol=1e-12)
        _, stats = retargeter.retarget(own)
        assert stats["loss"] < 1e-6
        assert stats["thumb_position_rmse"] < 1e-6
        assert stats["thumb_tip_error"] < 1e-6
        assert stats["thumb_direction_error_deg"] < 1e-3
        assert abs(stats["thumb_bend_error"]) < 1e-6


@pytest.mark.parametrize("side", ["left", "right"])
def test_thumb_retargeting_enforces_the_urdf_mimic_joint(side):
    # Build a reachable curled-thumb target from the robot itself. The old
    # unconstrained 21-DoF solver could fit this shape with MCP nearly straight
    # and the distal joint bent, even though one G20 actuator drives both. That
    # made the mapper's average substantially under-command physical curl.
    with L20Retargeter(
        urdf_for(side),
        side,
        smooth_weight=0.0,
        filter_alpha=1.0,
        max_iterations=100,
    ) as retargeter:
        desired = np.zeros(retargeter.dof)
        desired[retargeter.joint_names.index("thumb_mcp")] = 0.8
        retargeter._reset_joints(desired)
        curled_thumb = retargeter.robot_landmarks()

        retargeter.reset()
        solved, stats = retargeter.retarget(curled_thumb)
        values = dict(zip(retargeter.joint_names, solved))
        distal = "thumb_ip" if side == "left" else "thumb_dip"

        assert stats["loss"] < 1e-8
        assert values["thumb_mcp"] == pytest.approx(0.8, abs=2e-4)
        assert values[distal] == pytest.approx(
            retargeter._thumb_mimic_multiplier * values["thumb_mcp"], abs=1e-9
        )
        assert stats["thumb_flex_target"] == pytest.approx(0.8, abs=2e-4)
        assert stats["thumb_flex_emitted"] == pytest.approx(0.8, abs=2e-4)
        assert abs(stats["thumb_bend_error"]) < 2e-4


@pytest.mark.parametrize("side", ["left", "right"])
def test_default_filter_reaches_70_percent_of_a_new_pose_in_one_frame(side):
    # Filtering is gesture smoothing, not the hardware safety slew limiter.
    # Alpha 0.7 keeps one-frame jitter suppression while avoiding the roughly
    # 0.2 s response tail of the old alpha 0.35 at the 30 Hz hand update rate.
    with L20Retargeter(
        urdf_for(side),
        side,
        smooth_weight=0.0,
        max_iterations=100,
    ) as retargeter:
        open_hand = retargeter.robot_landmarks()
        retargeter.retarget(open_hand)

        desired = np.zeros(retargeter.dof)
        desired[retargeter.joint_names.index("thumb_mcp")] = 0.8
        retargeter._reset_joints(desired)
        curled_thumb = retargeter.robot_landmarks()
        solved, _ = retargeter.retarget(curled_thumb)

        thumb_mcp = solved[retargeter.joint_names.index("thumb_mcp")]
        assert thumb_mcp == pytest.approx(0.7 * 0.8, abs=2e-4)


@pytest.mark.parametrize("side", ["left", "right"])
def test_retargeting_is_invariant_to_similarity_transforms(side):
    # Global pose and hand size must not affect the solution: that invariance is
    # what lets the optical skeleton drive fingers while the wrist trackers
    # independently drive the arm.
    marks = hl.to_canonical_landmarks(synthetic_skeleton(mirror=(side == "left"), flex=0.5))
    angle = 0.7
    rotation = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    transformed = (marks * 2.7) @ rotation.T + np.array([0.4, -1.2, 3.3])

    with L20Retargeter(urdf_for(side), side) as retargeter:
        baseline, _ = retargeter.retarget(marks)
        retargeter.reset()
        moved, _ = retargeter.retarget(transformed)
    assert np.allclose(baseline, moved, atol=1e-4)


@pytest.mark.parametrize("side", ["left", "right"])
def test_retargeting_respects_urdf_limits(side):
    with L20Retargeter(urdf_for(side), side) as retargeter:
        for flex in (0.0, 0.4, 0.9, 1.3):
            marks = hl.to_canonical_landmarks(
                synthetic_skeleton(mirror=(side == "left"), flex=flex)
            )
            qpos, _ = retargeter.retarget(marks)
            assert np.all(qpos >= retargeter.lower - 1e-9)
            assert np.all(qpos <= retargeter.upper + 1e-9)


def test_retargeter_rejects_malformed_landmarks():
    with L20Retargeter(urdf_for("right"), "right") as retargeter:
        with pytest.raises(ValueError):
            retargeter.target_positions(np.zeros((26, 3)))
        bad = hl.to_canonical_landmarks(synthetic_skeleton())
        bad[3, 1] = np.inf
        with pytest.raises(ValueError):
            retargeter.target_positions(bad)


@pytest.mark.parametrize("side", ["left", "right"])
def test_normalization_matches_the_robots_own_finger_lengths(side):
    # The L20's four fingers are all one length while a human's are not. Without
    # this, an over-long robot finger over-curls to reach a shorter human tip and
    # the fingers foul each other.
    marks = hl.to_canonical_landmarks(
        synthetic_skeleton(mirror=(side == "left"), flex=0.5)
    )
    with L20Retargeter(urdf_for(side), side) as retargeter:
        points = retargeter.target_positions(marks)
        by_landmark = {
            t.landmark_index: p for t, p in zip(retargeter.targets, points)
        }
        for finger, chain in CANONICAL_FINGERS.items():
            length = sum(
                float(np.linalg.norm(by_landmark[chain[i + 1]] - by_landmark[chain[i]]))
                for i in range(3)
            )
            expected = retargeter.robot_finger_lengths[finger]
            # The tip target carries a link-local offset, so compare the three
            # joint-to-joint segments against the same three on the robot.
            assert length == pytest.approx(expected, rel=1e-6), finger


@pytest.mark.parametrize("side", ["left", "right"])
def test_normalization_preserves_curl_shape(side):
    # Only segment lengths may change. Directions carry the gesture.
    marks = hl.to_canonical_landmarks(
        synthetic_skeleton(mirror=(side == "left"), flex=0.8)
    )
    with L20Retargeter(urdf_for(side), side, normalize_finger_length=False) as plain:
        raw = plain.target_positions(marks)
        raw_by = {t.landmark_index: p for t, p in zip(plain.targets, raw)}
    with L20Retargeter(urdf_for(side), side, normalize_finger_length=True) as scaled:
        norm = scaled.target_positions(marks)
        norm_by = {t.landmark_index: p for t, p in zip(scaled.targets, norm)}

    for chain in CANONICAL_FINGERS.values():
        for i in range(3):
            a = raw_by[chain[i + 1]] - raw_by[chain[i]]
            b = norm_by[chain[i + 1]] - norm_by[chain[i]]
            cosine = float(
                np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
            )
            assert cosine == pytest.approx(1.0, abs=1e-9)


def test_mirrored_gesture_gives_consistent_flexion():
    # Left now uses the verified V10.1 mechanism while the unverified right G20
    # fallback retains the old model. Their flexion chains remain comparable,
    # but root locations, roll axes, and roll limits are intentionally different.
    right_marks = hl.to_canonical_landmarks(synthetic_skeleton(flex=0.7))
    mirrored = synthetic_skeleton(flex=0.7)
    mirrored[:, 0] *= -1.0
    left_marks = hl.to_canonical_landmarks(mirrored)

    with L20Retargeter(urdf_for("right"), "right") as right:
        qr = dict(zip(right.joint_names, right.retarget(right_marks)[0]))
    with L20Retargeter(urdf_for("left"), "left") as left:
        ql = dict(zip(left.joint_names, left.retarget(left_marks)[0]))

    for finger in ("index", "middle", "ring", "pinky"):
        for suffix in ("mcp_pitch", "pip", "dip"):
            name = f"{finger}_{suffix}"
            assert qr[name] == pytest.approx(ql[name], abs=0.12), name


def test_hand_packet_round_trip_is_lossless():
    payload = build_hand_packet(
        "pico-hand", 7, 1234.5, "left", ("a", "b", "c"), (0.1, -0.2, 0.3)
    )
    message = json.loads(payload.decode("utf-8"))
    assert message["side"] == "left"
    assert message["sequence"] == 7
    assert message["joint_names"] == ["a", "b", "c"]
    assert message["qpos"] == pytest.approx([0.1, -0.2, 0.3])


@pytest.mark.parametrize(
    "packet",
    [
        HandQposPacket("s", 0, 0.0, "middle", ("a",), (0.0,)),
        HandQposPacket("", 0, 0.0, "left", ("a",), (0.0,)),
        HandQposPacket("s", -1, 0.0, "left", ("a",), (0.0,)),
        HandQposPacket("s", 0, 0.0, "left", ("a", "a"), (0.0, 0.0)),
        HandQposPacket("s", 0, 0.0, "left", ("a", "b"), (0.0,)),
        HandQposPacket("s", 0, float("nan"), "left", ("a",), (0.0,)),
        HandQposPacket("s", 0, 0.0, "left", ("a",), (float("inf"),)),
    ],
)
def test_hand_packet_rejects_invalid_content(packet):
    with pytest.raises(ValueError):
        encode_hand_packet(packet)


def _bridge_core():
    """Load the bridge's mapper without importing ROS.

    The bridge lives in a separate host workspace and must not depend on PICO, so
    it is not importable as a package from here. This test reaches across anyway,
    because the abduction polarity is a claim about the URDF and only this side of
    the repository has the URDF and forward kinematics to check it.
    """
    import importlib.util

    path = (
        REPO_ROOT
        / "ros_ws"
        / "src"
        / "linker_hand_bridge"
        / "linker_hand_bridge"
        / "core.py"
    )
    spec = importlib.util.spec_from_file_location("linker_bridge_core", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["linker_bridge_core"] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("side", ["left", "right"])
def test_abduction_polarity_matches_urdf_geometry(side):
    # Observed once on hardware: commanding the left index abduction slot to 255
    # moves the index finger toward the THUMB side. The vendor's radian tables
    # cannot supply this, since they describe only its internal joint convention.
    # Given that one fact, the URDF fixes the polarity for both hands, and this
    # test re-derives it so replacing the assets cannot silently invert abduction.
    core = _bridge_core()
    with L20Retargeter(urdf_for(side), side) as retargeter:
        retargeter.reset()
        zero = retargeter.robot_landmarks()
        index_base = CANONICAL_FINGERS["index"][0]
        pinky_base = CANONICAL_FINGERS["pinky"][0]
        # Lateral axis pointing from the little finger toward the index, so
        # positive is the thumb side of the hand.
        lateral = zero[index_base] - zero[pinky_base]
        lateral = lateral / np.linalg.norm(lateral)

        joint = retargeter.joint_names.index("index_mcp_roll")
        tip = CANONICAL_FINGERS["index"][-1]
        lower, upper = retargeter.lower[joint], retargeter.upper[joint]

        def tip_offset(value):
            qpos = np.zeros(retargeter.dof)
            qpos[joint] = value
            retargeter._reset_joints(qpos)
            return float(np.dot(retargeter.robot_landmarks()[tip], lateral))

        # Which roll extreme puts the index tip furthest toward the thumb side?
        thumbward_roll = upper if tip_offset(upper) > tip_offset(lower) else lower

        # Which roll extreme does slot 255 actually command, under this polarity?
        names = list(retargeter.joint_names)
        at_upper = core.G20Mapper().map_qpos(
            side, names, [upper if n == "index_mcp_roll" else 0.0 for n in names]
        )[6]
        at_lower = core.G20Mapper().map_qpos(
            side, names, [lower if n == "index_mcp_roll" else 0.0 for n in names]
        )[6]
        roll_at_255 = upper if at_upper > at_lower else lower

        assert roll_at_255 == pytest.approx(thumbward_roll), (
            f"{side}: slot 255 must move the index toward the thumb side, as "
            f"observed on hardware"
        )


@pytest.mark.parametrize("side", ["left", "right"])
def test_calibrated_finger_endpoint_reaches_full_mechanical_curl(side):
    with L20Retargeter(urdf_for(side), side, filter_alpha=1.0) as oracle:
        desired = np.zeros(oracle.dof)
        for name in ("pinky_mcp_pitch", "pinky_pip", "pinky_dip"):
            index = oracle.joint_names.index(name)
            desired[index] = oracle.upper[index]
        oracle._reset_joints(desired)
        curled = oracle.robot_landmarks()
        bend_value = chain_bend_angle(
            curled[list(CANONICAL_FINGERS["pinky"])]
        )

    with L20Retargeter(
        urdf_for(side),
        side,
        filter_alpha=1.0,
        finger_curl_ranges={"pinky": (bend_value - 0.2, bend_value - 0.1)},
    ) as retargeter:
        qpos, _ = retargeter.retarget(curled)
        for name in ("pinky_mcp_pitch", "pinky_pip", "pinky_dip"):
            index = retargeter.joint_names.index(name)
            assert qpos[index] == pytest.approx(retargeter.upper[index], abs=5e-5)


@pytest.mark.parametrize("side", ["left", "right"])
def test_thumb_orientation_activation_releases_gradually(side):
    # A raw activation that flickers around zero re-tilts the near-degenerate
    # open-thumb CMC valley and used to flip the solve between far-apart
    # minima (121 raw jumps >0.3 rad on the 20260730 recording). The rise
    # must stay instant so pinch engages the same tick; only release decays.
    from pico_bimanual_franka_teleop.hand_retarget import (
        THUMB_ACTIVATION_RELEASE,
    )

    with L20Retargeter(urdf_for(side), side) as retargeter:
        open_hand = retargeter.robot_landmarks()
        pinch = open_hand.copy()
        pinch[4] = pinch[8]  # human thumb tip touching the index tip

        _, stats = retargeter.retarget(pinch)
        engaged = stats["thumb_orientation_activation"]
        assert engaged == pytest.approx(1.0)

        _, stats = retargeter.retarget(open_hand)
        assert stats["thumb_orientation_activation"] == pytest.approx(
            THUMB_ACTIVATION_RELEASE * engaged
        )
        for _ in range(60):
            _, stats = retargeter.retarget(open_hand)
        assert stats["thumb_orientation_activation"] < 0.01

        retargeter.reset()
        _, stats = retargeter.retarget(open_hand)
        assert stats["thumb_orientation_activation"] < 0.05


@pytest.mark.parametrize("side", ["left", "right"])
def test_thumb_solution_is_trust_region_bounded_per_call(side):
    # An activation step may move the thumb equilibrium across the whole CMC
    # box; the raw solution must ramp there over multiple calls, never snap
    # in one tick.
    from pico_bimanual_franka_teleop.hand_retarget import THUMB_TRUST_REGION

    with L20Retargeter(urdf_for(side), side) as retargeter:
        thumb_cols = [
            retargeter._active_joint_names.index(name)
            for name in ("thumb_cmc_yaw", "thumb_cmc_roll", "thumb_cmc_pitch")
        ]
        open_hand = retargeter.robot_landmarks()
        pinch = open_hand.copy()
        pinch[4] = pinch[8]

        previous = retargeter.last_qpos[thumb_cols].copy()
        for landmarks in (pinch, pinch, open_hand, pinch, open_hand):
            retargeter.retarget(landmarks)
            current = retargeter.last_qpos[thumb_cols].copy()
            step = np.abs(current - previous)
            assert np.all(step <= THUMB_TRUST_REGION + 1e-9), step
            previous = current
