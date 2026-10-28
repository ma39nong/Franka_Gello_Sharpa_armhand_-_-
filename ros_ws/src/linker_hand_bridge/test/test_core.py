import json

import pytest

from linker_hand_bridge.core import (
    ABDUCTION_INVERTED,
    COMMAND_SLOTS,
    G20_JOINT_NAMES,
    FINGER_LIMITS,
    LEFT_V101_FINGER_LIMITS,
    LEFT_V101_THUMB_LIMITS,
    THUMB_LIMITS,
    CommandLimiter,
    G20Mapper,
    QposPacket,
    decode_qpos_packet,
    validate_hand_state,
)
from linker_hand_bridge.profiles import O30I_JOINT_NAMES, create_hand_profile

L20_JOINTS = (
    "pinky_mcp_roll",
    "pinky_mcp_pitch",
    "pinky_pip",
    "pinky_dip",
    "ring_mcp_roll",
    "ring_mcp_pitch",
    "ring_pip",
    "ring_dip",
    "middle_mcp_roll",
    "middle_mcp_pitch",
    "middle_pip",
    "middle_dip",
    "index_mcp_roll",
    "index_mcp_pitch",
    "index_pip",
    "index_dip",
    "thumb_cmc_yaw",
    "thumb_cmc_roll",
    "thumb_cmc_pitch",
    "thumb_mcp",
)


def joints_for(side: str) -> tuple[str, ...]:
    return L20_JOINTS + (("thumb_ip",) if side == "left" else ("thumb_dip",))


def packet_bytes(default_side: str, qpos=None, **overrides) -> bytes:
    names = joints_for(default_side)
    payload = {
        "version": 1,
        "stream_id": "test",
        "sequence": 1,
        "timestamp": 1.0,
        "side": default_side,
        "joint_names": list(names),
        "qpos": list(qpos if qpos is not None else [0.0] * len(names)),
    }
    payload.update(overrides)
    return json.dumps(payload).encode("utf-8")


def test_decode_round_trip():
    packet = decode_qpos_packet(packet_bytes("left"))
    assert packet.side == "left"
    assert packet.sequence == 1
    assert len(packet.qpos) == 21
    assert packet.model is None


def test_decode_accepts_a_model_tag():
    packet = decode_qpos_packet(packet_bytes("right", model="g20"))
    assert packet.model == "g20"


@pytest.mark.parametrize(
    "overrides",
    [
        {"version": 2},
        {"side": "middle"},
        {"sequence": -1},
        {"timestamp": "soon"},
        {"joint_names": ["a", "a"], "qpos": [0.0, 0.0]},
        {"qpos": [0.0]},
    ],
)
def test_decode_rejects_invalid_packets(overrides):
    with pytest.raises(ValueError):
        decode_qpos_packet(packet_bytes("left", **overrides))


def test_decode_rejects_non_json_and_oversize():
    with pytest.raises(ValueError):
        decode_qpos_packet(b"not json")
    with pytest.raises(ValueError):
        decode_qpos_packet(b"x" * 20_000)


@pytest.mark.parametrize("side", ["left", "right"])
def test_mapping_produces_20_valid_slots(side):
    mapper = G20Mapper()
    command = mapper.map_packet(decode_qpos_packet(packet_bytes(side)))
    assert len(command) == COMMAND_SLOTS == len(G20_JOINT_NAMES)
    assert all(0.0 <= value <= 255.0 for value in command)
    assert all(float(value).is_integer() for value in command)
    # Reserved slots must always be zero, matching what the hand reports back.
    assert command[11:15] == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize("side", ["left", "right"])
def test_zero_flexion_maps_to_an_open_hand(side):
    # Lower vendor values mean more flexion, so a zero-radian URDF pose must
    # produce near-255 on every flexion slot.
    mapper = G20Mapper()
    command = mapper.map_packet(decode_qpos_packet(packet_bytes(side)))
    flexion_slots = [0, 1, 2, 3, 4, 15, 16, 17, 18, 19]
    for slot in flexion_slots:
        assert command[slot] == 255.0, f"slot {slot} ({G20_JOINT_NAMES[slot]})"


@pytest.mark.parametrize("side", ["left", "right"])
def test_full_flexion_maps_to_a_closed_hand(side):
    mapper = G20Mapper()
    names = joints_for(side)
    # Drive every flexion joint to the selected model's upper limit.
    finger_limits = LEFT_V101_FINGER_LIMITS if side == "left" else FINGER_LIMITS
    thumb_limits = LEFT_V101_THUMB_LIMITS if side == "left" else THUMB_LIMITS
    upper = {
        suffix: limits[1] for suffix, limits in finger_limits.items()
    }
    upper.update(
        {
            "thumb_cmc_pitch": thumb_limits["thumb_cmc_pitch"][1],
            "thumb_mcp": thumb_limits["thumb_mcp"][1],
            "thumb_ip": thumb_limits["thumb_tip"][1],
            "thumb_dip": thumb_limits["thumb_tip"][1],
        }
    )
    qpos = []
    for name in names:
        if name.endswith("mcp_roll"):
            qpos.append(0.0)
        elif name in ("thumb_cmc_yaw", "thumb_cmc_roll"):
            qpos.append(0.0)
        else:
            suffix = name.split("_", 1)[1] if not name.startswith("thumb") else name
            qpos.append(upper.get(suffix, upper.get(name, 0.0)))
    command = mapper.map_qpos(side, names, qpos)
    for slot in (0, 1, 2, 3, 4, 15, 16, 17, 18, 19):
        assert command[slot] == 0.0, f"slot {slot} ({G20_JOINT_NAMES[slot]})"


@pytest.mark.parametrize("side", ["left", "right"])
def test_all_four_abduction_slots_share_one_polarity(side):
    # Each vendor slot sets its own finger's lateral angle, with the same positive
    # direction on all four, so one sign must apply to all of them. Per-finger
    # signs would cancel the opposition between fingers that produces spread.
    mapper = G20Mapper()
    names = joints_for(side)
    for roll in (-0.17, 0.0, 0.17):
        values = [roll if n.endswith("mcp_roll") else 0.0 for n in names]
        command = mapper.map_qpos(side, names, values)
        assert len(set(command[6:10])) == 1, (side, roll, command[6:10])


def test_abduction_polarity_tracks_each_selected_urdf():
    # V10.1 reversed the left roll axis. Both currently selected URDFs therefore
    # need inversion even though they are not a mutually mirrored model pair.
    assert ABDUCTION_INVERTED == {"left": True, "right": True}


@pytest.mark.parametrize("side", ["left", "right"])
def test_abduction_invert_flips_every_slot(side):
    names = joints_for(side)
    values = [0.17 if n.endswith("mcp_roll") else 0.0 for n in names]
    plain = G20Mapper(abduction_invert=False).map_qpos(side, names, values)
    flipped = G20Mapper(abduction_invert=True).map_qpos(side, names, values)
    for slot in range(6, 10):
        assert plain[slot] + flipped[slot] == pytest.approx(255.0, abs=1.0), slot


@pytest.mark.parametrize("side", ["left", "right"])
def test_opposed_rolls_drive_slots_apart(side):
    # Spread reaches the hardware as fingers at opposite ends of the range. This is
    # the invariant the earlier per-finger signs destroyed.
    mapper = G20Mapper()
    names = joints_for(side)
    limits = LEFT_V101_FINGER_LIMITS if side == "left" else FINGER_LIMITS
    roll_limit = limits["mcp_roll"][1]
    values = []
    for name in names:
        if name.endswith("mcp_roll"):
            finger = name.split("_")[0]
            values.append(
                roll_limit if finger in ("ring", "pinky") else -roll_limit
            )
        else:
            values.append(0.0)
    command = mapper.map_qpos(side, names, values)
    assert abs(command[6] - command[9]) == pytest.approx(255.0, abs=1.0)


@pytest.mark.parametrize("side", ["left", "right"])
def test_mapping_requires_every_joint(side):
    mapper = G20Mapper()
    names = joints_for(side)[:-1]
    with pytest.raises(ValueError, match="missing required L20 joints"):
        mapper.map_qpos(side, names, [0.0] * len(names))


@pytest.mark.parametrize("side", ["left", "right"])
def test_home_is_an_open_hand(side):
    command = G20Mapper().home(side)
    assert len(command) == COMMAND_SLOTS
    for slot in (0, 1, 2, 3, 4, 15, 16, 17, 18, 19):
        assert command[slot] == 255.0
    assert command[11:15] == (0.0, 0.0, 0.0, 0.0)


def test_validate_hand_state_rejects_the_drivers_first_message():
    # The vendor driver publishes its [-1] * 10 initializer before the first
    # hardware poll. Accepting it would seed the limiter from a fiction, and -1
    # clamped to 0 would look like a fully closed hand.
    assert validate_hand_state([-1.0] * 10) is None
    assert validate_hand_state([-1.0] * 20) is None
    assert validate_hand_state([float("nan")] * 20) is None
    assert validate_hand_state([300.0] * 20) is None
    good = [12.0] * 20
    assert validate_hand_state(good) == tuple(good)


def test_g20_profile_owns_device_specific_contract():
    profile = create_hand_profile("G20")
    assert profile.model == "g20"
    assert profile.command_slots == 20
    assert profile.command_joint_names == G20_JOINT_NAMES
    assert profile.fixed_values == {11: 0.0, 12: 0.0, 13: 0.0, 14: 0.0}
    assert profile.max_publish_rate == 30.0
    settings = profile.startup_settings(255, 200, 250)
    assert [setting.command for setting in settings] == [
        "set_speed",
        "set_max_torque_limits",
    ]


def test_profile_rejects_unknown_and_mismatched_models():
    with pytest.raises(ValueError, match="unsupported hand model"):
        create_hand_profile("not-a-hand")
    profile = create_hand_profile("g20")
    packet = decode_qpos_packet(packet_bytes("left", model="o30"))
    with pytest.raises(ValueError, match="does not match"):
        profile.map_packet(packet)


def test_o30i_profile_preserves_canonical_urdf_radians():
    profile = create_hand_profile("o30i", side="right")
    assert profile.model == "o30i"
    assert profile.command_joint_names == O30I_JOINT_NAMES
    assert profile.max_slew_rate == 12.0
    qpos = tuple(
        (lower + upper) / 2.0
        for lower, upper in zip(
            profile.lower_bounds, profile.upper_bounds, strict=True
        )
    )
    packet = QposPacket(
        "test",
        0,
        1.0,
        "right",
        "o30i",
        tuple(reversed(O30I_JOINT_NAMES)),
        tuple(reversed(qpos)),
    )
    assert profile.map_packet(packet) == pytest.approx(qpos)
    assert profile.startup_settings(255, 200, 250) == ()
    # Both sides serve the same contract since the left mount became an O30i
    # (2026-08-04); the vendor's left URDF carries identical names and limits.
    left = create_hand_profile("o30i", side="left")
    assert left.lower_bounds == profile.lower_bounds
    assert left.upper_bounds == profile.upper_bounds
    with pytest.raises(ValueError, match="unknown side"):
        create_hand_profile("o30i", side="up")


def test_limiter_supports_profile_defined_dimensions_and_bounds():
    limiter = CommandLimiter(
        [0.0, -1.0, 5.0],
        10.0,
        lower_bounds=[-2.0, -2.0, 0.0],
        upper_bounds=[2.0, 2.0, 6.0],
        fixed_values={2: 5.0},
    )
    limiter.step([2.0, 2.0, 6.0], 0.0)
    assert limiter.step([20.0, 20.0, 20.0], 0.1) == pytest.approx(
        (1.0, 0.0, 5.0)
    )


def test_limiter_first_step_holds_and_then_slews():
    limiter = CommandLimiter([0.0] * COMMAND_SLOTS, 100.0)
    target = [255.0] * COMMAND_SLOTS
    # The first call only establishes the clock.
    assert limiter.step(target, 0.0) == tuple([0.0] * COMMAND_SLOTS)
    # 0.1 s at 100 units/s is 10 units. Note dt is clamped to 0.25 s on every
    # step, so intervals longer than that do not scale linearly.
    stepped = limiter.step(target, 0.1)
    assert stepped[0] == pytest.approx(10.0)
    assert stepped[11:15] == (0.0, 0.0, 0.0, 0.0)


def test_limiter_dt_clamp_applies_to_every_step():
    limiter = CommandLimiter([0.0] * COMMAND_SLOTS, 100.0)
    limiter.step([255.0] * COMMAND_SLOTS, 0.0)
    # A 1 s interval is clamped to 0.25 s, giving 25 rather than 100 units.
    assert limiter.step([255.0] * COMMAND_SLOTS, 1.0)[0] == pytest.approx(25.0)


def test_limiter_clamps_a_long_stall_into_one_bounded_step():
    limiter = CommandLimiter([0.0] * COMMAND_SLOTS, 100.0)
    limiter.step([255.0] * COMMAND_SLOTS, 0.0)
    # A 10 s gap must not become a 1000-unit jump; dt is clamped to 0.25 s.
    stepped = limiter.step([255.0] * COMMAND_SLOTS, 10.0)
    assert stepped[0] == pytest.approx(25.0)


def test_limiter_stays_within_range_and_validates_lengths():
    limiter = CommandLimiter([0.0] * COMMAND_SLOTS, 1e6)
    limiter.step([255.0] * COMMAND_SLOTS, 0.0)
    stepped = limiter.step([1e9] * COMMAND_SLOTS, 1.0)
    assert all(0.0 <= value <= 255.0 for value in stepped)
    with pytest.raises(ValueError):
        CommandLimiter([0.0] * 10, 100.0)
    with pytest.raises(ValueError):
        CommandLimiter([0.0] * COMMAND_SLOTS, 0.0)
    with pytest.raises(ValueError):
        limiter.step([0.0] * 5, 2.0)
