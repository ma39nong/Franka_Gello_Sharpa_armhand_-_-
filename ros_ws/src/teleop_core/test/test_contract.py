import pytest

from teleop_core.contract import (
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    DATA_TELEOPERATOR,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
    dataset_arm_joint_name,
    require_exact_joint_names,
    wuji_joint_names,
)


def test_wuji_contract_has_twenty_unique_side_specific_names_and_topics():
    assert len(WUJI_LEFT_JOINT_NAMES) == 20
    assert len(WUJI_RIGHT_JOINT_NAMES) == 20
    assert len(set(WUJI_LEFT_JOINT_NAMES)) == 20
    assert len(set(WUJI_RIGHT_JOINT_NAMES)) == 20
    assert all(name.startswith("l_") for name in WUJI_LEFT_JOINT_NAMES)
    assert all(name.startswith("r_") for name in WUJI_RIGHT_JOINT_NAMES)
    assert WUJI_RIGHT_JOINT_NAMES == tuple(
        name.replace("l_", "r_", 1) for name in WUJI_LEFT_JOINT_NAMES
    )
    assert WUJI_COMMAND_TOPIC.format(side="left") == "/teleop/wuji/left/command"
    assert WUJI_STATE_TOPIC.format(side="right") == "/teleop/wuji/right/joint_states"
    assert WUJI_TELEMETRY_STATUS_TOPIC == "/teleop/wuji/telemetry_status"
    assert wuji_joint_names("left") == WUJI_LEFT_JOINT_NAMES
    assert wuji_joint_names("right") == WUJI_RIGHT_JOINT_NAMES


def test_dataset_arm_names_normalize_fr3v2_without_using_array_index():
    assert DATA_TELEOPERATOR == "gello"
    assert tuple(dataset_arm_joint_name(name) for name in LEFT_COMMAND_JOINT_NAMES) == (
        DATA_LEFT_ARM_JOINT_NAMES
    )
    assert tuple(dataset_arm_joint_name(name) for name in RIGHT_COMMAND_JOINT_NAMES) == (
        DATA_RIGHT_ARM_JOINT_NAMES
    )
    with pytest.raises(ValueError, match="not a validated FR3 command"):
        dataset_arm_joint_name("left_fr3_joint1")


def test_require_exact_joint_names_rejects_missing_duplicate_and_reordered():
    expected = WUJI_LEFT_JOINT_NAMES
    assert require_exact_joint_names(expected, expected, label="left Wuji") == expected
    with pytest.raises(ValueError, match="missing"):
        require_exact_joint_names(expected[1:], expected, label="left Wuji")
    with pytest.raises(ValueError, match="unique"):
        require_exact_joint_names(
            expected[:19] + (expected[0],), expected, label="left Wuji"
        )
    with pytest.raises(ValueError, match="out of order"):
        require_exact_joint_names(
            expected[1:] + expected[:1], expected, label="left Wuji"
        )
    with pytest.raises(ValueError, match="empty"):
        require_exact_joint_names(("",) + expected[1:], expected, label="left Wuji")
