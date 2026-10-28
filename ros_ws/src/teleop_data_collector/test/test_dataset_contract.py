from pathlib import Path

import yaml
from teleop_core.contract import (
    COMMAND_STATUS_TOPIC,
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    DATA_TELEOPERATOR,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONTRACT_PATH = REPO_ROOT / "data_collection/contract/dataset_contract.yaml"


def _contract():
    return yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))


def _flatten(feature):
    return [joint for group in feature["order"] for joint in group["joints"]]


def test_action_contract_has_exact_harvest_54_order():
    contract = _contract()
    feature = contract["features"]["action"]
    assert feature["dimension"] == 54
    assert [group["count"] for group in feature["order"]] == [7, 7, 20, 20]
    assert len(_flatten(feature)) == 54
    assert [group["name"] for group in feature["order"]] == [
        "left_arm.position",
        "right_arm.position",
        "left_hand.command",
        "right_hand.command",
    ]


def test_observation_contract_has_exact_harvest_108_order():
    contract = _contract()
    feature = contract["features"]["observation.state"]
    assert feature["dimension"] == 108
    assert [group["count"] for group in feature["order"]] == [
        7,
        7,
        7,
        7,
        20,
        20,
        20,
        20,
    ]
    assert len(_flatten(feature)) == 108


def test_engagement_contract_is_explicit_and_side_ordered():
    feature = _contract()["features"]["observation.engaged"]
    assert feature == {
        "dimension": 4,
        "dtype": "float32",
        "order": ["left_arm", "right_arm", "left_hand", "right_hand"],
    }


def test_action_uses_only_post_gateway_arm_topic():
    topics = _contract()["topics"]
    assert topics["validated_arm_action"]["topic"] == (
        "/teleop/validated_arm_commands"
    )
    assert all(
        item["topic"] != "/teleop/arm_commands" for item in topics.values()
    )
    assert topics["arm_command_status"]["topic"] == COMMAND_STATUS_TOPIC


def test_joint_mappings_are_explicit_unique_and_side_specific():
    contract = _contract()
    for names in contract["source_joint_names"].values():
        assert len(names) in {7, 20}
        assert len(names) == len(set(names))
    assert contract["source_joint_names"]["validated_left_arm"] != contract[
        "contract_joint_names"
    ]["left_arm"]


def test_three_camera_features_are_fixed():
    assert _contract()["features"]["images"] == {
        "observation.images.cam0": "/cam0/color/image_raw",
        "observation.images.cam1": "/cam1/color/image_raw",
        "observation.images.cam2": "/cam2/color/image_raw",
    }


def test_raw_head_depth_feature_is_fixed():
    assert _contract()["features"]["depths"] == {
        "observation.depths.cam0": "/cam0/depth/image_raw",
    }


def test_yaml_joint_and_action_mapping_matches_teleop_core_source_of_truth():
    contract = _contract()
    source = contract["source_joint_names"]
    stable = contract["contract_joint_names"]
    assert contract["teleoperator"] == DATA_TELEOPERATOR
    assert contract["topics"]["validated_arm_action"]["topic"] == (
        VALIDATED_COMMAND_TOPIC
    )
    assert tuple(source["validated_left_arm"]) == LEFT_COMMAND_JOINT_NAMES
    assert tuple(source["validated_right_arm"]) == RIGHT_COMMAND_JOINT_NAMES
    assert tuple(stable["left_arm"]) == DATA_LEFT_ARM_JOINT_NAMES
    assert tuple(stable["right_arm"]) == DATA_RIGHT_ARM_JOINT_NAMES
    assert tuple(stable["left_hand"]) == WUJI_LEFT_JOINT_NAMES
    assert tuple(stable["right_hand"]) == WUJI_RIGHT_JOINT_NAMES
