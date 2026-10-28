SIDES = ("left", "right")

LEFT_COMMAND_JOINT_NAMES = tuple(
    f"left_fr3v2_joint{index}" for index in range(1, 8)
)
RIGHT_COMMAND_JOINT_NAMES = tuple(
    f"right_fr3v2_joint{index}" for index in range(1, 8)
)
COMMAND_JOINT_NAMES = LEFT_COMMAND_JOINT_NAMES + RIGHT_COMMAND_JOINT_NAMES
CONTROLLER_JOINT_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))

SOURCE_COMMAND_TOPIC = "/teleop/arm_commands"
COMMAND_STATUS_TOPIC = "/teleop/arm_command_status"
VALIDATED_COMMAND_TOPIC = "/teleop/validated_arm_commands"
ARM_COMMAND_TOPIC = "/target_robot/joint_commands"
ARM_STATE_TOPIC = "/{side}/franka/joint_states"
EXTERNAL_TORQUES_TOPIC = (
    "/{side}/franka_robot_state_broadcaster/external_joint_torques"
)
CONTROLLER_COMMAND_TOPIC = "/{side}/gello/joint_states"
RESET_ACTIVE_TOPIC = "/reset_to_initial_pose/active"

# Read-only data-collection contract. These names never appear on a hardware
# command publisher; they normalize controller/GELLO naming for Harvest-format
# datasets and validate the Wuji firmware command order exported by Operator.
DATA_TELEOPERATOR = "gello"
DATA_TELEOPERATORS = (DATA_TELEOPERATOR,)
DATA_LEFT_ARM_JOINT_NAMES = tuple(
    f"left_fr3_joint{index}" for index in range(1, 8)
)
DATA_RIGHT_ARM_JOINT_NAMES = tuple(
    f"right_fr3_joint{index}" for index in range(1, 8)
)
WUJI_LEFT_JOINT_NAMES = (
    "l_thumb_cmc_flex",
    "l_thumb_cmc_abd",
    "l_thumb_mcp",
    "l_thumb_ip",
    "l_index_finger_mcp_flex",
    "l_index_finger_mcp_abd",
    "l_index_finger_pip",
    "l_index_finger_dip",
    "l_middle_finger_mcp_flex",
    "l_middle_finger_mcp_abd",
    "l_middle_finger_pip",
    "l_middle_finger_dip",
    "l_ring_finger_mcp_flex",
    "l_ring_finger_mcp_abd",
    "l_ring_finger_pip",
    "l_ring_finger_dip",
    "l_pinky_mcp_flex",
    "l_pinky_mcp_abd",
    "l_pinky_pip",
    "l_pinky_dip",
)
WUJI_RIGHT_JOINT_NAMES = tuple(
    name.replace("l_", "r_", 1) for name in WUJI_LEFT_JOINT_NAMES
)
WUJI_COMMAND_TOPIC = "/teleop/wuji/{side}/command"
WUJI_STATE_TOPIC = "/teleop/wuji/{side}/joint_states"
WUJI_TELEMETRY_STATUS_TOPIC = "/teleop/wuji/telemetry_status"
CAMERA_COLOR_TOPIC = "/cam{index}/color/image_raw"


def command_names(active_sides):
    unknown = set(active_sides).difference(SIDES)
    if unknown or len(active_sides) != len(set(active_sides)):
        raise ValueError("Active sides must be unique values from left and right.")
    return tuple(
        name
        for side in active_sides
        for name in (
            LEFT_COMMAND_JOINT_NAMES if side == "left" else RIGHT_COMMAND_JOINT_NAMES
        )
    )


def dataset_arm_joint_name(command_name):
    """Normalize left/right_fr3v2_joint* onto the stable dataset names."""
    name = str(command_name)
    if name in LEFT_COMMAND_JOINT_NAMES:
        return DATA_LEFT_ARM_JOINT_NAMES[LEFT_COMMAND_JOINT_NAMES.index(name)]
    if name in RIGHT_COMMAND_JOINT_NAMES:
        return DATA_RIGHT_ARM_JOINT_NAMES[RIGHT_COMMAND_JOINT_NAMES.index(name)]
    raise ValueError(f"not a validated FR3 command joint name: {command_name!r}")


def wuji_joint_names(side):
    if side == "left":
        return WUJI_LEFT_JOINT_NAMES
    if side == "right":
        return WUJI_RIGHT_JOINT_NAMES
    raise ValueError(f"Wuji side must be left or right, not {side!r}")


def require_exact_joint_names(actual, expected, *, label):
    """Reject empty, duplicate, missing, or reordered joint names."""
    names = tuple(str(name) for name in actual)
    expected = tuple(expected)
    if any(not name for name in names):
        raise ValueError(f"{label} contains an empty joint name")
    if len(names) != len(set(names)):
        raise ValueError(f"{label} joint names must be unique")
    missing = [name for name in expected if name not in names]
    if missing:
        raise ValueError(f"{label} is missing joint names: {missing}")
    extra = [name for name in names if name not in expected]
    if extra:
        raise ValueError(f"{label} has unexpected joint names: {extra}")
    if names != expected:
        raise ValueError(f"{label} joint names are out of order")
    return names
