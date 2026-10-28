"""Bring up model-specific Linker Hand drivers and the safety bridge.

Either side may be a G20 or an O30i; the current robot mounts an O30i on
both. PICO optical hand tracking remains a G20 source; the O30i source is
the MANUS pipeline. Two O30i sides need two USB-CANFD adapters, named
per side via o30_canfd_device_left / o30_canfd_device_right (indices from
identify_canfd_devices.py).

The G20 bus-to-side mapping is confirmed from each hand's own reported comm ID:
`can0` is the left hand at 0x28. The current O30i uses HOP CAN-FD through the
vendor `libcanbus` USB adapter, device 0/channel 0, with right-hand request ID
0x01 and response ID 0x401. A transparent SocketCAN adapter may use `can1`.

`linker_hand_sdk` with `hand_joint:=G20` is motionless at startup, unlike
`linker_hand_advanced_g20`, which snaps to a default pose at full speed and
torque during construction. The bridge sets the requested operational speed
shortly after startup, because the vendor driver never initializes speed for
G20.

Output is disabled by default. The Compose `hand-control` service explicitly
enables it as an operator-facing hardware action.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from linker_hand_bridge.profiles import create_hand_profile

SIDES = (("left", "can0"), ("right", "can1"))


def _nodes(context):
    # Resolved eagerly so a typo like `sides:=letf` fails the launch instead
    # of silently starting zero hand drivers.
    sides = LaunchConfiguration("sides").perform(context)
    if sides not in ("left", "right", "both"):
        raise ValueError(f"sides must be left, right, or both, got {sides!r}")
    models = {
        side: LaunchConfiguration(f"{side}_model").perform(context).strip().lower()
        for side, _ in SIDES
    }
    if any(not model for model in models.values()):
        raise ValueError("left_model and right_model must be non-empty")
    calibration_verified = (
        LaunchConfiguration("o30_calibration_verified")
        .perform(context)
        .strip()
        .lower()
    )
    if calibration_verified not in {"true", "false"}:
        raise ValueError("o30_calibration_verified must be true or false")

    def calibration_vector(argument):
        text = LaunchConfiguration(argument).perform(context)
        try:
            values = [float(value.strip()) for value in text.split(",")]
        except ValueError as error:
            raise ValueError(f"{argument} must be a comma-separated numeric vector") from error
        if len(values) != 20 or any(
            not math.isfinite(value) or value < 0.0 or value > 255.0
            for value in values
        ):
            raise ValueError(f"{argument} must contain 20 values in [0, 255]")
        return values

    tick_at_lower = calibration_vector("o30_tick_at_lower")
    tick_at_upper = calibration_vector("o30_tick_at_upper")
    for side, _ in SIDES:
        if sides in ("both", side):
            # Fail before any driver process starts if the safety bridge has no
            # contract for the requested hardware.
            create_hand_profile(models[side], side=side)
    selected_o30 = [
        side
        for side, _ in SIDES
        if sides in ("both", side) and models[side] == "o30i"
    ]
    o30_transport = (
        LaunchConfiguration("o30_transport").perform(context).strip().lower()
    )
    if selected_o30 and o30_transport not in {"socketcan", "libcanbus"}:
        raise ValueError("o30_transport must be socketcan or libcanbus")
    # Two O30i sides mean two USB-CANFD adapters, selected by integer index
    # (USB enumeration order). Each side may name its own; the historical
    # shared `o30_canfd_device` remains the fallback so single-sided
    # launches keep working unchanged.
    o30_canfd_devices = {}
    for side, _ in SIDES:
        argument = f"o30_canfd_device_{side}"
        text = LaunchConfiguration(argument).perform(context).strip()
        if not text:
            argument = "o30_canfd_device"
            text = LaunchConfiguration(argument).perform(context).strip()
        try:
            o30_canfd_devices[side] = int(text)
        except ValueError as error:
            raise ValueError(f"{argument} must be an integer") from error
        if side in selected_o30 and o30_canfd_devices[side] < 0:
            raise ValueError(f"{argument} must not be negative")
    if (
        len(selected_o30) == 2
        and o30_canfd_devices["left"] == o30_canfd_devices["right"]
    ):
        raise ValueError(
            "left and right O30i cannot share one CANFD adapter: set "
            "o30_canfd_device_left and o30_canfd_device_right to the two "
            "indices reported by identify_canfd_devices.py"
        )
    try:
        channel_index = int(
            LaunchConfiguration("o30_canfd_channel").perform(context)
        )
    except ValueError as error:
        raise ValueError("o30_canfd_channel must be an integer") from error
    if selected_o30 and channel_index < 0:
        raise ValueError("o30_canfd_channel must not be negative")
    if selected_o30 and calibration_verified != "true":
        raise ValueError(
            "real O30i output requires o30_calibration_verified:=true"
        )
    if selected_o30:
        if any(
            lower == upper
            for lower, upper in zip(tick_at_lower, tick_at_upper, strict=True)
        ):
            raise ValueError(
                "O30i lower/upper calibration ticks must differ for every joint"
            )
        for argument in ("o30_command_timeout", "o30_state_timeout"):
            try:
                timeout = float(LaunchConfiguration(argument).perform(context))
            except ValueError as error:
                raise ValueError(f"{argument} must be numeric") from error
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError(f"{argument} must be positive and finite")

    drivers = []
    for side, channel in SIDES:
        if sides not in ("both", side):
            continue
        if models[side] == "o30i":
            drivers.append(
                Node(
                    package="linker_hand_ros2_sdk",
                    executable="linker_hand_o30i",
                    name=f"linker_hand_{side}_o30i",
                    output="screen",
                    parameters=[
                        {
                            "hand_type": side,
                            "transport": LaunchConfiguration(
                                "o30_transport"
                            ),
                            "can": channel,
                            "frame_id": 1 if side == "right" else 2,
                            "canfd_device": o30_canfd_devices[side],
                            "canfd_channel": LaunchConfiguration(
                                "o30_canfd_channel"
                            ),
                            "libcanbus_path": LaunchConfiguration(
                                "o30_libcanbus_path"
                            ),
                            "output_enabled": True,
                            "calibration_verified": True,
                            "tick_at_lower": tick_at_lower,
                            "tick_at_upper": tick_at_upper,
                            "initial_velocity": ParameterValue(
                                LaunchConfiguration("o30_initial_velocity"),
                                value_type=int,
                            ),
                            "initial_stall_current": ParameterValue(
                                LaunchConfiguration("o30_initial_stall_current"),
                                value_type=int,
                            ),
                            "command_timeout": LaunchConfiguration(
                                "o30_command_timeout"
                            ),
                            "state_timeout": LaunchConfiguration(
                                "o30_state_timeout"
                            ),
                        }
                    ],
                )
            )
            continue
        drivers.append(
            Node(
                package="linker_hand_ros2_sdk",
                executable="linker_hand_sdk",
                # Both driver instances hardcode the same node name, so rename
                # them or the second one collides with the first.
                name=f"linker_hand_{side}",
                output="screen",
                remappings=[
                    (
                        "/cb_hand_setting_cmd",
                        f"/cb_{side}_hand_setting_cmd",
                    )
                ],
                parameters=[
                    {
                        "hand_type": side,
                        "hand_joint": models[side].upper(),
                        "can": channel,
                        "is_touch": LaunchConfiguration("is_touch"),
                    }
                ],
            )
        )
    bridge = Node(
        package="linker_hand_bridge",
        executable="bridge",
        name="linker_hand_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "sides": LaunchConfiguration("sides"),
                "left_model": models["left"],
                "right_model": models["right"],
                "max_command_rate": LaunchConfiguration("max_command_rate"),
                "initial_speed": LaunchConfiguration("initial_speed"),
                "initial_torque": LaunchConfiguration("initial_torque"),
                "initial_thumb_torque": LaunchConfiguration(
                    "initial_thumb_torque"
                ),
                "abduction_invert": LaunchConfiguration("abduction_invert"),
            }
        ],
    )
    return [*drivers, bridge]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("port", default_value="5570"),
        DeclareLaunchArgument(
            "sides",
            default_value="both",
            description="left, right, or both",
        ),
        DeclareLaunchArgument(
            "left_model",
            default_value="g20",
            description="Registered bridge and vendor-driver model for the left hand.",
        ),
        DeclareLaunchArgument(
            "right_model",
            default_value="g20",
            description="Registered bridge and vendor-driver model for the right hand.",
        ),
        DeclareLaunchArgument(
            "o30_calibration_verified",
            default_value="false",
            description=(
                "Required true before real O30i output. Confirms that the "
                "configured radians-to-tick mapping was reviewed."
            ),
        ),
        DeclareLaunchArgument(
            "o30_transport",
            default_value="libcanbus",
            description=(
                "O30i adapter transport: libcanbus for the connected metal "
                "USB-CANFD adapter, or socketcan for a native CAN interface."
            ),
        ),
        DeclareLaunchArgument(
            "o30_canfd_device",
            default_value="0",
            description=(
                "Fallback CANFD adapter index for any O30i side that does not "
                "name its own via o30_canfd_device_<side>."
            ),
        ),
        DeclareLaunchArgument(
            "o30_canfd_device_left",
            default_value="",
            description=(
                "CANFD adapter index for the LEFT O30i; empty falls back to "
                "o30_canfd_device."
            ),
        ),
        DeclareLaunchArgument(
            "o30_canfd_device_right",
            default_value="",
            description=(
                "CANFD adapter index for the RIGHT O30i; empty falls back to "
                "o30_canfd_device."
            ),
        ),
        DeclareLaunchArgument("o30_canfd_channel", default_value="0"),
        DeclareLaunchArgument(
            "o30_libcanbus_path",
            default_value="",
            description=(
                "Optional libcanbus.so override; empty uses the packaged runtime."
            ),
        ),
        DeclareLaunchArgument(
            "o30_tick_at_lower",
            default_value=",".join(["0"] * 20),
            description="Twenty O30i ticks corresponding to the URDF lower limits.",
        ),
        DeclareLaunchArgument(
            "o30_tick_at_upper",
            default_value=",".join(["255"] * 20),
            description="Twenty O30i ticks corresponding to the URDF upper limits.",
        ),
        DeclareLaunchArgument(
            "o30_initial_velocity",
            default_value="-1",
            description=(
                "O30i joint velocity 0..255 applied at startup; -1 leaves the "
                "device's power-up value. Unlike the G20, this hand was never "
                "configured at all -- the driver logs what it booted with "
                "either way."
            ),
        ),
        DeclareLaunchArgument(
            "o30_initial_stall_current",
            default_value="-1",
            description=(
                "O30i holding current after stall, 0..255, applied at startup; "
                "-1 leaves the device's power-up value. This is grip force and "
                "motor heating: raise it deliberately and in steps."
            ),
        ),
        DeclareLaunchArgument(
            "o30_command_timeout",
            default_value="0.25",
            description=(
                "Seconds without a command before the O30i driver disables all "
                "joints terminally."
            ),
        ),
        DeclareLaunchArgument(
            "o30_state_timeout",
            default_value="0.5",
            description=(
                "Seconds without valid O30i position feedback before the driver "
                "disables all joints terminally."
            ),
        ),
        DeclareLaunchArgument(
            "max_command_rate",
            default_value="1500.0",
            description=(
                "Requested per-second slew limit in each profile's canonical "
                "units. Profiles impose their own upper cap: G20 uses ticks and "
                "O30i uses URDF radians."
            ),
        ),
        DeclareLaunchArgument(
            "initial_speed",
            default_value="255",
            description=(
                "Joint speed 0..255 requested at startup; 0 disables. This also "
                "raises the force the fingers apply before the firmware backs off, "
                "so lower it, for example to 120, when grasping something fragile."
            ),
        ),
        DeclareLaunchArgument(
            "initial_torque",
            default_value="200",
            description=(
                "Maximum torque 0..255 for the four non-thumb fingers at "
                "startup; 0 disables (must match initial_thumb_torque's "
                "enablement). 200 is the vendor's own convention for models "
                "it does initialize. The vendor driver never initializes G20 "
                "torque on its own. Lower for fragile objects."
            ),
        ),
        DeclareLaunchArgument(
            "initial_thumb_torque",
            default_value="250",
            description=(
                "Maximum thumb torque 0..255 at startup; raised above the "
                "other fingers because the operator's button-press needs the "
                "thumb pad force (2026-07-26)."
            ),
        ),
        DeclareLaunchArgument(
            "abduction_invert",
            default_value="false",
            description=(
                "Flip the abduction polarity away from the vendor default. Set "
                "it if the operator spreads their fingers and the hand closes its "
                "finger gaps instead of opening them."
            ),
        ),
        DeclareLaunchArgument("is_touch", default_value="false"),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_nodes)])
