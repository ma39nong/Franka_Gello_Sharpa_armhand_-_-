"""Common data types shared by hardware interfaces and higher-level workflows."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar, Mapping, TypeAlias

NumberSequence: TypeAlias = tuple[float, ...]
FeatureSpec: TypeAlias = dict[str, object]
RobotAction: TypeAlias = dict[str, object]
RobotObservation: TypeAlias = dict[str, object]
RobotFeedback: TypeAlias = dict[str, object]
TeleoperatorAction: TypeAlias = dict[str, object]
TeleoperatorFeedback: TypeAlias = dict[str, object]
CameraMetadata: TypeAlias = Mapping[str, object]


@dataclass(frozen=True, slots=True)
class JointState:
    """Joint-space position, velocity, and effort values for one hardware device."""

    position: NumberSequence
    velocity: NumberSequence = ()
    effort: NumberSequence = ()


@dataclass(frozen=True, slots=True)
class Pose:
    """Cartesian pose as position and quaternion orientation."""

    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """Single camera frame payload with an optional timestamp."""

    image: object
    timestamp: float | None = None
    metadata: CameraMetadata = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class TrackerPose:
    """Pose payload produced by tracker teleoperators such as HTC Vive trackers."""

    pose: Pose
    linear_velocity: tuple[float, float, float] | None = None
    angular_velocity: tuple[float, float, float] | None = None
    timestamp: float | None = None


@dataclass(frozen=True, slots=True)
class FrankaRobotState:
    """Complete Franka RobotState snapshot exposed by libfranka/pylibfranka."""

    FIELD_NAMES: ClassVar[tuple[str, ...]] = (
        "O_T_EE",
        "O_T_EE_d",
        "F_T_EE",
        "F_T_NE",
        "NE_T_EE",
        "EE_T_K",
        "m_ee",
        "I_ee",
        "F_x_Cee",
        "m_load",
        "I_load",
        "F_x_Cload",
        "m_total",
        "I_total",
        "F_x_Ctotal",
        "elbow",
        "elbow_d",
        "elbow_c",
        "delbow_c",
        "ddelbow_c",
        "tau_J",
        "tau_J_d",
        "dtau_J",
        "q",
        "q_d",
        "dq",
        "dq_d",
        "ddq_d",
        "joint_contact",
        "cartesian_contact",
        "joint_collision",
        "cartesian_collision",
        "tau_ext_hat_filtered",
        "O_F_ext_hat_K",
        "K_F_ext_hat_K",
        "O_dP_EE_d",
        "O_ddP_O",
        "O_T_EE_c",
        "O_dP_EE_c",
        "O_ddP_EE_c",
        "theta",
        "dtheta",
        "accelerometer_top",
        "accelerometer_bottom",
        "current_errors",
        "last_motion_errors",
        "control_command_success_rate",
        "robot_mode",
        "time",
    )

    values: Mapping[str, object] = field(default_factory=dict)

    @classmethod
    def from_object(cls, state: object) -> "FrankaRobotState":
        """Build a state snapshot from a pylibfranka/franky/ROS state-like object."""

        if isinstance(state, Mapping):
            return cls(
                {
                    field_name: state[field_name]
                    for field_name in cls.FIELD_NAMES
                    if field_name in state
                }
            )
        return cls(
            {
                field_name: getattr(state, field_name)
                for field_name in cls.FIELD_NAMES
                if hasattr(state, field_name)
            }
        )

    @classmethod
    def zero(cls) -> "FrankaRobotState":
        """Create a zero-filled state useful for mocks and interface smoke tests."""

        vector_defaults: dict[str, object] = {
            "O_T_EE": (0.0,) * 16,
            "O_T_EE_d": (0.0,) * 16,
            "F_T_EE": (0.0,) * 16,
            "F_T_NE": (0.0,) * 16,
            "NE_T_EE": (0.0,) * 16,
            "EE_T_K": (0.0,) * 16,
            "I_ee": (0.0,) * 9,
            "F_x_Cee": (0.0,) * 3,
            "I_load": (0.0,) * 9,
            "F_x_Cload": (0.0,) * 3,
            "I_total": (0.0,) * 9,
            "F_x_Ctotal": (0.0,) * 3,
            "elbow": (0.0,) * 2,
            "elbow_d": (0.0,) * 2,
            "elbow_c": (0.0,) * 2,
            "delbow_c": (0.0,) * 2,
            "ddelbow_c": (0.0,) * 2,
            "tau_J": (0.0,) * 7,
            "tau_J_d": (0.0,) * 7,
            "dtau_J": (0.0,) * 7,
            "q": (0.0,) * 7,
            "q_d": (0.0,) * 7,
            "dq": (0.0,) * 7,
            "dq_d": (0.0,) * 7,
            "ddq_d": (0.0,) * 7,
            "joint_contact": (0.0,) * 7,
            "cartesian_contact": (0.0,) * 6,
            "joint_collision": (0.0,) * 7,
            "cartesian_collision": (0.0,) * 6,
            "tau_ext_hat_filtered": (0.0,) * 7,
            "O_F_ext_hat_K": (0.0,) * 6,
            "K_F_ext_hat_K": (0.0,) * 6,
            "O_dP_EE_d": (0.0,) * 6,
            "O_ddP_O": (0.0, 0.0, -9.81),
            "O_T_EE_c": (0.0,) * 16,
            "O_dP_EE_c": (0.0,) * 6,
            "O_ddP_EE_c": (0.0,) * 6,
            "theta": (0.0,) * 7,
            "dtheta": (0.0,) * 7,
            "accelerometer_top": ((0.0, 0.0, 0.0),) * 6,
            "accelerometer_bottom": ((0.0, 0.0, 0.0),) * 6,
            "m_ee": 0.0,
            "m_load": 0.0,
            "m_total": 0.0,
            "control_command_success_rate": 0.0,
            "current_errors": None,
            "last_motion_errors": None,
            "robot_mode": "kUserStopped",
            "time": 0.0,
        }
        return cls(vector_defaults)

    def as_dict(self) -> RobotObservation:
        """Return a plain dictionary snapshot suitable for logging or serialization."""

        return dict(self.values)
