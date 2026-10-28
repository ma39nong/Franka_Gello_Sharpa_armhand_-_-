"""Shared helpers for Franka drivers implementing the core FrankaRobotInterface."""

from __future__ import annotations

import math
from abc import ABC
from collections.abc import Iterable, Mapping, Sequence
from typing import SupportsFloat, SupportsIndex, cast

from barmate.core.robot import FrankaRobotInterface
from barmate.core.types import (
    FeatureSpec,
    JointState,
    Pose,
    RobotAction,
    RobotObservation,
)


FRANKA_VECTOR_FEATURES: FeatureSpec = {
    "O_T_EE": (16,),
    "O_T_EE_d": (16,),
    "F_T_EE": (16,),
    "F_T_NE": (16,),
    "NE_T_EE": (16,),
    "EE_T_K": (16,),
    "I_ee": (9,),
    "F_x_Cee": (3,),
    "I_load": (9,),
    "F_x_Cload": (3,),
    "I_total": (9,),
    "F_x_Ctotal": (3,),
    "elbow": (2,),
    "elbow_d": (2,),
    "elbow_c": (2,),
    "delbow_c": (2,),
    "ddelbow_c": (2,),
    "tau_J": (7,),
    "tau_J_d": (7,),
    "dtau_J": (7,),
    "q": (7,),
    "q_d": (7,),
    "dq": (7,),
    "dq_d": (7,),
    "ddq_d": (7,),
    "joint_contact": (7,),
    "cartesian_contact": (6,),
    "joint_collision": (7,),
    "cartesian_collision": (6,),
    "tau_ext_hat_filtered": (7,),
    "O_F_ext_hat_K": (6,),
    "K_F_ext_hat_K": (6,),
    "O_dP_EE_d": (6,),
    "O_ddP_O": (3,),
    "O_T_EE_c": (16,),
    "O_dP_EE_c": (6,),
    "O_ddP_EE_c": (6,),
    "theta": (7,),
    "dtheta": (7,),
    "accelerometer_top": (6, 3),
    "accelerometer_bottom": (6, 3),
}

FRANKA_SCALAR_FEATURES: FeatureSpec = {
    "m_ee": float,
    "m_load": float,
    "m_total": float,
    "control_command_success_rate": float,
    "current_errors": object,
    "last_motion_errors": object,
    "robot_mode": object,
    "time": float,
}

FRANKA_STATE_FEATURES: FeatureSpec = {
    **FRANKA_VECTOR_FEATURES,
    **FRANKA_SCALAR_FEATURES,
}

LEGACY_TRAJECTORY_OBSERVATION_FIELDS = (
    "obs_joint",
    "obs_joint_velocity",
    "control_command_success_rate",
)

Floatable = str | bytes | SupportsFloat | SupportsIndex


def _as_float_tuple(value: object, *, width: int | None = None) -> tuple[float, ...]:
    if value is None:
        return (0.0,) * (width or 0)
    if isinstance(value, Sequence) and not isinstance(value, str):
        values = tuple(float(cast(Floatable, item)) for item in value)
    elif isinstance(value, Iterable) and not isinstance(value, (str, bytes, Mapping)):
        values = tuple(float(cast(Floatable, item)) for item in value)
    else:
        values = (float(cast(Floatable, value)),)
    if width is None:
        return values
    if len(values) >= width:
        return values[:width]
    return values + (0.0,) * (width - len(values))


def _affine_to_pose(affine_like: object) -> Pose | None:
    translation = cast(object | None, getattr(affine_like, "translation", None))
    quaternion = cast(object | None, getattr(affine_like, "quaternion", None))
    if translation is None or quaternion is None:
        return None

    position = _as_float_tuple(translation, width=3)
    orientation = _as_float_tuple(quaternion, width=4)

    return Pose(
        position=(position[0], position[1], position[2]),
        orientation_xyzw=(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        ),
    )


def _matrix_to_pose(matrix_like: object) -> Pose | None:
    affine_pose = _affine_to_pose(matrix_like)
    if affine_pose is not None:
        return affine_pose

    values = _as_float_tuple(matrix_like)
    if len(values) == 7:
        return Pose(
            position=(values[0], values[1], values[2]),
            orientation_xyzw=(values[3], values[4], values[5], values[6]),
        )
    if len(values) != 16:
        return None

    matrix = [
        tuple(values[column * 4 + row] for column in range(4)) for row in range(4)
    ]
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2][1] - matrix[1][2]) / scale
        qy = (matrix[0][2] - matrix[2][0]) / scale
        qz = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        qw = (matrix[2][1] - matrix[1][2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0][1] + matrix[1][0]) / scale
        qz = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        qw = (matrix[0][2] - matrix[2][0]) / scale
        qx = (matrix[0][1] + matrix[1][0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        qw = (matrix[1][0] - matrix[0][1]) / scale
        qx = (matrix[0][2] + matrix[2][0]) / scale
        qy = (matrix[1][2] + matrix[2][1]) / scale
        qz = 0.25 * scale

    return Pose(
        position=(matrix[0][3], matrix[1][3], matrix[2][3]),
        orientation_xyzw=(qx, qy, qz, qw),
    )


class FrankaDriverBase(FrankaRobotInterface, ABC):
    """Small base class for Franka drivers that expose complete RobotState snapshots."""

    def __init__(self, name: str = "franka") -> None:
        self.name: str = name
        self._connected: bool = False

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def observation_features(self) -> FeatureSpec:
        return {
            "obs_joint": (7,),
            "obs_joint_velocity": (7,),
            "obs_joint_effort": (7,),
            "obs_tcp_pose": (7,),
            "control_command_success_rate": float,
            **FRANKA_STATE_FEATURES,
        }

    @property
    def action_features(self) -> FeatureSpec:
        return {
            "action_joint": (7,),
            "action_tcp_pose": (7,),
        }

    def read_joint_state(self) -> JointState:
        state = self.read_franka_state().as_dict()
        return JointState(
            position=_as_float_tuple(state.get("q"), width=7),
            velocity=_as_float_tuple(state.get("dq"), width=7),
            effort=_as_float_tuple(state.get("tau_J"), width=7),
        )

    def read_tcp_pose(self) -> Pose | None:
        return _matrix_to_pose(self.read_franka_state().as_dict().get("O_T_EE"))

    def get_observation(self) -> RobotObservation:
        franka_state = self.read_franka_state().as_dict()
        joint_state = JointState(
            position=_as_float_tuple(franka_state.get("q"), width=7),
            velocity=_as_float_tuple(franka_state.get("dq"), width=7),
            effort=_as_float_tuple(franka_state.get("tau_J"), width=7),
        )
        tcp_pose = _matrix_to_pose(franka_state.get("O_T_EE"))
        observation: RobotObservation = {
            "obs_joint": joint_state.position,
            "obs_joint_velocity": joint_state.velocity,
            "obs_joint_effort": joint_state.effort,
            "obs_tcp_pose": ()
            if tcp_pose is None
            else (*tcp_pose.position, *tcp_pose.orientation_xyzw),
            "control_command_success_rate": franka_state.get(
                "control_command_success_rate", 0.0
            ),
        }
        observation.update(franka_state)
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        joint_position = action.get("action_joint")
        tcp_pose = action.get("action_tcp_pose")
        result: RobotAction = {}
        if joint_position is not None:
            target = _as_float_tuple(joint_position, width=7)
            self.command_joint_position(target)
            result["action_joint"] = target
        if tcp_pose is not None:
            target = _as_float_tuple(tcp_pose, width=7)
            self.command_tcp_pose(target)
            result["action_tcp_pose"] = target
        if not result:
            raise ValueError(
                "Franka action must contain action_joint or action_tcp_pose"
            )
        return result

    def command_joint_position(self, joint_position: Sequence[float]) -> None:
        """Send a joint position command in backend-specific drivers."""

        _ = joint_position
        raise NotImplementedError(
            "This Franka backend does not command joint positions"
        )

    def command_tcp_pose(self, tcp_pose: Sequence[float]) -> None:
        """Send a TCP pose command in backend-specific drivers."""

        _ = tcp_pose
        raise NotImplementedError("This Franka backend does not command TCP poses")

    def legacy_observation(self) -> RobotObservation:
        """Return only fields present in older recorded trajectory schemas."""

        return self.filter_observation(
            self.get_observation(), LEGACY_TRAJECTORY_OBSERVATION_FIELDS
        )

    @staticmethod
    def filter_observation(
        observation: Mapping[str, object], feature_names: Iterable[str]
    ) -> RobotObservation:
        """Filter a new full observation down to a requested feature set."""

        return {
            name: observation[name] for name in feature_names if name in observation
        }

    @staticmethod
    def filter_action(
        action: Mapping[str, object], feature_names: Iterable[str]
    ) -> RobotAction:
        """Filter a command payload down to features accepted by older callers."""

        return {name: action[name] for name in feature_names if name in action}
