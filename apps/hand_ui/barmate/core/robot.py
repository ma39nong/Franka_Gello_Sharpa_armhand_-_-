"""Abstract robot hardware interfaces used by hardware drivers and apps."""

from __future__ import annotations

from abc import ABC, abstractmethod

from barmate.core.types import (
    FeatureSpec,
    FrankaRobotState,
    JointState,
    Pose,
    RobotAction,
    RobotObservation,
)


class RobotInterface(ABC):
    """Low-level contract implemented by robot hardware backends."""

    @abstractmethod
    def connect(self) -> None:
        """Open the physical or simulated robot connection."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close the robot connection."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the robot connection is active."""

    @abstractmethod
    def read_joint_state(self) -> JointState:
        """Read the current joint-space state."""

    @abstractmethod
    def read_tcp_pose(self) -> Pose | None:
        """Read the current tool pose when available."""

    @property
    @abstractmethod
    def observation_features(self) -> FeatureSpec:
        """Describe the structure and types returned by ``get_observation``."""

    @property
    @abstractmethod
    def action_features(self) -> FeatureSpec:
        """Describe the structure and types accepted by ``send_action``."""

    @abstractmethod
    def get_observation(self) -> RobotObservation:
        """Return observation fields for higher-level record/replay code."""

    @abstractmethod
    def send_action(self, action: RobotAction) -> RobotAction:
        """Send a low-level robot command and return command feedback."""

    def stop(self) -> None:
        """Stop active robot motion if the backend supports it."""


class FrankaRobotInterface(RobotInterface):
    """Robot interface extension for complete Franka RobotState access."""

    @abstractmethod
    def read_franka_state(self) -> FrankaRobotState:
        """Read all available Franka state fields exposed by the backend."""
