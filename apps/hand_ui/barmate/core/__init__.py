"""Core hardware interface contracts for the BarMate refactor."""

from barmate.core.camera import CameraInterface
from barmate.core.robot import FrankaRobotInterface, RobotInterface
from barmate.core.teleoperator import SpaceMouseInterface, TeleoperatorInterface
from barmate.core.types import (
    CameraFrame,
    FeatureSpec,
    FrankaRobotState,
    JointState,
    Pose,
    RobotAction,
    RobotObservation,
    TeleoperatorAction,
    TeleoperatorFeedback,
    TrackerPose,
)

__all__ = [
    "CameraFrame",
    "CameraInterface",
    "FeatureSpec",
    "FrankaRobotInterface",
    "FrankaRobotState",
    "JointState",
    "Pose",
    "RobotAction",
    "RobotInterface",
    "RobotObservation",
    "SpaceMouseInterface",
    "TeleoperatorAction",
    "TeleoperatorFeedback",
    "TeleoperatorInterface",
    "TrackerPose",
]
