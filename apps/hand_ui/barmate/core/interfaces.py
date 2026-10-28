"""Convenience re-exports for hardware-agnostic imports from core interfaces."""

from barmate.core.camera import CameraInterface
from barmate.core.robot import FrankaRobotInterface, RobotInterface
from barmate.core.teleoperator import SpaceMouseInterface, TeleoperatorInterface

__all__ = [
    "CameraInterface",
    "FrankaRobotInterface",
    "RobotInterface",
    "SpaceMouseInterface",
    "TeleoperatorInterface",
]
