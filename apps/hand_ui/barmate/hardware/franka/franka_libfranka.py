"""Franka robot driver via libfranka/pylibfranka implementing FrankaRobotInterface."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from barmate.core.types import FrankaRobotState
from barmate.hardware.franka.base import FrankaDriverBase

# Implements barmate.core.robot.FrankaRobotInterface using libfranka/pylibfranka.


class FrankaLibfrankaRobot(FrankaDriverBase):
    """Thin placeholder for the low-level libfranka-backed Franka driver."""

    def __init__(self, host: str, name: str = "franka") -> None:
        super().__init__(name=name)
        self.host = host
        self._robot: Any | None = None
        self._pylibfranka: Any | None = None
        self._joint_position_control: Any | None = None
        self._cartesian_pose_control: Any | None = None

    def connect(self) -> None:
        import pylibfranka

        self._pylibfranka = pylibfranka
        self._robot = pylibfranka.Robot(self.host)
        self._connected = True

    def disconnect(self) -> None:
        self.stop()
        self._joint_position_control = None
        self._cartesian_pose_control = None
        self._robot = None
        self._pylibfranka = None
        self._connected = False

    def read_franka_state(self) -> FrankaRobotState:
        if self._robot is None:
            raise RuntimeError("Franka libfranka robot is not connected")
        return FrankaRobotState.from_object(self._robot.read_once())

    def stop(self) -> None:
        if self._robot is not None:
            self._robot.stop()

    def command_joint_position(self, joint_position: Sequence[float]) -> None:
        if self._robot is None or self._pylibfranka is None:
            raise RuntimeError("Franka libfranka robot is not connected")
        if self._joint_position_control is None:
            self._joint_position_control = self._robot.start_joint_position_control(
                self._pylibfranka.ControllerMode.CartesianImpedance
            )
        self._joint_position_control.writeOnce(
            self._pylibfranka.JointPositions(tuple(float(value) for value in joint_position))
        )

    def command_tcp_pose(self, tcp_pose: Sequence[float]) -> None:
        if self._robot is None or self._pylibfranka is None:
            raise RuntimeError("Franka libfranka robot is not connected")
        if self._cartesian_pose_control is None:
            self._cartesian_pose_control = self._robot.start_cartesian_pose_control(
                self._pylibfranka.ControllerMode.JointImpedance
            )
        self._cartesian_pose_control.writeOnce(
            self._pylibfranka.CartesianPose(tuple(float(value) for value in tcp_pose))
        )
