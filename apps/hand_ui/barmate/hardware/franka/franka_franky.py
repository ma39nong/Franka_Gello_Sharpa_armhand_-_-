"""Franka robot driver via the franky Python library implementing FrankaRobotInterface."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from barmate.core.types import FrankaRobotState
from barmate.hardware.franka.base import FrankaDriverBase

# Implements barmate.core.robot.FrankaRobotInterface using franky.


class FrankaFrankyRobot(FrankaDriverBase):
    """Thin placeholder for the franky-backed Franka driver."""

    def __init__(self, host: str, name: str = "franka") -> None:
        super().__init__(name=name)
        self.host = host
        self._robot: Any | None = None
        self._franky: Any | None = None

    def connect(self) -> None:
        import franky

        self._franky = franky
        self._robot = franky.Robot(self.host)
        self._connected = True

    def disconnect(self) -> None:
        self.stop()
        self._robot = None
        self._franky = None
        self._connected = False

    def read_franka_state(self) -> FrankaRobotState:
        if self._robot is None:
            raise RuntimeError("Franka franky robot is not connected")
        return FrankaRobotState.from_object(self._robot.state)

    def stop(self) -> None:
        if self._robot is not None and self._franky is not None:
            self._robot.move(self._franky.JointStopMotion(relative_dynamics_factor=0.1))

    def command_joint_position(self, joint_position: Sequence[float]) -> None:
        if self._robot is None or self._franky is None:
            raise RuntimeError("Franka franky robot is not connected")
        self._robot.move(
            self._franky.JointMotion(tuple(float(value) for value in joint_position))
        )

    def command_tcp_pose(self, tcp_pose: Sequence[float]) -> None:
        if self._robot is None or self._franky is None:
            raise RuntimeError("Franka franky robot is not connected")
        pose = tuple(float(value) for value in tcp_pose)
        target = self._franky.Affine(pose[:3], pose[3:])
        self._robot.move(self._franky.CartesianMotion(target))
