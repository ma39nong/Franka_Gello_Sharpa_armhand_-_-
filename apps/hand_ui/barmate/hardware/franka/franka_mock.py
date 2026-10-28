"""Franka mock robot driver implementing the core FrankaRobotInterface."""

from __future__ import annotations

from collections.abc import Sequence

from barmate.core.types import FrankaRobotState, JointState, RobotAction
from barmate.hardware.franka.base import FrankaDriverBase


class FrankaMockRobot(FrankaDriverBase):
    """In-process Franka mock used for import checks and dry-run recording."""

    def __init__(
        self,
        name: str = "franka",
        joint_position: Sequence[float] | None = None,
        joint_velocity: Sequence[float] | None = None,
        joint_effort: Sequence[float] | None = None,
        tcp_pose: Sequence[float] | None = None,
    ) -> None:
        super().__init__(name=name)
        self._joint_position = tuple(
            float(value) for value in ((0.0,) * 7 if joint_position is None else joint_position)
        )
        self._joint_velocity = tuple(
            float(value) for value in ((0.0,) * 7 if joint_velocity is None else joint_velocity)
        )
        self._joint_effort = tuple(
            float(value) for value in ((0.0,) * 7 if joint_effort is None else joint_effort)
        )
        self._tcp_pose = tuple(
            float(value)
            for value in (
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                if tcp_pose is None
                else tcp_pose
            )
        )

    def read_franka_state(self) -> FrankaRobotState:
        state = FrankaRobotState.zero().as_dict()
        state["q"] = self._joint_position
        state["q_d"] = self._joint_position
        state["dq"] = self._joint_velocity
        state["dq_d"] = self._joint_velocity
        state["tau_J"] = self._joint_effort
        state["O_T_EE"] = self._tcp_pose
        state["control_command_success_rate"] = 1.0
        state["robot_mode"] = "kIdle"
        return FrankaRobotState(state)

    def send_action(self, action: RobotAction) -> RobotAction:
        result = super().send_action(action)
        result["accepted"] = True
        return result

    def command_joint_position(self, joint_position: Sequence[float]) -> None:
        target = tuple(float(value) for value in joint_position)
        self._joint_velocity = tuple(
            target[index] - self._joint_position[index] for index in range(7)
        )
        self._joint_position = target

    def command_tcp_pose(self, tcp_pose: Sequence[float]) -> None:
        self._tcp_pose = tuple(float(value) for value in tcp_pose)

    def read_joint_state(self) -> JointState:
        return JointState(
            position=self._joint_position,
            velocity=self._joint_velocity,
            effort=self._joint_effort,
        )
