"""Linker Glove driver placeholder implementing the core TeleoperatorInterface."""

from __future__ import annotations

from barmate.core.teleoperator import TeleoperatorInterface
from barmate.core.types import FeatureSpec, TeleoperatorAction, TeleoperatorFeedback

# Implements barmate.core.teleoperator.TeleoperatorInterface for Linker Glove.


class LinkerGloveDriver(TeleoperatorInterface):
    """Placeholder for low-level Linker Glove input."""

    def __init__(self) -> None:
        self._connected = False

    def configure(self) -> None:
        """Apply Linker Glove runtime configuration."""

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def action_features(self) -> FeatureSpec:
        return {"hand_pose": tuple}

    @property
    def feedback_features(self) -> FeatureSpec:
        return {}

    def get_action(self) -> TeleoperatorAction:
        return {"hand_pose": ()}

    def send_feedback(self, feedback: TeleoperatorFeedback) -> None:
        _ = feedback
