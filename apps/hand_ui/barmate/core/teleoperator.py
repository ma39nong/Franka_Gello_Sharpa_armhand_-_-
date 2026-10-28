"""Abstract teleoperator interfaces for human input devices."""

from __future__ import annotations

from abc import ABC, abstractmethod

from barmate.core.types import FeatureSpec, TeleoperatorAction, TeleoperatorFeedback


class TeleoperatorInterface(ABC):
    """Low-level contract for teleoperation input devices."""

    @abstractmethod
    def configure(self) -> None:
        """Apply one-time or runtime configuration to the teleoperator."""

    @abstractmethod
    def connect(self) -> None:
        """Open the teleoperator device connection."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close the teleoperator device connection."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the teleoperator is connected."""

    @property
    @abstractmethod
    def action_features(self) -> FeatureSpec:
        """Describe the structure and types returned by ``get_action``."""

    @property
    @abstractmethod
    def feedback_features(self) -> FeatureSpec:
        """Describe the structure and types accepted by ``send_feedback``."""

    @abstractmethod
    def get_action(self) -> TeleoperatorAction:
        """Retrieve the current action from the teleoperator."""

    @abstractmethod
    def send_feedback(self, feedback: TeleoperatorFeedback) -> None:
        """Send feedback to the teleoperator."""


class SpaceMouseInterface(TeleoperatorInterface):
    """Specialized contract for six-degree-of-freedom SpaceMouse input."""
