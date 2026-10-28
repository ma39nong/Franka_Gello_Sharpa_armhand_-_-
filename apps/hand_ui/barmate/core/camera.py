"""Abstract camera interface for low-level image devices."""

from __future__ import annotations

import warnings
from abc import ABC, abstractmethod
from typing import Any

from numpy.typing import NDArray

from barmate.core.types import CameraFrame


class CameraInterface(ABC):
    """Low-level contract for cameras such as Intel RealSense."""

    @abstractmethod
    def connect(self) -> None:
        """Open the camera stream."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close the camera stream."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the camera stream is active."""

    @abstractmethod
    def read_frame(self) -> CameraFrame:
        """Read one frame from the camera."""

    @abstractmethod
    def async_read_frame(self, timeout_ms: float = 200.0) -> NDArray[Any]:
        """Return the most recent new frame, waiting up to ``timeout_ms``."""

    @abstractmethod
    def async_read_capture(self, timeout_ms: float = 200.0) -> Any:
        """Return the next backend-specific capture, waiting up to ``timeout_ms``."""

    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        """Return the most recent frame captured immediately."""

        _ = max_age_ms
        warnings.warn(
            (
                f"{self.__class__.__name__}.read_latest() is not implemented. "
                "Please override read_latest(); it will be required in future releases."
            ),
            FutureWarning,
            stacklevel=2,
        )
        return self.async_read_frame()
