"""Command-line tools for camera hardware."""

from barmate.hardware.cameras.tools.process import (
    CameraToolProcess,
    start_camera_recording,
    start_camera_viewer,
    start_realsense_recording,
    start_realsense_viewer,
)
from barmate.hardware.cameras.tools.backends import backend_choices
from barmate.hardware.cameras.tools.session import (
    CameraRecordingConfig,
    CameraRecordingSession,
)

__all__ = [
    "CameraRecordingConfig",
    "CameraRecordingSession",
    "CameraToolProcess",
    "backend_choices",
    "start_camera_recording",
    "start_camera_viewer",
    "start_realsense_recording",
    "start_realsense_viewer",
]
