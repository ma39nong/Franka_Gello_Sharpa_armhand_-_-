"""Orbbec camera driver implementing the core CameraInterface."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from importlib import import_module
from typing import Any

import numpy as np
from numpy.typing import NDArray

from barmate.core.camera import CameraInterface
from barmate.core.types import CameraFrame, CameraMetadata


@dataclass(frozen=True, slots=True)
class OrbbecConfig:
    """Stream configuration for one Orbbec camera."""

    width: int = 640
    height: int = 480
    fps: int = 30
    enable_depth: bool = True
    align_depth_to_color: bool = True
    color_format: str = "RGB"


@dataclass(frozen=True, slots=True)
class OrbbecCapture:
    """Color/depth frame pair captured from one Orbbec camera."""

    color: NDArray[np.uint8]
    depth: NDArray[np.uint16] | None = None
    timestamp: float | None = None
    timestamp_ms: float | None = None
    color_frame_number: int | None = None
    depth_frame_number: int | None = None
    metadata: CameraMetadata = field(default_factory=dict)
    _frames: Any = field(default=None, repr=False, compare=False)
    _color_frame: Any = field(default=None, repr=False, compare=False)
    _depth_frame: Any = field(default=None, repr=False, compare=False)


class OrbbecCamera(CameraInterface):
    """Read color and optionally aligned depth frames from one Orbbec camera."""

    def __init__(
        self,
        serial: str | None = None,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        enable_depth: bool = True,
        align_depth_to_color: bool = True,
        color_format: str = "RGB",
        ob_module: Any | None = None,
    ) -> None:
        if width < 1:
            raise ValueError("width must be at least 1")
        if height < 1:
            raise ValueError("height must be at least 1")
        if fps < 1:
            raise ValueError("fps must be at least 1")
        if color_format.upper() not in {"BGR", "RGB"}:
            raise ValueError("color_format must be 'BGR' or 'RGB'")

        self.serial = serial
        self.config = OrbbecConfig(
            width=width,
            height=height,
            fps=fps,
            enable_depth=enable_depth,
            align_depth_to_color=align_depth_to_color,
            color_format=color_format.upper(),
        )
        self._ob = ob_module
        self._pipeline: Any | None = None
        self._last_capture: OrbbecCapture | None = None
        self._last_capture_monotonic = 0.0
        self.depth_scale_meters: float | None = None
        self.color_intrinsic_matrix: list[list[float]] | None = None

    def connect(self) -> None:
        """Open the Orbbec pipeline."""

        if self._pipeline is not None:
            return

        ob = self._load_orbbec_module()
        pipeline = self._create_pipeline(ob)
        config = ob.Config()

        color_profile = self._get_video_stream_profile(
            ob,
            pipeline,
            ob.OBSensorType.COLOR_SENSOR,
            getattr(ob.OBFormat, self.config.color_format),
        )
        config.enable_stream(color_profile)

        if self.config.enable_depth:
            depth_profile = self._get_video_stream_profile(
                ob,
                pipeline,
                ob.OBSensorType.DEPTH_SENSOR,
                ob.OBFormat.Y16,
            )
            config.enable_stream(depth_profile)
            config.set_frame_aggregate_output_mode(
                ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
            )
            config.set_align_mode(
                ob.OBAlignMode.HW_MODE
                if self.config.align_depth_to_color
                else ob.OBAlignMode.DISABLE
            )

        try:
            pipeline.start(config)
            self._read_stream_metadata(pipeline)
        except Exception:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            self._pipeline = None
            self.depth_scale_meters = None
            self.color_intrinsic_matrix = None
            raise

        self._pipeline = pipeline

    def disconnect(self) -> None:
        """Close the Orbbec pipeline."""

        try:
            if self._pipeline is not None:
                self._pipeline.stop()
        finally:
            self._pipeline = None
            self._last_capture = None
            self._last_capture_monotonic = 0.0

    @property
    def is_connected(self) -> bool:
        """Whether the Orbbec pipeline is active."""

        return self._pipeline is not None

    def read_frame(self) -> CameraFrame:
        """Read one blocking color frame with depth metadata when enabled."""

        capture = self.read_capture()
        return CameraFrame(
            image=capture.color,
            timestamp=capture.timestamp,
            metadata={
                **capture.metadata,
                "depth_image": capture.depth,
                "depth_scale_meters": self.depth_scale_meters,
                "intrinsic_matrix": self.color_intrinsic_matrix,
                "color_intrinsic_matrix": self.color_intrinsic_matrix,
            },
        )

    def read_capture(self) -> OrbbecCapture:
        """Read the next color/depth capture, waiting until the camera produces one."""

        if self._pipeline is None:
            raise RuntimeError("Orbbec camera is not connected")
        frames = None
        while frames is None:
            frames = self._pipeline.wait_for_frames(1000)
        return self._capture_from_frames(frames)

    def async_read_frame(self, timeout_ms: float = 200.0) -> NDArray[np.uint8]:
        """Return the next color frame, waiting up to ``timeout_ms``."""

        return self.async_read_capture(timeout_ms=timeout_ms).color

    def async_read_capture(self, timeout_ms: float = 200.0) -> OrbbecCapture:
        """Return the next color/depth capture, waiting up to ``timeout_ms``."""

        if self._pipeline is None:
            raise RuntimeError("Orbbec camera is not connected")
        frames = self._pipeline.wait_for_frames(int(timeout_ms))
        if frames is None:
            raise TimeoutError(f"No Orbbec frame arrived within {timeout_ms:.1f} ms")
        return self._capture_from_frames(frames)

    def read_latest(self, max_age_ms: int = 500) -> NDArray[np.uint8]:
        """Return the most recent color frame if fresh, otherwise read a new one."""

        if self._last_capture is not None:
            age_ms = (time.monotonic() - self._last_capture_monotonic) * 1000.0
            if age_ms <= max_age_ms:
                return self._last_capture.color
        return self.async_read_frame(timeout_ms=float(max_age_ms))

    def _load_orbbec_module(self) -> Any:
        if self._ob is None:
            self._ob = import_module("pyorbbecsdk")
        return self._ob

    def _create_pipeline(self, ob: Any) -> Any:
        if self.serial is None:
            return ob.Pipeline()

        context = ob.Context()
        device = context.query_devices().get_device_by_serial_number(self.serial)
        return ob.Pipeline(device)

    def _get_video_stream_profile(
        self,
        ob: Any,
        pipeline: Any,
        sensor_type: Any,
        frame_format: Any,
    ) -> Any:
        profile_list = pipeline.get_stream_profile_list(sensor_type)
        try:
            return profile_list.get_video_stream_profile(
                self.config.width,
                self.config.height,
                frame_format,
                self.config.fps,
            )
        except Exception:
            return profile_list.get_default_video_stream_profile()

    def _read_stream_metadata(self, pipeline: Any) -> None:
        camera_param = pipeline.get_camera_param()
        self.color_intrinsic_matrix = _intrinsics_to_matrix(camera_param.rgb_intrinsic)

    def _capture_from_frames(self, frames: Any) -> OrbbecCapture:
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame() if self.config.enable_depth else None
        if color_frame is None:
            raise RuntimeError("Orbbec frame set did not contain a color frame")
        if self.config.enable_depth and depth_frame is None:
            raise RuntimeError("Orbbec frame set did not contain a depth frame")

        color = self._color_image_from_frame(color_frame)
        depth = self._depth_image_from_frame(depth_frame) if depth_frame else None
        timestamp_ms = float(color_frame.get_timestamp())
        self.depth_scale_meters = (
            float(depth_frame.get_depth_scale()) / 1000.0
            if depth_frame
            else self.depth_scale_meters
        )
        capture = OrbbecCapture(
            color=color,
            depth=depth,
            timestamp=timestamp_ms / 1000.0,
            timestamp_ms=timestamp_ms,
            color_frame_number=int(color_frame.get_index()),
            depth_frame_number=int(depth_frame.get_index()) if depth_frame else None,
            metadata={
                "serial": self.serial,
                "width": self.config.width,
                "height": self.config.height,
                "fps": self.config.fps,
                "color_format": "bgr8",
                "depth_format": "y16" if self.config.enable_depth else None,
            },
            _frames=frames,
            _color_frame=color_frame,
            _depth_frame=depth_frame,
        )
        self._last_capture = capture
        self._last_capture_monotonic = time.monotonic()
        return capture

    def _color_image_from_frame(self, frame: Any) -> NDArray[np.uint8]:
        image = np.asanyarray(frame.get_data(), dtype=np.uint8)
        width = int(frame.get_width())
        height = int(frame.get_height())
        frame_format = frame.get_format()
        ob = self._load_orbbec_module()

        if frame_format == ob.OBFormat.BGR:
            return image.reshape((height, width, 3))
        if frame_format == ob.OBFormat.RGB:
            return image.reshape((height, width, 3))[:, :, ::-1]
        converted = self._convert_to_rgb_frame(ob, frame, frame_format)
        if converted is not None:
            return np.asanyarray(converted.get_data(), dtype=np.uint8).reshape(
                (height, width, 3)
            )[:, :, ::-1]
        raise RuntimeError(f"Unsupported Orbbec color frame format: {frame_format}")

    def _convert_to_rgb_frame(
        self,
        ob: Any,
        frame: Any,
        frame_format: Any,
    ) -> Any | None:
        convert_formats = {
            ob.OBFormat.I420: ob.OBConvertFormat.I420_TO_RGB888,
            ob.OBFormat.MJPG: ob.OBConvertFormat.MJPG_TO_RGB888,
            ob.OBFormat.NV12: ob.OBConvertFormat.NV12_TO_RGB888,
            ob.OBFormat.NV21: ob.OBConvertFormat.NV21_TO_RGB888,
            ob.OBFormat.UYVY: ob.OBConvertFormat.UYVY_TO_RGB888,
            ob.OBFormat.YUYV: ob.OBConvertFormat.YUYV_TO_RGB888,
        }
        convert_format = convert_formats.get(frame_format)
        if convert_format is None:
            return None
        convert_filter = ob.FormatConvertFilter()
        convert_filter.set_format_convert_format(convert_format)
        converted = convert_filter.process(frame)
        if converted is None:
            raise RuntimeError(f"Failed to convert Orbbec frame format: {frame_format}")
        return converted

    def _depth_image_from_frame(self, frame: Any) -> NDArray[np.uint16]:
        if frame.get_format() != self._load_orbbec_module().OBFormat.Y16:
            raise RuntimeError(f"Unsupported Orbbec depth frame format: {frame.get_format()}")
        return np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(
            (int(frame.get_height()), int(frame.get_width()))
        )


def list_orbbec_serials(ob_module: Any | None = None) -> list[str]:
    """Return connected Orbbec device serial numbers."""

    ob = import_module("pyorbbecsdk") if ob_module is None else ob_module
    devices = ob.Context().query_devices()
    return [
        devices.get_device_serial_number_by_index(index)
        for index in range(devices.get_count())
    ]


def _intrinsics_to_matrix(intrinsics: Any) -> list[list[float]]:
    return [
        [float(intrinsics.fx), 0.0, float(intrinsics.cx)],
        [0.0, float(intrinsics.fy), float(intrinsics.cy)],
        [0.0, 0.0, 1.0],
    ]


__all__ = [
    "OrbbecCamera",
    "OrbbecCapture",
    "OrbbecConfig",
    "list_orbbec_serials",
]
