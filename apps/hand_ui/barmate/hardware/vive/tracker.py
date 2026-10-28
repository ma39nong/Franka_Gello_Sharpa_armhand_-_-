"""HTC Vive tracker driver implementing the core TeleoperatorInterface."""

from __future__ import annotations

import math
from typing import Any

from barmate.core.teleoperator import TeleoperatorInterface
from barmate.core.types import (
    FeatureSpec,
    Pose,
    TeleoperatorAction,
    TeleoperatorFeedback,
    TrackerPose,
)

# Implements barmate.core.teleoperator.TeleoperatorInterface using OpenVR/triad_openvr-style readers.


class ViveTracker(TeleoperatorInterface):
    """Read pose actions from an HTC Vive tracker."""

    def __init__(self, serial: str | None = None, *, reader: Any | None = None) -> None:
        self.serial = serial
        self._reader = reader
        self._owns_reader = reader is None
        self._connected = False

    def configure(self) -> None:
        """Vive tracker currently has no runtime configuration."""

    def connect(self) -> None:
        if self._connected:
            return
        if self._reader is None:
            try:
                import triad_openvr
            except ModuleNotFoundError:
                self._reader = _OpenVRReader(self.serial)
                connect = getattr(self._reader, "connect", None)
                if callable(connect):
                    connect()
            else:
                vr_context = triad_openvr.triad_openvr()
                self._reader = self._select_triad_device(vr_context.devices)
        self._connected = True

    def disconnect(self) -> None:
        if self._owns_reader:
            close = getattr(self._reader, "close", None)
            if callable(close):
                close()
        self._connected = False
        if self._owns_reader:
            self._reader = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def action_features(self) -> FeatureSpec:
        return {
            "tracker_pose": TrackerPose,
            "teleop_xyz": (3,),
            "teleop_quat": (4,),
            "teleop_pose_valid": float,
        }

    @property
    def feedback_features(self) -> FeatureSpec:
        return {}

    def read_pose(self) -> TrackerPose:
        if not self._connected or self._reader is None:
            raise RuntimeError("Vive tracker is not connected")

        get_pose_quaternion = getattr(self._reader, "get_pose_quaternion", None)
        if callable(get_pose_quaternion):
            values = tuple(float(value) for value in get_pose_quaternion())
            if len(values) >= 7:
                return TrackerPose(
                    pose=Pose(
                        position=(values[0], values[1], values[2]),
                        orientation_xyzw=(values[4], values[5], values[6], values[3]),
                    ),
                    timestamp=None,
                )

        get_pose_euler = getattr(self._reader, "get_pose_euler", None)
        if callable(get_pose_euler):
            values = tuple(float(value) for value in get_pose_euler())
            if len(values) >= 6:
                return TrackerPose(
                    pose=Pose(
                        position=(values[0], values[1], values[2]),
                        orientation_xyzw=_euler_xyz_to_quaternion(
                            values[5], values[4], values[3]
                        ),
                    ),
                    timestamp=None,
                )

        read = getattr(self._reader, "read", None)
        if callable(read):
            sample = read()
            translation = tuple(float(value) for value in sample.translation_m)
            quaternion = _matrix_to_quaternion(sample.rotation_matrix)
            return TrackerPose(
                pose=Pose(
                    position=(translation[0], translation[1], translation[2]),
                    orientation_xyzw=quaternion,
                ),
                linear_velocity=None,
                angular_velocity=None,
                timestamp=None,
            )

        raise RuntimeError("Connected Vive reader does not expose a pose method")

    def get_action(self) -> TeleoperatorAction:
        tracker_pose = self.read_pose()
        return {
            "tracker_pose": tracker_pose,
            "teleop_xyz": tracker_pose.pose.position,
            "teleop_quat": tracker_pose.pose.orientation_xyzw,
            "teleop_pose_valid": 1.0,
        }

    def send_feedback(self, feedback: TeleoperatorFeedback) -> None:
        _ = feedback

    def _select_triad_device(self, devices: dict[str, Any]) -> Any:
        if self.serial is not None:
            for name, device in devices.items():
                serial = getattr(device, "serial", None) or getattr(device, "serial_number", None)
                if serial == self.serial or name == self.serial:
                    return device
            raise RuntimeError(f"Vive tracker with serial {self.serial!r} was not found")
        for name, device in devices.items():
            if name.startswith("tracker"):
                return device
        raise RuntimeError("No Vive tracker device was found")


class _OpenVRReader:
    """Small OpenVR reader used when triad_openvr is not installed."""

    def __init__(self, serial: str | None) -> None:
        self.serial = serial
        self._openvr: Any | None = None
        self._vr_system: Any | None = None
        self._device_index: int | None = None

    def connect(self) -> None:
        import openvr

        self._openvr = openvr
        init_result = openvr.init(getattr(openvr, "VRApplication_Other", 0))
        vr_system_factory = getattr(openvr, "VRSystem", None)
        self._vr_system = vr_system_factory() if callable(vr_system_factory) else init_result
        self._device_index = self._find_tracker_index()

    def close(self) -> None:
        if self._openvr is not None:
            shutdown = getattr(self._openvr, "shutdown", None)
            if callable(shutdown):
                shutdown()
        self._openvr = None
        self._vr_system = None
        self._device_index = None

    def get_pose_quaternion(self) -> tuple[float, float, float, float, float, float, float]:
        if self._openvr is None or self._vr_system is None or self._device_index is None:
            raise RuntimeError("OpenVR reader is not connected")
        origin = getattr(self._openvr, "TrackingUniverseStanding", 0)
        poses = self._vr_system.getDeviceToAbsoluteTrackingPose(origin, 0, self._device_index + 1)
        pose = poses[self._device_index]
        if not getattr(pose, "bPoseIsValid", False):
            raise RuntimeError("OpenVR tracker pose is not valid")
        matrix = getattr(pose, "mDeviceToAbsoluteTracking")
        rows = [tuple(float(value) for value in row) for row in matrix]
        quaternion = _matrix_to_quaternion(rows)
        return (
            rows[0][3],
            rows[1][3],
            rows[2][3],
            quaternion[3],
            quaternion[0],
            quaternion[1],
            quaternion[2],
        )

    def _find_tracker_index(self) -> int:
        if self._openvr is None or self._vr_system is None:
            raise RuntimeError("OpenVR reader is not connected")
        max_devices = getattr(self._openvr, "k_unMaxTrackedDeviceCount", 64)
        tracker_class = getattr(self._openvr, "TrackedDeviceClass_GenericTracker", None)
        serial_prop = getattr(self._openvr, "Prop_SerialNumber_String", None)
        for index in range(max_devices):
            if not self._vr_system.isTrackedDeviceConnected(index):
                continue
            device_class = self._vr_system.getTrackedDeviceClass(index)
            if tracker_class is not None and device_class != tracker_class:
                continue
            if self.serial is None:
                return index
            if serial_prop is not None:
                serial = self._vr_system.getStringTrackedDeviceProperty(index, serial_prop)
                if serial == self.serial:
                    return index
        raise RuntimeError("No matching OpenVR tracker was found")


def _euler_xyz_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _matrix_to_quaternion(matrix_like: Any) -> tuple[float, float, float, float]:
    rows = [tuple(float(value) for value in row) for row in matrix_like]
    trace = rows[0][0] + rows[1][1] + rows[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (rows[2][1] - rows[1][2]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
            (rows[1][0] - rows[0][1]) / scale,
            0.25 * scale,
        )
    return (0.0, 0.0, 0.0, 1.0)
