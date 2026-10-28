"""Read-only pyopenvr access with serial-number device assignment."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import numpy as np


def project_rotation_to_so3(value: np.ndarray) -> np.ndarray:
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    left, _, right_t = np.linalg.svd(rotation)
    projected = left @ right_t
    if np.linalg.det(projected) < 0.0:
        left[:, -1] *= -1.0
        projected = left @ right_t
    return projected


@dataclass(frozen=True)
class DeviceInfo:
    index: int
    device_class: str
    serial: str
    model: str
    connected: bool
    pose_valid: bool
    tracking_result: int
    tracking_ok: bool
    battery: float | None
    transform: np.ndarray | None


@dataclass(frozen=True)
class TrackerReading:
    serial: str
    transform: np.ndarray
    velocity: np.ndarray
    angular_velocity: np.ndarray
    tracking_result: int


class OpenVRClient:
    """Own one background OpenVR client and read all poses in one batch."""

    def __init__(self, *, discovery_interval: float = 1.0) -> None:
        if discovery_interval <= 0.0:
            raise ValueError("discovery_interval must be positive")
        self.discovery_interval = float(discovery_interval)
        self.openvr: Any | None = None
        self.vr: Any | None = None
        self.serial_to_index: dict[str, int] = {}
        self.last_discovery_at: float | None = None
        self.last_status: dict[str, str] = {}

    def open(self) -> None:
        if self.vr is not None:
            raise RuntimeError("OpenVR client is already open")
        import openvr

        self.openvr = openvr
        self.vr = openvr.init(openvr.VRApplication_Background)
        self._discover(force=True)

    def close(self) -> None:
        if self.openvr is not None and self.vr is not None:
            self.openvr.shutdown()
        self.openvr = None
        self.vr = None
        self.serial_to_index = {}
        self.last_discovery_at = None
        self.last_status = {}

    def _require_open(self) -> tuple[Any, Any]:
        if self.openvr is None or self.vr is None:
            raise RuntimeError("OpenVR client is not open")
        return self.openvr, self.vr

    def _discover(self, *, force: bool = False) -> None:
        openvr, vr = self._require_open()
        now = time.monotonic()
        if (
            not force
            and self.last_discovery_at is not None
            and now - self.last_discovery_at < self.discovery_interval
        ):
            return
        found: dict[str, int] = {}
        for index in range(openvr.k_unMaxTrackedDeviceCount):
            if (
                vr.getTrackedDeviceClass(index)
                != openvr.TrackedDeviceClass_GenericTracker
            ):
                continue
            try:
                serial = vr.getStringTrackedDeviceProperty(
                    index, openvr.Prop_SerialNumber_String
                )
            except Exception:  # device can disappear during enumeration
                continue
            if serial:
                found[str(serial)] = index
        self.serial_to_index = found
        self.last_discovery_at = now

    @staticmethod
    def _vector(raw: Any) -> np.ndarray:
        values = raw.v if hasattr(raw, "v") else raw
        result = np.asarray([float(values[i]) for i in range(3)], dtype=float)
        if not np.all(np.isfinite(result)):
            raise RuntimeError("OpenVR velocity contains NaN or infinity")
        return result

    @staticmethod
    def _transform(pose: Any) -> np.ndarray:
        raw = pose.mDeviceToAbsoluteTracking.m
        transform = np.eye(4, dtype=float)
        for row in range(3):
            for column in range(4):
                transform[row, column] = float(raw[row][column])
        if not np.all(np.isfinite(transform)):
            raise RuntimeError("OpenVR pose contains NaN or infinity")
        transform[:3, :3] = project_rotation_to_so3(transform[:3, :3])
        return transform

    def read(self, serials: dict[str, str]) -> dict[str, TrackerReading]:
        """Return available requested trackers from one OpenVR pose query."""
        openvr, vr = self._require_open()
        self._discover()
        poses = vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount,
        )
        readings: dict[str, TrackerReading] = {}
        status: dict[str, str] = {}
        for role, serial in serials.items():
            index = self.serial_to_index.get(serial)
            if index is None:
                status[role] = "serial not discovered by SteamVR"
                continue
            pose = poses[index]
            if not pose.bDeviceIsConnected:
                status[role] = "device disconnected"
                continue
            if not pose.bPoseIsValid:
                status[role] = "OpenVR pose invalid"
                continue
            tracking_result = int(pose.eTrackingResult)
            if tracking_result != int(openvr.TrackingResult_Running_OK):
                status[role] = f"OpenVR tracking result {tracking_result}, not Running_OK"
                continue
            readings[role] = TrackerReading(
                serial=serial,
                transform=self._transform(pose),
                velocity=self._vector(pose.vVelocity),
                angular_velocity=self._vector(pose.vAngularVelocity),
                tracking_result=tracking_result,
            )
            status[role] = "ready"
        self.last_status = status
        return readings

    def list_devices(self) -> list[DeviceInfo]:
        """Inspect every connected OpenVR slot using one synchronous pose batch."""
        openvr, vr = self._require_open()
        poses = vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount,
        )
        class_names = {
            int(openvr.TrackedDeviceClass_HMD): "HMD",
            int(openvr.TrackedDeviceClass_Controller): "Controller",
            int(openvr.TrackedDeviceClass_GenericTracker): "GenericTracker",
            int(openvr.TrackedDeviceClass_TrackingReference): "TrackingReference",
        }
        devices = []
        for index in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = int(vr.getTrackedDeviceClass(index))
            if device_class == int(openvr.TrackedDeviceClass_Invalid):
                continue
            def string_property(prop: int) -> str:
                try:
                    return str(vr.getStringTrackedDeviceProperty(index, prop))
                except Exception:
                    return ""

            pose = poses[index]
            battery = None
            try:
                battery = float(
                    vr.getFloatTrackedDeviceProperty(
                        index, openvr.Prop_DeviceBatteryPercentage_Float
                    )
                )
            except Exception:
                pass
            transform = None
            if pose.bDeviceIsConnected and pose.bPoseIsValid:
                try:
                    transform = self._transform(pose)
                except RuntimeError:
                    pass
            devices.append(
                DeviceInfo(
                    index=index,
                    device_class=class_names.get(device_class, str(device_class)),
                    serial=string_property(openvr.Prop_SerialNumber_String),
                    model=string_property(openvr.Prop_ModelNumber_String),
                    connected=bool(pose.bDeviceIsConnected),
                    pose_valid=bool(pose.bPoseIsValid),
                    tracking_result=int(pose.eTrackingResult),
                    tracking_ok=(
                        bool(pose.bDeviceIsConnected)
                        and bool(pose.bPoseIsValid)
                        and int(pose.eTrackingResult)
                        == int(openvr.TrackingResult_Running_OK)
                    ),
                    battery=battery,
                    transform=transform,
                )
            )
        return devices

    def __enter__(self) -> "OpenVRClient":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
