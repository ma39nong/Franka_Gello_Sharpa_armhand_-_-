"""Utilities for inspecting and preparing HTC Vive tracker setups."""

from __future__ import annotations

import argparse
import importlib
import json
import shutil
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class ViveDeviceInfo:
    """Connected OpenVR device metadata."""

    index: int
    device_class: int
    class_name: str
    serial: str
    model: str
    pose_valid: bool
    tracking_result: int

    @property
    def is_tracker_candidate(self) -> bool:
        normalized_serial = self.serial.upper()
        normalized_model = self.model.lower()
        return (
            self.class_name == "GenericTracker"
            or normalized_serial.startswith("LHR-")
            or "tracker" in normalized_model
        )


@dataclass(slots=True)
class SteamVRPatchResult:
    """Result returned after patching SteamVR settings."""

    settings_path: Path
    backup_path: Path | None
    changed_values: dict[str, object]
    patched_data: dict[str, Any]
    dry_run: bool


@dataclass(slots=True)
class TrackerReadTestResult:
    """Summary from probing one tracker for valid OpenVR pose reads."""

    device: ViveDeviceInfo
    samples_requested: int
    samples_read: int
    valid_samples: int
    first_valid_position_m: tuple[float, float, float] | None
    last_valid_position_m: tuple[float, float, float] | None
    trigger_min: float | None
    trigger_max: float | None

    @property
    def ok(self) -> bool:
        return self.samples_read > 0 and self.valid_samples > 0

    @property
    def valid_ratio(self) -> float:
        if self.samples_read == 0:
            return 0.0
        return self.valid_samples / self.samples_read


def require_openvr() -> Any:
    """Import openvr with a helpful hardware-oriented error."""

    try:
        return importlib.import_module("openvr")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "openvr is not available. Install it, then start SteamVR before using Vive utilities."
        ) from exc


def list_vive_devices(origin: str = "standing") -> list[ViveDeviceInfo]:
    """Return connected OpenVR devices visible to SteamVR."""

    openvr = require_openvr()
    vr_system = _init_openvr(openvr)
    try:
        return _list_vive_devices(openvr, vr_system, origin)
    finally:
        _shutdown_openvr(openvr)


def test_tracker_readings(
    serial: str | None = None,
    *,
    origin: str = "standing",
    samples: int = 60,
    interval_sec: float = 0.05,
) -> TrackerReadTestResult:
    """Probe a tracker and report whether valid pose readings are arriving."""

    if samples < 1:
        raise ValueError("samples must be >= 1")
    if interval_sec < 0:
        raise ValueError("interval_sec must be >= 0")

    openvr = require_openvr()
    vr_system = _init_openvr(openvr)
    try:
        devices = _list_vive_devices(openvr, vr_system, origin)
        device = _select_tracker(devices, serial)
        origin_enum = _tracking_origin_enum(openvr, origin)
        samples_read = 0
        valid_samples = 0
        first_valid_position: tuple[float, float, float] | None = None
        last_valid_position: tuple[float, float, float] | None = None
        trigger_values: list[float] = []

        for index in range(samples):
            poses = _query_poses(openvr, vr_system, origin_enum)
            pose = poses[device.index]
            samples_read += 1
            if bool(getattr(pose, "bPoseIsValid", False)):
                position = _extract_position(pose)
                first_valid_position = first_valid_position or position
                last_valid_position = position
                valid_samples += 1

            trigger_value = _read_trigger_value(vr_system, device.index)
            if trigger_value is not None:
                trigger_values.append(trigger_value)

            if interval_sec and index < samples - 1:
                time.sleep(interval_sec)

        return TrackerReadTestResult(
            device=device,
            samples_requested=samples,
            samples_read=samples_read,
            valid_samples=valid_samples,
            first_valid_position_m=first_valid_position,
            last_valid_position_m=last_valid_position,
            trigger_min=min(trigger_values) if trigger_values else None,
            trigger_max=max(trigger_values) if trigger_values else None,
        )
    finally:
        _shutdown_openvr(openvr)


def candidate_steamvr_settings_paths(home: Path | None = None) -> list[Path]:
    """Return common Linux SteamVR settings locations."""

    base_home = home or Path.home()
    return [
        base_home / ".local/share/Steam/config/steamvr.vrsettings",
        base_home / ".steam/steam/config/steamvr.vrsettings",
        base_home / ".steam/debian-installation/config/steamvr.vrsettings",
        base_home
        / ".var/app/com.valvesoftware.Steam/.steam/steam/config/steamvr.vrsettings",
        base_home
        / ".var/app/com.valvesoftware.Steam/.local/share/Steam/config/steamvr.vrsettings",
    ]


def locate_steamvr_settings_path(
    path_override: Path | None = None,
    *,
    home: Path | None = None,
) -> Path:
    """Find the SteamVR settings file, or return the preferred default path."""

    if path_override is not None:
        return path_override.expanduser()

    candidates = candidate_steamvr_settings_paths(home)
    return next((path for path in candidates if path.exists()), candidates[0])


def patch_steamvr_settings_file(
    path_override: Path | None = None,
    *,
    home: Path | None = None,
    dry_run: bool = False,
) -> SteamVRPatchResult:
    """Set steamvr.requireHmd=false so tracker-only sessions can start."""

    settings_path = locate_steamvr_settings_path(path_override, home=home)
    current_data = _load_settings_file(settings_path)
    patched_data = _build_tracker_only_settings(current_data)
    changed_values = {"steamvr.requireHmd": False}

    if dry_run:
        return SteamVRPatchResult(
            settings_path=settings_path,
            backup_path=None,
            changed_values=changed_values,
            patched_data=patched_data,
            dry_run=True,
        )

    settings_path.parent.mkdir(parents=True, exist_ok=True)
    backup_path = _write_backup(settings_path) if settings_path.exists() else None
    settings_path.write_text(_serialize_settings(patched_data), encoding="utf-8")
    return SteamVRPatchResult(
        settings_path=settings_path,
        backup_path=backup_path,
        changed_values=changed_values,
        patched_data=patched_data,
        dry_run=False,
    )


def format_device_table(devices: Sequence[ViveDeviceInfo]) -> str:
    """Format OpenVR device metadata as a compact table."""

    if not devices:
        return "No OpenVR devices are currently visible."

    rows = [
        (
            str(device.index),
            device.class_name,
            device.serial or "-",
            device.model or "-",
            "yes" if device.pose_valid else "no",
            "yes" if device.is_tracker_candidate else "no",
            str(device.tracking_result),
        )
        for device in devices
    ]
    headers = ("index", "class", "serial", "model", "pose", "tracker", "tracking")
    widths = [
        max(len(headers[column]), *(len(row[column]) for row in rows))
        for column in range(len(headers))
    ]
    header = "  ".join(
        value.ljust(widths[column]) for column, value in enumerate(headers)
    )
    separator = "  ".join("-" * width for width in widths)
    body = "\n".join(
        "  ".join(value.ljust(widths[column]) for column, value in enumerate(row))
        for row in rows
    )
    return f"{header}\n{separator}\n{body}"


def _list_vive_devices(
    openvr: Any,
    vr_system: Any,
    origin: str,
) -> list[ViveDeviceInfo]:
    origin_enum = _tracking_origin_enum(openvr, origin)
    poses = _query_poses(openvr, vr_system, origin_enum)
    devices: list[ViveDeviceInfo] = []
    for index, pose in enumerate(poses):
        if not _is_device_connected(vr_system, index, pose):
            continue
        device_class = int(vr_system.getTrackedDeviceClass(index))
        devices.append(
            ViveDeviceInfo(
                index=index,
                device_class=device_class,
                class_name=_device_class_name(openvr, device_class),
                serial=_safe_get_string(
                    vr_system,
                    index,
                    getattr(openvr, "Prop_SerialNumber_String", 1002),
                ),
                model=_safe_get_string(
                    vr_system,
                    index,
                    getattr(openvr, "Prop_ModelNumber_String", 1001),
                ),
                pose_valid=bool(getattr(pose, "bPoseIsValid", False)),
                tracking_result=int(getattr(pose, "eTrackingResult", 0)),
            )
        )
    return devices


def _init_openvr(openvr: Any) -> Any:
    try:
        init_result = openvr.init(
            getattr(openvr, "VRApplication_Other", getattr(openvr, "VRApplication_Background", 0))
        )
    except Exception as exc:
        raise RuntimeError("OpenVR init failed. Start SteamVR, then retry.") from exc

    vr_system_factory = getattr(openvr, "VRSystem", None)
    vr_system = vr_system_factory() if callable(vr_system_factory) else init_result
    if vr_system is None:
        raise RuntimeError("OpenVR did not provide a VRSystem interface.")
    return vr_system


def _shutdown_openvr(openvr: Any) -> None:
    shutdown = getattr(openvr, "shutdown", None)
    if callable(shutdown):
        shutdown()


def _select_tracker(
    devices: Sequence[ViveDeviceInfo],
    serial: str | None,
) -> ViveDeviceInfo:
    trackers = [device for device in devices if device.is_tracker_candidate]
    if serial is not None:
        for device in trackers:
            if device.serial == serial:
                return device
        available = [device.serial or f"index={device.index}" for device in trackers]
        raise RuntimeError(
            f"No Vive tracker found for serial={serial!r}. Available tracker serials: {available}"
        )

    if trackers:
        return trackers[0]

    raise RuntimeError(
        "No Vive tracker devices are currently visible in OpenVR. "
        "Start SteamVR and confirm the trackers are connected."
    )


def _tracking_origin_enum(openvr: Any, origin: str) -> int:
    if origin == "raw":
        return getattr(
            openvr,
            "TrackingUniverseRawAndUncalibrated",
            getattr(openvr, "TrackingUniverseRaw", 2),
        )
    return getattr(openvr, "TrackingUniverseStanding", 1)


def _query_poses(openvr: Any, vr_system: Any, origin_enum: int) -> list[Any]:
    max_devices = int(getattr(openvr, "k_unMaxTrackedDeviceCount", 64))
    try:
        poses = vr_system.getDeviceToAbsoluteTrackingPose(
            origin_enum,
            0.0,
            max_devices,
        )
        if poses is not None:
            return [poses[index] for index in range(len(poses))]
    except TypeError:
        pass

    poses_type = openvr.TrackedDevicePose_t * max_devices
    pose_buffer = poses_type()
    vr_system.getDeviceToAbsoluteTrackingPose(origin_enum, 0.0, pose_buffer)
    return [pose_buffer[index] for index in range(max_devices)]


def _is_device_connected(vr_system: Any, index: int, pose: Any) -> bool:
    is_connected = getattr(vr_system, "isTrackedDeviceConnected", None)
    if callable(is_connected):
        return bool(is_connected(index))
    return bool(getattr(pose, "bDeviceIsConnected", False))


def _safe_get_string(vr_system: Any, device_index: int, property_id: int) -> str:
    try:
        return str(vr_system.getStringTrackedDeviceProperty(device_index, property_id))
    except Exception:
        return ""


def _device_class_name(openvr: Any, device_class: int) -> str:
    classes = {
        getattr(openvr, "TrackedDeviceClass_Invalid", 0): "Invalid",
        getattr(openvr, "TrackedDeviceClass_HMD", 1): "HMD",
        getattr(openvr, "TrackedDeviceClass_Controller", 2): "Controller",
        getattr(openvr, "TrackedDeviceClass_GenericTracker", 3): "GenericTracker",
        getattr(openvr, "TrackedDeviceClass_TrackingReference", 4): "TrackingReference",
        getattr(openvr, "TrackedDeviceClass_DisplayRedirect", 5): "DisplayRedirect",
    }
    return classes.get(device_class, f"Unknown({device_class})")


def _extract_position(pose: Any) -> tuple[float, float, float]:
    matrix = getattr(pose, "mDeviceToAbsoluteTracking")
    return (float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3]))


def _read_trigger_value(vr_system: Any, device_index: int) -> float | None:
    try:
        _, controller_state = vr_system.getControllerState(device_index)
    except Exception:
        return None

    axis = getattr(controller_state, "rAxis", None)
    if axis is None or len(axis) <= 1:
        return None
    return float(getattr(axis[1], "x", 0.0))


def _load_settings_file(settings_path: Path) -> dict[str, Any]:
    if not settings_path.exists():
        return {}

    raw = settings_path.read_text(encoding="utf-8", errors="ignore").strip()
    if not raw:
        return {}

    try:
        loaded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Invalid JSON in {settings_path}: {exc}") from exc

    if not isinstance(loaded, dict):
        raise RuntimeError(
            f"Expected JSON object in {settings_path}, got {type(loaded)!r}"
        )
    return loaded


def _build_tracker_only_settings(current_data: dict[str, Any]) -> dict[str, Any]:
    patched = dict(current_data)
    steamvr_section = dict(patched.get("steamvr", {}))
    steamvr_section["requireHmd"] = False
    patched["steamvr"] = steamvr_section
    return patched


def _write_backup(settings_path: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = settings_path.with_suffix(settings_path.suffix + f".bak-{timestamp}")
    shutil.copy2(settings_path, backup_path)
    return backup_path


def _serialize_settings(data: dict[str, Any]) -> str:
    return json.dumps(data, indent=2, sort_keys=True) + "\n"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manage HTC Vive tracker diagnostics for BarMate."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="List connected OpenVR devices.")
    list_parser.add_argument(
        "--origin",
        choices=("standing", "raw"),
        default="standing",
        help="OpenVR tracking origin used while querying poses.",
    )

    patch_parser = subparsers.add_parser(
        "patch-steamvr",
        aliases=("patch",),
        help="Patch SteamVR settings for tracker-only sessions.",
    )
    patch_parser.add_argument(
        "--settings",
        type=Path,
        help="Explicit steamvr.vrsettings path. Defaults to common Linux Steam paths.",
    )
    patch_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show the planned settings path and JSON without writing files.",
    )

    test_parser = subparsers.add_parser(
        "test",
        help="Probe one tracker and report whether valid pose readings arrive.",
    )
    test_parser.add_argument("--serial", help="Tracker serial to test. Defaults to the first tracker.")
    test_parser.add_argument(
        "--origin",
        choices=("standing", "raw"),
        default="standing",
        help="OpenVR tracking origin used while querying poses.",
    )
    test_parser.add_argument(
        "--samples",
        type=int,
        default=60,
        help="Number of pose samples to read.",
    )
    test_parser.add_argument(
        "--interval-sec",
        type=float,
        default=0.05,
        help="Delay between samples.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        if args.command == "list":
            print(format_device_table(list_vive_devices(origin=args.origin)))
            return 0

        if args.command in {"patch-steamvr", "patch"}:
            result = patch_steamvr_settings_file(
                args.settings,
                dry_run=args.dry_run,
            )
            print(f"settings: {result.settings_path}")
            if result.backup_path is not None:
                print(f"backup:   {result.backup_path}")
            print(f"dry_run:  {'yes' if result.dry_run else 'no'}")
            print(f"changed:  {result.changed_values}")
            if result.dry_run:
                print(_serialize_settings(result.patched_data), end="")
            return 0

        if args.command == "test":
            result = test_tracker_readings(
                args.serial,
                origin=args.origin,
                samples=args.samples,
                interval_sec=args.interval_sec,
            )
            print(f"serial:        {result.device.serial or '-'}")
            print(f"index:         {result.device.index}")
            print(f"model:         {result.device.model or '-'}")
            print(f"samples:       {result.samples_read}/{result.samples_requested}")
            print(f"valid_samples: {result.valid_samples}")
            print(f"valid_ratio:   {result.valid_ratio:.2%}")
            print(f"first_xyz_m:   {result.first_valid_position_m}")
            print(f"last_xyz_m:    {result.last_valid_position_m}")
            print(f"trigger_min:   {result.trigger_min}")
            print(f"trigger_max:   {result.trigger_max}")
            print(f"status:        {'ok' if result.ok else 'failed'}")
            return 0 if result.ok else 2

    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    parser.error(f"Unhandled command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
