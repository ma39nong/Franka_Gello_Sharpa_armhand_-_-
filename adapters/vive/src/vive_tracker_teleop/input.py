from __future__ import annotations

import math
import time

import numpy as np

from pico_bimanual_franka_teleop.types import Pose, SIDES, TeleopSample

from .config import ViveTrackerConfig
from .openvr_source import OpenVRClient, TrackerReading


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.acos(float(np.clip(cosine, -1.0, 1.0)))


def _control_pose(
    reading: TrackerReading,
    config: ViveTrackerConfig,
    side: str,
) -> Pose:
    world_rotation = config.world_to_control_rotation
    tracker_position = world_rotation @ reading.transform[:3, 3]
    tracker_rotation = (
        world_rotation @ reading.transform[:3, :3] @ world_rotation.T
    )
    local = config.tracker_to_control[side]
    return Pose(
        tracker_position + tracker_rotation @ local.translation,
        tracker_rotation @ local.rotation,
    )


class ViveTrackerInput:
    """Two-hand VIVE Tracker arm source with PICO-equivalent fault semantics."""

    def __init__(
        self,
        config: ViveTrackerConfig,
        operator,
        *,
        client: OpenVRClient | None = None,
    ) -> None:
        self.config = config
        self.operator = operator
        self.client = client if client is not None else OpenVRClient()
        self.last_poses: dict[str, Pose | None] = {side: None for side in SIDES}
        self.last_sample_at: float | None = None
        self.last_position_changed_at: dict[str, float | None] = {
            side: None for side in SIDES
        }
        self.last_rotation_changed_at: dict[str, float | None] = {
            side: None for side in SIDES
        }
        self.last_activations = {side: False for side in SIDES}
        self.readiness = {side: "waiting" for side in SIDES}
        self.last_readings: dict[str, TrackerReading] = {}
        try:
            self.client.open()
            self._wait_until_ready(config.ready_timeout)
        except BaseException:
            self.close()
            raise

    def _read(self) -> tuple[dict[str, Pose], dict[str, TrackerReading]]:
        readings = self.client.read(self.config.serials)
        poses = {
            side: _control_pose(reading, self.config, side)
            for side, reading in readings.items()
        }
        client_status = getattr(self.client, "last_status", {})
        for side in SIDES:
            self.readiness[side] = (
                "ready"
                if side in poses
                else client_status.get(
                    side, f"unavailable ({self.config.serials[side]})"
                )
            )
        return poses, readings

    def _wait_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        next_report = 0.0
        while time.monotonic() < deadline:
            self.operator.poll()
            try:
                poses, readings = self._read()
            except Exception as error:  # OpenVR runtime/API failure
                poses, readings = {}, {}
                for side in SIDES:
                    self.readiness[side] = f"OpenVR read failed: {error}"
            if poses:
                now = time.monotonic()
                self.last_sample_at = now
                self.last_readings = readings
                for side, pose in poses.items():
                    self.last_poses[side] = pose
                    self.last_position_changed_at[side] = now
                    self.last_rotation_changed_at[side] = now
                self.operator.disable_all("VIVE Trackers initialized")
                self.operator.show("VIVE Tracker state: " + self.status_summary())
                return
            now = time.monotonic()
            if now >= next_report:
                self.operator.show(
                    "Waiting for VIVE Trackers: "
                    + self.status_summary()
                    + f" | proceeding without them in {max(0.0, deadline - now):.0f}s"
                )
                next_report = now + 1.0
            time.sleep(0.05)
        self.operator.show(
            "No configured VIVE Tracker is currently usable; arms cannot "
            "engage until a tracker appears. " + self.status_summary()
        )

    def status_summary(self) -> str:
        return " | ".join(
            f"{side}={self.readiness[side]}, "
            f"configured={self.config.serials[side]}, "
            f"control={'ON' if self.last_activations[side] else 'off'}"
            for side in SIDES
        )

    def debug_raw_poses(self) -> dict[str, Pose]:
        return {
            side: pose
            for side, pose in self.last_poses.items()
            if pose is not None
        }

    def debug_feed_state(self) -> dict:
        return {
            "source": "openvr",
            "age": (
                None
                if self.last_sample_at is None
                else round(time.monotonic() - self.last_sample_at, 4)
            ),
            "n": len(self.last_readings),
            "serials": {
                side: reading.serial
                for side, reading in self.last_readings.items()
            },
            "readiness": dict(self.readiness),
        }

    def _motion_fault(
        self,
        poses: dict[str, Pose],
        activations: dict[str, bool],
        now: float,
        elapsed: float | None,
    ) -> str | None:
        faults = []
        for side, pose in poses.items():
            previous = self.last_poses[side]
            if (
                not activations[side]
                or not self.last_activations[side]
                or previous is None
            ):
                self.last_poses[side] = pose
                self.last_position_changed_at[side] = now
                self.last_rotation_changed_at[side] = now
                continue
            position_delta = float(np.linalg.norm(pose.position - previous.position))
            rotation_delta = _rotation_angle(pose.rotation @ previous.rotation.T)
            linear_speed = (
                position_delta / elapsed
                if elapsed is not None and elapsed > 0.0
                else None
            )
            angular_speed = (
                rotation_delta / elapsed
                if elapsed is not None and elapsed > 0.0
                else None
            )
            fault = None
            if position_delta > self.config.max_position_jump:
                fault = f"{side} VIVE Tracker position jumped {position_delta:.3f} m"
            elif rotation_delta > self.config.max_rotation_jump:
                fault = f"{side} VIVE Tracker rotation jumped {rotation_delta:.3f} rad"
            elif linear_speed is not None and linear_speed > self.config.max_linear_speed:
                fault = f"{side} VIVE Tracker linear speed {linear_speed:.3f} m/s"
            elif angular_speed is not None and angular_speed > self.config.max_angular_speed:
                fault = f"{side} VIVE Tracker angular speed {angular_speed:.3f} rad/s"

            if position_delta > 1e-5:
                self.last_position_changed_at[side] = now
            if rotation_delta > 1e-4:
                self.last_rotation_changed_at[side] = now
            position_changed_at = self.last_position_changed_at[side]
            rotation_changed_at = self.last_rotation_changed_at[side]
            if (
                position_changed_at is None
                or now - position_changed_at > self.config.frozen_timeout
            ):
                fault = f"{side} VIVE Tracker position is frozen"
            elif (
                rotation_changed_at is None
                or now - rotation_changed_at > self.config.frozen_timeout
            ):
                fault = f"{side} VIVE Tracker rotation is frozen"
            self.readiness[side] = fault or "ready"
            if fault is not None:
                faults.append(fault)
            self.last_poses[side] = pose
        return "; ".join(faults) if faults else None

    def sample(self) -> TeleopSample | None:
        activations = self.operator.poll()
        now = time.monotonic()
        try:
            poses, readings = self._read()
        except Exception as error:  # never continue through an invalid batch
            self.operator.disable_all(f"OpenVR read failed: {error}")
            self.last_activations = {side: False for side in SIDES}
            for side in SIDES:
                self.readiness[side] = f"OpenVR read failed: {error}"
            return None
        elapsed = None if self.last_sample_at is None else now - self.last_sample_at
        self.last_sample_at = now
        self.last_readings = readings

        for side in SIDES:
            if side in poses:
                continue
            if activations[side] and self.last_activations[side]:
                self.operator.disable_all(
                    f"{side} VIVE Tracker lost: {self.readiness[side]}"
                )
                self.last_activations = {selected: False for selected in SIDES}
                return None
            self.operator.deny(side, self.readiness[side])
            activations[side] = False
            self.last_poses[side] = None
            self.last_position_changed_at[side] = None
            self.last_rotation_changed_at[side] = None

        fault = self._motion_fault(poses, activations, now, elapsed)
        if fault is not None:
            self.operator.disable_all(fault)
            self.last_activations = {side: False for side in SIDES}
            return None
        self.last_activations = dict(activations)
        output_poses = {
            side: poses.get(side, Pose(np.zeros(3), np.eye(3)))
            for side in SIDES
        }
        return TeleopSample(output_poses, activations, now)

    def close(self) -> None:
        client = getattr(self, "client", None)
        self.client = None
        if client is not None:
            client.close()
