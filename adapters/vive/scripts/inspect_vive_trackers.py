#!/usr/bin/env python3
"""Read-only SteamVR connectivity and VIVE Tracker pose inspector."""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from vive_tracker_teleop import OpenVRClient, load_vive_config


def quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        values = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        indices = [(1, 2), (2, 0), (0, 1)]
        first, second = indices[index]
        scale = 2.0 * math.sqrt(
            max(0.0, 1.0 + matrix[index, index] - matrix[first, first] - matrix[second, second])
        )
        values = np.zeros(4)
        values[index + 1] = 0.25 * scale
        values[0] = (matrix[second, first] - matrix[first, second]) / scale
        values[first + 1] = (matrix[first, index] + matrix[index, first]) / scale
        values[second + 1] = (matrix[second, index] + matrix[index, second]) / scale
    values /= np.linalg.norm(values)
    if values[0] < 0.0:
        values *= -1.0
    return tuple(float(value) for value in values)


def print_devices(client: OpenVRClient, configured: dict[str, str]) -> bool:
    devices = client.list_devices()
    roles = {serial: role for role, serial in configured.items()}
    print("idx class              role   serial          connected valid battery model")
    print("-" * 100)
    usable = set()
    for device in devices:
        role = roles.get(device.serial, "-")
        battery = "-" if device.battery is None else f"{100 * device.battery:.0f}%"
        print(
            f"{device.index:>3} {device.device_class:<18} {role:<6} "
            f"{device.serial:<15} {str(device.connected):<9} "
            f"{str(device.pose_valid):<5} {battery:<7} {device.model}"
        )
        if role != "-" and device.tracking_ok:
            usable.add(role)
            if device.transform is not None:
                position = device.transform[:3, 3]
                quat = quaternion_wxyz(device.transform[:3, :3])
                print(
                    "    "
                    f"P=({position[0]:+.4f}, {position[1]:+.4f}, {position[2]:+.4f}) m "
                    f"Qwxyz=({quat[0]:+.4f}, {quat[1]:+.4f}, "
                    f"{quat[2]:+.4f}, {quat[3]:+.4f}) "
                    f"tracking_result={device.tracking_result}"
                )
    for role, serial in configured.items():
        if role not in usable:
            print(
                f"ERROR: configured {role} Tracker {serial} is not "
                "connected with Running_OK tracking"
            )
    return usable == set(configured)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect SteamVR devices without starting ROS or robot control"
    )
    parser.add_argument("--config", default="config/modes/vive.yaml")
    parser.add_argument(
        "--watch",
        action="store_true",
        help="refresh continuously instead of printing one snapshot",
    )
    parser.add_argument("--rate", type=float, default=5.0)
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    config = load_vive_config(args.config)
    try:
        with OpenVRClient(discovery_interval=0.2) as client:
            while True:
                ok = print_devices(client, config.serials)
                print("\nconfigured Trackers: " + ("READY" if ok else "NOT READY"))
                if not args.watch:
                    raise SystemExit(0 if ok else 2)
                print("\nCtrl-C to stop\n")
                time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
