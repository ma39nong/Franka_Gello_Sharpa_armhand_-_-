#!/usr/bin/env python3
"""Read-only guided calibration of OpenVR horizontal axes."""

from __future__ import annotations

import argparse
import time

import numpy as np

from vive_tracker_teleop import (
    OpenVRClient,
    horizontal_world_to_control,
    load_vive_config,
    write_world_to_control_rotation,
)


def capture_position(
    client: OpenVRClient,
    role: str,
    serial: str,
    duration: float,
) -> np.ndarray:
    positions = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        readings = client.read({role: serial})
        reading = readings.get(role)
        if reading is None:
            reason = client.last_status.get(role, "unavailable")
            raise RuntimeError(f"{role} Tracker {serial} is not usable: {reason}")
        positions.append(reading.transform[:3, 3].copy())
        time.sleep(0.01)
    if len(positions) < 5:
        raise RuntimeError("too few valid OpenVR samples")
    values = np.asarray(positions)
    spread = float(np.linalg.norm(np.std(values, axis=0)))
    if spread > 0.01:
        raise RuntimeError(
            f"Tracker moved during capture (position spread {spread:.3f} m); "
            "hold it still after pressing Enter"
        )
    return np.median(values, axis=0)


def matrix_yaml(rotation: np.ndarray) -> str:
    return "\n".join(
        "  - [" + ", ".join(f"{value:.9f}" for value in row) + "]"
        for row in rotation
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate VIVE XY directions without starting ROS or connecting "
            "to either robot"
        )
    )
    parser.add_argument("--config", default="config/modes/vive.yaml")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--capture-seconds", type=float, default=0.5)
    parser.add_argument("--minimum-movement", type=float, default=0.08)
    parser.add_argument(
        "--write",
        action="store_true",
        help="update world_to_control_rotation in the selected config",
    )
    args = parser.parse_args()
    if args.capture_seconds <= 0.0 or args.minimum_movement <= 0.0:
        parser.error("capture duration and minimum movement must be positive")
    config = load_vive_config(args.config)
    serial = config.serials[args.side]

    print("This tool is read-only: it starts no ROS node and sends no robot command.")
    print(f"Using {args.side} Tracker {serial}.")
    print("Choose one repeatable physical starting point for all three captures.")
    with OpenVRClient(discovery_interval=0.1) as client:
        input("\nHold the Tracker at the starting point, then press Enter: ")
        origin = capture_position(
            client, args.side, serial, args.capture_seconds
        )
        input(
            "\nMove and hold the Tracker at least "
            f"{args.minimum_movement:.2f} m in intended robot +X "
            "(forward), then press Enter: "
        )
        forward = capture_position(
            client, args.side, serial, args.capture_seconds
        )
        input(
            "\nReturn to the same starting point, then move and hold at least "
            f"{args.minimum_movement:.2f} m in intended robot +Y (left), "
            "then press Enter: "
        )
        left = capture_position(
            client, args.side, serial, args.capture_seconds
        )

    forward_delta = forward - origin
    left_delta = left - origin
    rotation = horizontal_world_to_control(
        forward_delta,
        left_delta,
        minimum_displacement=args.minimum_movement,
    )
    print("\nMeasured OpenVR displacement:")
    print(f"  intended +X: {forward_delta.round(5).tolist()}")
    print(f"  intended +Y: {left_delta.round(5).tolist()}")
    print("\nCalibrated world_to_control_rotation:")
    print(matrix_yaml(rotation))
    if args.write:
        write_world_to_control_rotation(args.config, rotation)
        load_vive_config(args.config)  # validate the written document
        print(f"\nUpdated {args.config}")
    else:
        print("\nDry run only; repeat with --write to update the configuration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
