#!/usr/bin/env python3
"""Read-only validation of configured dual-GELLO stable device paths."""

from __future__ import annotations

import argparse
import grp
import os
import pwd
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "adapters" / "pico" / "src"))

from pico_bimanual_franka_teleop.gello_input import load_gello_config  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate GELLO by-id identity and access without opening motors."
    )
    parser.add_argument(
        "--config", default=str(REPO_ROOT / "config" / "modes" / "gello.yaml")
    )
    args = parser.parse_args()
    config = load_gello_config(args.config)
    failures = 0
    resolved = {}
    for side in ("left", "right"):
        selected = getattr(config, side)
        path = Path(selected.port)
        if not path.exists():
            print(f"[FAIL] {side}: missing {path}")
            failures += 1
            continue
        resolved[side] = path.resolve()
        if not os.access(path, os.R_OK | os.W_OK):
            device_group = grp.getgrgid(resolved[side].stat().st_gid)
            current_groups = set(os.getgroups())
            user = pwd.getpwuid(os.getuid()).pw_name
            stale_membership = (
                user in device_group.gr_mem and device_group.gr_gid not in current_groups
            )
            detail = (
                f"; {user} is configured in {device_group.gr_name}, but this "
                "login session is stale—log out completely and log in again"
                if stale_membership
                else f"; device group is {device_group.gr_name}"
            )
            print(f"[FAIL] {side}: not readable+writable: {path}{detail}")
            failures += 1
            continue
        print(
            f"[PASS] {side}: {selected.expected_serial} -> {resolved[side]} "
            f"(motors {config.joint_ids}, baud {config.baudrate})"
        )
    if len(resolved) == 2 and resolved["left"] == resolved["right"]:
        print("[FAIL] left and right resolve to the same character device")
        failures += 1
    if failures:
        print(f"GELLO preflight failed with {failures} issue(s).", file=sys.stderr)
        return 1
    print("GELLO port preflight passed. No motor or robot command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
