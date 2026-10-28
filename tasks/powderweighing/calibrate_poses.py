#!/usr/bin/env python3
"""Interactively tune and save right O30i pitch_ready and pitch ticks.

The normal hand-control container must be stopped because this program needs
exclusive CAN access.  It starts from the hand's reported position, never from
an assumed all-zero tick vector, and parks at neutral on every exit path.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import termios
import time
import tty
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SDK = ROOT / "ros_ws" / "src" / "linker_hand_ros2_sdk"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SDK))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_contract import (  # noqa: E402
    O30I_DRIVER_TO_URDF,
    O30I_RIGHT_LOWER,
    O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES,
)
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path,
    make_libcanbus_communication,
)
o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path()
)


def neutral_pose() -> dict[str, int]:
    result = {}
    for index, name in enumerate(O30I_URDF_JOINT_NAMES):
        lower, upper = O30I_RIGHT_LOWER[index], O30I_RIGHT_UPPER[index]
        q = min(max(0.0, lower), upper)
        result[name] = round((q - lower) / (upper - lower) * 255)
    return result


def driver_vector(pose: dict[str, int]) -> list[int]:
    return [
        int(pose[O30I_DRIVER_TO_URDF[name]])
        for name in o30i_control.JOINT_NAMES
    ]


def feedback_pose(hand) -> dict[str, int] | None:
    values = hand.get_current_position()
    if values is None or len(values) != len(o30i_control.JOINT_NAMES):
        return None
    return {
        O30I_DRIVER_TO_URDF[name]: int(round(value))
        for name, value in zip(o30i_control.JOINT_NAMES, values, strict=True)
    }


def parse_ticks(text: str) -> dict[str, int]:
    parts = [part for part in text.replace(",", " ").split() if part]
    if len(parts) != 20:
        raise ValueError(f"expected 20 ticks, got {len(parts)}")
    values = [int(float(part)) for part in parts]
    if any(value < 0 or value > 255 for value in values):
        raise ValueError("ticks must be within 0..255")
    return dict(zip(O30I_URDF_JOINT_NAMES, values, strict=True))


def save_config(path: Path, saved: dict[str, list[int]]) -> None:
    data = json.loads(path.read_text())
    data["poses_ticks"].update(saved)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)
    # Docker normally runs this script as root while the bind-mounted checkout
    # belongs to the host user.  Match the parent directory so the host-side
    # runner and git can read/edit the saved calibration afterwards.
    parent = path.parent.stat()
    try:
        os.chown(path, parent.st_uid, parent.st_gid)
    except PermissionError:
        pass
    os.chmod(path, 0o664)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=int, default=1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("config") / "poses.json",
    )
    parser.add_argument(
        "--seed-ticks",
        help="optional 20-value URDF-order seed; default is current feedback",
    )
    parser.add_argument(
        "--start-joint",
        default="thumb_cmc_roll",
        choices=sorted(O30I_URDF_JOINT_NAMES),
    )
    args = parser.parse_args()
    seed_override = None
    if args.seed_ticks:
        try:
            seed_override = parse_ticks(args.seed_ticks)
        except ValueError as error:
            parser.error(str(error))
    if not args.output.is_file():
        parser.error(f"configuration template does not exist: {args.output}")
    hand = o30i_control.LinkerHandO30IController(
        hand_type="right",
        canfd_device=args.device,
        channel=args.channel,
        frame_id=1,
        comm_type="libcanbus",
    )
    if not hand.is_connected:
        print("No transport. Stop hand-control; this tool needs exclusive CAN access.")
        return 1

    names = list(O30I_URDF_JOINT_NAMES)
    selected = names.index(args.start_joint)
    stages = ("pitch_ready", "pitch")
    stage_index = 0
    saved: dict[str, list[int]] = {}
    pose: dict[str, int] = {}
    stage_seed: dict[str, int] = {}

    def vector() -> list[int]:
        return [pose[name] for name in names]

    def send() -> None:
        hand.set_target_position(driver_vector(pose))

    def print_vector(label: str) -> None:
        print(f"\r\n  {label} ticks (URDF order):\r")
        print("    " + ",".join(str(value) for value in vector()) + "\r")

    def status() -> None:
        name = names[selected]
        print(
            f"\r  [{stage_index + 1}/2 {stages[stage_index]:<11}] "
            f"[{selected + 1:2}/20] {name:<20} tick {pose[name]:>3}      ",
            end="",
        )
        sys.stdout.flush()

    try:
        hand.setup()
        time.sleep(0.3)
        measured = feedback_pose(hand)
        if seed_override is not None:
            pose = seed_override.copy()
        elif measured is not None:
            pose = measured.copy()
        else:
            pose = neutral_pose()
            print("WARNING: no feedback; using URDF neutral as the seed.")
        stage_seed = pose.copy()

        print("\nThis will tune the RIGHT O30i and save two hardcoded poses.")
        print("hand-control must be stopped. Keep the workspace clear.")
        input("Press Enter to hold the current seed pose, or Ctrl-C to abort: ")
        send()
        time.sleep(0.8)
        print(
            "\nKeys: ,/. previous/next joint; -/= change 1 tick; "
            "_/+ change 5; p print; m measure; r reset this stage; "
            "s save/next; q abort."
        )
        status()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while True:
                key = sys.stdin.read(1)
                if key in (",", "["):
                    selected = (selected - 1) % len(names)
                elif key in (".", "]"):
                    selected = (selected + 1) % len(names)
                elif key in ("-", "=", "_", "+"):
                    step = {"-": -1, "=": 1, "_": -5, "+": 5}[key]
                    name = names[selected]
                    pose[name] = min(255, max(0, pose[name] + step))
                    send()
                elif key == "r":
                    pose = stage_seed.copy()
                    send()
                elif key == "p":
                    print_vector(stages[stage_index])
                elif key == "m":
                    time.sleep(0.2)
                    measured = feedback_pose(hand)
                    if measured is None:
                        print("\r\n  no position feedback\r")
                    else:
                        print(f"\r\n  {'joint':<20}{'cmd':>6}{'meas':>7}{'diff':>7}\r")
                        for name in names:
                            print(
                                f"  {name:<20}{pose[name]:>6}{measured[name]:>7}"
                                f"{measured[name] - pose[name]:>+7}\r"
                            )
                elif key == "s":
                    label = stages[stage_index]
                    saved[label] = vector()
                    print_vector(f"saved {label}")
                    if stage_index == len(stages) - 1:
                        break
                    stage_index += 1
                    stage_seed = pose.copy()
                    print("\r\n  Now tune pitch, starting from pitch_ready.\r")
                elif key == "q":
                    print("\r\nAborted; poses.json was not changed.\r")
                    return 2
                status()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

        save_config(args.output, saved)
        print(f"\nSaved both poses to {args.output}")
        return 0
    except KeyboardInterrupt:
        print("\nAborted; poses.json was not changed.")
        return 130
    finally:
        try:
            hand.set_target_position(driver_vector(neutral_pose()))
            time.sleep(1.0)
        finally:
            hand.close()


if __name__ == "__main__":
    raise SystemExit(main())
