"""Park the hand at a NEUTRAL pose and leave it there (no return-to-zero).

    python3 park_neutral.py --device 1

Neutral = every joint at URDF 0 deg. On the yaw/roll joints tick 0 is NOT
neutral -- it is the lateral extreme (index -22.9 deg), which splays the
fingers and, on the index, presses the pinched cable. hold_pose.py's
"straight" (all ticks 0) has exactly that problem, so parking uses this
instead: index_yaw 233, middle_yaw 223, ring_yaw 152, little_yaw 127,
everything else 0.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_contract import (  # noqa: E402
    O30I_DRIVER_TO_URDF, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES)
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())

# computed from the contract's ranges so a URDF/limits update cannot leave
# stale hardcoded ticks behind (bitten once: 0706 -> 0803 changed them all)
NEUTRAL = {
    name: round((min(max(0.0, lo), hi) - lo) / (hi - lo) * 255)
    for name, lo, hi in zip(
        O30I_URDF_JOINT_NAMES, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER)
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--device", type=int, default=None,
                        help="CANFD adapter index; defaults right=1, left=0")
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--yaws", default=None,
                        help="index,middle,ring,pinky yaw ticks overriding the "
                             "URDF-neutral 233,223,152,127 -- for finding the "
                             "METAL's parallel by eye")
    args = parser.parse_args()
    if args.yaws:
        vals = [int(v) for v in args.yaws.replace(",", " ").split()]
        if len(vals) != 4 or any(not 0 <= v <= 255 for v in vals):
            raise SystemExit("--yaws needs 4 ticks 0..255")
        for name, v in zip(("index_mcp_roll", "middle_mcp_roll",
                            "ring_mcp_roll", "pinky_mcp_roll"), vals):
            NEUTRAL[name] = v

    if args.device is None:
        args.device = 1 if args.side == "right" else 0
    hand = o30i_control.LinkerHandO30IController(
        hand_type=args.side, canfd_device=args.device, channel=args.channel,
        frame_id=1 if args.side == "right" else 2,
        comm_type="libcanbus")
    if not hand.is_connected:
        print("no transport; is the bridge still holding the bus?")
        return 1
    try:
        model = str(hand.get_product_model() or "")
        if "O30" not in model.upper():
            print(f"index {args.device} is not an O30i ({model!r})")
            return 1
        hand.setup()
        time.sleep(0.2)
        driver_names = list(o30i_control.JOINT_NAMES)
        vec = [NEUTRAL[O30I_DRIVER_TO_URDF[d]] for d in driver_names]
        hand.set_target_position(vec)
        time.sleep(2.0)
        pos = hand.get_current_position()
        if pos is not None:
            urdf_index = {n: i for i, n in enumerate(O30I_URDF_JOINT_NAMES)}
            print(f"  {'joint':<20}{'cmd':>5}{'reported':>9}")
            for d, v in zip(driver_names, pos):
                n = O30I_DRIVER_TO_URDF[d]
                print(f"  {n:<20}{NEUTRAL[n]:>5}{v:>9}")
        print("parked at neutral; leaving it there.")
        return 0
    finally:
        hand.close()


if __name__ == "__main__":
    sys.exit(main())
