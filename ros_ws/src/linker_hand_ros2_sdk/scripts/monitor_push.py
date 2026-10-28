"""Hold a pose and stream realtime positions while a human pushes a finger.

    python3 monitor_push.py --device 1 --seconds 25

Purpose: a finger that holds its commanded pose but can be displaced by hand
has play somewhere. If the reported position moves while it is pushed, the
firmware sees the deflection (servo compliance / current limit). If the
finger visibly moves but the report stays flat, the play is mechanical,
after the encoder -- invisible to every readback.

Holds all fingers mid-curl (96 ticks on pitch/pip/dip, yaw mid, thumb
straight and out of the way), samples get_current_position as fast as it
answers for --seconds, then prints per-joint min/max/span and saves the
trace next to this script as push_trace.npz.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_contract import (  # noqa: E402
    O30I_DRIVER_TO_URDF, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES)
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())

POSE = {name: 96 for name in O30I_URDF_JOINT_NAMES}
for name in ("thumb_cmc_roll", "thumb_cmc_yaw", "thumb_mcp", "thumb_ip"):
    POSE[name] = 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=25.0)
    args = parser.parse_args()

    hand = o30i_control.LinkerHandO30IController(
        hand_type="right", canfd_device=args.device, channel=args.channel,
        comm_type="libcanbus")
    if not hand.is_connected:
        print("no transport; is the bridge still holding the bus?")
        return 1
    model = str(hand.get_product_model() or "")
    if "O30" not in model.upper():
        print(f"index {args.device} is not an O30i ({model!r})")
        hand.close()
        return 1
    try:
        hand.setup()
        time.sleep(0.2)
        driver_names = list(o30i_control.JOINT_NAMES)
        urdf_index = {n: i for i, n in enumerate(O30I_URDF_JOINT_NAMES)}
        vec = [POSE[O30I_DRIVER_TO_URDF[d]] for d in driver_names]
        hand.set_target_position(vec)
        time.sleep(1.5)

        print(f"holding; PUSH NOW -- sampling for {args.seconds:.0f} s", flush=True)
        rows, stamps = [], []
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            pos = hand.get_current_position()
            if pos is not None:
                by_urdf = [0] * len(O30I_URDF_JOINT_NAMES)
                for d, v in zip(driver_names, pos):
                    by_urdf[urdf_index[O30I_DRIVER_TO_URDF[d]]] = v
                rows.append(by_urdf)
                stamps.append(time.monotonic() - t0)
            time.sleep(0.01)
        data = np.array(rows)
        np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "push_trace.npz"),
                 positions=data, stamps=np.array(stamps),
                 commanded=[POSE[n] for n in O30I_URDF_JOINT_NAMES])
        print(f"{len(rows)} samples ({len(rows) / args.seconds:.0f} Hz)")
        print(f"  {'joint':<20}{'cmd':>5}{'min':>5}{'max':>5}{'span':>6}")
        for j, name in enumerate(O30I_URDF_JOINT_NAMES):
            span = data[:, j].max() - data[:, j].min()
            flag = "  <-- moved under push" if span > 5 else ""
            print(f"  {name:<20}{POSE[name]:>5}{data[:, j].min():>5}"
                  f"{data[:, j].max():>5}{span:>6}{flag}")
        return 0
    finally:
        try:
            # neutral (URDF 0 deg), not all-zero: yaw tick 0 is the lateral
            # extreme and the index's extreme presses the pinched cable.
            # Computed from the contract so limit updates cannot go stale.
            neutral = {
                name: round((min(max(0.0, lo), hi) - lo) / (hi - lo) * 255)
                for name, lo, hi in zip(
                    O30I_URDF_JOINT_NAMES, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER)}
            hand.set_target_position(
                [neutral[O30I_DRIVER_TO_URDF[d]] for d in driver_names])
            time.sleep(1.0)
        finally:
            hand.close()


if __name__ == "__main__":
    sys.exit(main())
