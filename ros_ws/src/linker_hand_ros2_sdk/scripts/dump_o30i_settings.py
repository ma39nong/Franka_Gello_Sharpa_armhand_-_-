"""Read the O30i's live motion settings and its own unit/range declaration.

    python3 dump_o30i_settings.py --device 1

Read-only: every access is a HOP read request. No joint is enabled, no
position, torque or configuration is written, nothing is saved to flash.

WHY. Two questions came out of a following failure and neither can be answered
from the driver's own logs.

1. What are the hand's motion parameters? The bridge sets speed and torque
   limits for the G20 and NOTHING for the O30i (`_no_startup_settings` in
   linker_hand_bridge/profiles.py), so the O30i runs on whatever it powers up
   with. Before choosing values, read the ones in force.

2. Is the tick-to-angle map right? `o30_tick_at_lower/upper` default to
   0..255 spanning each URDF joint limit, and nothing has ever checked that
   against the hardware. A wrong map is invisible in command-versus-measured
   comparisons -- both sides use it -- so it has to come from the device.
   MI 0x43 (UNIT_RANGE, "单位与量程") is the hand's own declaration; if it
   carries per-joint ranges, it settles the question without a ruler.

Anything this prints as None means the device did not answer that request,
which is information too: it means the field cannot be trusted as configured.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())

MI = o30i_control.MI
JOINT_NAMES = o30i_control.JOINT_NAMES


def show_vector(label: str, values, names=None) -> None:
    if values is None:
        print(f"  {label:<22} (no answer)")
        return
    if names is None:
        print(f"  {label:<22} {list(values)}")
        return
    print(f"  {label}")
    for name, value in zip(names, values):
        print(f"      {name:<16}{value}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=1,
                        help="canfd device index (identify_canfd_devices.py finds it)")
    parser.add_argument("--channel", type=int, default=0)
    args = parser.parse_args()

    hand = o30i_control.LinkerHandO30IController(
        hand_type="right", canfd_device=args.device, channel=args.channel,
        comm_type="libcanbus")
    try:
        if not hand.is_connected:
            print("  no transport")
            return 1
        model = hand.get_product_model()
        print(f"\n  device: {model!r} {hand.get_hand_side()!r} "
              f"uid={hand.get_device_uid()!r}")
        span = hand.phys_span

        print("\n  --- motion settings currently in force ---")
        for label, mi in (("velocity (MI 0x02)", MI.VELOCITY),
                          ("accel (0x03)", MI.ACCEL),
                          ("current (0x04)", MI.CURRENT),
                          ("torque (0x06)", MI.TORQUE),
                          ("move_time (0x08)", MI.MOVE_TIME),
                          ("stall_time (0x09)", MI.STALL_TIME),
                          ("stall_thresh (0x0A)", MI.STALL_THRESH),
                          ("stall_current (0x0B)", MI.STALL_CURRENT)):
            body = hand._request(mi, 0x00, span)
            show_vector(label, list(body) if body else None)

        print("\n  --- position loop gains ---")
        for label, mi in (("pos P (0x21)", MI.POS_PID_P),
                          ("pos I (0x22)", MI.POS_PID_I),
                          ("pos D (0x23)", MI.POS_PID_D)):
            body = hand._request(mi, 0x00, span)
            show_vector(label, list(body) if body else None)

        print("\n  --- unit and range declaration (MI 0x43) ---")
        body = hand._request(MI.UNIT_RANGE, 0x00, 162)
        if body is None:
            print("  (no answer -- the tick-to-angle map cannot be confirmed "
                  "from the device; it needs an external measurement)")
        else:
            print(f"  {len(body)} bytes:")
            for offset in range(0, len(body), 16):
                chunk = body[offset:offset + 16]
                hexed = " ".join(f"{b:02x}" for b in chunk)
                ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
                print(f"    {offset:04x}  {hexed:<48}  {ascii_}")

        print("\n  --- where the joints are sitting right now (ticks) ---")
        show_vector("position", hand.get_current_position(), JOINT_NAMES)
        return 0
    finally:
        try:
            hand.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
