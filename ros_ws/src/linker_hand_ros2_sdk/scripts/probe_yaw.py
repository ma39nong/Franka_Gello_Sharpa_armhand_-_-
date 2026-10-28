"""Diagnose the two yaw joints that do not reach their commanded ticks.

Read-only part: per-joint capability/enable masks (existence, controllable,
enabled, feedback), current straight-pose positions.

Motion part: command index_yaw (si 0x06) alone to a few ticks; after each,
read BOTH the realtime position (RTS=0) and the firmware's accepted setpoint
(RTS=1). Setpoint==command but position~0 means the motor does not go;
setpoint!=command means the firmware rejected or clamped the command.
Also probes little_yaw (si 0x09) at one tick. Ends straight.
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())


def show(label, values, names):
    if values is None:
        print(f"  {label:<10} (no answer)")
        return
    print(f"  {label}:")
    for n, v in zip(names, values):
        print(f"    {n:<16}{v}")


def main() -> int:
    hand = o30i_control.LinkerHandO30IController(
        hand_type="right", canfd_device=1, channel=0, comm_type="libcanbus")
    if not hand.is_connected:
        print("no transport")
        return 1
    try:
        model = str(hand.get_product_model() or "")
        if "O30" not in model.upper():
            print(f"index 1 is not an O30i ({model!r})")
            return 1
        hand.setup()
        time.sleep(0.2)
        names = list(hand.joint_names)

        print("\n--- capability masks (get_joint_fault / get_joint_enable) ---")
        show("fault", hand.get_joint_fault(), names)
        show("enable", hand.get_joint_enable(), names)

        n = hand.num_joints
        straight = [0] * n
        i_index = names.index("index_yaw")
        i_little = names.index("little_yaw")
        i_middle = names.index("middle_yaw")
        i_ring = names.index("ring_yaw")

        hand.set_target_position(straight)
        time.sleep(1.2)

        def probe(joint_idx, tick):
            vec = list(straight)
            vec[joint_idx] = tick
            hand.set_target_position(vec)
            time.sleep(1.5)
            pos = hand.get_current_position()
            tgt = hand.get_target_position()
            name = names[joint_idx]
            print(f"  {name:<12} cmd={tick:>3}  "
                  f"setpoint={'?' if tgt is None else tgt[joint_idx]:>3}  "
                  f"realtime={'?' if pos is None else pos[joint_idx]:>3}")

        print("\n--- index_yaw sweep, everything else straight ---")
        for tick in (32, 64, 96, 128, 178, 255):
            probe(i_index, tick)

        print("\n--- the other finger yaws ---")
        probe(i_little, 96)
        probe(i_little, 255)
        probe(i_middle, 96)
        probe(i_middle, 178)
        probe(i_ring, 96)
        probe(i_ring, 178)

        return 0
    finally:
        try:
            hand.set_target_position([0] * hand.num_joints)
            time.sleep(1.0)
        finally:
            hand.close()


if __name__ == "__main__":
    sys.exit(main())
