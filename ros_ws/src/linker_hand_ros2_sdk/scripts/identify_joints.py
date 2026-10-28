"""Move ONE joint at a time and see which part of the real hand responds.

    # the bridge must be stopped: this needs exclusive CAN access
    python3 identify_joints.py --device 1 --joint index_pip
    python3 identify_joints.py --device 1 --joint index_pip --hold 200

THIS MOVES THE HAND, one joint, in steps you confirm. `q` or Ctrl-C returns it
to the open pose.

WHAT IS ACTUALLY UNKNOWN HERE

Three things about this hand have been assumed and never checked, and all three
produce the same symptom -- the model showing a finger curled where the metal is
straight:

1. That the vendor's joint names mean what the URDF's do. The device calls the
   index joints 指根1, 指根2, 指尖 -- "finger base 1", "finger base 2",
   "fingertip". The driver maps 指根2 to the PIP, but a second joint called
   "base" could as easily be a second base degree of freedom or a linkage, and
   the mapping was never demonstrated.

2. That each joint is independently actuated in free space. Many hands couple or
   underactuate the distal joints so they only curl against resistance.

3. That a tick means the angle the URDF says. MI 0x43 reports the position
   channel's unit as the literal string "normalization", so the firmware holds
   no angular quantity and cannot settle this.

None of the three can be separated in software. Command and feedback both pass
through the same assumed map, so they agree with each other no matter what the
metal does: on a real capture every joint's measured range equalled its
commanded range to a tenth of a degree while the fingers sat visibly straight.
That equality says the MOTOR moved. It does not say the joint did.

So this drives one joint and asks you what moved. Watch for three things and
report all of them, because they distinguish the cases:

  - WHICH physical joint moves (base, middle, or tip knuckle)?
  - Does it move over the WHOLE sweep, or only part of it?
  - Roughly how far does it travel at the end -- straight, half bent, fully?

A joint that reads 0 to 93 degrees here while the metal barely leaves straight
is case 2 or 3; one where the wrong knuckle moves is case 1.
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

# Everything straight and slightly spread, so one moving joint is unmistakable
# and nothing can collide with the thumb while it moves.
NEUTRAL = {name: 0 for name in O30I_URDF_JOINT_NAMES}
NEUTRAL["thumb_cmc_roll"] = 0
NEUTRAL["thumb_cmc_yaw"] = 0


def driver_vector(by_urdf_name: dict[str, int]) -> list[int]:
    names = list(o30i_control.JOINT_NAMES)
    return [int(by_urdf_name[O30I_DRIVER_TO_URDF[d]]) for d in names]


def urdf_degrees(name: str, tick: int) -> float:
    import math
    index = O30I_URDF_JOINT_NAMES.index(name)
    lower, upper = O30I_RIGHT_LOWER[index], O30I_RIGHT_UPPER[index]
    return math.degrees(lower + (tick / 255.0) * (upper - lower))


def vendor_name_for(urdf_name: str) -> str:
    for driver, urdf in O30I_DRIVER_TO_URDF.items():
        if urdf == urdf_name:
            return driver
    return "?"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--joint", default="index_pip",
                        choices=sorted(O30I_URDF_JOINT_NAMES))
    parser.add_argument("--step", type=int, default=32, help="ticks per step")
    parser.add_argument("--hold", type=int, default=None,
                        help="skip the sweep, just hold this tick and exit on Enter")
    args = parser.parse_args()

    hand = o30i_control.LinkerHandO30IController(
        hand_type="right", canfd_device=args.device, channel=args.channel,
        comm_type="libcanbus")
    if not hand.is_connected:
        print("  no transport; is the bridge still holding the bus?")
        return 1
    model = str(hand.get_product_model() or "")
    if "O30" not in model.upper():
        print(f"  index {args.device} is not an O30i ({model!r}); "
              f"run identify_canfd_devices.py")
        hand.close()
        return 1

    vendor = vendor_name_for(args.joint)
    print(f"\n  {model} {hand.get_hand_side()}  |  driving URDF {args.joint!r} "
          f"= vendor {vendor!r}")
    print(f"  URDF says this joint spans "
          f"{urdf_degrees(args.joint, 0):.1f} to {urdf_degrees(args.joint, 255):.1f} deg\n")

    pose = dict(NEUTRAL)

    def send(ticks: dict[str, int]) -> None:
        hand.set_target_position(driver_vector(ticks))

    def measured() -> int | None:
        position = hand.get_current_position()
        if position is None:
            return None
        for driver, value in zip(list(o30i_control.JOINT_NAMES), position):
            if O30I_DRIVER_TO_URDF[driver] == args.joint:
                return value
        return None

    try:
        hand.setup()
        time.sleep(0.2)
        print("  moving everything to STRAIGHT; check the hand is fully open")
        send(pose)
        time.sleep(1.5)
        input("  press Enter when the hand is open and you are watching "
              f"the {args.joint.split('_')[0]} finger ")

        if args.hold is not None:
            tick = max(0, min(255, args.hold))
            pose[args.joint] = tick
            send(pose)
            time.sleep(1.0)
            print(f"  holding {args.joint} at tick {tick} "
                  f"(URDF: {urdf_degrees(args.joint, tick):.1f} deg), "
                  f"measured {measured()}")
            input("  press Enter to release ")
            return 0

        print(f"\n    {'tick':>6}{'measured':>10}{'URDF deg':>10}")
        for tick in list(range(0, 256, args.step)) + [255]:
            pose[args.joint] = tick
            send(pose)
            time.sleep(0.6)
            print(f"    {tick:>6}{str(measured()):>10}"
                  f"{urdf_degrees(args.joint, tick):>10.1f}")
            reply = input("      [Enter]=next  q=stop > ").strip().lower()
            if reply == "q":
                break
        print("\n  Report three things: which knuckle moved, whether it moved "
              "over the whole sweep, and how far it got at the end.")
        return 0
    except KeyboardInterrupt:
        print("\n  interrupted")
        return 130
    finally:
        try:
            print("  returning to straight")
            send(NEUTRAL)
            time.sleep(1.0)
        finally:
            hand.close()


if __name__ == "__main__":
    sys.exit(main())
