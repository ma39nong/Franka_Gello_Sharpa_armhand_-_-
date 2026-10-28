"""Ask every CANFD adapter what hand is behind it.

    python3 identify_canfd_devices.py            # probe indices 0..3
    python3 identify_canfd_devices.py --max 8

Read-only: it opens each adapter, reads the product-info strings, and closes.
No joint is enabled and no motion command is sent.

WHY THIS EXISTS. This host has two physically identical CANFD analysers
(a8fa:8598), and the driver selects one by integer index (`o30_canfd_device`).
Nothing pins index 0 to a particular adapter -- the order comes from USB
enumeration, so replugging a hand, or swapping one for another, can silently
move the O30i to the other index.

The failure that produces is confusing rather than obvious, because two of the
driver's startup messages do NOT verify the hardware:

  "有效关节 20 个"  -- ACTIVE_JOINTS is a static list in o30i_control.py;
                      NUM_JOINTS is the constant 20 whatever is attached.
  "初始化完成"      -- the CANFD transport opened, which any adapter will do.

The first message that actually checks is the product-info read, and on the
wrong adapter it comes back empty:

  RuntimeError: connected device is not O30i: ''

which reads like a dead hand and is really a wrong index. This script tells the
two apart in one shot, and prints the USB serial beside each index so the
mapping can be recorded rather than rediscovered.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

# o30i_control.CANFDCommunication.initialize() dlopens
# /usr/local/lib/libcanbus.so and /usr/local/lib/libusb-1.0.so, neither of which
# exists on this host or in the container image. The driver never uses that
# path: o30i_node swaps the class for one bound to the runtime bundled in
# LinkerHand/lib/. Do the same here, or the probe reports "no transport" on
# every index and looks exactly like an unplugged hand.
o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())
O30IController = o30i_control.LinkerHandO30IController


def usb_adapters() -> list[tuple[str, str]]:
    """(sysfs name, serial) for every a8fa:8598 adapter, in sysfs order."""
    found = []
    for path in sorted(glob.glob("/sys/bus/usb/devices/*")):
        try:
            with open(os.path.join(path, "idVendor")) as handle:
                if handle.read().strip() != "a8fa":
                    continue
            serial = "(none)"
            serial_path = os.path.join(path, "serial")
            if os.path.exists(serial_path):
                with open(serial_path) as handle:
                    serial = handle.read().strip()
            found.append((os.path.basename(path), serial))
        except OSError:
            continue
    return found


def probe(index: int) -> dict:
    """Try the right request ID (0x01), then the left (0x02).

    A hand answers only its own device ID, so probing with one side's ID
    reports a healthy other-side hand as silence -- exactly what happened
    when the left mount became an O30i and index 0 looked dead.
    """
    last = {"index": index, "status": "no transport"}
    for hand_type, frame_id in (("right", 1), ("left", 2)):
        controller = None
        try:
            controller = O30IController(
                hand_type=hand_type, canfd_device=index,
                frame_id=frame_id, comm_type="libcanbus")
            if not controller.is_connected:
                return {"index": index, "status": "no transport"}
            model = str(controller.get_product_model() or "")
            side = str(controller.get_hand_side() or "")
            uid = str(controller.get_device_uid() or "")
            if model:
                return {"index": index, "status": "OK", "model": model,
                        "side": side, "uid": uid}
            last = {"index": index,
                    "status": "opened, but no product info on either request "
                              "ID (nothing answering on this bus)"}
        except Exception as error:  # noqa: BLE001 - survive any adapter
            last = {"index": index,
                    "status": f"error: {type(error).__name__}: {error}"}
        finally:
            if controller is not None:
                try:
                    controller.close()
                except Exception:  # noqa: BLE001
                    pass
    return last


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--max", type=int, default=4,
                        help="highest device index to probe (default 4)")
    args = parser.parse_args()

    adapters = usb_adapters()
    print(f"\n  {len(adapters)} CANFD adapter(s) present on USB:")
    for name, serial in adapters:
        print(f"    {name:<10} serial={serial}")
    if len(adapters) > 1:
        print("    NOTE identical adapters: the index below is USB enumeration "
              "order and is not stable across replugs.")

    print("\n  probing device indices (read-only):")
    results = []
    for index in range(args.max):
        result = probe(index)
        results.append(result)
        detail = result.get("model", "")
        if detail:
            detail = (f"model={result['model']!r} side={result['side']!r} "
                      f"uid={result['uid']!r}")
        print(f"    index {index}: {result['status']}  {detail}")

    matches = [r for r in results if "O30" in r.get("model", "").upper()]
    print()
    if not matches:
        print("  no O30i found on any index. Is the hand powered and cabled?")
        return 1
    for result in matches:
        side = str(result["side"]).strip().lower() or "unknown"
        print(f"  O30i is at index {result['index']} "
              f"({result['side']}). Launch with "
              f"o30_canfd_device_{side}:={result['index']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
