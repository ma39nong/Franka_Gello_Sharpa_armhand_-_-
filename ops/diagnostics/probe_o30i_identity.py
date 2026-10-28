#!/usr/bin/env python3
"""Read-only O30i identity probe over the libcanbus CANFD adapter.

Tries each (device, channel) combination and reports what answers the
model/uid/side queries. Sends only query frames, never joint commands.
Run inside the hand-control container while the O30i driver is NOT
holding the adapter:

    docker compose run --rm hand-control python3 \
        /workspace/franka_upper_body_teleop/ops/diagnostics/probe_o30i_identity.py
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "linker_hand_ros2_sdk"))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path,
    make_libcanbus_communication,
)


def main() -> int:
    o30i_control.CANFDCommunication = make_libcanbus_communication(
        o30i_control, bundled_libcanbus_path()
    )
    found = False
    for device in (0, 1):
        for channel in (0, 1):
            label = f"device={device} channel={channel}"
            try:
                controller = o30i_control.LinkerHandO30IController(
                    hand_type="right",
                    canfd_device=device,
                    frame_id=1,
                    comm_type="libcanbus",
                    channel=channel,
                )
            except Exception as error:  # noqa: BLE001 - report and move on
                print(f"{label}: open failed: {error}", flush=True)
                continue
            try:
                if not controller.is_connected:
                    print(f"{label}: transport did not connect", flush=True)
                    continue
                model = str(controller.get_product_model() or "")
                uid = str(controller.get_device_uid() or "")
                side = str(controller.get_hand_side() or "")
                if model:
                    found = True
                    print(
                        f"{label}: model={model!r} side={side!r} uid={uid!r}",
                        flush=True,
                    )
                else:
                    print(f"{label}: opened, but no device answered", flush=True)
            finally:
                try:
                    controller.close()
                except Exception:  # noqa: BLE001
                    pass
    if not found:
        print(
            "No O30i answered on any channel - check hand power and the "
            "CANFD cable into the a8fa:8598 analyser.",
            flush=True,
        )
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
