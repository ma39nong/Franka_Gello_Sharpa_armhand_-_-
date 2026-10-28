"""Transport selection for the preserved O30i HOP controller."""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import platform
import threading
import time
from pathlib import Path
from types import ModuleType
from typing import Any


def bundled_libcanbus_path() -> Path:
    """Resolve the packaged vendor USB-CANFD runtime for this machine."""
    override = os.environ.get("LINKER_HAND_LIBCANBUS")
    if override:
        candidate = Path(override).expanduser().resolve()
    else:
        platform_name = {
            "x86_64": "linux-x86_64-ubuntu22",
            "amd64": "linux-x86_64-ubuntu22",
        }.get(platform.machine().lower())
        if platform_name is None:
            raise RuntimeError(
                f"no packaged O30i libcanbus runtime for {platform.machine()!r}"
            )
        candidate = (
            Path(__file__).resolve().parent
            / "LinkerHand"
            / "lib"
            / platform_name
            / "libcanbus.so"
        )
    if not candidate.is_file():
        raise FileNotFoundError(f"O30i libcanbus runtime not found: {candidate}")
    return candidate


def make_libcanbus_communication(
    sdk: ModuleType, library_path: Path
) -> type[Any]:
    """Bind the controller's vendor transport to a packaged USB runtime."""

    class BundledCANFDCommunication(sdk.CANFDCommunication):
        def initialize(self) -> bool:
            try:
                usb = ctypes.util.find_library("usb-1.0")
                if usb:
                    ctypes.CDLL(usb, mode=ctypes.RTLD_GLOBAL)
                self.canDLL = ctypes.CDLL(str(library_path))
                if self.canDLL.CAN_ScanDevice() <= 0:
                    return False
                result = self.canDLL.CAN_OpenDevice(
                    self.canfd_device, self.channel
                )
                if result != sdk.STATUS_OK:
                    return False
                config = sdk.CanFD_Config(
                    1_000_000,
                    5_000_000,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0x04,
                    0x0,
                    0x1,
                )
                result = self.canDLL.CANFD_Init(
                    self.canfd_device,
                    self.channel,
                    ctypes.byref(config),
                )
                if result != sdk.STATUS_OK:
                    self.canDLL.CAN_CloseDevice(
                        self.canfd_device, self.channel
                    )
                    return False
                result = self.canDLL.CAN_SetFilter(
                    self.canfd_device,
                    self.channel,
                    0,
                    0,
                    0,
                    0,
                    1,
                )
                if result != sdk.STATUS_OK:
                    self.canDLL.CAN_CloseDevice(
                        self.canfd_device, self.channel
                    )
                    return False
                self.is_connected = True
                self.running = True
                self._thread = threading.Thread(
                    target=self._receive_loop,
                    daemon=True,
                )
                self._thread.start()
                time.sleep(0.05)
                return self._thread.is_alive()
            except (OSError, AttributeError):
                self.is_connected = False
                return False

    BundledCANFDCommunication.__name__ = "BundledCANFDCommunication"
    return BundledCANFDCommunication
