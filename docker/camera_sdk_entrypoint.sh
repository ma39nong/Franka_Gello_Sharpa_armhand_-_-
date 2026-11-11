#!/usr/bin/env bash
set -euo pipefail

# Source the existing ROS overlays before applying the camera-only SDK override.
# Robot/hand services continue to use the original shared runtime image.
exec /entrypoint.sh bash -e -c '
  export LD_LIBRARY_PATH="/opt/teleop-orbbec-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  python3 - <<\PY
import ctypes
import os
from pathlib import Path

sdk = ctypes.CDLL("libOrbbecSDK.so.2")
version = tuple(getattr(sdk, "ob_get_" + part + "_version")() for part in ("major", "minor", "patch"))
if version != (2, 10, 6):
    raise RuntimeError(f"Camera SDK mismatch: {version}, expected 2.10.6")
driver = Path("/workspace/franka_upper_body_teleop/ros_ws/install/orbbec_camera/lib/liborbbec_camera.so")
if driver.exists():
    ctypes.CDLL(str(driver), mode=os.RTLD_NOW)
print("[camera-sdk] Orbbec SDK 2.10.6; existing ROS camera driver retained", flush=True)
PY
  exec "$@"
' camera-sdk "$@"
