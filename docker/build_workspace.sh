#!/usr/bin/env bash
set -eo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source /opt/vendor_ws/install/setup.bash
set -u
cd "${repo}/ros_ws"
cmake_args=(-DCMAKE_BUILD_TYPE=Release)
if [[ "${LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME:-}" == "ON" ]]; then
  cmake_args+=(-DLIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME=ON)
fi
colcon build --symlink-install --cmake-args "${cmake_args[@]}" "$@"
