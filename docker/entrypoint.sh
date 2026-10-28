#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/vendor_ws/install/setup.bash
if [[ -f /workspace/franka_upper_body_teleop/ros_ws/install/setup.bash ]]; then
  source /workspace/franka_upper_body_teleop/ros_ws/install/setup.bash
fi

exec "$@"
