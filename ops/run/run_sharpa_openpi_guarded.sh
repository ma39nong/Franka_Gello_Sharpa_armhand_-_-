#!/usr/bin/env bash
# Explicit, supervised OpenPI hand host. Never subscribes to public command topics.
set -euo pipefail
[[ $# -eq 1 ]] || { echo "usage: $0 PRIVATE_FULL_CONFIG_JSON" >&2; exit 2; }
FULL_CONFIG="$1"
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
LITCHI="$REPO/portable_deps/litchi_hardware"
[[ -f "$LITCHI/ros2/install/setup.bash" ]] || { echo "ROS2 overlay missing" >&2; exit 2; }
mapfile -t BINDING < <(PYTHONPATH="$REPO" "$LITCHI/.pixi/envs/ros2/bin/python" - "$FULL_CONFIG" <<'PY'
import sys
from adapters.litchibot.full_control import load_full_config
config=load_full_config(sys.argv[1])
if config['source']!='openpi' or config['mode']!='supervised_hardware_validation':
    raise SystemExit('OpenPI requires a supervised full config and permit')
print(config['policy']['serials']['left'])
print(config['policy']['serials']['right'])
PY
)
[[ ${#BINDING[@]} -eq 2 && -n ${BINDING[0]} && -n ${BINDING[1]} ]] || {
  echo "Invalid OpenPI full config or device binding" >&2; exit 2;
}
ARMS_DOMAIN="$(awk -F= '$1 == "TELEOP_ROS_DOMAIN_ID" {print $2; exit}' "$REPO/docker/.env")"
[[ "$ARMS_DOMAIN" =~ ^[0-9]+$ ]] || { echo "Missing ROS domain" >&2; exit 2; }
export ROS_DOMAIN_ID="$ARMS_DOMAIN" ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$REPO/config/cyclonedds.xml"
cd "$LITCHI"
export OPENPI_GELLO_REPO="$REPO" OPENPI_FULL_CONFIG="$FULL_CONFIG"
export OPENPI_LEFT_SERIAL="${BINDING[0]}" OPENPI_RIGHT_SERIAL="${BINDING[1]}"
exec pixi run -e ros2 bash -lc '
  source "$OPENPI_GELLO_REPO/portable_deps/litchi_hardware/ros2/install/setup.bash"
  export PYTHONPATH="$OPENPI_GELLO_REPO:${PYTHONPATH:-}"
  exec python "$OPENPI_GELLO_REPO/adapters/litchibot/validation_driver.py" \
    --full-config "$OPENPI_FULL_CONFIG" --ros-args -r __node:=sharpa_driver \
    -p use_fake_hardware:=false -p enable_left:=true -p enable_right:=true \
    -p "left_serial:=$OPENPI_LEFT_SERIAL" -p "right_serial:=$OPENPI_RIGHT_SERIAL" \
    -p stop_on_command_timeout_s:=0.2 -p speed_coefficient:=0.3 \
    -p current_coefficient:=0.6 -p state_rate_hz:=30.0 \
    -p publish_tactile:=false -p tactile_poll_rate_hz:=60.0
'
