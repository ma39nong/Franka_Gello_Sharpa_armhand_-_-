#!/usr/bin/env bash
# Container-side supervisor for the read-only collection processes.
set -euo pipefail

REPO_ROOT=/workspace/franka_upper_body_teleop
receiver_pid=""
camera_pid=""

pid_alive() {
  [[ -n "$1" ]] && kill -0 -- "$1" 2>/dev/null
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "$camera_pid" "$receiver_pid"; do
    if pid_alive "$pid"; then
      kill -INT -- "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$camera_pid" "$receiver_pid"; do
    [[ -z "$pid" ]] || wait "$pid" 2>/dev/null || true
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

: "${COLLECTION_OUTPUT_DIR:?}"
: "${COLLECTION_CAMERA_CONFIG:?}"
: "${COLLECTION_WORKCELL_HASH:?}"
: "${COLLECTION_CAM0_SERIAL:?}"
: "${COLLECTION_CAM0_SEMANTIC:?}"
: "${COLLECTION_CAM1_SERIAL:?}"
: "${COLLECTION_CAM1_SEMANTIC:?}"
: "${COLLECTION_CAM2_SERIAL:?}"
: "${COLLECTION_CAM2_SEMANTIC:?}"
: "${ORBBEC_SDK_LICENSE_ACCEPTED:?}"

ros2 run teleop_hand_telemetry hand_telemetry_receiver --ros-args \
  -p bind_host:=127.0.0.1 -p bind_port:=5602 -p max_staleness_ms:=150.0 &
receiver_pid=$!

ros2 launch teleop_camera_bringup triple_camera.launch.py \
  deployment_config:="$COLLECTION_CAMERA_CONFIG" &
camera_pid=$!

sleep 1
pid_alive "$receiver_pid" || {
  echo "Hand telemetry receiver exited during startup" >&2
  exit 1
}
pid_alive "$camera_pid" || {
  echo "Camera bringup exited during startup" >&2
  exit 1
}

ros2 run teleop_data_collector rosbag_data_collector --ros-args \
  --params-file "$REPO_ROOT/data_collection/config/record_gello.yaml" \
  -p output_dir:="$COLLECTION_OUTPUT_DIR" \
  -p quality_dir:="/collection_data/数据分类" \
  -p workcell_config_hash:="$COLLECTION_WORKCELL_HASH" \
  -p device_identities.cam0_serial:="$COLLECTION_CAM0_SERIAL" \
  -p device_identities.cam0_semantic:="$COLLECTION_CAM0_SEMANTIC" \
  -p device_identities.cam1_serial:="$COLLECTION_CAM1_SERIAL" \
  -p device_identities.cam1_semantic:="$COLLECTION_CAM1_SEMANTIC" \
  -p device_identities.cam2_serial:="$COLLECTION_CAM2_SERIAL" \
  -p device_identities.cam2_semantic:="$COLLECTION_CAM2_SEMANTIC"
