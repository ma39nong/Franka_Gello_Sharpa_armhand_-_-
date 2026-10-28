#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
docker compose --file "${repo}/docker/compose.yaml" run --rm tools \
  bash -lc '
    export PYTHONPATH=/workspace/franka_upper_body_teleop/ros_ws/src/teleop_data:/workspace/franka_upper_body_teleop/ros_ws/src/teleop_core
    exec /opt/lerobot_venv/bin/python -m teleop_data.export_lerobot "$@"
  ' bash "$@"
