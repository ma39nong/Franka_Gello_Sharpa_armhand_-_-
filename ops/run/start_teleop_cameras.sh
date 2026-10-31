#!/usr/bin/env bash
# Shared, camera-only owner. Safe to run without powered arms/hands.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
config="${1:-$REPO_ROOT/data_collection/config/cameras.yaml}"
export TELEOP_CAMERA_CONFIG="$(realpath -- "$config")"
[[ -f "$TELEOP_CAMERA_CONFIG" ]] || { echo "Missing camera config: $config" >&2; exit 2; }
export ORBBEC_SDK_LICENSE_ACCEPTED="${ORBBEC_SDK_LICENSE_ACCEPTED:-$(deployment_env_value "$REPO_ROOT/docker/.env" ORBBEC_SDK_LICENSE_ACCEPTED)}"
[[ "$ORBBEC_SDK_LICENSE_ACCEPTED" == YES ]] || { echo 'Orbbec SDK license acceptance is required in docker/.env.' >&2; exit 2; }
export TELEOP_CAMERA_CONFIG_HASH="$(sha256sum "$TELEOP_CAMERA_CONFIG" | awk '{print $1}')"
export TELEOP_CAMERA_UID="$(id -u)" TELEOP_CAMERA_GID="$(id -g)"
cd "$REPO_ROOT/docker"
# Serialize GUI and recorder startup so only one driver owner can be created.
exec 9>"/tmp/gello-cameras-$(id -u).lock"
flock -w 120 9
camera_id="$(docker compose ps -q --status running teleop-cameras)"
if [[ -n "$camera_id" ]]; then
  current_hash="$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$camera_id" | sed -n 's/^TELEOP_CAMERA_CONFIG_HASH=//p')"
  [[ "$current_hash" == "$TELEOP_CAMERA_CONFIG_HASH" ]] || {
    echo 'Shared cameras use a different deployment config; stop them only after recording has finished.' >&2; exit 1;
  }
  echo 'Reusing shared cam0/cam1/cam2 service.'
else
  # Do not acquire devices from an existing legacy collector or other owner.
  if [[ -n "$(docker ps --filter label=com.docker.compose.service=data-collection --format '{{.Names}}')" ]]; then
    echo 'An existing collector may own the cameras; refusing a second driver. Stop that collector before migrating to shared cameras.' >&2
    exit 1
  fi
  if [[ -n "$(docker compose ps -q --status running orbbec)" ]] || pgrep -x OrbbecViewer >/dev/null; then
    echo 'Close the standalone Orbbec driver/viewer before starting shared cameras.' >&2; exit 1
  fi
  docker compose up -d --no-deps teleop-cameras
fi
# No robot nodes are started. Reuse the recorder's original four-stream gate.
docker compose exec -T teleop-cameras /entrypoint.sh \
  python3 -m teleop_data_collector.camera_stream_health wait \
  --timeout-sec "${CAMERA_READY_TIMEOUT_SEC:-90}" --stable-sec 3 --max-gap-ms 150
