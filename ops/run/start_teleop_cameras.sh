#!/usr/bin/env bash
# Shared, camera-only owner. Safe to run without powered arms/hands.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
takeover=false
config="$REPO_ROOT/data_collection/config/cameras.yaml"
config_set=false
for arg in "$@"; do
  case "$arg" in
    --takeover) takeover=true ;;
    --*) echo "Unknown camera startup option: $arg" >&2; exit 2 ;;
    *)
      [[ "$config_set" == false ]] || { echo 'Only one camera config is allowed.' >&2; exit 2; }
      config="$arg"; config_set=true ;;
  esac
done
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
  reused=true
  echo 'Reusing shared cam0/cam1/cam2 service.'
else
  reused=false
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
# Probe real ROS frames independently of the GUI preview. Existing services
# should already be warm; a freshly started/rebuilt service gets the full budget.
ready_timeout="${CAMERA_READY_TIMEOUT_SEC:-90}"
probe_timeout="$ready_timeout"
[[ "$reused" == false ]] || probe_timeout="${CAMERA_REUSE_PROBE_SEC:-8}"
wait_frames() {
  docker compose exec -T teleop-cameras /entrypoint.sh \
    python3 -m teleop_data_collector.camera_stream_health wait \
    --timeout-sec "$1" --stable-sec 3 --max-gap-ms 150 --no-frames-exit-code 3
}
if wait_frames "$probe_timeout"; then exit 0; else result=$?; fi
# An interrupted probe, Docker error, low FPS or timestamp fault must not trigger
# recreation. The readiness tool returns 3 only for actual missing/stale frames.
[[ "$result" == 3 ]] || exit "$result"
if [[ "$takeover" != true ]]; then
  echo '相机无帧或断流；普通启动不重建服务。如需尝试恢复，请使用统一入口的 --takeover。' >&2
  exit "$result"
fi

# Recorder wrappers hold a shared lease from container creation through exit;
# recovery takes the exclusive lease to close the check/start race.
exec 8>"/tmp/gello-camera-use-$(id -u).lock"
if ! flock -n -x 8; then
  echo '相机自动恢复已阻止：采集器正在启动、运行或收尾。' >&2; exit 1
fi
collectors="$(docker ps --filter label=com.docker.compose.service=data-collection --format '{{.Names}}')"
[[ -z "$collectors" ]] || { echo '相机自动恢复已阻止：存在运行中的采集器，请先退出采集终端。' >&2; exit 1; }
# Any control listener (including an idle collector) conservatively blocks recovery.
COLLECTION_CONTROL_PORT="${COLLECTION_CONTROL_PORT:-$(deployment_env_default "$REPO_ROOT/docker/.env" COLLECTION_CONTROL_PORT 5592)}" \
  python3 "$REPO_ROOT/ops/lib/camera_recovery_guard.py" || exit 1
if [[ -n "$(docker compose ps -q --status running orbbec)" ]] || pgrep -x OrbbecViewer >/dev/null; then
  echo '相机自动恢复已阻止：请先关闭独立相机驱动或 Viewer。' >&2; exit 1
fi
# Exactly one attempt, camera service only. Never stop arm/hand dependencies.
echo '检测到相机无帧或断流，正在自动重建共享相机服务（仅尝试一次）……'
docker compose up -d --no-deps --force-recreate teleop-cameras
if wait_frames "$ready_timeout"; then
  echo '相机已恢复，图像已连续稳定 3 秒。'
else
  result=$?
  echo '相机重建后仍未就绪，停止自动重试；请检查日志、供电、USB 或网络。' >&2
  exit "$result"
fi
