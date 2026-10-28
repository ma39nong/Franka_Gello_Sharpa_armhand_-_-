#!/usr/bin/env bash
# Start only read-only hand telemetry, three cameras, and the ROS bag recorder.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA_ROOT=""
CAMERA_CONFIG="$REPO_ROOT/data_collection/config/cameras.yaml"

usage() {
  echo "Usage: $0 --data-root ABSOLUTE_PATH [--camera-config YAML]" >&2
}

while (($#)); do
  case "$1" in
    --data-root)
      (($# >= 2)) || { usage; exit 2; }
      DATA_ROOT=$2
      shift 2
      ;;
    --camera-config)
      (($# >= 2)) || { usage; exit 2; }
      CAMERA_CONFIG=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

[[ -n "$DATA_ROOT" ]] || { usage; exit 2; }
[[ "$DATA_ROOT" == /* ]] || {
  echo "--data-root must be an absolute path" >&2
  exit 2
}
[[ -f "$CAMERA_CONFIG" ]] || {
  echo "Camera deployment config not found: $CAMERA_CONFIG" >&2
  exit 2
}

# Persist acceptance in the already-gitignored docker/.env so each recording
# session does not need a shell export. The image and git tree still default
# to not accepted.
if [[ "${ORBBEC_SDK_LICENSE_ACCEPTED:-}" != "YES" && -f "$REPO_ROOT/docker/.env" ]]; then
  ORBBEC_SDK_LICENSE_ACCEPTED="$(
    awk -F= '$1 == "ORBBEC_SDK_LICENSE_ACCEPTED" {
      sub(/^[^=]*=/, ""); gsub(/\r/, ""); print; exit
    }' "$REPO_ROOT/docker/.env"
  )"
fi
if [[ "${ORBBEC_SDK_LICENSE_ACCEPTED:-}" != "YES" ]]; then
  echo "Orbbec runtime is license-gated." >&2
  echo "Review ros_ws/src/orbbec_camera/SDK/End User License Agreement.txt." >&2
  echo "If you accept, set ORBBEC_SDK_LICENSE_ACCEPTED=YES in docker/.env." >&2
  exit 2
fi

DATA_ROOT="$(realpath -m -- "$DATA_ROOT")"
CAMERA_CONFIG="$(realpath -- "$CAMERA_CONFIG")"
case "$DATA_ROOT/" in
  "$REPO_ROOT/"*)
    echo "Data output must be outside the Git repository: $DATA_ROOT" >&2
    exit 2
    ;;
esac

mapfile -t CAMERA_VALUES < <(python3 - "$CAMERA_CONFIG" <<'PY'
import sys
from pathlib import Path
import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
cameras = config.get("camera_bringup") or {}
for name in ("cam0", "cam1", "cam2"):
    entry = cameras.get(name) or {}
    serial = str(entry.get("serial_number", "")).strip()
    semantic = str(entry.get("semantic", "")).strip()
    if not serial or serial.startswith("REPLACE_"):
        raise SystemExit(f"{name} serial_number is not deployed")
    if not semantic or semantic.startswith("REPLACE_"):
        raise SystemExit(f"{name} semantic is not deployed")
    print(serial)
    print(semantic)
PY
)
[[ ${#CAMERA_VALUES[@]} -eq 6 ]] || exit 2
[[ $(printf '%s\n' "${CAMERA_VALUES[0]}" "${CAMERA_VALUES[2]}" "${CAMERA_VALUES[4]}" | sort -u | wc -l) -eq 3 ]] || {
  echo "Camera serial numbers must be unique" >&2
  exit 2
}

running="$(cd "$REPO_ROOT/docker" && docker compose ps --services --status running 2>/dev/null || true)"
if grep -qx hand-control <<<"$running"; then
  echo "Refusing to start Harvest recording while the O30i/G20 hand-control service is running." >&2
  echo "Stop the default teleop stack first; collection is Wuji-only and read-only." >&2
  exit 2
fi
if grep -qx orbbec <<<"$running"; then
  echo "Refusing to start three-camera recording while compose service 'orbbec' is running." >&2
  echo "The existing single-camera Orbbec service and Harvest cam0/1/2 must not share a device." >&2
  exit 2
fi
if pgrep -a OrbbecViewer >/dev/null 2>&1; then
  echo "Refusing to start three-camera recording while OrbbecViewer is running." >&2
  exit 2
fi

mkdir -p -- "$DATA_ROOT/bags/gello" "$DATA_ROOT/datasets"
WORKCELL_HASH="$(sha256sum "$REPO_ROOT/config/workcell/current.yaml" | awk '{print $1}')"

[[ -f "$REPO_ROOT/docker/.env" ]] || {
  echo "docker/.env is missing; run setup first" >&2
  exit 2
}
docker image inspect franka-upper-body-teleop:latest >/dev/null

ACTIVE_COLLECTORS="$(
  docker ps \
    --filter label=com.docker.compose.service=data-collection \
    --format '{{.Names}}'
)"
if [[ -n "$ACTIVE_COLLECTORS" ]]; then
  echo "Refusing to start a second data-collection instance." >&2
  echo "Already running: $ACTIVE_COLLECTORS" >&2
  exit 2
fi
if ss -H -lun 'sport = :5602' | awk 'NF { found=1 } END { exit !found }'; then
  echo "Refusing to start: UDP 127.0.0.1:5602 is already in use." >&2
  echo "Stop the existing hand telemetry receiver or recording session first." >&2
  exit 2
fi
if ss -H -ltn 'sport = :5592' | awk 'NF { found=1 } END { exit !found }'; then
  echo "Refusing to start: collector UI control TCP port 127.0.0.1:5592 is already in use." >&2
  echo "Stop the existing recording session first." >&2
  exit 2
fi

(
  cd "$REPO_ROOT/docker"
  docker compose config --quiet
  docker compose run --rm \
    -e ORBBEC_SDK_LICENSE_ACCEPTED=YES \
    -e COLLECTION_OUTPUT_DIR=/collection_data/bags/gello \
    -e COLLECTION_CAMERA_CONFIG=/collection_camera_config.yaml \
    -e COLLECTION_WORKCELL_HASH="$WORKCELL_HASH" \
    -e COLLECTION_CAM0_SERIAL="${CAMERA_VALUES[0]}" \
    -e COLLECTION_CAM0_SEMANTIC="${CAMERA_VALUES[1]}" \
    -e COLLECTION_CAM1_SERIAL="${CAMERA_VALUES[2]}" \
    -e COLLECTION_CAM1_SEMANTIC="${CAMERA_VALUES[3]}" \
    -e COLLECTION_CAM2_SERIAL="${CAMERA_VALUES[4]}" \
    -e COLLECTION_CAM2_SEMANTIC="${CAMERA_VALUES[5]}" \
    -v /dev:/dev \
    -v "$DATA_ROOT:/collection_data" \
    -v "$CAMERA_CONFIG:/collection_camera_config.yaml:ro" \
    data-collection
)
