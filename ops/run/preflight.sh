#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
TELEOP_CONDA_ENV="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"
CHECK_HANDS=true
CHECK_GELLO=true

while (($# > 0)); do
  case "$1" in
    --arms-only)
      CHECK_HANDS=false
      ;;
    --skip-gello)
      CHECK_GELLO=false
      ;;
    *)
      echo "Usage: $0 [--arms-only] [--skip-gello]" >&2
      exit 2
      ;;
  esac
  shift
done

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

run_with_dialout() {
  if [[ " $(id -nG) " == *" dialout "* ]]; then
    "$@"
    return
  fi
  local command_string
  printf -v command_string '%q ' "$@"
  sg dialout -c "$command_string"
}

[[ -f "$REPO_ROOT/docker/.env" ]] || fail \
  "docker/.env is missing; run: cp docker/.env.example docker/.env"
if [[ "$CHECK_HANDS" == true ]]; then
  [[ -f "$REPO_ROOT/adapters/manus/config/Calibration_left.mcal" ]] || \
    fail "missing MANUS left calibration"
  [[ -f "$REPO_ROOT/adapters/manus/config/Calibration_right.mcal" ]] || \
    fail "missing MANUS right calibration"
  [[ -f "$REPO_ROOT/adapters/manus/build/libmanus_skeleton_bridge.so" ]] || \
    fail "MANUS bridge is not built; run: adapters/manus/scripts/build.sh"
fi
[[ -f "$REPO_ROOT/ros_ws/install/setup.bash" ]] || \
  fail "ROS workspace is not built; run: ./ops/setup/build.sh"

command -v conda >/dev/null || fail "conda is not available"
command -v docker >/dev/null || fail "docker is not available"
docker info >/dev/null 2>&1 || fail "Docker daemon is unavailable"

if [[ "$CHECK_GELLO" == true ]]; then
  run_with_dialout conda run --no-capture-output -n "$TELEOP_CONDA_ENV" \
    python "$REPO_ROOT/ops/diagnostics/check_gello_ports.py"
fi
conda run --no-capture-output -n base python -c \
  "import PySide6; print('[PASS] operator GUI: PySide6 ready')"

(
  cd "$REPO_ROOT/docker"
  docker compose config --quiet
)
echo "[PASS] Docker Compose configuration"
"$REPO_ROOT/ops/diagnostics/check_franka_cpu_layout.sh"
echo "Preflight passed. No robot, hand, or motor command was sent."
