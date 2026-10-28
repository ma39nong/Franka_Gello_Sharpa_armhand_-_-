#!/usr/bin/env bash
# Gello arms plus optional MANUS-to-Wuji hands. O30i remains untouched/default.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
FRANKA_SERVICE=franka-control
OPERATOR_ARGS=()
for argument in "$@"; do
  if [[ "$argument" == "--fake-franka" ]]; then
    FRANKA_SERVICE=fake-franka-control
  else
    OPERATOR_ARGS+=("$argument")
  fi
done
SERVICES=("$FRANKA_SERVICE" teleop-control moveit-ik preset-ik gello-bridge)
stack_started=false
operator_pid=""

pid_alive() {
  local pid=$1
  [[ -n "$pid" ]] && kill -0 -- "$pid" 2>/dev/null
}

wait_until_dead() {
  local pid=$1
  local attempts=$2
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    pid_alive "$pid" || return 0
    sleep 0.1
  done
  return 1
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  # run_operator owns the separately-sessioned backend and GUI.  Signal and
  # reap that supervisor before stopping Docker; otherwise an outer Ctrl-C can
  # return the prompt while the native MANUS/Wuji backend remains alive.
  if pid_alive "$operator_pid"; then
    kill -TERM -- "$operator_pid" 2>/dev/null || true
    if ! wait_until_dead "$operator_pid" 200; then
      echo "Operator supervisor did not stop; sending SIGKILL..." >&2
      kill -KILL -- "$operator_pid" 2>/dev/null || true
    fi
  fi
  [[ -z "$operator_pid" ]] || wait "$operator_pid" 2>/dev/null || true
  if [[ "$stack_started" == true ]]; then
    (cd "$REPO_ROOT/docker" && docker compose stop --timeout 10 "${SERVICES[@]}") || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

running="$(cd "$REPO_ROOT/docker" && docker compose ps --services --status running)"
if grep -qx hand-control <<<"$running"; then
  echo "Refusing to start Wuji mode while the O30i/G20 hand-control service is running." >&2
  echo "Stop the existing teleop stack first. O30i configuration has not been changed." >&2
  exit 1
fi
if grep -Eq '^(moveit-fake|moveit-real|arm-ui)$' <<<"$running"; then
  echo "Refusing to start Wuji/Gello mode while a MoveIt service is running." >&2
  echo "Stop MoveIt first so only one controller stack owns the FR3 arms." >&2
  exit 1
fi
if [[ "$FRANKA_SERVICE" == fake-franka-control ]] && grep -qx franka-control <<<"$running"; then
  echo "Refusing fake-FR3 mode while real franka-control is running." >&2
  exit 1
fi
if [[ "$FRANKA_SERVICE" == franka-control ]] && grep -qx fake-franka-control <<<"$running"; then
  echo "Refusing real-FR3 mode while fake-franka-control is running." >&2
  exit 1
fi

"$REPO_ROOT/ops/run/preflight.sh"
if [[ "$FRANKA_SERVICE" == fake-franka-control ]]; then
  echo "Using fake FR3 hardware; no physical Franka connection is required."
fi
cd "$REPO_ROOT/docker"
docker compose up -d "${SERVICES[@]}"
stack_started=true
docker compose ps "${SERVICES[@]}"

cd "$REPO_ROOT"
"$REPO_ROOT/ops/run/run_operator.sh" \
  --hand-source wuji \
  "${OPERATOR_ARGS[@]}" &
operator_pid=$!

set +e
wait "$operator_pid"
status=$?
set -e
operator_pid=""
exit "$status"
