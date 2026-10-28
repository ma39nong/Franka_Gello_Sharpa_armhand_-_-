#!/usr/bin/env bash
# One-command dual GELLO + dual FR3 + MANUS teleoperation.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICES=(franka-control teleop-control moveit-ik preset-ik gello-bridge hand-control)
stack_started=false

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ "$stack_started" == true ]]; then
    echo "Stopping teleoperation stack..."
    (
      cd "$REPO_ROOT/docker"
      docker compose stop --timeout 10 "${SERVICES[@]}"
    ) || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

"$REPO_ROOT/ops/run/preflight.sh"
cd "$REPO_ROOT/docker"
running="$(docker compose ps --services --status running)"
if grep -Eq '^(moveit-fake|moveit-real|arm-ui)$' <<<"$running"; then
  echo "Refusing to start teleoperation while MoveIt/arm-ui is running." >&2
  exit 1
fi
docker compose up -d "${SERVICES[@]}"
stack_started=true
docker compose ps "${SERVICES[@]}"

cd "$REPO_ROOT"
"$REPO_ROOT/ops/run/run_operator.sh" "$@"
