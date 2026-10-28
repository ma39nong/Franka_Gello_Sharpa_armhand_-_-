#!/usr/bin/env bash
# One-command dual GELLO + dual FR3 arm-only teleoperation.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICES=(franka-control teleop-control moveit-ik preset-ik gello-bridge)
stack_started=false

for argument in "$@"; do
  if [[ "$argument" == "--hand-source" || "$argument" == --hand-source=* ]]; then
    echo "run_gello_arms_only.sh does not accept --hand-source" >&2
    exit 2
  fi
done

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ "$stack_started" == true ]]; then
    echo "Stopping arm-only teleoperation stack..."
    (
      cd "$REPO_ROOT/docker"
      docker compose stop --timeout 10 "${SERVICES[@]}"
    ) || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

"$REPO_ROOT/ops/run/preflight.sh" --arms-only
cd "$REPO_ROOT/docker"
running="$(docker compose ps --services --status running)"
if grep -Eq '^(moveit-fake|moveit-real|arm-ui)$' <<<"$running"; then
  echo "Refusing arm-only teleoperation while MoveIt/arm-ui is running." >&2
  exit 1
fi
# Ensure a hand service left by an earlier session cannot command either hand.
docker compose stop --timeout 10 hand-control
docker compose up -d "${SERVICES[@]}"
stack_started=true
docker compose ps "${SERVICES[@]}"

cd "$REPO_ROOT"
"$REPO_ROOT/ops/run/run_operator.sh" --hand-source none "$@"
