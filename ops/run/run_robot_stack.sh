#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
"$REPO_ROOT/ops/run/preflight.sh"

cd "$REPO_ROOT/docker"
running="$(docker compose ps --services --status running)"
if grep -Eq '^(moveit-fake|moveit-real|arm-ui)$' <<<"$running"; then
  echo "Refusing to start the teleop robot stack while MoveIt/arm-ui is running." >&2
  exit 1
fi
exec docker compose up \
  franka-control teleop-control moveit-ik preset-ik gello-bridge hand-control
