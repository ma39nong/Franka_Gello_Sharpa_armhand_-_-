#!/usr/bin/env bash
# Manual dual-hand control through the safety bridge and NiceGUI.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTROL_PORT="${TELEOP_CONTROL_PORT:-5590}"

occupied="$(lsof -t -iTCP:"$CONTROL_PORT" -sTCP:LISTEN 2>/dev/null || true)"
if [[ -n "$occupied" ]]; then
  echo "The teleop operator is active on port $CONTROL_PORT (PID(s): $occupied)." >&2
  echo "Disengage it before opening the manual hand UI." >&2
  exit 1
fi

cd "$REPO_ROOT/docker"
echo "Hand UI: http://127.0.0.1:8080"
exec docker compose up hand-control hand-ui
