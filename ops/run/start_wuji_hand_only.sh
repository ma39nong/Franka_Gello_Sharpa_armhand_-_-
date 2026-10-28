#!/usr/bin/env bash
# MANUS-to-Wuji hands only: no GELLO, FR3, Docker, or arm controller.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONDA_ENV="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"
BRIDGE="$REPO_ROOT/adapters/manus/build/libmanus_skeleton_bridge.so"

if [[ ! -f "$BRIDGE" ]]; then
  echo "Missing MANUS bridge: $BRIDGE" >&2
  echo "Build it first with: $REPO_ROOT/adapters/manus/scripts/build.sh" >&2
  exit 1
fi
if ldd "$BRIDGE" 2>/dev/null | grep -q 'not found'; then
  echo "The MANUS bridge has unresolved native libraries:" >&2
  ldd "$BRIDGE" | grep 'not found' >&2
  exit 1
fi

running="$(cd "$REPO_ROOT/docker" && docker compose ps --services --status running 2>/dev/null || true)"
if grep -qx hand-control <<<"$running"; then
  echo "Refusing to start while the O30i/G20 hand-control service is running." >&2
  echo "Stop the existing hand-control service first so hands have one owner." >&2
  exit 1
fi

cd "$REPO_ROOT"
exec conda run --no-capture-output -n "$CONDA_ENV" \
  python -m adapters.wuji.hand_only "$@"
