#!/usr/bin/env bash
# Optional VIVE Tracker arms with the default MANUS hands.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${repo_root}/ops/run/run_teleop.sh" \
  --arm-source vive-trackers \
  --vive-config "${repo_root}/config/modes/vive.yaml" \
  "$@"
