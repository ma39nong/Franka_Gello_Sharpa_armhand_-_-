#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
viewer=$(ls -d /opt/OrbbecSDK_v2.*/tools/OrbbecViewer 2>/dev/null | sort -V | tail -1)
viewer_dir=$(dirname "${viewer:-/nonexistent}")

if [[ -z ${viewer} || ! -x ${viewer} ]]; then
  echo "Orbbec SDK v2 Viewer is missing under /opt/OrbbecSDK_v2.*/tools/" >&2
  echo "The SDK v1 Viewer cannot open a Gemini 435Le (pid 0x0815)." >&2
  exit 1
fi
if docker compose --file "${repo}/docker/compose.yaml" \
    ps --status running --services | grep -Fxq orbbec; then
  echo "The ROS Orbbec service is running; stop it before opening Viewer:" >&2
  echo "  docker compose --file ${repo}/docker/compose.yaml stop orbbec" >&2
  exit 1
fi
if pgrep -x OrbbecViewer >/dev/null; then
  echo "OrbbecViewer is already running." >&2
  exit 1
fi

display=${DISPLAY:-:1}
xauthority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
cd "${viewer_dir}"
exec env DISPLAY="${display}" XAUTHORITY="${xauthority}" "${viewer}"
