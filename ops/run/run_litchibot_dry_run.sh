#!/usr/bin/env bash
# Independent real-glove dry-run. No ROS/GELLO/FR3/Sharpa initialization.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${LITCHIBOT_PYTHON:-/home/user/litchibot_glove/.venv/bin/python}"
cd "$REPO_ROOT"
exec "$PYTHON_BIN" -m adapters.litchibot.cli --hand-source litchibot --dry-run "$@"
