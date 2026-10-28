#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"
CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"

python -m pip install \
  'nlopt>=2.7' \
  'pin>=3.8.0' \
  'nicegui>=2.24,<3' \
  'wuji-sdk>=0.10.0' \
  'wujihandpy>=1.8.0'

python - <<'PY'
import nlopt
import nicegui
import pinocchio
import wuji_sdk
import wujihandpy
print("Wuji Python dependencies import successfully")
PY
