#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
environment=${TELEOP_CONDA_ENV:-gello-upper-body-teleop}
gello_root=${GELLO_SOFTWARE_ROOT:-}

if [[ -z "$gello_root" || ! -d "$gello_root/gello" ]]; then
  echo "Set GELLO_SOFTWARE_ROOT to a gello_software checkout containing gello/." >&2
  exit 1
fi

conda run --name "$environment" python -m pip install -e "$gello_root"
if [[ ! -d "$gello_root/third_party/DynamixelSDK/python" ]]; then
  echo "Missing DynamixelSDK submodule; run:" >&2
  echo "  git -C $gello_root submodule update --init third_party/DynamixelSDK" >&2
  exit 1
fi
conda run --name "$environment" python -m pip install -e \
  "$gello_root/third_party/DynamixelSDK/python"
conda run --name "$environment" python -c \
  "from gello.dynamixel.driver import DynamixelDriver; print('GELLO driver ready')"
echo "Installed GELLO driver for $repo_root"
