#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
environment=${TELEOP_CONDA_ENV:-gello-upper-body-teleop}

if conda env list | awk '{print $1}' | grep -Fxq "${environment}"; then
  conda env update --name "${environment}" \
    --file "${repo}/environment.gello-upper-body-teleop.yml" --prune
else
  conda env create --name "${environment}" \
    --file "${repo}/environment.gello-upper-body-teleop.yml"
fi

conda run --name "${environment}" python -m pip install \
  "${repo}/vendor/xrobotoolkit_sdk"

conda run --name "${environment}" python -m pip install \
  -e "${repo}/ros_ws/src/teleop_core" \
  -e "${repo}/adapters/pico" \
  -e "${repo}/adapters/vive" \
  -e "${repo}/adapters/manus/python"

# The Operator GUI is deliberately launched from Conda base so it remains
# independent of ROS/native numerical libraries in the teleoperation env.
conda install --name base --channel conda-forge --yes pyside6
