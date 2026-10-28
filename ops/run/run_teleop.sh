#!/usr/bin/env bash
# One-command teleop backend: fresh RUN_DIR and both debug logs.
# The operator GUI is a separate process (apps/operator_gui/operator_gui.py).
#
# Exists because the equivalent multi-line paste has now buried three
# recordings: a clipboard missing its final newline leaves the command
# pending, and the next paste glues onto it. A single short command has no
# such failure mode.
#
# GELLO is the default arm source and MANUS is the default hand source.
# Passing an explicit --arm-source or --hand-source suppresses that default;
# ops/run/run_pico_teleop.sh is the convenience path for PICO. Other extra
# arguments pass through to teleop_runtime.cli. TELEOP_RUN_DIR overrides the
# run directory.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
TELEOP_CONDA_ENV="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"
DATA_ROOT="${TELEOP_DATA_ROOT:-}"
if [[ -z "$DATA_ROOT" && -f "$REPO_ROOT/docker/.env" ]]; then
  DATA_ROOT="$(awk -F= '$1 == "TELEOP_DATA_ROOT" {sub(/^[^=]*=/, ""); print; exit}' \
    "$REPO_ROOT/docker/.env")"
fi
if [[ -z "$DATA_ROOT" ]]; then
  echo "TELEOP_DATA_ROOT is not set and is missing from docker/.env" >&2
  exit 1
fi

# This is the common backend entrypoint for GELLO, PICO, VIVE, and the
# Operator supervisor.  Pin here as well as in run_operator.sh so direct
# invocations cannot drift onto the Franka realtime cores.
HOUSEKEEPING_CPUSET_FILE=""
if [[ -f "$REPO_ROOT/docker/.env" ]]; then
  HOUSEKEEPING_CPUSET_FILE="$(
    awk -F= '$1 == "HOUSEKEEPING_CPUSET" {
      sub(/^[^=]*=/, ""); print; exit
    }' "$REPO_ROOT/docker/.env"
  )"
fi
if [[ -z "$HOUSEKEEPING_CPUSET_FILE" ]]; then
  echo "HOUSEKEEPING_CPUSET is missing from docker/.env" >&2
  exit 1
fi
if [[ -n "${HOUSEKEEPING_CPUSET:-}" && \
      "$HOUSEKEEPING_CPUSET" != "$HOUSEKEEPING_CPUSET_FILE" ]]; then
  echo "Exported HOUSEKEEPING_CPUSET conflicts with docker/.env:" >&2
  echo "  exported=$HOUSEKEEPING_CPUSET" >&2
  echo "  configured=$HOUSEKEEPING_CPUSET_FILE" >&2
  exit 1
fi
HOUSEKEEPING_CPUSET="$HOUSEKEEPING_CPUSET_FILE"
if ! taskset -c "$HOUSEKEEPING_CPUSET" true >/dev/null 2>&1; then
  echo "Invalid HOUSEKEEPING_CPUSET: $HOUSEKEEPING_CPUSET" >&2
  exit 1
fi

RUN_PARENT="${TELEOP_DIAGNOSTICS_ROOT:-$DATA_ROOT/diagnostics}"
if [[ -n "${TELEOP_RUN_DIR:-}" ]]; then
  RUN_DIR="$TELEOP_RUN_DIR"
  if [[ -e "$RUN_DIR" ]]; then
    echo "Refusing to reuse TELEOP_RUN_DIR: $RUN_DIR" >&2
    exit 1
  fi
  if ! mkdir -- "$RUN_DIR"; then
    echo "Could not create TELEOP_RUN_DIR: $RUN_DIR" >&2
    exit 1
  fi
else
  mkdir -p -- "$RUN_PARENT"
  RUN_DIR="$(mktemp -d "$RUN_PARENT/$(date +%Y%m%d_%H%M%S).XXXXXX")"
fi
echo "RUN_DIR=$RUN_DIR"

ARM_ARGS=(--arm-source gello --gello-config "$REPO_ROOT/config/modes/gello.yaml")
HAND_ARGS=(--hand-source manus --hand-debug-log "$RUN_DIR/hand_fidelity.jsonl")
for argument in "$@"; do
  if [[ "$argument" == "--arm-source" || "$argument" == --arm-source=* ]]; then
    ARM_ARGS=()
  fi
  if [[ "$argument" == "--hand-source" || "$argument" == --hand-source=* ]]; then
    HAND_ARGS=()
  fi
done

cd "$REPO_ROOT"
# Match the container stack: keep host-side DDS off the physical interfaces
# (FCI robot subnets and hand networks). Override by exporting it beforehand.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# activate + exec, NOT `conda run`: conda run wraps python in a subprocess
# and dies first on Ctrl-C, orphaning python into the background where it
# cannot restore the terminal (tcsetattr EIO -> hidden cursor, stuck cbreak).
# With exec, python owns the foreground and cleanup completes properly.
CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$TELEOP_CONDA_ENV"
exec taskset -c "$HOUSEKEEPING_CPUSET" python -m teleop_runtime.cli \
  --config config/modes/pico.yaml "${ARM_ARGS[@]}" \
  --preset-config config/preset_actions.yaml \
  --preset-data-root "$DATA_ROOT" \
  "${HAND_ARGS[@]}" \
  --debug-log "$RUN_DIR/ee_jitter.jsonl" \
  "$@"
