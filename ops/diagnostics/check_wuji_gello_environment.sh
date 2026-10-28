#!/usr/bin/env bash
# Read-only deployment check for the GELLO + MANUS + Wuji teleoperation stack.
set -uo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_NAME="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"
HARDWARE=false
LEFT_ADDRESS=""
RIGHT_ADDRESS=""
failures=0
warnings=0

usage() {
  cat <<'EOF'
Usage: ./ops/diagnostics/check_wuji_gello_environment.sh [options]

Checks the installed software without enabling or commanding any hardware.

Options:
  --hardware                 Require GELLO devices and a MANUS USB dongle
  --left-address IP:PORT     Also check routing/ping to the left Wuji hand
  --right-address IP:PORT    Also check routing/ping to the right Wuji hand
  -h, --help                 Show this help

Examples:
  ./ops/diagnostics/check_wuji_gello_environment.sh
  ./ops/diagnostics/check_wuji_gello_environment.sh --hardware \
    --left-address 192.168.1.110:7447 \
    --right-address 192.168.2.111:7447

This script does not start MANUS Core, connect/enable a Wuji hand, open a
Dynamixel bus, start Docker containers, or send a robot/hand/motor command.
EOF
}

while (($#)); do
  case "$1" in
    --hardware)
      HARDWARE=true
      shift
      ;;
    --left-address|--right-address)
      option="$1"
      if (($# < 2)); then
        echo "Missing value for $option" >&2
        exit 2
      fi
      if [[ "$option" == --left-address ]]; then
        LEFT_ADDRESS="$2"
      else
        RIGHT_ADDRESS="$2"
      fi
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "command available: $1"
  else
    fail "missing command: $1"
  fi
}

check_file() {
  if [[ -f "$1" ]]; then
    pass "$2"
  else
    fail "$2 missing: $1"
  fi
}

echo "GELLO + MANUS + Wuji deployment check"
echo "  repo: $REPO_ROOT"
echo "  conda env: $ENV_NAME"
echo "  mode: $([[ "$HARDWARE" == true ]] && echo hardware || echo software-only)"
echo

if [[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]]; then
  pass "platform: Linux x86_64"
else
  fail "platform must be Linux x86_64; found $(uname -s) $(uname -m)"
fi

for command_name in conda cmake c++ ldd readelf docker lsof taskset; do
  check_command "$command_name"
done

env_ready=false
if command -v conda >/dev/null 2>&1; then
  if conda run --no-capture-output -n "$ENV_NAME" python -c \
    'import sys; assert sys.version_info[:2] == (3, 10), sys.version' \
    >/dev/null 2>&1; then
    pass "Conda environment exists with Python 3.10"
    env_ready=true
  else
    fail "Conda environment $ENV_NAME is missing or is not Python 3.10"
    echo "       run: ./ops/setup/setup_teleop_env.sh"
  fi
fi

if [[ "$env_ready" == true ]]; then
  python_probe='from importlib import import_module, metadata
modules = {
    "numpy": "numpy", "scipy": "scipy", "PyYAML": "yaml",
    "mujoco": "mujoco", "pin": "pinocchio", "nlopt": "nlopt",
    "wuji-sdk": "wuji_sdk", "wujihandpy": "wujihandpy",
    "GELLO driver": "gello.dynamixel.driver",
    "teleop adapter": "pico_bimanual_franka_teleop",
    "MANUS Python": "manus_teleop", "Wuji adapter": "adapters.wuji",
}
for label, module in modules.items():
    import_module(module)
    try:
        version = metadata.version(label)
    except metadata.PackageNotFoundError:
        version = "imported"
    print(f"  {label}: {version}")'
  if probe_output="$(cd "$REPO_ROOT" && conda run --no-capture-output \
      -n "$ENV_NAME" python -c "$python_probe" 2>&1)"; then
    pass "required Python modules import"
    printf '%s\n' "$probe_output"
  else
    fail "required Python module import failed"
    printf '%s\n' "$probe_output" | tail -n 12
    echo "       run: ./ops/setup/setup_teleop_env.sh"
    echo "       then: GELLO_SOFTWARE_ROOT=/path/to/gello_software ./ops/setup/setup_gello_driver.sh"
  fi

  if conda run --no-capture-output -n base python -c 'import PySide6' \
      >/dev/null 2>&1; then
    pass "Operator GUI dependency PySide6 imports in Conda base"
  else
    fail "PySide6 is missing from Conda base"
    echo "       run: conda install -n base -c conda-forge pyside6"
  fi
fi

check_file "$REPO_ROOT/vendor/manus_sdk/lib/libManusSDK_Integrated.so" \
  "vendored MANUS Core SDK library"
check_file "$REPO_ROOT/adapters/manus/config/Calibration_left.mcal" \
  "left MANUS calibration"
check_file "$REPO_ROOT/adapters/manus/config/Calibration_right.mcal" \
  "right MANUS calibration"

BRIDGE="$REPO_ROOT/adapters/manus/build/libmanus_skeleton_bridge.so"
if [[ -f "$BRIDGE" ]]; then
  pass "MANUS skeleton bridge is built"
  unresolved="$(ldd "$BRIDGE" 2>/dev/null | awk '/not found/{print}')"
  if [[ -z "$unresolved" ]]; then
    pass "MANUS bridge has no unresolved native dependency"
  else
    fail "MANUS bridge has unresolved native dependencies"
    printf '%s\n' "$unresolved"
  fi
  expected_runpath="$REPO_ROOT/vendor/manus_sdk/lib"
  if readelf -d "$BRIDGE" 2>/dev/null | grep -Fq "$expected_runpath"; then
    pass "MANUS bridge RUNPATH matches this repository"
  else
    fail "MANUS bridge RUNPATH points at another checkout"
    echo "       run: ./adapters/manus/scripts/build.sh"
  fi
  if [[ "$env_ready" == true ]]; then
    if conda run --no-capture-output -n "$ENV_NAME" python -c \
      'import ctypes, sys; ctypes.CDLL(sys.argv[1])' "$BRIDGE" \
      >/dev/null 2>&1; then
      pass "MANUS bridge loads without starting MANUS Core"
    else
      fail "MANUS bridge cannot be loaded"
      echo "       run: ./adapters/manus/scripts/build.sh"
    fi
  fi
else
  fail "MANUS skeleton bridge is not built"
  echo "       run: ./adapters/manus/scripts/build.sh"
fi

if [[ -f "$REPO_ROOT/ros_ws/install/setup.bash" ]]; then
  pass "ROS workspace install exists"
else
  fail "ROS workspace is not built"
  echo "       run: ./ops/setup/build.sh"
fi

if [[ -f "$REPO_ROOT/docker/.env" ]]; then
  pass "docker/.env exists"
  data_root="$(awk -F= '$1 == "TELEOP_DATA_ROOT" {sub(/^[^=]*=/, ""); print; exit}' \
    "$REPO_ROOT/docker/.env")"
  if [[ -n "$data_root" && -d "$data_root" && -w "$data_root" ]]; then
    pass "TELEOP_DATA_ROOT exists and is writable: $data_root"
  else
    fail "TELEOP_DATA_ROOT is missing or not writable: ${data_root:-unset}"
  fi
  if "$REPO_ROOT/ops/diagnostics/check_franka_cpu_layout.sh"; then
    pass "Franka CPU/SMT layout is internally consistent"
  else
    fail "Franka CPU/SMT layout is unsafe"
  fi
else
  fail "docker/.env is missing"
  echo "       run: cp docker/.env.example docker/.env, then edit host paths/CPUs"
fi

if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    pass "Docker daemon is accessible"
    if (cd "$REPO_ROOT/docker" && docker compose config --quiet) \
        >/dev/null 2>&1; then
      pass "Docker Compose configuration is valid"
    else
      fail "Docker Compose configuration is invalid"
    fi
    if docker image inspect franka-upper-body-teleop:latest >/dev/null 2>&1; then
      pass "Docker image exists: franka-upper-body-teleop:latest"
    else
      fail "Docker image is missing: franka-upper-body-teleop:latest"
      echo "       run: ./ops/setup/build.sh"
    fi
  else
    fail "Docker daemon is unavailable or current user lacks permission"
  fi
fi

if [[ "$env_ready" == true ]]; then
  for side in left right; do
    if (cd "$REPO_ROOT" && conda run --no-capture-output -n "$ENV_NAME" \
      python -m adapters.wuji.sim --side "$side" --headless --frames 30) \
      >/dev/null 2>&1; then
      pass "Wuji $side hand offline retarget/model smoke test"
    else
      fail "Wuji $side hand offline retarget/model smoke test failed"
    fi
  done
fi

if [[ "$HARDWARE" == true ]]; then
  if [[ "$env_ready" == true ]] && (cd "$REPO_ROOT" && \
    conda run --no-capture-output -n "$ENV_NAME" \
      python ops/diagnostics/check_gello_ports.py) >/dev/null 2>&1; then
    pass "both GELLO stable by-id paths are present"
  else
    fail "GELLO by-id preflight failed"
    echo "       inspect: ls -l /dev/serial/by-id"
  fi
  if command -v lsusb >/dev/null 2>&1 && lsusb | grep -qi 'Manus'; then
    pass "MANUS USB dongle is present"
  else
    fail "MANUS USB dongle was not detected"
  fi
else
  warn "hardware presence checks skipped (pass --hardware to require them)"
fi

check_address() {
  local side="$1"
  local address="$2"
  [[ -n "$address" ]] || return 0
  if [[ "$address" != *:* ]]; then
    fail "$side Wuji address must be IP:PORT: $address"
    return
  fi
  local host="${address%:*}"
  local port="${address##*:}"
  if [[ ! "$port" =~ ^[0-9]+$ ]] || ((port < 1 || port > 65535)); then
    fail "$side Wuji port is invalid: $port"
    return
  fi
  if ip route get "$host" >/dev/null 2>&1; then
    pass "$side Wuji route exists: $address"
  else
    fail "$side Wuji has no route: $host"
    return
  fi
  ping_output="$(ping -c 3 -W 2 "$host" 2>&1)"
  packet_loss="$(printf '%s\n' "$ping_output" | \
    sed -nE 's/.* ([0-9]+)% packet loss.*/\1/p' | tail -n 1)"
  if [[ "$packet_loss" == 0 ]]; then
    pass "$side Wuji ping is stable (0% loss): $host"
  elif [[ -n "$packet_loss" ]]; then
    fail "$side Wuji ping is unstable (${packet_loss}% loss): $host"
  else
    fail "$side Wuji ping failed: $host"
  fi
}

check_address left "$LEFT_ADDRESS"
check_address right "$RIGHT_ADDRESS"

echo
echo "Summary: $failures failure(s), $warnings warning(s)."
echo "No hardware was enabled and no robot, hand, or motor command was sent."
if ((failures)); then
  exit 1
fi
