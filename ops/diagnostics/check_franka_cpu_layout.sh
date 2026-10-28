#!/usr/bin/env bash
# Read-only guard for the host-specific dual-FCI CPU/SMT layout.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/.env"
WORKCELL_FILE="$REPO_ROOT/config/workcell/current.yaml"
TUNER_FILE="$REPO_ROOT/ops/setup/franka_rt_tuning.sh"
failures=0

pass() { printf '[PASS] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*" >&2; failures=$((failures + 1)); }

read_env_value() {
  local key=$1
  awk -F= -v key="$key" \
    '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "$ENV_FILE"
}

expand_cpuset() {
  local cpuset=$1
  local item first last cpu
  local -a items
  IFS=',' read -r -a items <<<"$cpuset"
  for item in "${items[@]}"; do
    if [[ "$item" == *-* ]]; then
      first=${item%-*}
      last=${item#*-}
      for ((cpu = first; cpu <= last; cpu++)); do
        printf '%s\n' "$cpu"
      done
    else
      printf '%s\n' "$item"
    fi
  done
}

sorted_cpuset() {
  expand_cpuset "$1" | LC_ALL=C sort -u
}

cpuset_intersection() {
  comm -12 <(sorted_cpuset "$1") <(sorted_cpuset "$2") | paste -sd, -
}

cpuset_difference() {
  comm -23 <(sorted_cpuset "$1") <(sorted_cpuset "$2") | paste -sd, -
}

if [[ ! -f "$ENV_FILE" || ! -f "$WORKCELL_FILE" || ! -f "$TUNER_FILE" ]]; then
  echo "Required CPU-layout input is missing." >&2
  exit 1
fi

declare -A cpu_sets=()
for variable in FRANKA_CPUSET FRANKA_TELEOP_CPUSET HOUSEKEEPING_CPUSET; do
  value="$(read_env_value "$variable")"
  cpu_sets[$variable]="$value"
  if [[ -z "$value" ]] || ! taskset -c "$value" true >/dev/null 2>&1; then
    fail "$variable is invalid: ${value:-unset}"
    continue
  fi
  if [[ -n "${!variable:-}" && "${!variable}" != "$value" ]]; then
    fail "exported $variable=${!variable} conflicts with docker/.env=$value"
  else
    pass "$variable=$value"
  fi
done

teleop_set=${cpu_sets[FRANKA_TELEOP_CPUSET]}
housekeeping_set=${cpu_sets[HOUSEKEEPING_CPUSET]}
if [[ -n "$teleop_set" && -n "$housekeeping_set" ]]; then
  overlap="$(cpuset_intersection "$teleop_set" "$housekeeping_set")"
  if [[ -z "$overlap" ]]; then
    pass "teleop and housekeeping CPU sets are disjoint"
  else
    fail "teleop and housekeeping CPU sets overlap on: $overlap"
  fi
fi

mapfile -t controller_sets < <(
  awk -F'"' '/^[[:space:]]+controller_cpus:/ {print $2}' "$WORKCELL_FILE"
)
if ((${#controller_sets[@]} != 2)); then
  fail "expected exactly two controller_cpus entries in $WORKCELL_FILE"
else
  controller_union="${controller_sets[0]},${controller_sets[1]}"
  outside="$(cpuset_difference "$controller_union" "$teleop_set")"
  extra="$(cpuset_difference "$teleop_set" "$controller_union")"
  if [[ -z "$outside" && -z "$extra" ]]; then
    pass "franka-control cpuset exactly matches both controller CPU sets"
  else
    fail "controller/container mismatch: controller-only=${outside:-none}, container-only=${extra:-none}"
  fi

  left_irq="$(awk -F= '$1 == "LEFT_IRQ_CPU" {print $2; exit}' "$TUNER_FILE")"
  right_irq="$(awk -F= '$1 == "RIGHT_IRQ_CPU" {print $2; exit}' "$TUNER_FILE")"
  irq_cpus=("$left_irq" "$right_irq")
  arm_names=(left right)
  for index in 0 1; do
    irq_cpu=${irq_cpus[$index]}
    sibling_file="/sys/devices/system/cpu/cpu${irq_cpu}/topology/thread_siblings_list"
    if [[ -z "$irq_cpu" || ! -r "$sibling_file" ]]; then
      fail "cannot read ${arm_names[$index]} FCI IRQ CPU topology"
      continue
    fi
    irq_siblings="$(<"$sibling_file")"
    controller_overlap="$(
      cpuset_intersection "${controller_sets[$index]}" "$irq_siblings"
    )"
    housekeeping_overlap="$(
      cpuset_intersection "$housekeeping_set" "$irq_siblings"
    )"
    if [[ -z "$controller_overlap" && -z "$housekeeping_overlap" ]]; then
      pass "${arm_names[$index]} IRQ CPU $irq_cpu core ($irq_siblings) is reserved"
    else
      fail "${arm_names[$index]} IRQ core $irq_siblings is used by controller=${controller_overlap:-none}, housekeeping=${housekeeping_overlap:-none}"
    fi
  done
fi

if ((failures != 0)); then
  echo "CPU layout check failed with $failures error(s)." >&2
  exit 1
fi
echo "CPU layout check passed. No process or hardware was started."
