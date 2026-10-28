#!/usr/bin/env bash
# Install the repository RT-tuning script with an automatic rollback.
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run with sudo: sudo ./ops/setup/deploy_franka_rt_tuning.sh" >&2
  exit 2
fi

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source_script="$repo_root/ops/setup/franka_rt_tuning.sh"
installed_script=/usr/local/sbin/franka-rt-tuning
service=franka-rt-tuning.service
backup_root=/var/backups/franka-rt-tuning
stamp="$(date +%Y%m%d_%H%M%S)"
backup_dir="$backup_root/irq-priority-$stamp"
status_file="$backup_dir/status.after"
backup_ready=0

rollback() {
  local rc=$?
  trap - ERR
  if ((backup_ready)); then
    echo "Deployment failed; restoring $backup_dir/franka-rt-tuning.before" >&2
    install -m 0755 "$backup_dir/franka-rt-tuning.before" "$installed_script"
    systemctl restart "$service" || true
  fi
  exit "$rc"
}
trap rollback ERR

bash -n "$source_script"
install -d -m 0755 "$backup_dir"
cp -a "$installed_script" "$backup_dir/franka-rt-tuning.before"
backup_ready=1

install -m 0755 "$source_script" "$installed_script"
systemctl restart "$service"
systemctl is-active --quiet "$service"
"$installed_script" status > "$status_file"

left_count=$(grep -c '^enp2s0f0 .*fifo_priority=80$' "$status_file" || true)
right_count=$(grep -c '^enp2s0f1 .*fifo_priority=80$' "$status_file" || true)
non_fci_count=$(grep -c '^non_fci_rt_irq=.*fifo_priority=20' "$status_file" || true)

if ((left_count == 0 || right_count == 0 || non_fci_count == 0)); then
  echo "IRQ priority validation failed: left=$left_count right=$right_count non_fci=$non_fci_count" >&2
  false
fi

trap - ERR
echo "Deployment succeeded. Backup and status: $backup_dir"
cat "$status_file"
