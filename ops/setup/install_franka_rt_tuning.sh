#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run this installer as root." >&2
  exit 1
fi

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
STATE_ROOT=/var/lib/franka-rt-tuning
STATE_FILE="$STATE_ROOT/install-state"
BACKUP_ROOT=/var/backups/franka-rt-tuning
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_DIR="$BACKUP_ROOT/$STAMP"

if [[ -e "$STATE_FILE" ]]; then
  echo "Franka RT tuning is already installed; roll it back before reinstalling." >&2
  exit 1
fi

install -d -m 0755 "$STATE_ROOT" "$BACKUP_DIR"
cp -a /etc/default/grub "$BACKUP_DIR/grub"

had_service=0
had_runtime=0
had_rollback=0
if [[ -e /etc/systemd/system/franka-rt-tuning.service ]]; then
  cp -a /etc/systemd/system/franka-rt-tuning.service "$BACKUP_DIR/service"
  had_service=1
fi
if [[ -e /usr/local/sbin/franka-rt-tuning ]]; then
  cp -a /usr/local/sbin/franka-rt-tuning "$BACKUP_DIR/runtime"
  had_runtime=1
fi
if [[ -e /usr/local/sbin/rollback-franka-rt-tuning ]]; then
  cp -a /usr/local/sbin/rollback-franka-rt-tuning "$BACKUP_DIR/rollback"
  had_rollback=1
fi

printf 'BACKUP_DIR=%q\n' "$BACKUP_DIR" > "$STATE_FILE"
printf 'HAD_SERVICE=%s\n' "$had_service" >> "$STATE_FILE"
printf 'HAD_RUNTIME=%s\n' "$had_runtime" >> "$STATE_FILE"
printf 'HAD_ROLLBACK=%s\n' "$had_rollback" >> "$STATE_FILE"
chmod 0600 "$STATE_FILE"

python3 - <<'PY'
from pathlib import Path

path = Path("/etc/default/grub")
lines = path.read_text().splitlines()
required = (
    "isolcpus=domain,managed_irq,8-15",
    "nohz_full=8-15",
    "rcu_nocbs=8-15",
    "irqaffinity=0-7,16-23",
)
output = []
seen_style = seen_timeout = seen_cmdline = False
for line in lines:
    if line.startswith("GRUB_TIMEOUT_STYLE="):
        output.append("GRUB_TIMEOUT_STYLE=menu")
        seen_style = True
    elif line.startswith("GRUB_TIMEOUT="):
        output.append("GRUB_TIMEOUT=5")
        seen_timeout = True
    elif line.startswith("GRUB_CMDLINE_LINUX_DEFAULT="):
        key, value = line.split("=", 1)
        value = value.strip().strip('"')
        words = value.split()
        for item in required:
            if item not in words:
                words.append(item)
        output.append(f'{key}="{" ".join(words)}"')
        seen_cmdline = True
    else:
        output.append(line)
if not seen_style:
    output.append("GRUB_TIMEOUT_STYLE=menu")
if not seen_timeout:
    output.append("GRUB_TIMEOUT=5")
if not seen_cmdline:
    output.append(f'GRUB_CMDLINE_LINUX_DEFAULT="{" ".join(required)}"')
path.write_text("\n".join(output) + "\n")
PY

install -m 0755 \
  "$REPO_ROOT/ops/setup/franka_rt_tuning.sh" \
  /usr/local/sbin/franka-rt-tuning
install -m 0755 \
  "$REPO_ROOT/ops/setup/rollback_franka_rt_tuning.sh" \
  /usr/local/sbin/rollback-franka-rt-tuning
install -m 0644 \
  "$REPO_ROOT/ops/setup/franka-rt-tuning.service" \
  /etc/systemd/system/franka-rt-tuning.service

update-grub
systemctl daemon-reload
systemctl enable franka-rt-tuning.service

echo "Franka RT tuning installed."
echo "Reboot to activate kernel isolation and the tuning service."
echo "Rollback command: sudo rollback-franka-rt-tuning"
