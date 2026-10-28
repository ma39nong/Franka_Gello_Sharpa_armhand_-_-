#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run this rollback as root." >&2
  exit 1
fi

STATE_ROOT=/var/lib/franka-rt-tuning
STATE_FILE="$STATE_ROOT/install-state"
if [[ ! -r "$STATE_FILE" ]]; then
  echo "No installed Franka RT tuning state was found." >&2
  exit 1
fi

# This file is root-owned, mode 0600, and only contains shell-escaped paths
# and integer flags written by install_franka_rt_tuning.sh.
# shellcheck disable=SC1090
source "$STATE_FILE"

systemctl disable --now franka-rt-tuning.service 2>/dev/null || true
cp -a "$BACKUP_DIR/grub" /etc/default/grub

if [[ "$HAD_SERVICE" == 1 ]]; then
  cp -a "$BACKUP_DIR/service" /etc/systemd/system/franka-rt-tuning.service
else
  rm -f /etc/systemd/system/franka-rt-tuning.service
fi
if [[ "$HAD_RUNTIME" == 1 ]]; then
  cp -a "$BACKUP_DIR/runtime" /usr/local/sbin/franka-rt-tuning
else
  rm -f /usr/local/sbin/franka-rt-tuning
fi

update-grub
systemctl daemon-reload
rm -f "$STATE_FILE"

echo "System-level Franka RT tuning was rolled back."
echo "Restore the repository CPU configuration backup separately, then reboot."

if [[ "$HAD_ROLLBACK" == 1 ]]; then
  cp -a "$BACKUP_DIR/rollback" /usr/local/sbin/rollback-franka-rt-tuning
else
  rm -f /usr/local/sbin/rollback-franka-rt-tuning
fi
