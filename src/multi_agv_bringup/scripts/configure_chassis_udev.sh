#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: sudo $0 /dev/ttyUSB0-or-ttyACM0" >&2
  exit 2
fi
if [[ "${EUID}" -ne 0 ]]; then
  echo "ERROR: run this hardware setup with sudo." >&2
  exit 3
fi

DEVICE="$1"
if [[ ! -c "${DEVICE}" ]]; then
  echo "ERROR: ${DEVICE} is not an existing character device." >&2
  exit 4
fi
if ! command -v udevadm >/dev/null 2>&1; then
  echo "ERROR: udevadm is unavailable." >&2
  exit 5
fi

ID_SERIAL="$({ udevadm info --query=property --name="${DEVICE}" || true; } |
  awk -F= '$1 == "ID_SERIAL" {print $2; exit}')"
if [[ -z "${ID_SERIAL}" ]]; then
  echo "ERROR: ${DEVICE} has no ID_SERIAL property; refusing an unstable rule." >&2
  echo "Inspect it with: udevadm info --query=property --name=${DEVICE}" >&2
  exit 6
fi
if [[ ! "${ID_SERIAL}" =~ ^[A-Za-z0-9._:+-]+$ ]]; then
  echo "ERROR: unsafe ID_SERIAL value: ${ID_SERIAL}" >&2
  exit 7
fi

RULE_FILE="/etc/udev/rules.d/99-agv-chassis.rules"
if [[ -e "${RULE_FILE}" ]]; then
  cp -a "${RULE_FILE}" "${RULE_FILE}.bak"
fi
printf '%s\n' \
  "SUBSYSTEM==\"tty\", ENV{ID_SERIAL}==\"${ID_SERIAL}\", SYMLINK+=\"chassis_driver\", GROUP=\"dialout\", MODE=\"0660\"" \
  >"${RULE_FILE}"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm settle

if [[ ! -e /dev/chassis_driver ]]; then
  echo "ERROR: rule installed but /dev/chassis_driver was not created." >&2
  echo "Unplug/replug the chassis USB cable, then check again." >&2
  exit 8
fi

echo "Stable chassis serial link installed:"
ls -l /dev/chassis_driver
echo "Rule: ${RULE_FILE}"

