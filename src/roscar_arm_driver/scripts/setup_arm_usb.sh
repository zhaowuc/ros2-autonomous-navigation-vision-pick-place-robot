#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this script with sudo." >&2
  exit 1
fi

RULE_SOURCE=$(cd "$(dirname "${BASH_SOURCE[0]}")/../udev" && pwd)/99-roscar-arm.rules
[[ -f ${RULE_SOURCE} ]] || { echo "Missing udev rule: ${RULE_SOURCE}" >&2; exit 1; }

systemctl mask --now brltty-udev.service brltty.service
install -m 0644 "${RULE_SOURCE}" /etc/udev/rules.d/99-roscar-arm.rules
udevadm control --reload-rules
modprobe ch341

for interface in /sys/bus/usb/devices/*:1.0; do
  device=${interface%:1.0}
  [[ -r ${device}/idVendor && -r ${device}/idProduct ]] || continue
  [[ $(<"${device}/idVendor") == 1a86 && $(<"${device}/idProduct") == 7523 ]] || continue
  if [[ ! -L ${interface}/driver ]]; then
    printf '%s' "$(basename "${interface}")" > /sys/bus/usb/drivers/ch341/bind
  fi
done

udevadm trigger --subsystem-match=tty --action=add
udevadm settle
test -e /dev/roscar_arm
ls -l /dev/roscar_arm
echo ROSCAR_ARM_USB_READY
