#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(id -u)" == 0 ]]; then
  echo "Run as the normal robot user, without sudo in front of the command." >&2
  exit 1
fi
sudo usermod -aG dialout,video "$(id -un)"
sudo install -m 0644 "$SCRIPT_DIR/99-limo-hardware.rules" /etc/udev/rules.d/99-limo-hardware.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm trigger --subsystem-match=usb
echo "Log out and back in to apply groups. Reconnect the USB lidar and camera."
echo "Check that /dev/ydlidar identifies your lidar before launching the robot."
