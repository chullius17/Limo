#!/usr/bin/env bash
set -e

# Only this profile's named volume; never reuse the lab/host build or install.
if [[ -d /workspace/.lab_wsl ]]; then
  sudo chown "$(id -u):$(id -g)" /workspace/.lab_wsl
fi
source /opt/ros/foxy/setup.bash
if [[ -f /workspace/.lab_wsl/install/setup.bash ]]; then
  source /workspace/.lab_wsl/install/setup.bash
fi
exec "$@"
