#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../workspace" && pwd)"
if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" || ! -f "$WORKSPACE_DIR/install/limo-camera-enabled" ]]; then
  echo "Build the robot workspace first: bash build-robot.sh [--without-camera]." >&2
  exit 1
fi
source /opt/ros/foxy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
camera_args=()
if [[ "$(cat "$WORKSPACE_DIR/install/limo-camera-enabled")" == 0 ]]; then
  camera_args+=(use_camera:=false)
fi
exec ros2 launch custom_start limo_real.launch.py "${camera_args[@]}" "$@"
