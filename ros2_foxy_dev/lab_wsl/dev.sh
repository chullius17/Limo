#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Load only explicit profile options; Compose itself reads .env for substitutions.
if [[ -f .env ]]; then
  while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%$'\r'}"
    case "$line" in
      ENABLE_GPU=0|ENABLE_GPU=1|ENABLE_HOST_NETWORK=0|ENABLE_HOST_NETWORK=1)
        export "$line"
        ;;
    esac
  done < .env
fi
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"
compose=(docker compose -f compose.yaml)
[[ "${ENABLE_GPU:-0}" != 1 ]] || compose+=(-f compose.gpu.yaml)
[[ "${ENABLE_HOST_NETWORK:-0}" != 1 ]] || compose+=(-f compose.host.yaml)

usage() {
  echo "Uso: $0 {build|up|shell|build-workspace|deps|check|gpu|config|down}" >&2
}

action="${1:-shell}"
case "$action" in
  build|up|shell|build-workspace|deps|check|gpu|config|down) ;;
  *) usage; exit 2 ;;
esac

if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
  echo "Docker Desktop/Compose non disponibile. Abilitare WSL Integration per questa distro." >&2
  exit 1
fi
if [[ "$action" == config ]]; then
  "${compose[@]}" config
  exit
fi
if ! docker info >/dev/null 2>&1; then
  echo "Docker non accessibile: avviare Docker Desktop e abilitare WSL Integration." >&2
  exit 1
fi
if [[ "$action" != build && "$action" != down ]]; then
  if [[ ! -d /mnt/wslg/.X11-unix ]]; then
    echo "WSLg non disponibile. Eseguire wsl --update su Windows e riavviare WSL." >&2
    exit 1
  fi
fi

case "$action" in
  build) "${compose[@]}" build ;;
  up) "${compose[@]}" up -d --build ;;
  shell)
    "${compose[@]}" up -d
    "${compose[@]}" exec foxy bash
    ;;
  build-workspace)
    "${compose[@]}" up -d
    "${compose[@]}" exec foxy bash -c \
      'source /opt/ros/foxy/setup.bash && colcon --log-base /workspace/.lab_wsl/log build --symlink-install --build-base /workspace/.lab_wsl/build --install-base /workspace/.lab_wsl/install --parallel-workers 2'
    ;;
  deps)
    "${compose[@]}" up -d
    "${compose[@]}" exec foxy workspace-deps
    ;;
  check)
    "${compose[@]}" up -d
    "${compose[@]}" exec -T foxy /usr/local/bin/ros-entrypoint bash -c \
      'python3 -c "import rclpy, cv2, numpy, scipy, yaml, matplotlib, onnxruntime, turbojpeg; from PyQt5 import QtCore; from cv_bridge import CvBridge; from turbojpeg import TurboJPEG; TurboJPEG(); print(\"Import e TurboJPEG OK\")" && python3 -m pip check && ros2 pkg prefix gazebo_ros && ros2 pkg prefix nav2_bringup && ros2 pkg prefix cartographer_ros && pkg-config --modversion libuvc libglog && test -f /usr/local/lib/libydlidar_sdk.so && colcon list --base-paths src'
    ;;
  gpu)
    if [[ "${ENABLE_GPU:-0}" != 1 ]]; then
      echo "Impostare ENABLE_GPU=1 in .env prima di usare gpu (richiede NVIDIA)." >&2
      exit 1
    fi
    "${compose[@]}" up -d
    "${compose[@]}" exec -T foxy nvidia-smi
    ;;
  down) "${compose[@]}" down ;;
esac
