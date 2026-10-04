#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../workspace" && pwd)"
PROFILE_DIR="$WORKSPACE_DIR/.limo"
with_camera=1
case "${1:-}" in
  "") ;;
  --without-camera) with_camera=0 ;;
  *) echo "Usage: bash build-robot.sh [--without-camera]" >&2; exit 2 ;;
esac
if [[ $# -gt 1 ]]; then
  echo "Usage: bash build-robot.sh [--without-camera]" >&2
  exit 2
fi
if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy is missing. Run bash install-deps.sh first." >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE_DIR/src/YDLidar-SDK/CMakeLists.txt" ]]; then
  echo "Missing workspace/src/YDLidar-SDK sources." >&2
  exit 1
fi
if [[ "$with_camera" == 1 ]]; then
  case "$(dpkg --print-architecture)" in
    arm64) camera_platform=arm64 ;;
    amd64) camera_platform=x64 ;;
    *) echo "Unsupported camera architecture." >&2; exit 1 ;;
  esac
  redist="$WORKSPACE_DIR/src/ros2_astra_camera/astra_camera/openni2_redist/$camera_platform"
  if [[ ! -f "$redist/libOpenNI2.so" ]] || ! compgen -G "$redist/OpenNI2/Drivers/*.so" >/dev/null; then
    echo "Missing Orbbec OpenNI2 runtime for $camera_platform: $redist" >&2
    echo "Restore the manufacturer's SDK files, or build with --without-camera." >&2
    exit 1
  fi
fi
source /opt/ros/foxy/setup.bash
mkdir -p "$PROFILE_DIR"
if [[ ! -f "$PROFILE_DIR/.gitignore" ]]; then
  printf '*\n' > "$PROFILE_DIR/.gitignore"
fi
prepare_args=()
[[ "$with_camera" == 1 ]] || prepare_args+=(--without-camera)
python3 "$SCRIPT_DIR/prepare-workspace.py" "$WORKSPACE_DIR/src" "$PROFILE_DIR/src" "${prepare_args[@]}"

# Build the C++ SDK before colcon: the driver does not declare this build edge.
# A private static SDK avoids replacing a system-wide or existing lab SDK.
cmake -S "$WORKSPACE_DIR/src/YDLidar-SDK" -B "$PROFILE_DIR/sdk-build" \
  -DCMAKE_INSTALL_PREFIX="$PROFILE_DIR/sdk-install" \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=ON -DSWIG_FOUND=FALSE
cmake --build "$PROFILE_DIR/sdk-build" --parallel "${BUILD_JOBS:-2}"
cmake --install "$PROFILE_DIR/sdk-build"
export CMAKE_PREFIX_PATH="$PROFILE_DIR/sdk-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd "$PROFILE_DIR"
colcon --log-base "$PROFILE_DIR/log" build \
  --base-paths "$PROFILE_DIR/src" \
  --build-base "$PROFILE_DIR/build" --install-base "$PROFILE_DIR/install" \
  --parallel-workers "${BUILD_JOBS:-2}" --cmake-args -DBUILD_TESTING=OFF
printf '%s\n' "$with_camera" > "$PROFILE_DIR/camera-enabled"
echo "Robot workspace built. Start it with bash launch-robot.sh."
