#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../workspace" && pwd)"
SDK_BUILD_DIR="$WORKSPACE_DIR/build/ydlidar_sdk"
SDK_INSTALL_DIR="$WORKSPACE_DIR/install/ydlidar_sdk"
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
package_paths=(
  "$WORKSPACE_DIR/src/limo_ros2/limo_msgs"
  "$WORKSPACE_DIR/src/limo_ros2/limo_base"
  "$WORKSPACE_DIR/src/limo_ros2/limo_bringup"
  "$WORKSPACE_DIR/src/limo_ros2/limo_description"
  "$WORKSPACE_DIR/src/custom_start"
  "$WORKSPACE_DIR/src/ydlidar_ros2_driver"
)
if [[ "$with_camera" == 1 ]]; then
  package_paths+=(
    "$WORKSPACE_DIR/src/ros2_astra_camera/astra_camera_msgs"
    "$WORKSPACE_DIR/src/ros2_astra_camera/astra_camera"
  )
fi
for package_path in "${package_paths[@]}"; do
  if [[ ! -f "$package_path/package.xml" ]]; then
    echo "Missing robot package manifest: $package_path/package.xml" >&2
    exit 1
  fi
done

# Build against Foxy, without chaining the previous generated robot workspace
# or another build of this workspace into the new install/setup.bash.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/foxy/setup.bash
for package_path in "${package_paths[@]}"; do
  if [[ -e "$package_path/COLCON_IGNORE" || -L "$package_path/COLCON_IGNORE" ]]; then
    rm -f "$package_path/COLCON_IGNORE"
  fi
done

# Build the C++ SDK before colcon: the driver does not declare this build edge.
# A private static SDK avoids replacing a system-wide or existing lab SDK.
cmake -S "$WORKSPACE_DIR/src/YDLidar-SDK" -B "$SDK_BUILD_DIR" \
  -DCMAKE_INSTALL_PREFIX="$SDK_INSTALL_DIR" \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=ON -DSWIG_FOUND=FALSE
cmake --build "$SDK_BUILD_DIR" --parallel "${BUILD_JOBS:-2}"
cmake --install "$SDK_BUILD_DIR"
export CMAKE_PREFIX_PATH="$SDK_INSTALL_DIR${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd "$WORKSPACE_DIR"
colcon --log-base "$WORKSPACE_DIR/log" build \
  --symlink-install \
  --base-paths "${package_paths[@]}" \
  --build-base "$WORKSPACE_DIR/build" --install-base "$WORKSPACE_DIR/install" \
  --parallel-workers "${BUILD_JOBS:-2}" --cmake-args -DBUILD_TESTING=OFF
printf '%s\n' "$with_camera" > "$WORKSPACE_DIR/install/limo-camera-enabled"
bash "$SCRIPT_DIR/setup-shell.sh"
echo "Robot workspace built. Start it with bash launch-robot.sh."
