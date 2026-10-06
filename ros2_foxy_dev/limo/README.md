# LIMO robot setup: Ubuntu 20.04 and ROS 2 Foxy

This tutorial installs the dependencies **directly on the physical robot**.
It supports Ubuntu 20.04 on ARM64 and x86_64. Docker is not required.
The installation includes ROS 2 Foxy, chassis and sensor dependencies,
localization, navigation, SLAM and optional RViz tools. **Gazebo is excluded.**

Run the commands below yourself on the robot, either in a local terminal or
through SSH. The installation and hardware behavior have not been tested on
the robot here; no installation or build was executed while preparing these files.

## 0. Robot info 

`NAME`:     jetson
`PASSWORD`: jetson

## 1. Check the robot and copy the project

Check the operating system and architecture:

```bash
cat /etc/os-release
dpkg --print-architecture
```

The system must be **Ubuntu 20.04**, and the architecture must be `arm64` or
`amd64`. The installer stops on other systems. Foxy is kept to match this
workspace; it is an older ROS distribution that is out of support.

Copy the complete project onto the robot, for example to:

```text
~/Limo/
  workspace/
    src/
  ros2_foxy_dev/
    limo/
```

Use the normal Linux account that will run the robot. Do not run the setup
scripts as root; they request `sudo` for system changes when needed.

```bash
cd ~/Limo/ros2_foxy_dev/limo
```

## 2. Install system dependencies

```bash
bash install-deps.sh
```

This installs the packages in `apt-packages.txt`, configures the ROS 2 apt
repository and installs additional dependencies from the physical packages'
manifests using rosdep. Internet access and sudo permissions are required.
It does not upgrade the operating system or install Docker.

Included components are:

- ROS 2 Foxy base, launch/XML support, message generation and build tools.
- LIMO chassis driver dependencies, TF and robot localization/EKF.
- YDLidar driver dependencies and the tools to compile the local C++ SDK.
- Orbbec/Astra/Dabai camera dependencies: OpenCV, Eigen, USB, libuvc,
  glog/gflags, image transport, camera info manager, cv_bridge and TF helpers.
- Nav2, SLAM Toolbox, Cartographer, teleop, twist mux and optional RViz tools.
- `rqt_image_view` for viewing the camera image topics.
- NumPy, OpenCV, SciPy and YAML for the available Python code.

`custom_start` declares both simulator and robot dependencies in its original
manifest. The helper creates temporary robot-only manifest copies, removing
`limo_car` and Gazebo dependencies before running rosdep. Original files remain
unchanged. A temporary apt preference blocks new Gazebo packages, including
transitive dependencies, during installation. Existing Gazebo installations
are not removed. If a future dependency requires Gazebo, installation fails
instead of adding it.

The current Foxy [Nav2 bringup manifest](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_bringup/bringup/package.xml)
does not declare Gazebo as a dependency.

## 3. Configure hardware permissions

```bash
bash setup-devices.sh
```

This adds your user to `dialout` and `video`, installs the supplied udev rules
and reloads them. **Log out and log back in** to apply group membership.
If using SSH, disconnect and reconnect. Reconnect USB sensors as needed.

Check the devices:

```bash
id
ls -l /dev/ttyTHS1
ls -l /dev/ydlidar
lsusb
```

The default chassis port is `/dev/ttyTHS1`. The lidar launch uses `/dev/ydlidar`,
which the rules create for the adapter IDs used by the workspace's driver.
If the adapter has a different USB ID, update `99-limo-hardware.rules` to match
it and rerun `setup-devices.sh`.

If multiple adapters share those IDs, identify the lidar by its serial number
and refine the rule so `/dev/ydlidar` points to the correct device. Camera USB
access is granted to the `video` group for Orbbec vendor ID `2bc5`.

## 4. Restore the camera runtime

The Astra source driver needs the **manufacturer's Orbbec OpenNI2 runtime**.
The required binaries are missing from this checkout. The procedure below
restores the ARM64 runtime from the
[official Orbbec ROS 2 camera repository](https://github.com/orbbec/ros2_astra_camera/tree/master/astra_camera/openni2_redist/arm64).
Alternatively, obtain a matching runtime from the manufacturer's SDK or the
robot's known working installation.

### Check an existing installation

On the robot, look for an existing OpenNI2 library and camera drivers:

```bash
find /home/jetson /opt /usr/local /usr/lib \
  -name 'libOpenNI2.so*' 2>/dev/null
find /usr/lib /usr/local /opt /home/jetson \
  -path '*/OpenNI2/Drivers/*' -type f 2>/dev/null
```

Replace `/home/jetson` if your robot account has a different home directory.
Finding `/usr/lib/libOpenNI2.so` alone is not enough. During setup on the
Jetson Nano, the system driver directory contained `libPSLink.so.0`,
`libOniFile.so.0`, `libDummyDevice.so.0` and `libPS1080.so.0`, but no
`liborbbec.so`. That installation did not provide the Orbbec driver needed
for this procedure.

### Download and copy the ARM64 runtime

Check the robot architecture:

```bash
dpkg --print-architecture
```

**Continue with the following commands only if the result is `arm64`.**
Download the official repository into a separate directory, without `sudo`:

```bash
git clone --depth 1 --branch master \
  https://github.com/orbbec/ros2_astra_camera.git \
  ~/orbbec-astra-runtime
```

If `~/orbbec-astra-runtime` already contains this checkout, reuse it instead
of cloning into the same directory again. The commands below assume the
project is in `~/Limo`; adjust that path if needed.

```bash
cd ~/Limo/workspace/src/ros2_astra_camera/astra_camera
```

If `cd` succeeds, copy the complete runtime directory, including its
configuration files:

```bash
mkdir -p openni2_redist/arm64
cp -a ~/orbbec-astra-runtime/astra_camera/openni2_redist/arm64/. \
  openni2_redist/arm64/
```

The runtime belongs in the original source package, matching the robot CPU:

| Robot architecture | Required source directory |
| --- | --- |
| `arm64` | `workspace/src/ros2_astra_camera/astra_camera/openni2_redist/arm64/` |
| `amd64` | `workspace/src/ros2_astra_camera/astra_camera/openni2_redist/x64/` |

The ARM64 directory should contain:

```text
openni2_redist/arm64/
  libOpenNI2.so
  OpenNI.ini
  OpenNI2/
    Drivers/
      liborbbec.so
      libOniFile.so
      orbbec.ini
      OniFile.ini
```

For `amd64`, obtain the matching `x64` runtime instead of copying ARM64 files.
Use binaries compatible with the robot's architecture and Ubuntu 20.04.
The generic Ubuntu OpenNI2 library is not a replacement for the bundled Orbbec
driver expected by this package.

### Verify the runtime before building

From the same `astra_camera` directory, run:

```bash
file openni2_redist/arm64/libOpenNI2.so \
  openni2_redist/arm64/OpenNI2/Drivers/liborbbec.so
ldd openni2_redist/arm64/libOpenNI2.so
ldd openni2_redist/arm64/OpenNI2/Drivers/liborbbec.so
```

`file` should identify both libraries as ARM aarch64 ELF binaries. The `ldd`
output must not contain `not found` or version errors. Resolve any missing
dependencies before building.

The `ldd` check for `liborbbec.so` succeeded on the Jetson Nano during this
setup: all listed dependencies resolved through the system's AArch64
libraries. This confirms that the driver library's dependencies are
available; camera detection and image streaming still need to be checked
after launching ROS.

Return to the setup directory before following step 5:

```bash
cd ~/Limo/ros2_foxy_dev/limo
```

If the camera files are not available yet, use the camera-free build in the
next step. Chassis, lidar and localization can still be prepared independently.

## 5. Build the physical robot workspace for the first time

Before building, remove an unused JSON include from the original camera
source to avoid a missing-header error. Run from `ros2_foxy_dev/limo`:

```bash
sed -i '/^#include <nlohmann\/json\.hpp>$/d' \
  ../../workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_info.cpp
```

This command is safe to repeat if the include has already been removed.

With the camera runtime restored:

```bash
bash build-robot.sh
```

Or, to build chassis, lidar and localization without the camera:

```bash
bash build-robot.sh --without-camera
```

The script first checks required files. It then:

1. Checks the physical packages in the original `workspace/src`.
2. Enables those packages by removing any `COLCON_IGNORE` files they contain.
3. Compiles the C++ YDLidar SDK in `workspace/build/ydlidar_sdk` and installs it
   into `workspace/install/ydlidar_sdk`.
4. Compiles the selected ROS packages directly from their original source
   directories with `colcon build --symlink-install`, using that SDK.
5. Configures the ROS environment and robot aliases in your `~/.bashrc`.

The SDK is built before the lidar driver because the original driver manifest
does not declare that build dependency. The SDK is installed privately, without
replacing any existing system installation.

Build outputs use the standard `workspace/build`, `workspace/install` and
`workspace/log` directories. The script selects only the physical packages;
simulator packages are excluded from discovery. Source files and manifests
are used directly, without creating package copies. Edit files in
`workspace/src`, then rerun the build. The default build uses two workers to
limit RAM usage; for a lower-memory robot, use:

```bash
BUILD_JOBS=1 bash build-robot.sh
```

### Bash environment and aliases

After a successful build, `build-robot.sh` runs `setup-shell.sh`. This adds a
managed block to your `~/.bashrc` that loads `/opt/ros/foxy/setup.bash`, then
`workspace/install/setup.bash` when that file exists, and finally
`aliases/limo.bash`. The script replaces its own block when rerun, without
duplicating it or changing other shell settings. The original `.bashrc` is
saved once as `~/.bashrc.limo-backup` before the first change.

To configure the shell before the first build, or refresh the paths after
moving the project, run from `ros2_foxy_dev/limo`:

```bash
bash setup-shell.sh
```

Open a new terminal, or load the configuration into the current one:

```bash
source ~/.bashrc
```

| Alias | Action |
| --- | --- |
| `cb_limo` | Run the robot build wrapper with `colcon build --symlink-install` |
| `wsp` | Change to the project's `workspace` directory |
| `start` | Launch the real robot through `launch-robot.sh` |

The build alias compiles the YDLidar SDK before colcon, then builds the physical
packages directly in `workspace`. The directory selected by `wsp` contains
`src`, `build`, `install` and `log`. Build and launch options can be passed
after the alias, for example:

```bash
BUILD_JOBS=1 cb_limo --without-camera
start use_camera:=false use_lidar:=false
```

After rebuilding, reload `~/.bashrc` or open a new terminal to load the updated
workspace environment. `start` also loads that environment itself.

## 6. Launch the LIMO for the first time

From `ros2_foxy_dev/limo`, run:

```bash
bash launch-robot.sh
```

The wrapper loads Foxy and the robot workspace, then starts:

```text
custom_start/limo_real.launch.py
```

This starts the chassis, IMU transform, lidar, camera and EKF localization.
RViz is disabled by default. A camera-free build automatically passes
`use_camera:=false`.

The launch uses the workspace's current **Ackermann configuration** and
steering calibration. Confirm that these settings match the physical chassis.
It does not automatically start navigation or SLAM.

If the chassis uses a USB serial adapter, pass its **device name without
`/dev/`**, for example:

```bash
bash launch-robot.sh port_name:=ttyUSB0
```

The current chassis driver adds `/dev/` itself when the name contains `tty`.
The lidar port is configured separately in
`workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml`.

To launch only the chassis and EKF while troubleshooting sensor setup:

```bash
bash launch-robot.sh use_camera:=false use_lidar:=false
```

For RViz on a local desktop session:

```bash
bash launch-robot.sh open_rviz:=true
```

Use **Ctrl+C** in the launch terminal to stop the nodes.

## 7. Check ROS topics

Open a second terminal on the robot and load the same environment:

```bash
source /opt/ros/foxy/setup.bash
source ~/Limo/workspace/install/setup.bash
ros2 topic list
```

Check topics for the components you enabled:

```bash
ros2 topic hz /odom
ros2 topic hz /limo/imu
ros2 topic hz /odometry/filtered
ros2 topic hz /scan
ros2 topic hz /rgb/image_raw
ros2 topic hz /depth_camera/depth/image_raw
```

Stop each topic check with Ctrl+C before starting the next one.

To display an image topic, run from a local desktop terminal with the ROS
environment loaded:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/rgb/image_raw` or `/depth_camera/depth/image_raw` in the topic list.
See the [rqt_image_view package documentation](https://index.ros.org/p/rqt_image_view/).

The current camera profile disables point-cloud publication, so `/depth/points`
is not expected by default. Nav2 and SLAM require their own launch commands and
map/settings in addition to this hardware bringup.

For another ROS computer on the same network, use the same `ROS_DOMAIN_ID`
on both systems. This wrapper defaults to domain `0`; to change it:

```bash
ROS_DOMAIN_ID=10 bash launch-robot.sh
```

Use the same value in the second terminal before inspecting topics.

## Files and source limitations

| File | Purpose |
| --- | --- |
| `install-deps.sh` | Install Ubuntu/ROS system dependencies and run rosdep |
| `apt-packages.txt` | Explicit package list without Gazebo |
| `no-gazebo.pref` | Block new Gazebo packages during dependency installation |
| `setup-devices.sh` | Install hardware rules and configure user groups |
| `99-limo-hardware.rules` | Chassis, lidar and camera permissions |
| `prepare-workspace.py` | Create temporary robot-only manifests for dependency installation |
| `build-robot.sh` | Build the SDK and physical robot packages |
| `setup-shell.sh` | Configure Foxy, the robot install and aliases in `.bashrc` |
| `aliases/limo.bash` | Define `cb_limo`, `wsp` and `start` |
| `launch-robot.sh` | Load the environment and start the real robot launch |

The five directories in `workspace/src/ros2_ws` currently contain Python caches
but no source modules or package manifests. They are not included in this
build. Restore their original sources before adding controller, trajectory or
perception packages. This profile does not install ONNX Runtime or PyTurboJPEG:
the available physical launch and drivers do not import them, and the missing
perception sources must be checked before selecting robot-compatible versions.
