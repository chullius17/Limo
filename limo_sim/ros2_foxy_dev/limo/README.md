# LIMO robot setup: Ubuntu 20.04 and ROS 2 Foxy

This tutorial installs the dependencies **directly on the physical robot**.
It supports Ubuntu 20.04 on ARM64 and x86_64. Docker is not required.
The installation includes ROS 2 Foxy, chassis and sensor dependencies,
localization, navigation, SLAM and optional RViz tools. **Gazebo is excluded.**

Run the commands below yourself on the robot, either in a local terminal or
through SSH. The installation and hardware behavior have not been tested on
the robot here; no installation or build was executed while preparing these files.

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
~/limo_foxy/limo_sim/
  workspace/
    src/
  ros2_foxy_dev/
    limo/
```

Use the normal Linux account that will run the robot. Do not run the setup
scripts as root; they request `sudo` for system changes when needed.

```bash
cd ~/limo_foxy/limo_sim/ros2_foxy_dev/limo
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
Those binary files are currently missing from this checkout and are not
provided by apt. Obtain the matching SDK from the manufacturer or the robot's
known working installation.

Place the runtime in the original source package, matching the robot CPU:

| Robot architecture | Required source directory |
| --- | --- |
| `arm64` | `workspace/src/ros2_astra_camera/astra_camera/openni2_redist/arm64/` |
| `amd64` | `workspace/src/ros2_astra_camera/astra_camera/openni2_redist/x64/` |

That directory must include at least:

```text
libOpenNI2.so
OpenNI2/
  Drivers/
    ... manufacturer's driver libraries (.so)
```

Copy the complete matching runtime directory, including its configuration
files. Use binaries compatible with the robot's architecture and Ubuntu 20.04.
The generic Ubuntu OpenNI2 library is not a replacement for the bundled Orbbec
driver expected by this package.

If the camera files are not available yet, use the camera-free build in the
next step. Chassis, lidar and localization can still be prepared independently.

## 5. Build the physical robot workspace

With the camera runtime restored:

```bash
bash build-robot.sh
```

Or, to build chassis, lidar and localization without the camera:

```bash
bash build-robot.sh --without-camera
```

The script first checks required files. It then:

1. Copies the physical packages into `workspace/.limo/src`.
2. Enables the copied sensor drivers by omitting their `COLCON_IGNORE` files.
3. Removes simulator dependencies and simulator launch resources from the
   copied `custom_start` package.
4. Compiles and installs the C++ YDLidar SDK into `workspace/.limo/sdk-install`.
5. Compiles the selected ROS packages with colcon, using that SDK.

The SDK is built before the lidar driver because the original driver manifest
does not declare that build dependency. The SDK is installed privately, without
replacing any existing system installation.

Original sources and `COLCON_IGNORE` files are preserved. Build outputs are in
`workspace/.limo`, separate from the existing lab and WSL build directories.
Edit files in the original `workspace/src`, then rerun the build to update the
robot's generated workspace. The default build uses two workers to limit RAM
usage; for a lower-memory robot, use:

```bash
BUILD_JOBS=1 bash build-robot.sh
```

## 6. Launch the LIMO

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
source ~/limo_foxy/limo_sim/workspace/.limo/install/setup.bash
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
| `prepare-workspace.py` | Create robot-only copies of manifests or sources |
| `build-robot.sh` | Build the SDK and physical robot packages |
| `launch-robot.sh` | Load the environment and start the real robot launch |

The five directories in `workspace/src/ros2_ws` currently contain Python caches
but no source modules or package manifests. They are not included in this
build. Restore their original sources before adding controller, trajectory or
perception packages. This profile does not install ONNX Runtime or PyTurboJPEG:
the available physical launch and drivers do not import them, and the missing
perception sources must be checked before selecting robot-compatible versions.
