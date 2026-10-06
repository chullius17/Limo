# LIMO on ROS 2 Foxy: Gazebo Setup and Hardware Integration

This document reconstructs the work needed to run the LIMO robot in Gazebo
Classic on ROS 2 Foxy and bring up its chassis, lidar, RGB/depth camera, and IMU
on the physical robot. It explains the original compatibility problems, the
changes that addressed them, and the later corrections needed to make commands
and sensor data consistent across the two environments.

The history covers the `foxy` branch from **3 September to 4 October 2026**,
ending at commit `1a62922`. It focuses on `custom_start`, `limo_ros2`,
`YDLidar-SDK`, `ydlidar_ros2_driver`, `ros2_astra_camera`, and the Docker setup.
Changes inside `ros2_ws/` and the `navigation2/` source tree are outside its scope.

Validation results below come from the reports committed during development.
They describe the tests performed at the time; they are not new measurements
or a claim that every remaining hardware issue has been resolved.

## Contents

- [Starting point](#starting-point)
- [Development timeline](#development-timeline)
- [Foxy environment and workspace](#foxy-environment-and-workspace)
- [Gazebo simulation](#gazebo-simulation)
- [Physical robot and sensors](#physical-robot-and-sensors)
- [Configuration and launch commands](#configuration-and-launch-commands)
- [Validation and remaining limitations](#validation-and-remaining-limitations)
- [Supporting reports](#supporting-reports)

## Starting point

The initial Foxy workspace contained an Ackermann configuration in
[`limo_description`](../limo_ros2/limo_description/urdf/limo_ackerman.gazebo)
that referenced ROS 1 plugins:

```text
libgazebo_ros_control.so
libgazebo_ros_ackerman.so
```

The original setup report identifies these as incompatible with the intended
ROS 2 simulation. The working Gazebo path was therefore built by importing
`limo_car` and `custom_start` from the working Humble environment and configuring
the ROS 2 plugin:

```text
libgazebo_ros_ackermann_drive.so
```

This established a working Ackermann model, a custom circuit, simulated sensors,
and an EKF. Later work corrected the plugin's command interface and integrated
the physical robot. The compatibility finding concerns the Ackermann path in
the initial snapshot; it is not an assessment of every launch in the upstream
repository.

## Development timeline

| Date | Commit(s) | Main change |
| --- | --- | --- |
| 3 Sep 2026 | `16fa470` | Initial Foxy workspace and Docker environment. |
| 3 Sep 2026 | `27b0c10` | Standard Colcon layout, corrected workspace mount, imported Ackermann model, circuit, simulated sensors, and EKF. |
| 3 Sep 2026 | `3a2a8f0`, `e2fdc8c` | Additional pipeline dependencies and container recreation after an image rebuild. |
| 7 Sep 2026 | `717253c` | Added hardware drivers and the physical robot launch. |
| 7 Sep 2026 | `cd0525c` | Selected the DaBai U3 profile, corrected depth resolution, added camera mount TF, and fixed the OpenNI2 runtime name. |
| 8 Sep 2026 | `b9dcd14` | Added an Ackermann command-mode override for the physical chassis. |
| 12 Sep 2026 | `0e500b0`, `a82139f` | Camera synchronization, disabled driver point cloud, component-registration fix, and static optical TFs. |
| 15 Sep 2026 | `4cc0076` | Added the physical-camera TF convention to the simulation. |
| 18 Sep 2026 | `d43c9de` | Added the laboratory Docker profile. |
| 22 Sep 2026 | `045fe7d` | Added the Gazebo Twist adapter and remapped the plugin command topic. |
| 23 Sep 2026 | `e892edc` | Corrected odometry yaw initialization and IMU angle unwrapping. |
| 24 Sep 2026 | `0aa6c66` | Corrected real-robot EKF inputs, lidar scan geometry, and invalid ranges. |
| 28 Sep 2026 | `caf9d9d` | Limited SDK warning output and selected the matching SDK library during linking. |
| 29–30 Sep 2026 | `14e4fc0`, `c2d0676` | Disabled the camera IR stream and documented the camera geometry investigation. |
| 1 Oct 2026 | `d56c5d7` | Exposed steering calibration scales and selected new values for the physical chassis. |
| 1 Oct 2026 | `ae814b0`, `4ce8f01` | Moved launch defaults into YAML profiles and generalized camera argument forwarding. |
| 4 Oct 2026 | `1a62922` | Added an obstacle world and a launch that reuses the circuit setup. |

## Foxy environment and workspace

### Docker setup

The development image uses Ubuntu 20.04 with ROS 2 Foxy Desktop, Gazebo 11,
`gazebo_ros_pkgs`, Xacro, Colcon, and rosdep. The initial simulation setup also
added `robot_localization` and `rqt_robot_steering`.

The container configuration provides NVIDIA GPU access, X11 forwarding, and
host networking and IPC. The workspace mount was corrected to:

```bash
-v "${SCRIPT_DIR}/../workspace:/workspace"
```

The startup script was subsequently changed to compare the existing container's
image with the rebuilt image and recreate the container when they differ.

The laboratory profile added Docker Compose, USB/serial device access, and an
application user whose UID/GID matches the host. Its instructions are in the
[`lab` README](../../../ros2_foxy_dev/lab/README.md).

### Workspace layout

The ROS packages were moved into the standard Colcon source directory:

```text
Limo/
├── ros2_foxy_dev/
│   ├── asus/
│   │   ├── Dockerfile
│   │   ├── build.sh
│   │   ├── run.sh
│   │   └── start_limo_docker.sh
│   ├── lab/
│   ├── lab_wsl/
│   └── limo/
└── workspace/
    ├── src/
    │   ├── custom_start/
    │   ├── limo_ros2/
    │   ├── YDLidar-SDK/
    │   ├── ydlidar_ros2_driver/
    │   └── ros2_astra_camera/
    ├── build/
    ├── install/
    └── log/
```

The initial report also records host-side shell aliases for entering the Foxy
container and restoring workspace file ownership. These are local environment
changes rather than ROS package functionality.

## Gazebo simulation

### Ackermann model and controller

**Introduced in `27b0c10`.**

The imported `limo_car` model supplies four wheel joints and two front steering
joints. Its Gazebo configuration uses the ROS 2 Ackermann drive plugin, publishes
odometry on `/odom`, and includes a joint-state publisher for the steering
joints. The setup report also records removal of a reference to a nonexistent
steering-wheel joint.

The relevant files are:

- [`ackermann.xacro`](../limo_ros2/limo_car/gazebo/ackermann.xacro): robot geometry and drive plugin.
- [`ackermann_with_sensor.xacro`](../limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro): sensor assembly and mounting joints.
- [`sensor.xacro`](../limo_ros2/limo_car/gazebo/sensor.xacro): lidar, camera, and IMU plugins.
- [`ackermann.launch.py`](../limo_ros2/limo_car/launch/ackermann.launch.py): robot description, state publisher, and command adapter.

### Circuit and launch orchestration

[`limo_circuit.launch.py`](launch/limo_circuit.launch.py) coordinates the robot
description, Gazebo server, optional graphical client, robot spawning, camera
TFs, and EKF.

The world includes a textured circuit plane and surrounding walls. The package
installs the world, model definitions, materials, and textures, and adds its
model directory to `GAZEBO_MODEL_PATH`. The default spawn position is
`(0, 0, 0.30)` metres with zero yaw.

The initial report records a startup failure caused by another Gazebo process
occupying port `11345`. Releasing that port resolved the server and spawn
failures in that session.

### Simulated sensors and EKF

The following settings were introduced with the model:

| Component | Plugin or source | Configuration |
| --- | --- | --- |
| Lidar | `libgazebo_ros_ray_sensor.so` | `/scan`, 720 samples, approximately 240° field of view, 8 Hz, 0.2–8 m range. |
| RGB/depth camera | `libgazebo_ros_camera.so` | 320 × 240 images, 20 Hz, approximately 60° horizontal field of view. |
| IMU | `libgazebo_ros_imu_sensor.so` | `/limo/imu`, 100 Hz. |
| Raw odometry | Ackermann drive plugin | `/odom`. |
| Filtered odometry | `robot_localization` | `/odometry/filtered`, planar EKF configured at 100 Hz. |

The shared [`ekf.yaml`](config/ekf.yaml) combines odometry with IMU yaw rate.
The EKF publishes `odom → base_link`; the drive plugin's odometry TF is disabled
to give that transform a single publisher. The circuit launch enables simulated
time by default.

### Camera TF alignment

**Added in `4cc0076`.**

The simulation was extended with the camera TF convention used by the physical
robot:

```text
base_link → camera_link → depth_camera_frame_optical
```

The default camera mount is `(0.10, 0, 0.065)` metres relative to `base_link`.
The optical-frame rotation uses roll and yaw of `-π/2`. Foxy's positional
`static_transform_publisher` arguments use the order **yaw, pitch, roll**.

This change aligns the coordinate-frame convention. It does not make the
simulated camera resolution, intrinsics, or lidar coverage identical to the
physical sensors. The launch camera offsets configure the added TF chain; the
simulated sensor mounting joints are defined separately in the Xacro model.

### Correcting the Gazebo command interface

**Added in `045fe7d`.**

The Foxy Ackermann plugin expects a steering angle in `Twist.angular.z`, whereas
the application uses the standard `/cmd_vel` meaning: yaw rate in radians per
second. Feeding the standard command directly into the plugin produced an
incorrect turning response.

[`gazebo_twist_adapter.py`](../limo_ros2/limo_car/scripts/gazebo_twist_adapter.py)
converts between the two interfaces:

```text
/cmd_vel                  /cmd_vel_gazebo
speed + yaw rate  →  adapter  →  speed + plugin steering angle  →  Gazebo
```

For a nonzero speed, the adapter computes:

```text
plugin_angle = atan(wheelbase × yaw_rate / abs(speed))
```

The absolute speed accounts for the plugin's internal reverse-direction sign
handling. The adapter also limits steering, rejects nonfinite commands, stops
for near-zero speed, and sends a stop after a 0.5-second command timeout. The
plugin subscribes to `/cmd_vel_gazebo` through ROS remapping; the legacy
`command_topic` tag was removed.

Regression tests cover forward/reverse curvature, steering saturation,
invalid inputs, topic isolation, and timeout behavior.

## Physical robot and sensors

### Unified hardware launch

**Introduced in `717253c`.**

[`limo_real.launch.py`](launch/limo_real.launch.py) brings up the physical chassis,
lidar, camera, sensor transforms, and EKF. The original target was a LIMO with an
NVIDIA Jetson Nano running ROS 2 Foxy.

| Component | Driver or source | Main interface |
| --- | --- | --- |
| Chassis and raw odometry | `limo_base` | Serial device `ttyTHS1`, `/cmd_vel`, `/odom`, `/limo_status`. |
| Integrated IMU | Chassis protocol through `limo_base` | Remapped to `/limo/imu`. |
| YDLidar | `ydlidar_ros2_driver` and `YDLidar-SDK` | `/dev/ydlidar`, `/scan`, `laser_frame`. |
| Orbbec DaBai U3 | `astra_camera` | `/rgb/image_raw`, `/depth_camera/depth/image_raw`, and camera calibration topics. |
| Fused odometry | `robot_localization` | `/odometry/filtered` and `odom → base_link`. |

The physical launch disables the chassis odometry TF so the EKF remains its
publisher. It uses real time and can start without the camera or lidar for
diagnosis.

### DaBai U3 camera compatibility

**Main fix in `cd0525c`; subsequent refinements in `0e500b0`, `a82139f`, and `14e4fc0`.**

The first hardware launch used the generic `astra.launch.xml` profile. It was
replaced with `dabai_u3.launch.xml`, with RGB at **640 × 480** and depth at
**640 × 400**. A configurable `base_link → camera_link` transform was added.

Two runtime problems also required attention on the Jetson:

- The bundled OpenNI2 ARM64 runtime was missing and had to be restored,
  including `libOpenNI2.so`, `OpenNI2/Drivers/liborbbec.so`, and `orbbec.ini`.
- The executable requested `libOpenNI2.so.0`, but the Orbbec library was initially
  installed only as `libOpenNI2.so`. The CMake installation now provides the
  requested runtime name so the loader can select the workspace library.

The hardware report documents installation of the Orbbec USB rules and the
missing `robot_localization` dependency, followed by device enumeration and
library-resolution checks.

Later changes disabled the driver's point cloud, requested RGB/depth
synchronization, moved ROS component registration from the header into the
implementation file, and set `tf_publish_rate: 0.0`. The last setting publishes
the rigid optical transforms on `/tf_static`, avoiding time-extrapolation
failures. The IR stream was disabled on 29 September.

Synchronization aligns acquisition timing; it does not geometrically register
depth pixels to RGB pixels. That separate issue was investigated in the
[camera report](README_camera_reale.md), with application changes outside the
scope of this README.

### Chassis motion mode and steering calibration

**Command-mode override in `b9dcd14`; calibration parameters in `d56c5d7`.**

Although the robot had been mechanically converted to Ackermann, its firmware
still reported Mecanum mode (`motion_mode: 2`). The ROS driver therefore selected
the wrong command conversion.

The driver gained an independent `motion_mode` parameter:

| Value | Command conversion |
| --- | --- |
| `-1` | Use the mode reported by the chassis. |
| `0` | Differential. |
| `1` | Ackermann. |
| `2` | Mecanum. |

The physical launch selects `1`. `/limo_status` continues to show raw chassis
feedback, so its mode can remain `2` while the command conversion is Ackermann.
The override changes the ROS conversion and does not reprogram the firmware.

Later measurements showed less steering than requested. The fixed scale of
`2.47` was exposed as `steering_left_scale` and `steering_right_scale`, and the
physical profile selected **1.0** for both. The generic driver defaults remain
**2.47**. Commands divide the inner-wheel angle by the scale; feedback uses the
same scale in the opposite direction. The parameters are validated and fixed
at startup, and the inner-wheel angle limit remains 28°.

The report records successful serial tests and a Jetson build. A second physical
curve test was still needed to validate the new calibration.

### Odometry yaw and EKF inputs

**Driver fix in `e892edc`; EKF correction in `0aa6c66`.**

The yaw integration was changed to initialize from the first IMU sample, keep
state per driver instance, retain slow rotations, and unwrap the ±180° boundary
using `std::remainder`. This removed a per-sample deadband that discarded real
motion.

A separate problem affected fusion: the chassis odometry reported zero angular
velocity even during turns, while the IMU measured rotation. The real EKF was
changed to exclude odometry yaw rate and take it from `/limo/imu`. Odometry pose
yaw and the other previously enabled inputs remain in use.

An explicit root namespace was added to the EKF node to ensure the intended
Foxy parameter overrides applied. The override was later moved into
[`ekf_real.yaml`](config/ekf_real.yaml), loaded after the shared configuration.

The recorded maximum filtered-yaw variation during stops decreased from
**5.83° to 0.21°** in the comparison with the active correction. This validates
the observed EKF behavior in those recordings; the chassis angular-velocity
field and its zero velocity covariances were not repaired by this change.

### Lidar geometry, invalid ranges, and SDK selection

**Scan corrections in `0aa6c66`; SDK refinements in `caf9d9d`.**

The hardware YAML used `resolution_fixed`, but the driver reads
`fixed_resolution`. Correcting the name stabilized scan size within a session,
which matters for downstream scan matching. The number of rays is estimated
at startup and can differ between sessions.

Missing returns and unfilled angular bins could still appear as zero ranges.
The driver now applies `invalid_range_is_inf` after binning, converting zeros,
negative values, nonfinite readings, and out-of-range values to positive
infinity. Valid ranges and their angular indices are preserved.

The real lidar profile enables this behavior. Tests cover invalid samples,
empty bins, and compatibility when the option is disabled. The mapping report
records improved replay results while retaining the need for confirmation on
the moving robot.

The SDK's repeated fixed-scan-size warnings were subsequently rate-limited.
The ROS driver's CMake configuration also resolves the SDK library next to its
selected package configuration to avoid linking an obsolete system copy.

## Configuration and launch commands

### YAML profiles

**Introduced in `ae814b0` and extended in `4ce8f01`.**

| File | Purpose |
| --- | --- |
| [`limo_circuit.yaml`](config/limo_circuit.yaml) | Gazebo world, GUI, simulated time, camera TF, and spawn pose. |
| [`limo_real.yaml`](config/limo_real.yaml) | Serial device, sensor switches, camera TF, steering scales, RViz, and camera driver arguments. |
| [`ekf.yaml`](config/ekf.yaml) | Shared EKF configuration. |
| [`ekf_real.yaml`](config/ekf_real.yaml) | Real-robot overrides loaded after the shared EKF file. |

Launch arguments take precedence over the corresponding `launch` settings in
the selected profile. The `camera_driver` section forwards its entries to
`dabai_u3.launch.xml`. Distances are in metres and rotations are in radians.

The launch files read the installed package configuration. Rebuild and source
the workspace after changes, or pass an explicit source profile with
`config_file`.

### Simulation

With Docker, NVIDIA support, and X11 configured on the host, start the original
development container from the repository root:

```bash
cd ros2_foxy_dev/asus
./build.sh
./start_limo_docker.sh
```

Inside the configured container, the original build and launch sequence is:

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_start limo_circuit.launch.py
```

A full workspace build requires the dependencies of all packages present in
the checkout, including the hardware camera runtime. For an already provisioned
workspace, the simulation packages can also be rebuilt selectively:

```bash
colcon build --packages-select limo_car custom_start --symlink-install
source install/setup.bash
ros2 launch custom_start limo_circuit.launch.py gui:=false
```

### Physical robot

After provisioning the drivers, native libraries, and device permissions on the
Jetson, run from the physical robot's workspace:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch custom_start limo_real.launch.py
```

For a reduced session without the camera:

```bash
ros2 launch custom_start limo_real.launch.py use_camera:=false
```

Useful checks after startup include:

```bash
ros2 param get /limo_base motion_mode
ros2 param get /limo_base steering_left_scale
ros2 param get /limo_base steering_right_scale
ros2 param get /ekf_filter_node odom0_config
ros2 topic hz /scan
ros2 topic hz /limo/imu
ros2 topic hz /rgb/image_raw
ros2 topic hz /depth_camera/depth/image_raw
ldd install/astra_camera/lib/astra_camera/astra_camera_node
```

The physical profile selects motion mode `1` and steering scales `1.0`. The
EKF's `odom0_config` must have index 11 (`vyaw`) disabled. For the camera, verify
that `libOpenNI2.so.0` resolves to the intended workspace runtime.

An alternative profile can be selected for either launch:

```bash
ros2 launch custom_start limo_real.launch.py config_file:=/absolute/path/limo_real.yaml
ros2 launch custom_start limo_circuit.launch.py config_file:=/absolute/path/limo_circuit.yaml
```

### Historical obstacle-world extension

Commit `1a62922` added `limo_obstacles.yaml`, `limo_obstacles.launch.py`, and
`limo_obstacle_world.world`. The wrapper launch reuses the circuit setup with
a different profile. These files belong to the documented `foxy` snapshot;
their availability depends on the checkout being used.

## Validation and remaining limitations

The committed reports record the following checks:

- Docker image and simulation-package builds.
- Shell/Python syntax checks, Xacro generation, and URDF validation.
- Headless Gazebo startup, circuit loading, robot spawning, and controller loading.
- Controller command subscription and odometry publication.
- Camera enumeration, runtime-library resolution, and image streaming.
- Recorded EKF comparisons and lidar scan/replay diagnosis.
- Lidar invalid-range tests and steering serial tests.

The development history also leaves specific limits on reproducibility and
validation:

- The OpenNI2 binary runtime is absent from the versioned `foxy` tree. The
  appropriate architecture-specific Orbbec runtime must be restored before
  building the camera package.
- System packages, applied USB rules, shell aliases, and Jetson-side deployment
  steps are documented in reports but are not fully represented by source
  commits.
- Simulation and hardware share launch conventions and command semantics, but
  retain different sensor resolutions, fields of view, calibration, and mounts.
- The EKF improvement does not establish global mapping accuracy. Lidar replay
  improvements and the revised physical steering scale still require the
  physical validation described in the reports.
- The camera geometry report covers depth/RGB registration and plane estimation;
  its application implementation is under `ros2_ws/`, outside this history.