# limo_depth

C++ depth registration and ground-plane completion for ROS 2 Foxy. The node
estimates ground height and tilt from measured depth, freezes a lookup table,
and uses it to fill missing pixels. It needs no class labels or Python runtime;
Python is used only for launch files and integration tests.

## Inputs and output

| Direction | Default topic / source | Content |
| --- | --- | --- |
| Input | `/depth_camera/depth/image_raw` | `sensor_msgs/Image`: `32FC1` optical depth in metres, or `16UC1` / `mono16` in millimetres. Padded rows and either byte order are supported. |
| Input | `/depth_camera/depth/camera_info` | Full-resolution source pinhole intrinsics, dimensions and optical frame matching the depth image. |
| Input | `/rgb/camera_info` | Target RGB pinhole intrinsics, dimensions and optical frame; required when `register_to_rgb: true` (default). |
| Input | TF | Depth → RGB optical frame and both optical frames → Z-up `plane_frame`, at the input depth timestamp. |
| Output | `limo/cv_package/depth_correction/depth_corrected/raw` | Corrected `32FC1` depth in metres; default 320×120, lower half of the RGB image. |
| Output | `limo/cv_package/depth_correction/ground_depth/raw` | Plane-only `32FC1` depth LUT, published when subscribed; NaN until calibrated. |
| Output | `limo/cv_package/depth_correction/image_jet/compressed` | JPEG JET visualization of registered measurements before completion. |
| Output | `limo/cv_package/depth_correction/image_jet_corrected/compressed` | JPEG JET visualization after completion. |

Image/CameraInfo subscriptions and metric outputs use best effort, keep last 1.
Debug outputs are reliable, keep last 1, generated only when enabled and subscribed.
Output headers retain the **depth timestamp** and use the **RGB optical frame**
when registered; otherwise they retain the source depth frame. Missing
CameraInfo/TF or invalid input skips the frame with a throttled diagnostic.

Both calibrations must describe rectified/zero-distortion pinhole images. The
node uses `K`, checks `D`, and rejects skew, nontrivial binning or ROI metadata;
it does not rectify images. RGB images themselves are not needed. CameraInfo is
the latest calibration, not an image synchronization stream. Rigid extrinsics
are expected; TF is checked at each sensor timestamp.

## Procedure

1. Validate source layout and calibration. Cache source rays and rigid transforms;
   rebuild geometry and restart calibration when intrinsics, frames, dimensions
   or extrinsics change (TF roundoff within 1e-8 is ignored).
2. Collect lower-central **original measured** points, transform into the Z-up
   frame and fit `n·p + d = 0` with deterministic RANSAC and least squares.
   Reject walls, insufficient inliers, implausible height/tilt and narrow support.
3. After enough stable fits, normalize their component-wise median and freeze
   target ray/plane intersections as a depth LUT. Keep the robot on one plane
   while calibrating. Before readiness, measurements still flow with NaN holes.
4. Register depth into the cropped/resized RGB grid with calibrated forward
   projection and a nearest-Z buffer. Invalid/out-of-range depth becomes NaN.
   Preserve every valid projected measurement, including foreground obstacles;
   replace only missing pixels with finite in-range ground predictions.

The ground model assumes a planar road. A filled pixel is an **estimate**, not
proof of observed free space: missing returns on objects are also filled where
the plane intersects the ray. Upward rays, intersections beyond the configured
depth range, and holes without a usable plane remain NaN. Ground is not refitted
after calibration until reset or geometry changes; reset when road geometry changes.

`plane_frame` defaults to `camera_link`, with +Z up and the ground below its
origin. Automatic fitting defaults to 5–60 cm plane offset and at most 20° tilt;
adjust these limits for other installations. A finite `plane_height_m` bypasses
fitting and uses the horizontal plane `z = plane_height_m` in that frame, e.g.
`-0.18`. Its default `.nan` estimates both normal and offset.

## Run

Place this repository under the ROS workspace `src` directory. Colcon discovers
`limo_depth` alongside the other student packages. From the workspace root:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select limo_depth
source install/setup.bash
ros2 launch limo_depth depth.launch.py
ros2 launch limo_depth depth.launch.py profile:=real
```

Default `profile:=sim` selects [config/sim.yaml](config/sim.yaml), simulation time,
30 calibration frames and full source registration. `profile:=real` selects
[config/real.yaml](config/real.yaml), real time, 10 calibration frames, a 15 Hz
processing cap and reduced registration work, matching the existing CV profiles.
Reduction can discard small obstacles; disable `registration_reduce_input` to
project every source sample. Fitting always uses original source pixels.

Drivers/Gazebo, CameraInfo, TF, `/clock` when applicable, and the detector must
already run. This launch starts only `limo_depth`. **Disable the original
`cv_package` depth-correction node** when publishing on the same output topics.
The existing combined CV launch still owns its original producer; do not start
the two depth producers together. No changes to that launch are applied here.

Every parameter, default, unit and constraint is documented in
[config/depth.yaml](config/depth.yaml). Copy it and uncomment the desired entries:

```bash
ros2 launch limo_depth depth.launch.py profile:=real config_file:=/absolute/path/depth.yaml
# Explicit clock override for a bag or a simulated robot:
ros2 launch limo_depth depth.launch.py profile:=real use_sim_time:=true
# Direct execution: C++ defaults and real time, unless overridden:
ros2 run limo_depth limo_depth_node --ros-args --params-file /absolute/path/depth.yaml
```

`config_file` replaces the selected profile file; unspecified parameters follow
the C++ defaults documented in `depth.yaml`. The `use_sim_time` launch argument
always wins and follows `profile` when omitted. All geometry/calibration settings
are startup-only. `enable_telemetry` and `max_processing_rate_hz` can change live:

```bash
ros2 param set /limo_depth max_processing_rate_hz 15.0
ros2 param set /limo_depth enable_telemetry true
ros2 service call /limo_depth/reset_calibration std_srvs/srv/Trigger '{}'
```

The rate cap uses a wall-clock timer and retains only the latest pending depth
frame, processing it once. Reset clears calibration and pending input; the next
valid measurements recalibrate (or rebuild the configured-height LUT).

## Using with limo_cloud

Use the same RGB CameraInfo, output dimensions and upstream crop as the detector.
`limo_cloud` must consume this corrected topic and use matching ray geometry:

```yaml
limo_cloud:
  ros__parameters:
    depth_topic: limo/cv_package/depth_correction/depth_corrected/raw
    camera_info_topic: /rgb/camera_info
    require_rgb_depth: true
    input_crop_y_min: 0.5
```

Classified and corrected images must match in dimensions and RGB optical frame.
The package does not synchronize depth with classification or start `limo_cloud`.
For a depth-only consumer, `register_to_rgb: false` crops/resizes the native depth
grid and retains its optical frame; it does not align separate RGB/depth cameras.

## Validation

```bash
colcon test --packages-select limo_depth
colcon test-result --verbose
```

Tests exercise camera geometry, padded/big-endian inputs, nearest-Z visibility,
tilted ground fitting with outliers, degenerate/wall rejection and preservation
of measured obstacles. DDS integration uses synthetic calibrated cameras/TF to
check both profiles, known plane intersections, startup/invalid inputs, geometry
changes, reset, debug output and processing-rate behavior. No camera is required.
