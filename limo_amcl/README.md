# limo_amcl

C++ localization with a single particle filter combining laser likelihood and
classified point-cloud evidence. It follows the current mixed AMCL pipeline,
with configurable class roles and no dependency on the modified `nav2_amcl`.

## Inputs and outputs

| Direction | Default topic / source | Content |
| --- | --- | --- |
| Input | `/scan` | `sensor_msgs/LaserScan`, localization trigger. |
| Input | `/limo/map_package/online/maps/laser_map` | Static laser-only `nav_msgs/OccupancyGrid`. |
| Input | `/limo/map_package/online/maps/cv_obstacle` | Static semantic grid: road `0`, obstacles at or above `cv_occupied_threshold`. |
| Input | `/limo/cv_package/visual_ptcld/points` | `sensor_msgs/PointCloud2`: `x/y/z` FLOAT32, `class_id` UINT8. |
| Input | TF | Continuous `odom → base_link`, plus transforms to laser/cloud frames. |
| Input | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` in the global frame. |
| Output | `/amcl_pose` | Estimated pose and covariance, compatible with Nav2. |
| Output | TF | `map → odom`. |
| Output | `/particlecloud`, `/particle_cloud` | Standard PoseArray and Nav2 weighted ParticleCloud. |
| Output | `/amcl/cv_diagnostics`, `/amcl/laser_diagnostics` | DiagnosticArray: accepted/rejected updates, reasons and actual workload. |

The example also serves the complete saved map on `/map`. Nav2 can use this map
for planning while AMCL evaluates laser and CV against their separate maps.
All topics and coordinate frames are configurable.

## Procedure

1. Apply the odometry motion model and laser sensor model to the same particle filter.
2. Select the nearest buffered classified cloud within `cv_sync_tolerance` of the
   laser timestamp. Transform it into the robot frame at laser time through the
   fixed **odom** frame, compensating motion without depending on the estimated map pose.
3. Build XY voxel centroids separately for road and obstacle classes. Each centroid
   casts one vote; denser clouds cannot increase a voxel's weight. Ignore other IDs.
4. For each particle, compare the projected observation with the static CV grid.
   Road matches exactly free cells (`0`), obstacles match cells at or above the
   occupied threshold. Unknown and out-of-map cells count as mismatches.
5. Check minimum total/non-road voxel counts and, when enabled, information gain
   and CV-only posterior spread. Rejecting CV leaves the laser weights intact.
6. Fuse before resampling, with normalized weights:
   `w_final ∝ w_laser^laser_weight_factor × exp(-cv_weight_factor × cv_sad_gain × SAD)`.
   SAD is the mean binary mismatch. With useful laser data each cloud is used
   once. Reuse is allowed only without useful laser evidence (or with laser weight
   zero), still within the original cloud's synchronization tolerance.

`laser_weight_factor: 0.0` skips laser likelihood, but scans still trigger updates.
`cv_enabled: false` selects laser-only AMCL. No usable CV means laser fallback;
without usable evidence from either source, there is no localization correction.
The simulation profile preserves the previous laser/CV factors `2.0/1.0`; the real
profile uses `1.0/1.0`. Both preserve the previous 300–800 particle bounds.

## Build and run

From the ROS workspace root, after sourcing ROS Foxy:

```bash
colcon build --symlink-install --packages-up-to limo_amcl
source install/setup.bash
ros2 launch limo_amcl amcl.launch.py map_directory:=/workspace/ros2_maps/semantic
```

The default example starts the `limo_cloud_utilities` simulation CV pipeline, three
static map servers, C++ `limo_amcl_node`, lifecycle management and a localization
RViz configuration. Robot/simulator, sensor drivers and odometry/EKF must already
be running. It does not start SLAM. Stop the mapping launch before localization:
there must be only one publisher of `map → odom`.

Set the initial pose with RViz **2D Pose Estimate**, or set `set_initial_pose: true`
and the `initial_pose.*` parameters in your configuration. The services
`/reinitialize_global_localization` and `/request_nomotion_update` are available.

```bash
# Real robot: real CV profile, wall clock; time can be overridden independently.
ros2 launch limo_amcl amcl.launch.py profile:=real map_directory:=/path/to/maps

# Custom export name and AMCL/CV settings.
ros2 launch limo_amcl amcl.launch.py map_directory:=/path/to/maps map_name:=circuit \
  config_file:=/path/to/amcl.yaml cv_config_file:=/path/to/cv.yaml

# Reuse existing CV and map servers; run localization without a second viewer.
ros2 launch limo_amcl amcl.launch.py start_cv:=false start_maps:=false start_rviz:=false

# Use an existing Nav2 map server on /map for the laser observation.
ros2 launch limo_amcl amcl.launch.py start_maps:=false map_topic:=/map
```

`map_directory` is relative to the launch working directory unless absolute.
The expected `limo_map` exports are `<map_name>_complete.yaml`,
`<map_name>_laser.yaml` and `<map_name>_cv_obstacle.yaml` (default name: `limo_map`).
Individual paths can be overridden with `map_yaml`, `laser_map_yaml`, `cv_map_yaml`.
All three YAML files are required when `start_maps:=true`; their image references
are resolved by the standard Nav2 map server.

Additional launch overrides: `use_sim_time`, `cv_enabled`, `laser_weight_factor`,
`cv_weight_factor`, `map_topic`, `cv_map_topic`, `scan_topic`, `cv_cloud_topic`,
`complete_map_topic`, `rviz_config`, `start_amcl`. With `start_cv:=true`, cloud topic
changes must also be reflected in the upstream `cv_config_file`.

## Nav2 integration and isolation

This package adds its own executable, `limo_amcl_*` libraries, C++ namespace,
headers and package share directory. It does not overwrite or relink `nav2_amcl`,
its libraries, configs or launch files, and does not register a Nav2 component.
The example map servers and lifecycle manager also have dedicated names.

The runtime node is called `amcl` and intentionally exposes Nav2's standard
localization interfaces. Run **one** localization provider: disable the original
AMCL and SLAM when using this node. With an existing Nav2 deployment, launch its
navigation servers without its localization bringup. Either use this example's
map servers, or set `start_maps:=false` and configure the existing map topics.
If an existing Nav2 lifecycle manager already manages `amcl`, also set
`start_lifecycle_manager:=false`; it will configure and activate this node instead.
Do not let two lifecycle managers manage the same AMCL or map server nodes.
Existing planner, controller, BT navigator and behavior servers can consume the
same `/map`, `/amcl_pose` and TF tree without changes.

The CV obstacle map is still required when CV fusion is enabled. Serving only
`/map` does not create the separate semantic observation map automatically.

## Parameters

See [config/amcl.yaml](config/amcl.yaml) and
[config/amcl_real.yaml](config/amcl_real.yaml): active entries are profile
overrides; commented entries document the actual C++ defaults and units.
Parameters are read during lifecycle configuration; restart or reconfigure after
editing them. The launch selects simulation time from `profile` unless overridden.

Set `cv_road_class_ids` and `cv_obstacle_class_ids` to the IDs of your input
pipeline. Lists must contain unique, disjoint integers in `[0,255]`; IDs absent
from both lists are ignored. Defaults reproduce the existing pipeline: road
`[1,5]`, obstacles `[2,4,6]`; soft obstacles (`3`) do not vote. Keep these roles
consistent with the costs configured in `limo_map` when exporting the semantic
map. The localizer evaluates occupied/free agreement, not arbitrary cost labels.

## Verification and provenance

```bash
colcon build --symlink-install --packages-select limo_amcl --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select limo_amcl --event-handlers console_direct+
colcon test-result --test-result-base build/limo_amcl --verbose
```

Tests cover fusion/normalization, quality gates, cloud layouts, configurable
classes, odometry compensation, synchronization/reuse, laser fallback, profile
parity, complete launch wiring and a running lifecycle/map-server/localization
stack with synthetic inputs in an isolated ROS domain. These checks do not
replace validation of localization accuracy on a robot or recorded trajectory.

The particle filter, laser/map/motion code and lifecycle node derive from the
workspace's `navigation2/nav2_amcl`, preserving existing copyright notices and
LGPL-2.1-or-later licensing; see [COPYING.LGPL](COPYING.LGPL). Semantic fusion,
diagnostics and tests retain their Apache-2.0 notices; see
[LICENSE.Apache-2.0](LICENSE.Apache-2.0). Launch/configuration and class-role
integration are supplied with this package.
