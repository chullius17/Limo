# Trajectory planning

`traj_package` contains the Nav2 SMAC planner and `rviz_goal_bridge`, which
receives RViz goals on `/goal_pose`, searches for a feasible solution, and
publishes the path. It can be launched without `limo_controller`:

```bash
ros2 launch traj_package trajectory.launch.py
```

It requires the robot map and transforms. Pass a custom configuration with
`planner_params_file:=/path/to/parameters.yaml`.

The bridge defaults to `optimize_idle_io: true`. Its private TF receiver is
drained in idle batches at 5 Hz and immediately when a planning pose is queried,
avoiding Python executor wakeups for every TF publication while waiting for a
goal. Batching keeps the TF buffer warm without changing transform timestamps.
Current poses and bounded TF
waits still use the normal transform buffer. Set the parameter to false at
startup to restore continuous reception. Build the shared `limo_runtime`
package with `traj_package` after this update, then source and restart.

The built-in real and simulation global costmaps use `lethal_cost_threshold: 60`.
Semantic yellow lines (60), boardwalk (90) and occupied cells (100) become hard
obstacles for SMAC and start/goal validation; white markings (30) remain soft
costs. This is applied by the planner's StaticLayer without rewriting the source
map used by localization. The default path starts at the selected planning pose
and obeys these global obstacles throughout.

## Planning GUI

```bash
ros2 launch traj_package planning_gui.launch.py
```

The Qt window shows the global planner costmap from above, updating as complete
grids arrive. The image keeps its aspect ratio when resized, with positive grid
Y upward. Free cells are white, increasing costs light to dark purple, inscribed
obstacles purple, lethal obstacles black and unknown cells gray. The lower,
resizable, read-only log panel lists every attempted start/goal pair in order:
attempt number, start and arrival positions in meters, yaw in degrees and frame,
followed by the planner's failure reason or success. It also reports the actual
selected start on success, including virtual-start quantization. The transcript
starts over for each new goal and ends at the first success or exhausted search.
Preflight errors such as unavailable localization or planner are shown too.

A compact legend below the map shows the actual white-to-purple cost palette,
separate color cells for inscribed obstacles, lethal obstacles and unknown
space, and colored symbols for the path, goal arrows, robot footprint and axes.
The map itself has no text labels covering the robot or planning endpoints.

The validated path on `/limo/planning/path` is drawn as a thick turquoise line
with a dark outline, above the costmap and below the robot and endpoint arrows.
It follows the map origin's translation and orientation, and uses timestamped
TF if the path and displayed grid have different frames. A new goal or an empty
path clears the line; retained paths older than the current goal stay hidden.
The path is also included in the compressed PNG preview.

The bridge publishes the entire current transcript on `/limo/planning/log`
(`std_msgs/String`, reliable/transient-local, depth 1), so reopening the GUI
restores all attempts from the current search. Set `planning_log_topic` in the
bridge YAML to change both the publisher and viewer subscription. The built-in
explicit-start service supplies the exact error message; the optional Foxy
action backend reports its returned status when it provides no detailed reason.

The robot overlay follows live TF from `base_link` into the displayed grid's
frame, with red +X and green +Y arrows and a dark footprint outline with a white
halo, as in the MPC viewer. It uses the global costmap's `published_footprint`
(including padding), recovered in body coordinates at the polygon timestamp.
Before that polygon is available, it uses the footprint and padding from the
selected planner YAML. Robot pose updates independently of costmap updates;
missing TF hides the overlay and is reported above the map. The map origin's
translation and orientation are included when positioning the overlay.

A green arrow marks the goal selected on `/goal_pose`; a blue arrow marks the
accepted `/adjusted_goal_pose` once a matching validated `/limo/planning/path`
exists. Both include heading. A new request or an empty path removes the blue
arrow until planning succeeds again. Green and blue arrows use different
lengths so both remain visible when the two poses coincide. The selected goal
is received live; the accepted goal and path also support retained delivery.

A second blue arrow shows `/adjusted_start_pose`,
the start selected for that validated path. With start adjustment enabled, this
is the actual quantized virtual start where the published path begins. It
stays at the planning pose while the robot moves, and disappears when the path
is invalidated. Start topic overrides follow `adjusted_start_topic` in the
planner/bridge YAML. Both planning endpoints are included in the compressed view.

The complete map view, including the robot and goal overlays, is published as
PNG `sensor_msgs/CompressedImage` on `/limo/planning/preview/image/compressed`.
There is no raw image output. PNG preserves cell and arrow colors without loss;
output is capped at 960 × 720 pixels and defaults to 5 Hz. The GUI itself draws
locally; its ROS image output is compressed. `image_topic` and
`image_publish_rate` can be overridden in `planning_gui.launch.py`.

The viewer reads `rviz_goal_bridge.ros__parameters.costmap_topic` from
`planner_params_file` (default: `config/traj_sim.yaml`). This displays the global
planning costmap, including its layers, rather than the source `/map` or MPC's
local costmap. A `costmap_topic:=/custom/topic` override is also available.
`use_sim_time` defaults to true and can be set to false for a wall clock.
`footprint_topic` can override the default topic in the costmap namespace.

`user_package/limo_app_ideal_sim.launch.py` includes this viewer by default;
set `start_planning_gui:=false` to disable it.

## Outputs

| Topic | Type | Contents |
| --- | --- | --- |
| `/limo/planning/path` | `nav_msgs/Path` | Most recent validated path |
| `/limo/planning/status` | `std_msgs/String` | Planning state and failures |
| `/limo/planning/log` | `std_msgs/String` | Complete current attempt transcript |

These topics use reliable, transient-local QoS with depth 1. A consumer started
after planning can therefore receive the most recent path with the same QoS. A
new request first publishes an empty path to invalidate the preceding one; only
a successful search publishes a new non-empty path.

## Execution

The package exposes no control services and does not send `FollowPath` goals.
The former `enable_control` and `auto_start_control` flags were removed.

Both profiles now load `limo_smac_planner/ExplicitStartPlanner`, which wraps
Foxy SMAC and exposes `/compute_path_with_start`. This service supplies the
explicit start missing from Foxy's `ComputePathToPose` action. With
`enable_start_adjustment: true`, the bridge samples collision-free virtual
starts as well as goals, and tries at most `max_planning_attempts` start/goal
pairs. Exact starts retain normal obstacle checks.
Both `traj_real.yaml` and `traj_sim.yaml` enable this start adjustment, searching
within 0.75 m at 0.10 m steps and 22.5 degree heading increments.

The path starts at the selected planning pose. The wrapper quantizes that
pose to SMAC's grid and heading bins and checks its footprint. Foxy's backtrace
omits the initial node, so the wrapper restores the quantized start at the
beginning of an adjusted path. It generates no initial connector and applies
ordinary obstacle checks throughout planning and control.

The service returns the quantized planning start, which is also published on
`/adjusted_start_pose` and included in the planning log. The bridge publishes
the validated path on `/limo/planning/path`; a new goal clears it and stale
service results are ignored. Execution retains the controller's START and
tracking checks, including the requirement to start inside the Tube MPC path
corridor.

Execution belongs to `limo_controller`: its `path_executor` node receives the
path and handles START, pause, resume, and cancellation through
`/limo/control/set_active` and `/limo/control/set_enabled`. A new path
interrupts the active one and requires a new START. The GUI also displays the
planning state.

`user_package/limo_app_sim.launch.py` and `limo_app_real.launch.py` compose
mapping, planning, and control for the two runtime profiles.

## Real and simulation profiles

`trajectory.launch.py` selects `config/traj_real.yaml` or
`config/traj_sim.yaml` through `robot_model`:

```bash
ros2 launch traj_package trajectory.launch.py robot_model:=real
ros2 launch traj_package trajectory.launch.py robot_model:=sim
```

The default model is `real`. The clock defaults to wall time for real and
simulation time for sim; `use_sim_time` can override the clock without changing
the selected profile. The application launches pass their profile automatically.

Both files configure SMAC, the global costmap and `rviz_goal_bridge`, including
its goal-search budget and validation footprint. The profiles are tuned
separately. Their current geometry, together with the corresponding
`limo_controller/config/control_real.yaml` and `control_sim.yaml` profiles, is:

| Parameter | Real | Simulation |
| --- | --- | --- |
| Planner minimum turning radius | 0.50 m | 0.70 m |
| Global planner footprint, length × width | 0.322 × 0.200 m | 0.322 × 0.220 m |
| Bridge validation footprint, length × width | 0.322 × 0.220 m | 0.322 × 0.220 m |
| Controller collision footprint after padding, length × width | 0.322 × 0.220 m | 0.342 × 0.240 m |

Both global planner costmaps use `footprint_padding: 0.0`. In the real profile,
the planner footprint is 0.020 m narrower than the bridge validation and
controller footprints. In simulation, the planner, bridge and controller share
the same base footprint, but the controller adds `footprint_padding: 0.01`
(0.01 m per side), producing the effective collision footprint listed above.
The real controller uses zero padding.

An explicit `planner_params_file` takes precedence over the selected default.
`map_topic` continues to rewrite both the static-map and border-follow inputs.
The real/sim profiles are the only built-in planner parameter files.

The built-in YAML files target Foxy's `smac_planner/SmacPlanner` 0.4.7 through
the explicit-start wrapper.
They omit unsupported newer options: `analytic_expansion_max_length`,
`max_on_approach_iterations`, `max_planning_time`, `lookup_table_size`,
`cache_obstacle_heuristic`, and the newer `smoother` configuration block.
Search is bounded by `max_iterations`; the removed `max_planning_time: 7.5`
never imposed a 7.5-second search limit in this version. `smooth_path: false`
continues to disable smoothing. See the
[SMAC 0.4.7 configuration implementation](https://github.com/ros-navigation/navigation2/blob/0.4.7/smac_planner/src/smac_planner.cpp).

Automatic planner configuration and activation is controlled by
`lifecycle_manager_global_planner.ros__parameters.autostart` in the selected
YAML (true in both built-in profiles).
