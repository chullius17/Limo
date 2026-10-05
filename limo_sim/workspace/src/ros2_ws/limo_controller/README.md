# LIMO control profiles

`control.launch.py` selects `config/control_real.yaml` or
`config/control_sim.yaml` using `robot_model`, independently of the clock.

| Setting | Real | Simulation |
| --- | --- | --- |
| MPC wheelbase | 0.20 m | 0.24 m |
| Rear axle to base | 0.10 m | 0.12 m |
| Minimum turning radius | 0.462 m | 0.55 m |
| Controller frequency | 10 Hz | 20 Hz |
| MPC sequences / steps | 128 / 25 | 768 / 80 |
| Model timestep / horizon | 0.10 s / 2.5 s | 0.05 s / 4.0 s |
| Require chassis command mode | Yes | No |
| Default clock | Wall clock | Simulation clock |
| Default local GUI | Off | On |
| Local obstacle guidance | On | On |
| Local costmap size | 5 x 5 m | 5 x 5 m |

Simulation geometry is now in its YAML rather than applied over a parameter file
by the launch. Real control uses a smaller Nano CPU budget, retaining geometry,
velocity/acceleration limits, the prediction horizon and footprint collision checks.
The real local costmap updates at 10 Hz and publishes at 2 Hz.

Both profiles enable `FollowPath.MppiPath.ObstacleGuidance`: when a newly observed
obstacle blocks the reference, MPC scores progress through free space toward
a clear rejoin point instead of straight-line progress into the obstacle.
It temporarily relaxes reference alignment, rewards steering toward the free
passage, and retains collision checks. The DWB `Oscillation` critic is omitted
in both profiles so its sign locks cannot prevent exiting the detour. See the
[MPC documentation](../limo_dwb_critics/README.md#online-obstacle-detours)
for parameters and local-map/kinematic limitations. Rebuild both
`limo_dwb_critics` and `limo_controller`, then restart the control application.

Detours are triggered by the oriented footprint, so an already collision-free
planned path beside an obstacle keeps normal tracking. Simulation uses a 4 s
prediction horizon and soft costs for reverse distance and direction changes;
both profiles retain detour scoring until the robot has returned near the reference.
Real control retains its 2.5 s horizon and 128 candidates at 10 Hz. Both profiles
use a 5 x 5 m local costmap and a 2.50 m prune distance to include a free rejoin
point beyond the obstacle. The real footprint padding, chassis guard and
command timeouts remain active. The larger map and guidance field add work to
the real control cycle; verify the complete cycle still fits within 100 ms on
the Nano. The shorter real horizon and smaller sample budget can limit detours
that succeed with the simulation profile.

The real profile gives local obstacle costs twice their reference weight:
`FollowPath.MppiObstacle.scale` is 6.0 instead of 3.0, while path and goal
weights retain their reference values. This increases the preference for
lower-cost routes through the local costmap relative to following the planned
path; it does not impose an absolute priority between the soft costs.
Both profiles set `FollowPath.MppiObstacle.near_goal_distance` to 0.15 m, so
ordinary obstacle repulsion switches off when the current robot position is
less than 15 cm from the goal. The costmap inflation remains present; critical
costs remain active and footprint collisions remain hard rejections at every
predicted pose. These parameters are read when configuring
the controller; restart the application after changing the installed YAML.

Both profiles enable `FollowPath.MPC.StartConnector.ignore_obstacles`. This
applies only to the initial Dubins prefix declared by the planner on
`/limo/planning/start_connector`, with metadata matching the entire executed
path. During this prefix all obstacle costs and footprint collision checks are
bypassed, obstacle detours are disabled, and the controller follows only the
Dubins reference. The exception ends after progress along the prefix and arrival
within `position_tolerance: 0.05` m and `yaw_tolerance: 0.10` rad of its endpoint;
it cannot reactivate by returning near that point. The rest of the path retains
normal collision checks. Setting `ignore_obstacles: false` restores ordinary
collision handling on the prefix too. Chassis, actuator and command-timeout
checks remain in effect. Rebuild `limo_interfaces`, `limo_smac_planner`,
`limo_dwb_critics`, `traj_package` and `limo_controller`, source the install and
restart the application to use this feature.

```bash
ros2 launch limo_controller control.launch.py robot_model:=real
ros2 launch limo_controller control.launch.py robot_model:=sim
```

The real application runs the controller, path executor and velocity mux onboard;
use `ros2 launch user_package desktop_app.launch.py` on the PC for its GUI.
`start_gui` can still be overridden for standalone use.

`use_sim_time` defaults from `robot_model` and can be overridden without changing
the selected geometry. A custom `controller_params_file:=/path/to/control.yaml`
takes priority; no model geometry is injected over custom YAML.
Both profiles contain the controller, local costmap, `path_executor` and
`twist_mux` parameters. The executor and mux use the selected controller YAML.
The mux uses the same clock override as the controller:
autonomy publishes to `/cmd_vel_autonomy` (priority 10), teleop to
`/cmd_vel_teleop` (priority 100); `/cmd_vel` is published at 20 Hz on the real
robot and 50 Hz in simulation.
Both inputs time out after 0.5 s. A custom YAML can also override these mux
settings through its `twist_mux.ros__parameters` section.

On the real robot START/resume requires fresh `/limo_status` feedback (at most
1 s old), normal `vehicle_state=0`, `error_code=0` and `control_mode=1`.
Missing/stale feedback or a chassis fault blocks START. Losing command mode or
feedback during execution cancels FollowPath and requires a new explicit START;
restoring feedback never resumes automatically. The raw `motion_mode` is not
used as a guard: this robot reports mode 2 despite its mechanical Ackermann
conversion, and `custom_start/limo_real.launch.py` already forces conversion 1.

The phone/remote is a separate chassis control source, not ROS `/cmd_vel_teleop`.
Release/disconnect it before autonomous control and restore command mode. The
driver requests command mode only at startup; if needed, stop the application
and restart the base after releasing the phone. Do not run two base drivers.
This package never takes ownership from the phone automatically. After a base
restart, verify localization with RViz's 2D Pose Estimate before planning again.
The real launch enables the guard even if a custom YAML omits the executor
section; custom `commanded_control_mode` and timeout still come from that YAML.
Only an explicit `require_chassis_status:=false` launch argument disables it.

Do not work around a slow controller by extending command timeouts or progress
allowance. At 10 Hz the complete real control cycle must fit within 100 ms,
including costmap and footprint scoring; validate timing with the full CV and
desktop workload before supervised physical execution.

These real/sim profiles are the only built-in controller parameter files.

## Simulation MPC image preview

`control.launch.py` reads `FollowPath.MPC.Debug.enabled` from the selected
controller YAML to start `mpc_preview`, including when launched through
`user_package/limo_app_sim.launch.py`. This flag is explicitly `true` in
`control_sim.yaml` and `false` in `control_real.yaml`; it enables both telemetry
and the preview node. Clock overrides do not change it. A custom controller YAML
uses its own flag (missing means disabled). Setting `start_mpc_preview:=false`
can suppress the renderer; setting it to `true` cannot override a false YAML flag.

The node publishes `sensor_msgs/CompressedImage` on
`/limo/control/mpc_preview/image/compressed` (PNG by default). View this topic
with `rqt_image_view`, selecting the compressed transport. No local window is
opened by the node.

The top-down image is centered on the snapshot's `base_link`, with +x upward
and +y leftward. It contains the controller's local costmap, its effective
footprint, equivalent bicycle front/rear wheels, initial steering (orange),
first winning steering command (purple), and representative predicted
base_link trajectories. A magnified model inset makes the wheel positions and
footprint readable without changing the main map scale. These are model states
and virtual wheels, not measured steering joints or the four-wheel chassis.

| Trajectory family | Default color |
| --- | --- |
| Braking | Red |
| Nominal/warm start | Blue |
| Constant curvature | Purple |
| Perturbed nominal | Teal |
| Broad exploration | Orange |
| Selected trajectory | Green with a white outline |

Rejected or effort-pruned candidates can be shown as dashed lines. A finite
candidate score may have been short-circuited by DWB and does not certify that
all its critics were checked. The selected candidate is fully scored by the
controller. The preview is diagnostic and has no effect on selection.

Settings are split between two YAML files:

- `config/control_sim.yaml` / `config/control_real.yaml`, under `FollowPath.MPC.Debug`:
  enable telemetry and the preview node with `enabled` (sim: true, real: false),
  its topic, publication rate, samples per family and pose downsampling stride.
  Representatives are evenly distributed over each family's generated order;
  the winner is always included even if it was not among those representatives.
- `config/mpc_preview_sim.yaml`, under `mpc_preview.ros__parameters`: input/output
  topics, rendering rate, image dimensions, pixels per meter, grid spacing,
  colors, line widths, model inset, wheel glyph dimensions, PNG/JPEG format and
  compression settings. Input topic changes must match the telemetry topic.

Footprint, wheelbase and axle offset come directly from the controller snapshot,
so editing MPC geometry does not require duplicating it in the renderer YAML.
The default export is 4 Hz with six representatives per family (one each for
braking/nominal), plus the winner if necessary. No snapshot is constructed
without a telemetry subscriber. Rendering and image compression happen in the
separate Python node, using Pillow (`python3-pil`).

The `limo_interfaces/MpcDebug` snapshot carries raw Nav2 costmap bytes from the
same locked control cycle as the trajectories. The renderer transforms both
into that cycle's robot frame, without subscribing to an independently timed
costmap or requiring additional TF lookups. Output stamps retain the snapshot
time. The rendering timer uses a steady clock; after `stale_timeout` without a
new snapshot, the image displays a waiting/stale status and removes old paths.

After changing YAML settings, restart the application. A custom renderer file
can be selected with `mpc_preview_params_file:=/path/to/preview.yaml` in either
the standalone controller launch or the application launch.

Build the new interfaces before loading the updated plugin:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select limo_interfaces limo_dwb_critics limo_controller
source install/setup.bash
ros2 launch user_package limo_app_sim.launch.py
```

If the updated application launch is not installed by symlink, rebuild
`user_package` too. The renderer's geometry, colors and PNG encoding can also be
tested outside ROS:

```bash
PYTHONPATH=src/ros2_ws/limo_controller python3 -m unittest discover \
  -s src/ros2_ws/limo_controller/test -p test_mpc_preview_render.py
```
