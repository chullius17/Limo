# LIMO control profiles

`control.launch.py` selects `config/control_real.yaml` or
`config/control_sim.yaml` using `robot_model`, independently of the clock.

| Setting | Real | Simulation |
| --- | --- | --- |
| MPC wheelbase | 0.20 m | 0.24 m |
| Rear axle to base | 0.10 m | 0.12 m |
| Minimum turning radius | 0.462 m | 0.55 m |
| Controller frequency | 10 Hz | 20 Hz |
| MPC proposal budget / steps | Up to 59 / 40 | Up to 59 / 80 |
| Model timestep / horizon | 0.10 s / 4.0 s | 0.05 s / 4.0 s |
| Require chassis command mode | Yes | No |
| Default clock | Wall clock | Simulation clock |
| Default local GUI | Off | On |
| Local obstacle guidance | On | On |
| Local costmap size | 5 x 5 m | 5 x 5 m |

Simulation geometry is now in its YAML rather than applied over a parameter file
by the launch. Real control uses a smaller Nano CPU budget, retaining geometry,
velocity/acceleration limits, the prediction horizon and footprint collision checks.
The real local costmap updates at 10 Hz and publishes at 2 Hz.

`path_executor` defaults to `optimize_idle_io: true`: it receives current-pose
TF in idle batches and on demand through `limo_runtime`, and excludes arrival odometry from its
executor while control is stopped or paused. Arrival checks resume at the
original odometry rate when needed; chassis monitoring remains active.
Idle TF maintenance at 5 Hz preserves incoming samples and keeps the buffer
warm for START; active pose queries drain new TF immediately.
TF publishers, MPC settings and costmap update rates are unchanged. Set this
parameter to false at startup for comparison with continuous reception.
After updating, build `limo_runtime limo_controller traj_package` with
`colcon build --packages-select limo_runtime limo_controller traj_package
--symlink-install --executor sequential`, source the install, and restart the app.

`control.launch.py` uses `limo_dwb_critics/limo_controller_server`, which hosts
the same Nav2 controller server and parameters. On SIGINT it deactivates the
controller (including its zero-velocity command and costmap thread), then
cleans it up while ROS is still available, and finally shuts down ROS. The
launch sets `LIMO_FAST_EXIT=1` to bypass slow native-library finalization after
cleanup on the Nano; export `LIMO_FAST_EXIT=0` to debug ordinary finalization.
SIGTERM remains launch's forced-stop fallback. Rebuild `limo_dwb_critics`
before starting this launch after the update.

Both profiles add three slow-start candidates when the model's initial speed
is at most 0.05 m/s in magnitude: full right steering, straight, and full left
steering. Each ramps at 0.05 m/s² toward 0.03 m/s and holds that speed through
the horizon. Tune these under `FollowPath.MPC.Startup` (`enabled`,
`max_initial_speed`, `target_speed`, `acceleration`). They obey the same steering
dynamics, effort costs, full-footprint collision checks and terminal tube as
every other candidate. The regular 56 proposals remain available; at low speed
the budget is 59 before deduplication. Telemetry labels this family
`Partenza lenta` in the preview. See [Pianificazione.md](limo_controller/Pianificazione.md)
for all trajectory groups.

Both profiles enable `FollowPath.MPC.prefer_motion`. Outside the final global
goal's `FollowPath.xy_goal_tolerance` (0.10 m), MPC selects the cheapest feasible
candidate whose first reachable command moves forward, even if stopping has a
lower cost. Collision, steering/acceleration limits and the terminal tube remain
mandatory. Stopping is still available when no forward candidate is feasible
and near the final goal; a temporary avoidance target does not enable the goal
exception. Reconfigure the controller after changing this parameter.

Both profiles enable `FollowPath.MppiPath.ObstacleGuidance`: when a newly observed
obstacle blocks the reference, MPC scores progress through free space toward
a clear rejoin point instead of straight-line progress into the obstacle.
It temporarily relaxes reference alignment, rewards steering toward the free
passage, and retains collision checks. The DWB `Oscillation` critic is omitted
in both profiles so its sign locks cannot prevent exiting the detour. See the
[MPC documentation](../limo_dwb_critics/README.md#online-obstacle-detours)
for parameters and local-map/kinematic limitations. Rebuild both
`limo_dwb_critics` and `limo_controller`, then restart the control application.

Both profiles trigger detours only from `/scan` marks in `obstacle_layer`, when
the oriented reference footprint plus a uniform 5 cm Euclidean margin
(`trigger_margin: 0.05`), intersects a laser obstacle. The margin covers the
front, sides and rear, with round corners; `trigger_front_extension` defaults
to 0.0. Semantic costs and unknown sensor cells do not
initiate detours. Dijkstra and SMAC share the same logic: detection checks the
robot-to-reference connector and the local reference up to the separate
`ObstacleGuidance.lookahead_distance: 1.0` m. Normal path following retains
`PathFollowCritic.lookahead_distance: 1.25` m. The detour target starts at the
1.0 m lookahead point, advances to a free reference point
if blocked, then proceeds up to another 0.50 m along consecutive free points.
SMAC then checks the final candidate's full filled, oriented footprint against
the current map and skips invalid candidates forward without a search attempt.
The center-based straightening distance and Dijkstra selection remain unchanged.
Dijkstra replans on every active controller cycle. SMAC validates the full
cached route and robot connection on every active cycle, and requests a search on
activation, when the cache is missing or invalid, or when the desired rejoin
target moves at least `ObstacleGuidance.Smac.replan_goal_distance: 0.25` m or
changes heading by at least `replan_goal_angle: 0.35` rad (about 20 degrees).
Both changes are measured from the last accepted target; heading uses the
wrapped angular difference. A safe cached route with a stable target causes
no periodic searches. `Smac.replan_frequency: 4.0` is a maximum search rate
(at least 0.25 s between actual worker starts), including failed attempts.
The last actual start survives reference, detour and deactivation resets;
discarded worker results also retain their place in this cadence. Setting
the frequency to zero only removes this rate cap; setting both target
thresholds to zero forces searches on each eligible cycle for legacy benchmarks.
The MPC retains its configured frequency. In SMAC mode, alignment and the
terminal tube follow its reference, with the remaining local reference appended
after rejoining. After each actual failed search,
`Smac.failure_advance_distance: 0.25` m advances the next retry candidate along
the original reference to the next free point at or beyond that arc distance.
The advance clamps to the last available reference point if less than 0.25 m
remains, provided that endpoint is free.
Cooldown cycles do not advance it. Retry demand overrides the 0.25 m/0.35 rad
target-change thresholds and remains subject to the 4 Hz cap. A failed target
update retains the old accepted target and route while the route remains safe.
Success clears the pending retry but keeps the advanced target as the earliest
allowed target until a new reference or the end of the detour resets it. This
prevents the next cycle from retreating to an earlier nominal target.
The available reference and local map bound possible target extension, and the
target never advances beyond the final global goal. A clipped local reference
end is not necessarily that final goal; a later snapshot may expose more path.
If target selection finds no candidate, the safe cached route remains in use.
Without a safe route, the controller commands zero velocity and retries when
eligible without immediately aborting navigation; preview snapshots stay fresh
with old candidates cleared.

`ObstacleGuidance.Smac.async_planning: true` uses one persistent native worker
with no request queue or concurrent search. Requests own immutable copies of the
costmap, footprint, start and target. The active controller call consumes a
finished result without waiting for native completion, then fully revalidates
the route, robot and target connections, clearance and oriented footprint on
the current map. It continues using an old route only while that route remains
safe; otherwise it sends zero velocity while planning. New references, detour
resets and lifecycle deactivation invalidate old generations, preventing late
results from restoring obsolete routes. `async_planning: false` retains
synchronous search for deterministic benchmarks. Copying the map and validating
routes still consume controller time; Foxy's iteration bound is not a worker
wall-time deadline.

SMAC rejection logs separate start/goal footprint and clearance prechecks,
native exceptions or empty routes, snapshot route validation, and
`current_map_validation` when a completed route fails live-map checks. Failure
details retain pose/cell and clearance context; footprint, clearance and motion
constraints remain enforced.
The normal progress timeout still applies. This anticipates the
switch to detour scoring without enlarging the guidance grid's hard clearance.
Both profiles use a 4 s
prediction horizon, forward motion or stopping, and retain detour scoring until
the robot has returned near the reference. Real control runs at 10 Hz, simulation
at 20 Hz; both generate up to 59 proposals before deduplication. Both profiles
use a 5 x 5 m local costmap and a 2.50 m prune distance to include a free rejoin
point beyond the obstacle. The real controller footprint is 0.322 x 0.220 m
with zero padding. The chassis guard and
command timeouts remain active. The larger map and guidance field add work to
the real control cycle; verify the complete cycle still fits within 100 ms on
the Nano. Different geometry, model timesteps and obstacle weights can produce
different detours between the real and simulation profiles.

The real profile uses strong finite local obstacle repulsion:
`FollowPath.MppiObstacle.scale` is 12.0, twice its previous value of 6.0
(simulation uses 6.0; the reference weight is 3.0), while path and goal weights retain
their reference values. This increases the preference for
lower-cost routes through the local costmap relative to following the planned
path; it does not impose an absolute priority between the soft costs.
The real profile sets `FollowPath.MppiObstacle.near_goal_distance` to 0.0 m,
keeping repulsion active up to the goal. Simulation retains 0.15 m, disabling
ordinary repulsion within the last 15 cm. The costmap inflation remains present; critical
costs remain active and footprint collisions remain hard rejections at every
predicted pose. These parameters are read when configuring
the controller; restart the application after changing the installed YAML.

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

## Local lane inflation

The real local costmap uses `lethal_cost_threshold: 100`: semantic boardwalk
points with source cost 90 become high but traversable Nav2 costs (228).
They and their Gaussian halos receive the strong finite repulsion above.
Simulation retains threshold 86 and makes boardwalk cells lethal (254).
Laser obstacle cells remain lethal in both profiles, with full-footprint
collision rejection even near the goal.
The global planner already treats source costs 60 and above as lethal.

The standard ObstacleLayer marks measured laser hits. The controller costmap
fuses those cells with the semantic grid before Gaussian inflation; it does not
extend laser obstacles into the unseen space behind their measured hits.

The Gazebo scan explicitly uses `laser_link`, preserving the sensor's 12.36 cm
forward offset in TF. Its height is -1 cm in the odom/base plane, so the
simulation obstacle source sets `min_obstacle_height: -0.05`; the default zero
would discard those correctly transformed points. The standard obstacle layer
uses Foxy's `obstacle_range` and `raytrace_range` parameter names. Rebuild with
`colcon build --symlink-install`, then restart the controller to load the
configuration. A Gazebo model created before the explicit `laser_link` fix must
also be recreated once to load that sensor frame.

The optional `limo_inflation::LocalLaneLayer` described below is not selected
by the current control profiles, which use Gaussian inflation. The source
semantic grid remains raw; inflation is applied in the controller costmap.
The viewer defaults to the original cloud and laser points, colored by each
point's own class cost (`view_mode: sources`, `source_cost_colors: true`).
The tuning below is retained for experiments; set `enabled: true` to restore it.
The optional layer replaces the standard Nav2 inflation layer and runs last,
after laser and semantic fusion.
It reuses the trajectory planner's `computeBorderCost` function, with two
preferred distance bands and a traversable cost ridge between them. The global
planner and its configuration remain independent.

All tuning stays in `config/control_sim.yaml` and `config/control_real.yaml`:

| Parameter | Sim | Real | Meaning |
| --- | --- | --- | --- |
| `obstacle_cost_threshold` | 1 | 1 | Seed threshold in raw Nav2 costs, 1..254 |
| `robot_width` | 0.220 | 0.220 | Width used for the first preferred distance, m |
| `safety_margin` | 0.25 | 0.04 | First distance is half the width plus this margin |
| `second_lane_distance` | 0.80 | 0.29 | Second preferred distance, m |
| `distance_tolerance` | 0.02 | 0.02 | Half-width of each preferred band, m |
| `near_max_cost` | 120 | 120 | Near-obstacle cost, 0..252 |
| `near_cost_sigma` | 0.05 | 0.025 | Gaussian decay width, m; 0 restores the linear ramp |
| `near_band_start` | 0.15 | 0.10 | Start of the extra near-obstacle turquoise plateau, m |
| `near_band_width` | 0.10 | 0.0 | Width of the extra plateau, m; 0 disables it |
| `near_band_ramp` | 0.025 | 0.005 | Linear transition width on either edge, m |
| `near_band_cost` | 100 | 100 | Cost of the extra plateau, turquoise in Jet |
| `inter_lane_peak_cost` | 100 | 100 | Peak of the triangular ridge, 0..252 |
| `inter_lane_plateau_ratio` | 0.90 | 0.90 | Fraction of the inter-well gap held at peak cost |
| `far_cost_slope` | 10.0 | 50.0 | Cost increase per meter beyond the second band |
| `far_max_cost` | 8 | 40 | Maximum cost beyond the second band, 0..252 |
| `high_cost_expansion_radius` | 0.075 | 0.075 | Extra red band around high-cost source cells, m; 0 disables |
| `high_cost_threshold` | 200 | 200 | Split between lane/red sources and soft repulsion sources |
| `high_cost_expansion_cost` | 252 | 252 | Cost assigned to the extra band, 0..252 |

The extra red band expands high-cost source cells, including boardwalk and
lethal laser returns, by 7.5 cm between cell centers. It is a real MPC cost,
not a display effect. The additional band has cost 252 and does not make
surrounding cells lethal. The fused grid no longer distinguishes a lethal
boardwalk cell from a lethal laser return; lower-cost yellow/soft cells do
not seed this band. Unknown cells remain unchanged. The original two-lane
profile is combined by maximum and remains active outside this band.
With `near_cost_sigma > 0`, the near-obstacle ramp and the tail outside the red
plateau use Gaussian decay. A smaller sigma produces a narrower halo. The
Gaussian is truncated and normalized to reach zero at the first preferred
band; well distances and the inter-lane profile are unchanged. The red radius
and the decay width are independent parameters. With sigma 0 the original
linear ramp and hard edge of the red plateau are retained.
`near_band_start` and `near_band_width` add a second turquoise plateau near
high-cost obstacles. In simulation it extends from 0.15 to 0.25 m, with 2.5 cm
ramps. It is merged by maximum with the existing near-obstacle and red costs,
so it never lowers obstacle repulsion. Its outer ramp must end before the
first well's zero-cost band. The real profile leaves it disabled because its
closer wells provide less available space.

`inter_lane_plateau_ratio` widens the central turquoise region at cost 100 while
preserving the zero-cost wells. In simulation the gap from 0.38 to 0.78 m
has a 0.36 m-wide plateau, from 0.40 to 0.76 m, centered at 0.58 m, with short linear ramps
at its edges. Ratio 0 restores the original triangular ridge; values must
be in [0, 1). This modifies the MPC costs, not the Jet color scale.

The thresholds apply to the fused 0..255 costmap, not the source OccupancyGrid's
0..100 values. Sources at or above `high_cost_threshold` construct the lane/red
field. Lower-cost sources at or above `obstacle_cost_threshold` construct an
independent, monotone near-obstacle repulsion field. Yellow points therefore
cannot replace a boardwalk/laser distance source and erase its turquoise band.
They add to that band instead; for example, source cost 177 plus lane cost 100
saturates at 252. Within each source group the distance is to its nearest point;
individual point kernels are not summed within a group.

For ordinary cells the result is the sum of the high-source field and the soft
field, capped at 252. Each source's original cost is preserved within its own
field before summation. Critical (253), lethal (254), and unknown (255) source
cells remain unchanged. Road at cost 0 and unknown cells do not seed a field.
Soft points alone generate local repulsion, not the two-lane rings. Both fields
are rebuilt from fresh observations, so repeated updates do not accumulate
cost. This needs at most two distance transforms per changed grid, rather than
a distance computation for every point and MPC sample. The existing
oriented-footprint collision check still applies.

The master grid is rebuilt over the complete local window before inflation,
preventing feedback of the previous field or ghosts after obstacle removal.
The distance field is cached while fused cell data, size and resolution are
unchanged. Inflation is computed once per changed costmap, not per MPC sample.
It runs while control is inactive too, once the controller lifecycle is active.
Restart the controller/application after editing these YAML parameters.

The same nearest-obstacle construction as the planner creates preferred
distances, not labelled or directional lanes. The local grid remains 5 cm, so
the configured 4 cm-wide bands are represented only where cell distances allow.
Published output is `/local_costmap/costmap_raw`; MPC telemetry captures this
same final grid when evaluating candidates.

ABORT clears the control request and immediately publishes READY if a path is
stored, or IDLE otherwise. Cancellation of a pending FollowPath action runs
internally. A new START is accepted after that action finishes; late results
from the stopped action preserve READY/IDLE. PAUSE retains the path for RESUME.

## Confirmed arrival: SUCCESS

After START has been accepted, `path_executor` publishes a latched `SUCCESS`
status when the measured robot center stays inside the green goal circle while
remaining stopped continuously for the configured dwell time. Position is
`base_link` transformed into the global path frame at the odometry timestamp;
velocity comes from measured odometry, including planar lateral velocity and
yaw rate. Final orientation is not part of this SUCCESS condition.

Tune these in `path_executor.ros__parameters` of `control_sim.yaml` or
`control_real.yaml`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `goal_circle_radius` | real: 0.30 m; sim: 0.20 m | Shared radius for the green circle and arrival detection |
| `success_dwell_time` | 2.0 s | Continuous stopped time, measured on the ROS clock |
| `stopped_linear_velocity` | 0.01 m/s | Maximum measured planar speed considered stopped |
| `stopped_angular_velocity` | 0.05 rad/s | Maximum absolute measured yaw rate considered stopped |
| `success_odom_topic` | `/odometry/filtered` | Measured odometry source |
| `success_odom_timeout` | 0.5 s | Maximum measurement age and gap between qualifying samples |

Motion, leaving the circle, unavailable TF, stale or invalid odometry, clock
rewind, PAUSE, ABORT or a replacement path reset the dwell. Repeated timestamps
do not advance it. No SUCCESS is reported before START or during PAUSE.
The radius applies to the robot center, not the footprint.

On SUCCESS the executor ends its control request and cancels any remaining
FollowPath action. Late action results preserve SUCCESS. The GUI shows SUCCESS
in green, restores START and disables PAUSE. A new path, START or ABORT clears
the latched success state; there is no automatic restart.

Nav2's existing goal checker still decides when FollowPath finishes. If it
finishes first, the procedure stays ACTIVE while waiting for the measured
arrival condition, then reports SUCCESS instead of returning directly to READY.
Errors remain ERROR and cannot produce SUCCESS from subsequent idle samples.
Restart the control application after changing these parameters.

## MPC telemetry and optional image preview

The preview Python code is grouped in `limo_controller/mpc_preview/`:
`node.py` handles ROS subscriptions and image publication, `render.py` draws
the preview, and `sources.py` decodes laser and semantic observations.
The ROS executable is `mpc_preview`.

The active avoidance reference is drawn in dark blue and labeled `DIJKSTRA`
or `SMAC`; the selected MPC prediction remains green and the global path magenta.
An orange dot with a white border marks the actual temporary rejoin target.
The dot disappears when avoidance ends or the snapshot expires.
Labeled markers also show the normal MPC lookahead, the laser-trigger
lookahead and the prospective rejoin target, even without an avoidance path
or after a failed search. These are the actual reference points evaluated by
the controller in that snapshot; unavailable points and stale snapshots do
not leave old markers visible. Coincident lookaheads retain separate labels.
An orange contour shows a uniform 5 cm Euclidean margin around the padded
controller footprint (`ObstacleGuidance.trigger_margin: 0.05`), with round
corners and a clear interior. It remains visible from the live footprint before
START and while MPC telemetry is paused. Both application launches share the
controller's configured margin and optional front extension (0.0 m by default)
with the preview; the top legend shows their values.
The reference comes from the same controller snapshot, and disappears when no
detour is active or the snapshot is stale. Select the backend with
`FollowPath.MppiPath.ObstacleGuidance.use_smac` (`true` in simulation, `false`
on the real profile). Restart the controller after switching. After this
telemetry-interface update, rebuild `limo_interfaces`, `limo_dwb_critics` and
`limo_controller`, then restart both the controller and preview.

`FollowPath.MPC.Debug.enabled` enables telemetry on `/limo/control/mpc_debug`.
It is explicitly `true` in both `control_sim.yaml` and `control_real.yaml`.
The control GUI contains only the control buttons and current status label; it does
not subscribe to MPC telemetry or images. Read numerical telemetry in a separate
terminal with `ros2 topic echo /limo/control/mpc_debug --no-arr`.
Timing CSVs are written to `~/.ros/limo_timing` (`/root/.ros/limo_timing` in
Docker). `ros2 run limo_dwb_critics analyze_mpc_timing.py --latest` summarizes
MPC and SMAC/Dijkstra search times, including failures. In async mode,
`plan_called=1` records one consumed worker completion, including discarded
results; submissions and running cycles are not attempts or failures.
`plan_ms` measures background worker duration and can exceed `cycle_ms` while
the controller stays within budget. Controller `cpu_ms` excludes the worker;
live-map acceptance checks remain in controller preparation. The appended
`plan_async`, `plan_running` and `plan_discarded` columns preserve the original
30 column positions. Diagnostics and the analyzer count async completions,
in-flight cycles and discarded results separately. Actual worker-start
timestamps and result epochs let the report check SMAC's 250 ms spacing at
4 Hz within epochs independently of search duration; historical short intervals
across epochs are listed separately. The runtime cap also spans reference and
detour resets. Older CSVs without async columns remain
supported.
`FollowPath.MPC.Debug.preview_enabled` starts the image renderer: it is `true`
in `control_sim.yaml` and `false` in `control_real.yaml`. For the real robot,
`desktop_app.launch.py` starts the renderer on the PC by default; open
`rqt_image_view` manually. Both the standalone controller and application
launches use the selected controller YAML setting.
Use `start_mpc_preview:=true` or `start_mpc_preview:=false` to override it for
one run. The viewer can run with MPC telemetry disabled: it still shows the
planned global path and published local costmap before START and during PAUSE.
Clock overrides do not change these flags; a custom controller YAML uses its
own flags (missing means disabled).

The node publishes `sensor_msgs/CompressedImage` on
`/limo/control/mpc_preview/image/compressed` (PNG by default).
The renderer opens no window of its own; `rqt_image_view` can display the
compressed image separately from the control GUI.
Image output uses reliable delivery to support Foxy's `rqt_image_view`; the
telemetry subscription remains best effort.

The top-down image is centered on `base_link`, with +x upward and +y leftward.
The shipped viewer YAML defaults to `view_mode: sources` and
`source_cost_colors: true`, showing the original laser/cloud point coordinates
before rasterization, including before START and during PAUSE. Point labels
are retained through TF, and `source_class_costs` assigns their normal 0..100
costs: yellow 60, soft 30, boardwalk 90. `source_laser_cost: 100` renders laser
returns black; other source costs use Jet with a 0..100 point-cost legend.
These colors never come from a costmap-cell lookup. Boardwalk points at source
cost 90 appear red here even when the Nav2 lethal threshold makes their
rasterized cells lethal. Select `view_mode: costmap` to inspect that separate
0..255 Nav2 conversion; `costmap_as_points: false` shows it as a filled raster.
`costmap_colormap: jet` maps raw costs 0..253 to blue/cyan/green/yellow/red
with a fixed numerical color bar; lethal cells (254) are black and unknown
cells (255) remain gray. `lethal_color` configures the lethal-cell color in
both raster and cell-center views.
Set `costmap_colormap: classic` to use the previous configurable orange palette.
The source view shows every valid `/scan` endpoint and
every finite semantic obstacle point from
`/limo/map_package/online/local_ctrl_map/points`: classes 2/3/4/6 in blue.
Road classes 1/5 are excluded from the viewer before TF projection and drawing.
The cloud contains both recent
observations and semantic memory, before grid rasterization or Nav2 inflation.
Labels are preserved during decoding and rendering; no point subsampling is applied.
`source_cost_colors: false` restores the old laser-red/CV-blue diagnostic colors.
The laser range is the sensor's declared range unless `laser_max_range` is set
to a positive cap. Counts include all decoded points, including distant returns
outside the current viewport. `pixels_per_meter` controls the displayed extent.

Source timestamps are transformed through `fixed_frame: odom` into the displayed
robot pose. The source view works before START and without a master costmap;
stale or TF-unavailable inputs are identified in the status line. This changes
only diagnostics and does not alter MPC scoring or the costmap configuration.

Set `costmap_as_points: true` in costmap mode to draw every nonfree cell as a
point at its center, colored by its actual raw cost in Jet mode, including soft
semantic and inflation costs as well as lethal cells. Unknown cells are gray;
free cells remain blank. The legend reports the number of positive-cost cells.
`costmap_as_points: false` restores the colored raster in this mode.
`costmap_point_size` changes only marker size in either mode.

The view also contains the global reference in magenta and representative predicted base_link
trajectories. A green circle marks the last point of the global reference;
`path_executor.ros__parameters.goal_circle_radius` in the selected controller
profile sets its radius in meters (default 0.20). The launch passes this same
radius to the preview and the SUCCESS detector. A standalone preview uses
`goal_circle_radius` in `config/mpc_preview_sim.yaml` instead (0 hides its circle).
The circle is drawn before START and during PAUSE too, and disappears when the
global path is cleared. Initial and first
winning velocity/steering values are shown numerically in the legend. These are
model states, not measured steering joints. The bicycle drawing, steering arrows
and magnified model inset are omitted to leave the costmap visible.

| Trajectory family | Default color |
| --- | --- |
| Braking | Red |
| Nominal/warm start | Blue |
| Constant curvature | Purple |
| Perturbed nominal | Teal |
| Broad exploration | Orange |
| Structured blocks | Magenta |
| Monophase | Cyan |
| Slow starts | Violet |
| Selected trajectory | Green with a white outline |
| Global reference path | Magenta with a white outline |

Rejected or effort-pruned candidates can be shown as dashed lines. A finite
candidate score may have been short-circuited by DWB and does not certify that
all its critics were checked. The selected candidate is fully scored by the
controller. The preview is diagnostic and has no effect on selection.

Settings are split between two YAML files:

- `config/control_sim.yaml` / `config/control_real.yaml`, under `FollowPath.MPC.Debug`:
  enable telemetry with `enabled` (true in both profiles) and start the image
  renderer with `preview_enabled` (true in simulation; the real desktop runs it on the PC); configure the
  telemetry topic, publication rate, samples per family and pose downsampling stride.
  Representatives are evenly distributed over each family's generated order;
  the winner is always included even if it was not among those representatives.
- `config/mpc_preview_sim.yaml`, under `mpc_preview.ros__parameters`: telemetry,
  global-path, raw-costmap, scan, local-cloud and image topics, view mode,
  base/fixed frames, optional laser range limit, rendering rate, image
  dimensions, pixels per meter, grid spacing,
  colors, line widths, PNG/JPEG format and
  compression settings. Input topic changes must match the telemetry topic.

The footprint comes directly from the controller snapshot, so editing it does
not require duplicating it in the renderer YAML.
Simulation exports MPC snapshots and images at a target of 10 Hz; the real
profile retains 4 Hz MPC telemetry. Each snapshot contains up to 27 representatives
per family (one each for braking/nominal, up to three slow starts), plus the winner if necessary. Fonts
and nonfree cell indices are cached; cell transforms share one affine map per
image. PNG compression defaults to level 1, keeping every point and the full
960 x 960 image while reducing encoding work. No snapshot is constructed
without a telemetry subscriber. Rendering and image compression happen in the
separate Python node, using Pillow (`python3-pil`).

The `limo_interfaces/MpcDebug` snapshot carries raw Nav2 costmap bytes from the
same locked control cycle as the candidates. While telemetry is fresh, those
candidates and that costmap retain their shared pose and timestamp. The global
reference is independently read from `/limo/planning/path` and transformed into
the snapshot frame using TF at the snapshot time.

In `view_mode: costmap`, without a fresh MPC snapshot, the viewer uses
`/local_costmap/costmap_raw` and
the current TF robot pose instead. This is Nav2's complete master costmap after
laser/semantic fusion and inflation, preserving its raw 0--255 costs. It is
published at the costmap's `publish_frequency` (10 Hz in simulation, 2 Hz on the
real profile), independently of FollowPath execution. Both context subscriptions
use reliable, transient-local delivery so an existing map and planned path are
available to late viewers. An empty path clears the previous reference.

The rendering timer uses a steady clock. After `stale_timeout` without new MPC
telemetry, only the old candidates and command values disappear; the global
path and latest local costmap remain visible. Costmap updates and robot motion
continue to change the image while control is paused. If map messages stop for
`costmap_stale_timeout`, the last map remains with a stale label. Missing TF is
reported instead of drawing a misaligned layer. The stopped-control view also
works before the first MPC snapshot; its footprint appears once telemetry has
supplied it. Its image timestamp is the current render time.

After changing YAML settings, restart the application. A custom renderer file
can be selected with `mpc_preview_params_file:=/path/to/preview.yaml` in either
the standalone controller launch or the application launch.

### Controller costmap frequency

The MPC reads Nav2's internal local costmap, merging `/scan` and the raw
semantic grid `/limo/map_package/online/local_costmap` before inflation.
Its configured update and publication rates differ:

| Profile | Internal update target | Topic publication target | Controller target |
| --- | --- | --- | --- |
| sim | 15 Hz | 10 Hz | 20 Hz |
| real | 10 Hz | 2 Hz | 10 Hz |

Read the active configuration in the ROS environment running the backend:

```bash
ros2 param get /local_costmap/local_costmap update_frequency
ros2 param get /local_costmap/local_costmap publish_frequency
ros2 param get /controller_server controller_frequency
ros2 topic hz /limo/map_package/online/local_costmap
ros2 topic hz /local_costmap/costmap_updates
```

The first topic measures the incoming semantic grid; the second measures
published Nav2 map deltas. With `always_send_full_costmap: false`, full maps
and deltas use different topics. Neither topic measures the internal update
loop. The parameters are targets; measuring achieved internal update frequency
requires timing that loop. MPC debug snapshots are throttled to 10 Hz in
simulation and 4 Hz on the real profile, and their
costmap timestamps describe capture time, not the last internal update.

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
