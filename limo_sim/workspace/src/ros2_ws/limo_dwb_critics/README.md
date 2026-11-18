# LIMO sampled Ackermann MPC on Nav2 Foxy

`limo_dwb_critics::AckermannMPCController` is a Nav2 controller derived from
`dwb_core::DWBLocalPlanner`. It reuses DWB plan transformation, costmap locking,
critics, lifecycle and trajectory visualization, and replaces the command search
with finite-sample nonlinear shooting MPC. It does **not** implement MPPI's
soft-weighted update or a continuous constrained optimization solver.

Each cycle:

1. Read the measured pose. Initialize speed and bicycle steering from odometry
   on a fresh start, then update the actuator-state estimate according to
   `MPC.feedback`.
2. Shift the previous winning target sequence by the elapsed control steps.
3. Sample piecewise target velocity/steering sequences around that solution,
   plus broad exploration, constant-curvature seeds and a braking sequence.
   Perturbations repeat across cycles to reduce sampling-induced command noise;
   their nominal sequence still shifts and changes with every solution.
4. Roll out the constrained bicycle model over the full horizon. Score these
   trajectories with DWB critics plus normalized acceleration/steering effort.
5. Publish **only the first reachable command** of the winning sequence. Repeat
   from the measured pose at the next control cycle.

The model default budget is 768 sequences, 50 steps of 0.05 s, and five control
segments. The simulation profile uses 80 steps (4 s) to include the steering
reversal and straightening needed to pass an obstacle. This is a finite search,
not a guarantee of a globally optimal or
recursively feasible solution. If every candidate is invalid, the controller
raises DWB's normal no-legal-trajectories exception; it never reuses a stale
command as a fallback. A new path, deactivation, failed search, or a control gap
longer than three periods discards the warm start.

The control period/watchdog uses a monotonic wall clock, matching Foxy's
controller loop. Repeated Gazebo `/clock` stamps (for example 10 Hz clock
publication with 20 Hz control) do not erase the command ramp. Backward ROS
clock jumps still reset the prediction.

Reference association compares position and chassis orientation. Alternatives
within 0.10 m of the nearest spatial projection compete using
`distance² + (0.25 * wrapped_yaw_error)²`. Robot progress is retained across
cycles: the search allows 0.10 m behind the furthest reached arc length and
at least 0.50 m ahead (or twice the displacement since the last update).
This excludes earlier laps when a loop returns to the same position and yaw.
Completed branches are removed before DWB transforms/prunes the local plan;
tube progress and path critics use the same pose comparison. Prediction
resets and identical republished paths preserve this history; a changed
reference or cleanup resets it. Candidate scoring never updates robot history.

Simulation optionally exports `limo_interfaces/MpcDebug` snapshots for the
compressed-image preview in `limo_controller`. The sampler assigns an explicit
family and candidate ID at generation time. A bounded observer records
representative rollouts and the winner without changing the search objective,
candidate order or random perturbations. Costmap bytes, model geometry and
initial actuator state are captured in the same controller cycle. Disabled
telemetry, or a publisher without subscribers, constructs no snapshot. See the
[preview settings](../limo_controller/README.md#simulation-mpc-image-preview).

When obstacle guidance is enabled, every ordinary critic preparation refreshes
its full clearance/cost grid from the current costmap, including free paths.
Footprint rasterization and proximity checks reuse per-critic scratch buffers
across poses and candidates. These are allocation/work reductions; sampling,
scores, collision thresholds and control frequencies are unchanged. Rebuild
`limo_inflation` and `limo_dwb_critics`, then restart the application to load
the updated libraries. Scratch buffers rely on the controller's existing
serialized scoring; parallel scoring would need independent storage per worker.

Guidance also reuses the search heap and precomputes clearance offsets,
cost-to-penalty values and the eight neighbor steps. Only scalar/geometric
work is cached: every update still rebuilds blocked cells and penalties from
the current map, and every target search recomputes the complete distance field
with the same heap ordering and diagonal-collision rules.

The Gaussian and path-distance costmap layers reuse mask/output storage for
their precise OpenCV distance transforms. Gaussian broad-field seeds are filled
while scanning the original source cells for narrow inflation, avoiding a second
source scan. The source snapshot, MAX merging, metric distances and costmap lock
lifetime retain their existing semantics. These changes reduce work under the
lock; the resulting MPC waiting time must be checked in another robot capture.

### MPC timing telemetry

The real and simulation profiles enable `FollowPath.MPC.Timing.enabled` and
publish `diagnostic_msgs/DiagnosticArray` on `/limo/control/mpc_timing`, with
`publish_rate: 1.0` and `log_summary: true`. These parameters are read during
configuration; the plugin default is disabled. Set `enabled: false` to disable
timing collection, or `log_summary: false` to retain only the topic. The rate is
configurable up to 10 Hz. No background timer, image rendering or per-candidate
messages are added; reports are produced only by active control calls.

Each report contains per-window means and maxima in milliseconds:

| Metric | Measured work |
| --- | --- |
| `cycle_ms` | Controller plugin call, inside its serialization lock; excludes timing-report publication and the outer Nav2 loop. |
| `thread_cpu_ms` | CPU time used by the calling controller thread over the same work; excludes the async planner worker and time asleep or descheduled. |
| `prepare_ms` | Work before the MPC scoring core, including plan transforms, costmap-lock waits and critic preparation. |
| `plan_prepare_ms` / `plan_prepare_cpu_ms` | Wall / calling-thread CPU time in DWB plan preparation: path projection, pruning and transforms, transformed-plan publication and goal transform. This is not TF waiting alone. |
| `costmap_lock_wait_ms` | Time acquiring the costmap mutex, before any critic is prepared. |
| `critics_prepare_ms` / `critics_prepare_cpu_ms` | Wall / calling-thread CPU time preparing all critics, while holding the costmap mutex. |
| `guidance_grid_ms` | Clearance/cost grid update; included in preparation. |
| `solve_ms` | Candidate generation, rollout, scoring and preview observer work inside the solver. |
| `scoring_ms` | Candidate trajectory conversion, terminal checks and critic scoring, plus the winner's final rescore. |
| `debug_ms` | Preview snapshot capture, candidate observer and final publication; observer work is also included in `solve_ms`. |
| `debrief_ms` | Stateful critic updates after scoring, still under the costmap mutex. |
| `dwb_publish_ms` | Local-plan and cost-grid publication after releasing the costmap mutex; excludes the outer evaluation publication. |

Fields use suffixes `_mean` and `_max`, for example `cycle_ms_max`.
Each configured critic also has `critic_prepare/<name>_ms_mean` / `_max`,
`critic_prepare_cpu/<name>_ms_mean` / `_max`, and
`critic_prepare/<name>_calls`. These means count only preparations actually
entered, including a throwing call; later critics skipped by an exception are
not treated as zero-duration calls. Buffers and critic names are allocated during
configuration and reused on every control cycle. A greater wall time than CPU
time indicates time off CPU; the difference alone does not identify its cause.

The timed 2D preparation boundary follows the Foxy DWB sequence, including its
mutex lifetime, warning-only false prepare results and `NoLegalTrajectoriesException`
handling. With collection disabled, it delegates directly to DWB. Plan preparation
and critic exceptions still propagate with their original semantics; partial
stage measurements are retained for the failing cycle.

The timings overlap and must not be added together. Counters include `cycles`,
`failures`, `deadline_misses` (plugin duration strictly above `1000 * model_dt`),
`candidate_generated` (after deduplication), `candidate_evaluated`,
`candidate_rejected` (illegal evaluated candidates), `grid_updates` and
`detour_cycles`. Candidate counts exclude the final winner recheck and do not
equate effort-pruned or duplicate candidates with collision rejections.
`budget_ms` is 100 ms in the real profile and 50 ms in simulation. These are
plugin execution measurements, not command arrival intervals or the complete
Nav2 loop duration. A window reports after at least one configured interval of
control samples; lifecycle deactivation discards an incomplete window.
Diagnostic failures are contained and cannot replace a control result or error.

```bash
ros2 topic echo /limo/control/mpc_timing
```

Both profiles also set `MPC.Timing.csv_directory: "~/.ros/limo_timing"`.
Each controller configuration opens a unique `mpc_timing_<timestamp>_<pid>.csv`
and logs its complete path. In Docker this directory is
`/root/.ros/limo_timing`. Each active controller call records one row, including
idle guidance cycles and failed attempts. Set the directory to an empty string
to disable file logging; `Timing.enabled: false` disables both file and topic
collection. Rows are buffered, flushed once per second and on deactivation.
File errors disable only CSV output and leave control and diagnostics running.

For SMAC/Dijkstra comparisons, group rows by `backend` and `plan_async`, and
compare `plan_ms` only where `plan_called=1`, reporting `plan_failed` separately.
For synchronous planning, a call is an attempt performed in that control cycle.
With SMAC async planning, a call is one completed worker result consumed by the
controller, including a discarded stale result. Submission and in-flight cycles
record no call or failure. `plan_ms` is the worker's preparation, native search
and snapshot-validation duration; it can exceed `cycle_ms` because that work
runs in the background. The immutable map copy and current-map result validation
remain in controller preparation. `cpu_ms` measures only the controller thread.
`grid_ms` measures the shared clearance-grid preparation. Compare `cycle_ms`,
`cpu_ms`, `solve_ms` and `scoring_ms` on matching `detouring`/`success` conditions, using the same
scene and debug settings. Each row also records the ROS and monotonic timestamps,
session/cycle numbers, budget, deadline misses and candidate counts. A new
activation increments `session` without mixing its counters with earlier rows.
CSV formatting, flushing and timing-report publication happen after the
recorded plugin duration; they still consume controller-thread time.
The original 30 CSV columns retain their positions; `plan_async`, `plan_running`
and `plan_discarded` are appended. `plan_running` can coexist with a consumed
completion when another request has already started.

The CSV also records `plan_start_steady_ns`, the actual search-start timestamp,
`plan_epoch`, and the configured `replan_hz`. Async completion rows retain the
worker's actual start and result epoch, including discarded results. A new SMAC
detour or reference increments the epoch so its first search is
identifiable. The derived columns `plan_interval_valid`, `plan_interval_ms`,
`rate_limit_violation`, and `cadence_reset` check start-to-start spacing,
including failed attempts. At the configured maximum of 4 Hz, successive
searches must be at least 250 ms apart, including across reference and detour
resets. Reported intervals remain separated by epoch for CSV compatibility;
the analyzer lists historical short cross-epoch intervals separately. A 1 ns
rounding allowance is used. Event-driven searches may be farther apart, and a
valid cached route with a stable target can produce no search intervals. The first search
after a reset is recorded separately. Completion timestamps and counts in a
one-second reporting window cannot establish this constraint.

Console summaries and `/limo/control/mpc_timing` include
`guidance_replan_hz`, `guidance_observed_hz`,
`guidance_plan_interval_ms_min` / `_mean` / `_max`,
`guidance_plan_intervals`, and `guidance_rate_limit_violations`. There must be
at least one measured interval to assess the cadence. Cached successful
cycles without a search appear in `guidance_cache_cycles`; failed SMAC
cycles with no active route or search appear in `guidance_waiting_cycles`.
`guidance_async_cycles`, `guidance_plan_running_cycles`,
`guidance_async_completions` and `guidance_plan_discarded` count background-mode
cycles, in-flight cycles, consumed worker completions and discarded completions.
A rate-limit violation raises a warning independently of cycle overruns.

After restarting the controller with the updated logger, summarize the latest
CSV from a sourced ROS terminal inside Docker:

```bash
ros2 run limo_dwb_critics analyze_mpc_timing.py --latest
```

The report compares synchronous attempt and async worker times, including mean,
median, p95 and max, MPC cycle stages, failures and missed deadlines. It labels
async completions, discarded results and in-flight cycles separately from
controller elapsed time. It independently recomputes
the cadence from search-start timestamps and lists short intervals caused by
new epochs separately. CSVs without async columns remain supported; only CSVs
missing search-start metadata cannot certify the 4 Hz constraint.
To compare recorded SMAC and Dijkstra runs, pass both CSV
paths; `--json` produces machine-readable output.

## Model and parameters

`FollowPath.MPC.model_dt` must equal `1 / controller_frequency`. The prediction
horizon is `MPC.time_steps * MPC.model_dt`; `sim_time`, `vx_samples` and
`vtheta_samples` apply only to the standard DWB fallback. Parameters are read at
configuration time; reconfigure the controller after changing them.

`MPC.feedback: OPEN_LOOP` continues the velocity/steering ramp from the previous
issued command, like the command-state convention of Humble's OPEN_LOOP
velocity smoother. This avoids a startup lock where each cycle commands only
`v_odom + acceleration * dt` while actuator lag/friction keeps odometry near
zero. Position feedback, collision checks and the progress checker remain
active. The rollout assumes commanded speed/steering are tracked; it does not
estimate actuator lag. `CLOSED_LOOP` instead starts each rollout at measured
velocity and is appropriate when low-level tracking and odometry are reliable.
Commands are never retained across failed searches, new paths or stale cycles.

`MPC.prefer_motion: true` gives priority to feasible candidates with a first
reachable velocity above the existing stopped threshold (0.001 m/s), outside
`FollowPath.xy_goal_tolerance` of the final
global goal. The cheapest such candidate wins even when stopping costs less.
Moving and stopping candidates use independent cost bounds so a cheap stop
cannot prevent evaluation of a moving candidate. All collision, kinematic and
terminal-tube checks still apply; the winner is scored fully before issuing
its first command. If no positive first command is feasible, the best legal
fallback remains available and a throttled warning explains a stationary
selection. Within the goal tolerance, ordinary cost selection permits stopping.
The temporary SMAC target and the clipped reference endpoint do not count as
the final goal. Set this parameter to false to restore pure cost selection.

The model enforces the minimum radius, longitudinal acceleration/deceleration,
steering rate and yaw-rate limits at **every predicted step**. The smaller of
`acc_lim_theta` and `-decel_lim_theta` is used in both angular directions.
Reversing first brakes to zero. Observed overspeed is reduced at the configured
rates instead of being clipped instantaneously.

`AckermannKinematics.min_turning_radius` is shared by the model and the existing
critic. The critic remains a final check on the command sent to Nav2.
`MPC.max_steering_rate` limits the equivalent bicycle angle, not the inner wheel
angle. The YAML uses 0.5 rad/s (0.025 rad per 20 Hz command), a tuning estimate,
not a measured LIMO
actuator specification. `MPC.wheelbase: 0.20` and `MPC.rear_axle_to_base: 0.10`
match the Ackermann URDF; set the latter to zero if the navigation base frame
is actually located on the rear axle.

### Gazebo model and command conversion

`limo_app_sim.launch.py` passes `robot_model:=sim` to the controller launch.
This overrides the bicycle wheelbase to 0.24 m and rear-axle offset to
0.12 m, matching `limo_car/gazebo/ackermann.xacro`. Its minimum radius is 0.55 m,
leaving margin for the wheel collision-center offsets when enforcing the
simulated inner steering joint's 30-degree limit.
`robot_model:=real` preserves the physical/custom geometry from the YAML.
When launching `control.launch.py` directly for Gazebo, specify
`robot_model:=sim` explicitly; `use_sim_time` alone does not select geometry.

Foxy's `gazebo_ros_ackermann_drive` interprets `Twist.angular.z` as an equivalent
steering angle and multiplies it by the sign of longitudinal speed internally.
It is **not** the same command interface as Nav2 or the physical LIMO driver.
The simulation launch starts `limo_car/gazebo_twist_adapter.py`, converting
standard `/cmd_vel` to `/cmd_vel_gazebo` using
`angular.z = atan(wheelbase * yaw_rate / abs(speed))`, with steering saturation
and zero-speed handling. A wall-clock watchdog stops the plugin after 0.5 s
without commands. The physical robot continues receiving standard `/cmd_vel`.

Source: [Gazebo Foxy Ackermann drive implementation](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp).
The simulator must be restarted/robot respawned after updating its command
topic; restarting only the navigation app cannot change an already spawned
Gazebo plugin. Do not publish directly to `/cmd_vel_gazebo` using standard
yaw-rate commands.

The model assumes rear-axle longitudinal speed and no slip. It integrates the
rear-axle arc and translates the predicted pose to `base_link`. Output `Twist`
contains longitudinal speed and yaw rate, with `linear.y = 0`, matching the
driver command interface. It does not use observed lateral velocity. Verify
odometry and actuator response against this convention: the current LIMO driver
computes Ackermann odometry using additional steering projections, so agreement
with the physical robot is **not established by these software tests**.
Below `MPC.steering_feedback_min_velocity` (0.05 m/s), yaw-rate/velocity is too
noisy to estimate steering: retain the previous command's steering, or assume
centered wheels on a fresh start, as requested by the driver's zero-speed command. This
does not model steering feedback, actuator lag, wheel slip or moving obstacles.

`MPC.acceleration_weight` and `MPC.steering_weight` penalize squared normalized
input increments, averaged over the horizon. They are additional regularizers,
not a translation of MPPI's `gamma` or `temperature`.

`MPC.steering_command_weight: 0.2` additionally penalizes the squared first
steering increment normalized by `max_steering_rate * model_dt`.
`MPC.steering_rate_change_weight` (0.3 in simulation, 0.1 on the real robot)
penalizes its difference from the previous issued steering increment, in the
same normalized units. These immediate
costs are **not divided by the horizon length**, so increasing the prediction
horizon does not dilute command continuity. Steering-rate history resets with
the warm start. Both are soft costs: collision rejection can still force a
change or braking, and no command is filtered after trajectory evaluation.
The simulation YAML uses local steering sampling deviation 0.10 rad; the real
profile uses 0.20 rad. Simulation uses 768 candidates with 8 velocity samples
and 15 curvature samples. The real profile uses 128 candidates: 6 x 11
constant-curvature seeds, braking, warm start and 60 sampled sequences.
Both retain full-range seeds and broad exploration. The real profile uses four
times the reference obstacle weight to favor
local-costmap clearance relative to path tracking; other critic weights retain
their reference values.

Simulation additionally uses `MPC.reverse_distance_weight: 2.0` per metre of
predicted reverse motion and `MPC.direction_change_weight: 0.5` per transition
between forward and reverse. Direction history survives an intermediate stop
and records only the command actually issued. It resets with the warm start.
These soft costs discourage short alternating commands while allowing reverse
when it provides enough progress or clearance. Both weights default to zero;
the real profile retains its existing behavior.

## Cost correspondence with Humble MPPI

The reference is `origin/humble-navigation` at
`Limo_robot_dev_mk2/src/ros2_ws/navigation_pipeline/nav_limo_controller/config/mppi_control_params.yaml`.
The two new DWB critics deliberately replace raw DWB map-grid costs so the
Humble weights have comparable units. All cost powers are fixed at 1, as in
that reference profile.

| Humble MPPI term | Foxy configuration | Behavior |
| --- | --- | --- |
| CostCritic, weight 3 | Real `MppiObstacle.scale: 12`; simulation: 6 | Mean center cost / 254; critical cost 300; real repulsion remains active up to the goal, simulation disables it below 0.15 m; critical and collision checks remain active |
| GoalCritic, weight 5 | `MppiPath.GoalCritic` | Mean distance in meters, enabled within 1 m of the goal |
| GoalAngleCritic, weight 3 | `MppiPath.GoalAngleCritic` | Mean wrapped yaw error in radians, within 0.5 m |
| PathAlignCritic, weight 10 | `MppiPath.PathAlignCritic` | Mean path-pose error, with orientations; off within 0.5 m or if local path occupancy exceeds 15% |
| PathFollowCritic, weight 5 | `MppiPath.PathFollowCritic` | Terminal distance to a metric lookahead target; off within 1 m of the goal |
| PathAngleCritic, weight 2 | `MppiPath.PathAngleCritic` | Terminal heading toward the lookahead target, enabled for initial errors above 1 rad and outside 0.5 m; forward preference in simulation, forward/reverse symmetric on the real profile |
| ConstraintCritic / collision_cost | Model constraints and exceptions | Infeasible/colliding candidates are rejected, not assigned a finite penalty |

This is an adaptation, not a port of MPPI critics. DWB has no batch-wide
furthest-trajectory index: the MPPI point offsets are replaced by
`PathFollowCritic.lookahead_distance: 1.25` m. Alignment uses monotone,
distance-limited nearest path poses. Goal-distance gates use the actual goal rather than a clipped local
path endpoint. Every predicted pose is collision checked; there is no
`trajectory_point_step` skipping. Both footprint contour and center are
checked, while footprint interiors are not exhaustively rasterized.

Inflation now matches Humble (`0.30` m, scaling factor `8.0`), retaining this
branch's physical footprint and semantic lethal threshold 86. The finite set
of predictions uses discrete collision checks, not continuous swept-volume
checking or a terminal invariant-set constraint. Equivalent weights do not
guarantee identical behavior; simulation and hardware tuning remain necessary.

## Online obstacle detours

`FollowPath.MppiPath.ObstacleGuidance.use_smac` selects the backend: `false`
retains Dijkstra; `true` runs Foxy's SMAC Hybrid-A* with forward Dubins motion.
Simulation selects SMAC for the comparison; the real profile retains Dijkstra.
Restart the controller after switching. Only the selected planner runs.

Both backends share the laser trigger and initial target selection.
Detection checks the connector from the robot to the local reference and the
reference up to `ObstacleGuidance.lookahead_distance: 1.0` m, including intervals
between path samples. Only marked laser cells (254) from `trigger_layer: obstacle_layer`,
configured with `/scan`, initiate the detour; semantic costs and unknown sensor
cells do not. This distance only controls laser detours; normal tracking keeps
`PathFollowCritic.lookahead_distance: 1.25` m. The target starts at the 1.0 m detour
lookahead point. If that point is blocked in the common clearance grid, it
advances to the next free reference
point. It then advances along consecutive free points by up to
`rejoin_distance: 0.50` m to leave room to straighten. SMAC additionally checks
the final candidate's full filled, oriented footprint on the current map and
skips invalid candidates forward without invoking or counting a search. This
preserves the center-based straightening distance; Dijkstra selection is unchanged.
Dijkstra replans on every controller cycle while detouring. SMAC validates the full cached route,
including its robot connection and oriented footprint, against the current
map on every active cycle. It requests a search on first activation, when
the cache is missing or invalid, or when the desired rejoin target moves at
least `ObstacleGuidance.Smac.replan_goal_distance: 0.25` m or its wrapped yaw
difference reaches `replan_goal_angle: 0.35` rad (about 20 degrees). Position
and heading changes are measured from the last successfully accepted target,
so small changes accumulate relative to that target. A valid route with a
stable target remains cached without periodic searches.

`ObstacleGuidance.Smac.async_planning: true` runs native searches on one
persistent worker. Each accepted request owns an immutable costmap copy,
footprint, start, target and reference generation; there is no request queue or
second concurrent search. The controller consumes each available result once
without waiting for native planning. It fully revalidates the returned route, robot
connection, exact target connection, clearance and oriented footprint against
the current costmap before accepting it. While planning, MPC uses the old route
only while its current-map checks pass; without one it commands zero velocity
and keeps navigation active. New references, detour resets and lifecycle
deactivation invalidate older generations, so late results cannot restore an
obsolete route. Worker destruction joins outstanding native work.
Set `async_planning: false` for deterministic synchronous backend benchmarks;
this mode performs the search in the controller call.

`ObstacleGuidance.Smac.replan_frequency: 4.0` caps these event-driven searches
using a steady clock (at least 0.25 s between actual worker starts). Failed and
discarded attempts obey the same cap. Its last actual start survives reference,
detour and deactivation resets; a new reporting epoch cannot bypass the cap.
After each actual failed search,
`Smac.failure_advance_distance: 0.25` m advances the next retry candidate along
the original reference to the next free point at or beyond that arc distance.
The advance clamps to the last available reference point if less than 0.25 m
remains, provided that endpoint is free.
Cycles waiting for the rate cap do not advance the candidate. Retry demand
overrides the position and heading thresholds, including when a safe route is
still cached. A failed retry retains that safe route and the last accepted
target. Success clears the pending retry but keeps the advanced target as the
earliest allowed target until a new reference or the end of the detour resets
it. This prevents retreat to an earlier nominal target on the next cycle.
The available reference and local map bound possible extension;
the target never advances beyond the final global goal. A clipped local
reference end is not necessarily the final global goal, so a later snapshot
may expose further candidates.
When target selection finds no candidate, SMAC reuses a safe cached route.
Without one, the controller commands zero velocity until a search can succeed.

`SMAC target unavailable` logs summarize rejected candidates every two seconds
on a steady clock during an active detour. They distinguish center clearance,
off-map centers/footprints, lethal cells and unknown cells. Context includes
selection stages and forward-floor skips, reference end, map geometry, worker
state and cached point count. First/last rejected poses report center cost
separately from the offending footprint cell, or the off-map vertex.
`s_rel` measures arc distance from the current clipped reference start index;
stored goal arc hints retain their original selection context. A single
`SMAC target available again` line reports recovery within the same detour.

Setting `replan_frequency: 0.0` only removes the rate cap; event conditions
still apply. Setting both target thresholds to zero deliberately requests a
search on every eligible cycle for legacy benchmarks; also setting the
frequency to zero removes the interval limit. New reference plans and completed
detours reset route state and reporting epochs while retaining search cadence.
The MPC and lookahead telemetry retain their
configured frequencies. A detour remains active while the reference is blocked
or the robot is more than 0.15 m from it.

SMAC uses the measured pose and the full reference pose at the rejoin point,
including yaw. It searches a private snapshot of the current local costmap,
with the same padded footprint and extra `clearance_margin`. Snapshot inflation
adds one cell diagonal as a conservative quantization allowance for obstacle
and reference cells; this can narrow a passage by a cell. The robot radius is not inflated
twice, and the original costmap remains unchanged. The route
and MPC predictions must also satisfy the common clearance grid. Scoring uses
remaining cost-weighted route length plus distance from the route, and heading
toward a point 0.35 m ahead. The route, including its final connection to the
exact target, is validated with the full oriented footprint against the current
costmap. MPC follows the SMAC path with the remaining local reference appended
after rejoining to avoid an intermediate stop. Alignment, progress and the
terminal tube follow this active reference. Detection still uses the original
local reference on the next controller cycle.
A failed update is counted and retains the previous accepted target only with
a still collision-free cached route.
If no safe route remains, the controller commands zero velocity and keeps the
navigation action alive for the next controller cycle. Preview snapshots stay
fresh and clear the old candidates while waiting. The normal navigation progress
timeout still applies. There is no implicit Dijkstra fallback.

`ObstacleGuidance.Smac` configures `max_iterations` (50000 in simulation, 20000 on the real profile),
`angle_quantization_bins` (64), `tolerance` (0.05 m), `cost_penalty` (defaults to
the guidance weight), `analytic_expansion_ratio`, and `minimum_turning_radius`
(defaults to the controller's minimum). The radius must be at least the
controller's minimum. Forward motion, full resolution, known space and disabled
smoothing are required. Async execution removes native search waiting from the
active controller call; map copying and live route validation still take control
time. Foxy's iteration limit is not a hard wall-time deadline for the worker.

Rejection logs distinguish endpoint prechecks (`start_footprint`,
`goal_footprint`, `start_clearance`, `goal_clearance`), native exceptions or empty
routes, snapshot route/footprint/clearance failures, and
`current_map_validation` after completion. Details include the failing pose,
map cell or footprint/clearance context where available. These diagnostics do
not relax footprint, clearance, known-space or motion constraints. A failed
current-generation completion advances retry demand; a discarded old-generation
completion cannot update the accepted route or retry target.

`/limo/control/mpc_timing` now reports `guidance_backend`, `guidance_plan_calls`,
`guidance_plan_failures`, and `guidance_plan_ms_mean` / `_max`. Dijkstra timing
includes rejoin selection and field calculation. Async SMAC timing records a
consumed worker completion once; its worker duration excludes live-map acceptance
checks, which remain in preparation with cached-route checks. Submission and
in-flight cycles report no search call and zero search time. Means count
completed attempts, including failures and discarded results. Grid work remains
in `guidance_grid_ms`; compare `scoring_ms` and
`cycle_ms` too, because reference queries differ. Compare the same scene and
debug settings after warm-up. The backend test prints `GUIDANCE_BENCH` for
20 warm calls on a fixed 5 x 5 m, 2 cm grid.

The MPC preview draws the selected backend's reference in dark blue, labeled
`DIJKSTRA` or `SMAC`. `MpcDebug.guidance_path` contains at most 512 poses from
the same cycle as the candidates and map; it clears when detouring ends or the
snapshot expires. Dijkstra's display route is extracted on demand without
changing scoring. The same snapshot carries the normal path lookahead,
the laser-trigger lookahead and the prospective free rejoin target, each
with its own validity flag. These points remain available when the trigger
is inactive or planning fails; selecting the prospective target does not
run either planner. They are copies of the original reference poses, so an
active SMAC path cannot replace their coordinates. The GUI labels these
points independently of `guidance_path` and hides them when telemetry expires.
Rebuild `limo_interfaces`, `limo_dwb_critics` and
`limo_controller`, then restart the controller and preview together after the
message-interface change.

Both real and simulation profiles enable `MppiPath.ObstacleGuidance.enabled`.
A detour starts when the trigger footprint intersects a laser obstacle along
the local reference, including between path samples. Both profiles use
`trigger_margin: 0.05`, `trigger_front_extension: 0.0` and
`trigger_layer: obstacle_layer`. The trigger adds a 5 cm Euclidean margin
around the entire padded controller footprint, rotated with each reference
pose. Unknown sensor cells do not trigger it.
Cell area is conservatively covered
by a half-cell diagonal. This margin only selects detour scoring; it does not
change collision rejection or the traversable guidance grid, so a valid
trajectory can leave the trigger region.
The real planner's border-follow `safety_margin: 0.04` places its first lane
at 0.15 m from obstacles, using the declared 0.22 m footprint width.
Both planner profiles use `second_lane_enabled: false`, retaining only the
first preferred band and increasing cost beyond its outer edge. The second
lane settings apply only when explicitly re-enabled. Controller footprint
padding is configured separately.
In Dijkstra mode, the path critic builds an eight-connected
cost-to-go field through the current controller costmap. Occupied and unknown
cells are excluded and diagonal corner cutting is forbidden. Lethal cells are
expanded by the inscribed footprint radius plus `clearance_margin` (0.0 m in
both profiles) and a half-cell diagonal. This center reference leaves the full
oriented-footprint collision check responsible for the rectangular robot's
actual clearance. Inflated costs add a travel penalty controlled by
`cost_weight` (2.0).

In Dijkstra mode, MPC minimizes distance through free space and terminal motion heading toward
the field route (`heading_weight: 2.0`). The
heading follows a short route section to smooth individual grid-cell angles;
reverse motion uses its travel direction instead of the opposite chassis
orientation. Direct path alignment and goal attraction are suspended
during the detour. Every predicted center must remain in the clearance grid;
the existing footprint collision and Ackermann constraints still check every
prediction. Ordinary path/goal scoring resumes when the reference becomes clear
and the robot is within 0.15 m of it, avoiding abrupt scoring-mode changes while
the robot is still returning from the detour. A critic reset clears this history.
The global plan and FollowPath action are not replaced or restarted.

Both profiles use a 5 x 5 m rolling costmap and a 2.50 m DWB prune distance to
include the sides and exit of local obstacles. They omit DWB's `Oscillation`
critic: its direction locks can prevent the steering reversal needed to exit
a detour. MPC actuation constraints and command regularization remain active.
Real control uses a 0.312 x 0.210 m collision footprint with zero padding,
inset 5 mm per side from the planner's physical 0.322 x 0.220 m footprint.
This deliberately reduces the collision envelope below the physical dimensions.
Both profiles use a 4 s horizon: real control uses 128 sequences at 10 Hz
and 40 steps of 0.10 s; simulation uses 768 sequences at 20 Hz and 80 steps
of 0.05 s. The real local costmap
updates at 10 Hz and publishes at 2 Hz, with mux commands at 20 Hz. Simulation
uses 15 Hz, 4 Hz and 50 Hz respectively.
The real obstacle weight is 12.0, with semantic boardwalk kept traversable by
local lethal threshold 100. Real repulsion stays active up to the goal;
simulation disables ordinary repulsion below 0.15 m. Both retain inflation
and critical/collision checks. Simulation's reverse/direction penalties are not
applied to the real profile.
The larger local map and guidance field add computation; verify the full real
cycle fits within 100 ms on the Nano. The reduced real candidate budget
leaves more computation time for perception and costmap updates. Runtime
must still be checked with the application active and representative obstacles.

This field is a local scoring aid, not a kinematic global planner. It requires
a reachable free rejoin point inside the local map and enough room/time for
the sampled bicycle trajectories. A blocked passage, a blocked goal, an obstacle
detected too late, or no feasible sample can still require stopping and global
replanning. Sensor observations must actually reach the controller costmap;
unobserved space is treated according to that costmap's configured policy.

Regression tests cover a previously clear straight path with a cube inserted
after motion starts, model execution around it, return toward the original
path, and independent rectangle collision checks. This is a software model
test, not validation of Gazebo sensor timing or physical actuator tracking.
Both profile YAMLs are exercised for valid planned clearances, scoring-mode
continuity, reverse-motion heading and preference for a free-space detour.
Additional simulation regressions cover direction penalties through a stop and
progress on a frozen costmap/path captured from Gazebo. The frozen replay does not reproduce
live costmap changes or prove that every Gazebo interruption is resolved.

## Build and test

Inside the Foxy workspace:

Use GCC 9 or newer for this C++17 plugin. On the Jetson, GCC 8's
`std::filesystem::path` layout is incompatible with the newer runtime library;
the controller can segfault while pluginlib loads its trajectory generator.
Select the compiler explicitly and clear the package's CMake cache when
rebuilding an existing GCC 8 installation:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select limo_interfaces limo_dwb_critics limo_controller \
  --cmake-clean-cache --cmake-args \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 -DCMAKE_C_COMPILER=/usr/bin/gcc-9
source install/setup.bash
colcon test --packages-select limo_dwb_critics --event-handlers console_direct+
colcon test-result --verbose
```

The `limo_controller` launch selects `config/control_real.yaml` or
`config/control_sim.yaml` via `robot_model`; both select this plugin. For a baseline comparison, set `FollowPath.plugin` back to
`dwb_core::DWBLocalPlanner`; the original generator and its parameters remain.

Before physical operation, validate closed-loop simulation on a straight path,
tight curve, slalom, obstruction and goal approach. Measure lateral error,
clearance, steering increments and controller execution time. The whole
control cycle must fit within 100 ms at 10 Hz on the real robot and within
50 ms at 20 Hz in simulation. Hardware uses 128 sequences and simulation
uses 768. Both retain a 4 s horizon, with 40 steps of 0.10 s on hardware
and 80 steps of 0.05 s in simulation. A model-only benchmark does not
include costmap or footprint scoring. Enable `publish_evaluation` temporarily
for DWB candidate diagnostics (`MpcEffort` is the additional cost).
