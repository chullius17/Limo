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

Simulation optionally exports `limo_interfaces/MpcDebug` snapshots for the
compressed-image preview in `limo_controller`. The sampler assigns an explicit
family and candidate ID at generation time. A bounded observer records
representative rollouts and the winner without changing the search objective,
candidate order or random perturbations. Costmap bytes, model geometry and
initial actuator state are captured in the same controller cycle. Disabled
telemetry, or a publisher without subscribers, constructs no snapshot. See the
[preview settings](../limo_controller/README.md#simulation-mpc-image-preview).

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
`MPC.steering_rate_change_weight: 0.1` penalizes its difference from the previous
issued steering increment, in the same normalized units. These immediate
costs are **not divided by the horizon length**, so increasing the prediction
horizon does not dilute command continuity. Steering-rate history resets with
the warm start. Both are soft costs: collision rejection can still force a
change or braking, and no command is filtered after trajectory evaluation.
The simulation YAML uses local steering sampling deviation 0.10 rad; the real
profile uses 0.20 rad. Simulation uses 768 candidates with 8 velocity samples
and 15 curvature samples. The real profile uses 128 candidates: 6 x 11
constant-curvature seeds, braking, warm start and 60 sampled sequences.
Both retain full-range seeds and broad exploration. The real profile doubles
the reference obstacle weight to favor
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
| CostCritic, weight 3 | Real `MppiObstacle.scale: 6`; simulation: 3 | Mean center cost / 254; critical cost 300; ordinary repulsion off below 0.15 m from the goal in both profiles; critical and collision checks remain active |
| GoalCritic, weight 5 | `MppiPath.GoalCritic` | Mean distance in meters, enabled within 1 m of the goal |
| GoalAngleCritic, weight 3 | `MppiPath.GoalAngleCritic` | Mean wrapped yaw error in radians, within 0.5 m |
| PathAlignCritic, weight 10 | `MppiPath.PathAlignCritic` | Mean path-pose error, with orientations; off within 0.5 m or if local path occupancy exceeds 15% |
| PathFollowCritic, weight 5 | `MppiPath.PathFollowCritic` | Terminal distance to a metric lookahead target; off within 1 m of the goal |
| PathAngleCritic, weight 2 | `MppiPath.PathAngleCritic` | Terminal heading toward the lookahead target, enabled for initial errors above 1 rad and outside 0.5 m; forward preference in simulation, forward/reverse symmetric on the real profile |
| ConstraintCritic / collision_cost | Model constraints and exceptions | Infeasible/colliding candidates are rejected, not assigned a finite penalty |

This is an adaptation, not a port of MPPI critics. DWB has no batch-wide
furthest-trajectory index: the MPPI point offsets are replaced by
`lookahead_distance: 1.25` m. Alignment uses monotone, distance-limited nearest
path poses. Goal-distance gates use the actual goal rather than a clipped local
path endpoint. Every predicted pose is collision checked; there is no
`trajectory_point_step` skipping. Both footprint contour and center are
checked, while footprint interiors are not exhaustively rasterized.

Inflation now matches Humble (`0.30` m, scaling factor `8.0`), retaining this
branch's physical footprint and semantic lethal threshold 86. The finite set
of predictions uses discrete collision checks, not continuous swept-volume
checking or a terminal invariant-set constraint. Equivalent weights do not
guarantee identical behavior; simulation and hardware tuning remain necessary.

## Online obstacle detours

Both real and simulation profiles enable `MppiPath.ObstacleGuidance.enabled`.
A detour starts when the oriented robot footprint collides along the local reference,
including between path samples. The real profile also uses `trigger_margin: 0.05`
to anticipate a detour when an occupied or unknown cell comes within 5 cm of
the padded, oriented reference footprint. Cell area is conservatively covered
by a half-cell diagonal. This margin only selects detour scoring; it does not
change collision rejection or the traversable guidance grid, so a valid
trajectory can leave the proximity band. Simulation keeps the default
`trigger_margin: 0.0` and preserves collision-free planned passages.
The real planner's border-follow `safety_margin: 0.04` places its first lane
at 0.15 m from obstacles, using the declared 0.22 m footprint width.
`second_lane_distance: 0.29` sets the second lane independently, giving
0.14 m between lane centers. If omitted, the second lane defaults to 1.5
times `robot_width`, preserving the simulation profile. Controller footprint
padding is configured separately.
The path critic builds an eight-connected Dijkstra
cost-to-go field through the current controller costmap. Occupied and unknown
cells are excluded and diagonal corner cutting is forbidden. Lethal cells are
expanded by the inscribed footprint radius plus `clearance_margin` (0.0 m in
both profiles) and a half-cell diagonal. This center reference leaves the full
oriented-footprint collision check responsible for the rectangular robot's
actual clearance. Inflated costs add a travel penalty controlled by
`cost_weight` (2.0).

If the normal lookahead point is blocked, the target advances to a free point
on the reference path beyond it, then advances up to `rejoin_distance: 0.50` m
to leave room to straighten. MPC minimizes distance through free space and
terminal motion heading toward the field route (`heading_weight: 2.0`). The
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
The real obstacle weight remains 6.0. Both profiles disable ordinary obstacle
repulsion below 0.15 m from the goal, retaining inflation in the costmap and
critical/collision checks. Simulation's reverse/direction penalties are not
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
