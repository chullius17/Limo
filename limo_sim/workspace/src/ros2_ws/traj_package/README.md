# Trajectory planning

`traj_package` contains the Nav2 SMAC planner and `rviz_goal_bridge`, which
receives RViz goals on `/goal_pose`, searches for a feasible solution, and
publishes the path. It can be launched without `limo_controller`:

```bash
ros2 launch traj_package trajectory.launch.py
```

It requires the robot map and transforms. Pass a custom configuration with
`planner_params_file:=/path/to/parameters.yaml`.

## Outputs

| Topic | Type | Contents |
| --- | --- | --- |
| `/limo/planning/path` | `nav_msgs/Path` | Most recent validated path |
| `/limo/planning/status` | `std_msgs/String` | Planning state and failures |

Both topics use reliable, transient-local QoS with depth 1. A consumer started
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

When a virtual start differs in position or orientation, the planner prepends
one forward-only OMPL Dubins connection from the measured real start to the
virtual start quantized on SMAC's grid and heading bins. Foxy omits the initial
node in its backtrace, so the wrapper restores this pose before the first SMAC
step. `/adjusted_start_pose` shows this actual connector endpoint.
Its radius is `GridBased.minimum_turning_radius`;
`GridBased.start_connector_sample_step` is 0.02 m and
`GridBased.start_connector_max_length` is 2.0 m. The length limit applies to
the initial connector, independently of SMAC's analytic expansion. Connections
exceeding it are rejected and the bridge tries another pair. SMAC still checks
collisions throughout the virtual-start-to-goal suffix. The prefix deliberately
ignores obstacles, including footprint collisions.

The bridge publishes the complete path and `/limo/planning/start_connector`
(`limo_interfaces/StartConnector`), which binds the prefix end index to the
complete path geometry and unique header timestamp. The executor preserves this
identity while setting individual pose timestamps to zero for latest TF.
A new goal clears both outputs; stale service results are ignored.
The controller uses the declared prefix as its temporary reference, without
obstacle repulsion, critical costs, collision rejection or obstacle detours.
After advancing along the prefix and reaching its end within 0.05 m and 0.10 rad,
normal tracking and obstacle checks resume. This initial exception does not
change the final-goal behavior, which retains footprint collision rejection.

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
its goal-search budget and validation footprint. Planner tuning is preserved:
both profiles initially retain the conservative 0.7 m turning radius, and their
footprints match their controller costmaps. The files can now be tuned separately.

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
