# limo_smac_planner

Adds **explicit-start planning** to Nav2 Foxy's SMAC, allowing the planning
start to differ from the robot's current pose. The plugin runs inside
`planner_server`; SMAC performs the path search.

## `explicit_start_planner.cpp`

[This file](src/explicit_start_planner.cpp) implements the
`limo_smac_planner/ExplicitStartPlanner` plugin. It loads
`smac_planner/SmacPlanner` and manages its configuration, activation and
deactivation.

It exposes `/compute_path_with_start`, whose
[request](../limo_interfaces/srv/ComputePathWithStart.srv) contains:

- `real_start`: the actual robot pose, used for the optional connector;
- `start`: the selected planning start;
- `goal`: the selected target pose.

The service callback validates frames, coordinates and endpoint occupancy.
It computes the start quantized to SMAC's grid and heading bins and checks
its footprint when the start has been adjusted. It then calls
`planner_->createPlan(request->start, request->goal)`.

For an adjusted start, it restores the quantized start as the first path pose,
since Foxy's SMAC backtrace omits the initial node. It returns the path,
quantized planning start and diagnostics, or a failure reason. When requested,
it also prepends the connector described below.

Candidate generation and result publication belong to `rviz_goal_bridge` in
[traj_package](../traj_package/README.md).

## `dubins_connector.cpp`

[This file](src/dubins_connector.cpp) implements `dubinsConnector()`, the
optional **actual robot pose → virtual start** prefix for the SMAC path.
It is called when the selected start differs from the actual pose and
`connect_adjusted_start: true`.

It uses OMPL to construct a Dubins curve from circular arcs and straight
segments, with forward motion only and the configured `minimum_turning_radius`.
It rejects non-finite or excessive lengths, samples the curve and returns a
`nav_msgs/Path` with exact endpoints.

Defaults, also used by the real and simulation profiles:

| Parameter under `GridBased` | Value |
| --- | --- |
| `connect_adjusted_start` | `false` |
| `start_connector_sample_step` | `0.02` m |
| `start_connector_max_length` | `2.0` m |

The connector performs no costmap or collision checks. The wrapper validates
the virtual endpoint, and SMAC retains its collision checks for the suffix.
The connector is currently disabled: paths start at the selected planning pose,
without a prefix from the actual robot pose.
