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

- `real_start`: the measured robot pose, used to identify an adjusted start;
- `start`: the selected planning start;
- `goal`: the selected target pose.

The service callback validates frames, coordinates and endpoint occupancy.
It computes the start quantized to SMAC's grid and heading bins and checks
its footprint when the start has been adjusted. It then calls
`planner_->createPlan(request->start, request->goal)`.

For an adjusted start, it restores the quantized start as the first path pose,
since Foxy's SMAC backtrace omits the initial node. It returns the path,
quantized planning start, or a failure reason. The path begins at the selected
planning start and all planning uses the normal obstacle checks.

Candidate generation and result publication belong to `rviz_goal_bridge` in
[traj_package](../traj_package/README.md).

The initial connector has been removed. The legacy connector fields in the
service response remain zero/false for interface compatibility. Controller
START validation still applies when the selected pose differs from the robot.
