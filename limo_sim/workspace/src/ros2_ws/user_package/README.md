# LIMO application launch

The application includes online localization/mapping, trajectory planning and
control. Start sensors first, then launch the application:

```bash
ros2 launch user_package limo_app_real.launch.py
```

CV starts through online mapping when `launch.start_cv` is true in the selected
`online_map_package/config/mapping_real.yaml` or `mapping_sim.yaml`. Both shipped
profiles enable it. `start_cv` defaults to empty, so the YAML setting is preserved.
To override the config for one run, use `start_cv:=true` or `start_cv:=false`.
If CV is already running separately, disable its application startup:

```bash
ros2 launch cv_package cv_real.launch.py
ros2 launch user_package limo_app_real.launch.py start_cv:=false
```

Explicit `start_cv` overrides are forwarded to online mapping, whose
`mapping_real.yaml` selects
`cv_real.yaml` and the waterfall detector. `limo_app_sim.launch.py` selects
`cv_sim.yaml` and the HSV detector. `limo_app.launch.py` also uses the simulation
profile, with a wall clock; changing the clock does not change the detector.

An optional `cv_config:=/absolute/path/to/profile.yaml` overrides the selected
CV profile when starting CV. All consumers continue to receive the semantic
cloud on `/limo/cv_package/visual_ptcld/points`; the CV launch connects waterfall
labels to the point-cloud node. Detector runtime parameters are under `/lane_node`.

## Robot and desktop

On the physical LIMO, start the sensors and then the application:

```bash
ros2 launch custom_start limo_real.launch.py
ros2 launch user_package limo_app_real.launch.py
```

The real application runs online mapping/localization, the trajectory planner,
the RViz goal bridge, the Nav2 controller, the path executor and the velocity
mux on the robot. CV processing starts only when `launch.start_cv` in the
mapping YAML is true (or `start_cv:=true` is explicitly supplied).
The real application does not open RViz or the control GUI.
`start_control_gui:=true` is rejected for the real profile; use the desktop launch.
The simulation application retains its existing local RViz/control GUI behavior.

In the PC's ROS environment, start all graphical clients with:

```bash
ros2 launch user_package desktop_app.launch.py
```

This opens mapping/planning/local-costmap RViz and the control GUI. CV RViz also
opens when `launch.start_cv` in the mapping YAML is true. Each RViz client runs
in its own launch scope. The desktop launch starts no mapping, CV processing,
planner, controller, velocity mux, path executor or second goal bridge.
RViz publishes goals to the robot; the GUI uses the robot's existing control
services to start, pause, resume or abort a planned path.

Optional desktop switches:

```bash
ros2 launch user_package desktop_app.launch.py start_cv_rviz:=false
ros2 launch user_package desktop_app.launch.py start_mapping_rviz:=false
ros2 launch user_package desktop_app.launch.py start_control_gui:=false
```

`start_cv_rviz` only selects the desktop viewer; it does not change CV processing
on the robot. For a separately launched CV backend, use `start_cv:=false` on the
robot and `start_cv_rviz:=true` on the PC if its viewer is wanted.

Both launches accept `mapping_config:=/absolute/path/to/mapping.yaml`.
Use matching profiles on each machine; paths are local to that machine.
The desktop resolves `cv_config` from the mapping profile, with the same optional
`cv_config:=/absolute/path/to/cv.yaml` override as the robot. Relative CV profile
names are resolved in the installed `cv_package/config` directory.
All real nodes and desktop clients use the wall clock.

Build/install `user_package` in each ROS workspace before using the new launch.
PC and robot must share the same `ROS_DOMAIN_ID` and be discoverable over DDS;
otherwise RViz and the GUI cannot reach the robot.

## Planner and controller configuration

The app launch also passes its `robot_model` to both planning and control:

| Application | Trajectory YAML | Controller YAML |
| --- | --- | --- |
| `limo_app_real.launch.py` | `traj_real.yaml` | `control_real.yaml` |
| `limo_app_sim.launch.py` | `traj_sim.yaml` | `control_sim.yaml` |
| `limo_app.launch.py` | `traj_sim.yaml` | `control_sim.yaml` |

The legacy app keeps its wall clock while selecting simulation tuning.
Custom files can be supplied directly to the app:

```bash
ros2 launch user_package limo_app_real.launch.py \
  planner_params_file:=/absolute/path/to/planner.yaml \
  controller_params_file:=/absolute/path/to/control.yaml
```
