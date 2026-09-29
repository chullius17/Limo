# LIMO control profiles

`control.launch.py` selects `config/control_real.yaml` or
`config/control_sim.yaml` using `robot_model`, independently of the clock.

| Setting | Real | Simulation |
| --- | --- | --- |
| MPC wheelbase | 0.20 m | 0.24 m |
| Rear axle to base | 0.10 m | 0.12 m |
| Minimum turning radius | 0.462 m | 0.55 m |
| Controller frequency | 10 Hz | 20 Hz |
| MPC sequences / steps | 96 / 25 | 768 / 50 |
| Model timestep / horizon | 0.10 s / 2.5 s | 0.05 s / 2.5 s |
| Require chassis command mode | Yes | No |
| Default clock | Wall clock | Simulation clock |
| Default local GUI | Off | On |

Simulation geometry is now in its YAML rather than applied over a parameter file
by the launch. Real control uses a smaller Nano CPU budget, retaining geometry,
velocity/acceleration limits, the prediction horizon and footprint collision checks.
The real local costmap updates at 10 Hz and publishes at 2 Hz.

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
