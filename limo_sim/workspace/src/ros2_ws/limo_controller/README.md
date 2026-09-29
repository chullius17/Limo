# LIMO control profiles

`control.launch.py` selects `config/control_real.yaml` or
`config/control_sim.yaml` using `robot_model`, independently of the clock.

| Setting | Real | Simulation |
| --- | --- | --- |
| MPC wheelbase | 0.20 m | 0.24 m |
| Rear axle to base | 0.10 m | 0.12 m |
| Minimum turning radius | 0.462 m | 0.55 m |
| Default clock | Wall clock | Simulation clock |
| Default local GUI | Off | On |

These values preserve the previously effective YAML and launch overrides.
Simulation geometry is now in its YAML rather than applied over a parameter file
by the launch. Other controller and costmap tuning is preserved.

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
Both profiles contain the controller, local costmap and `twist_mux` parameters.
The mux uses the same selected YAML and clock override as the controller:
autonomy publishes to `/cmd_vel_autonomy` (priority 10), teleop to
`/cmd_vel_teleop` (priority 100); the output is `/cmd_vel` at 50 Hz.
Both inputs time out after 0.5 s. A custom YAML can also override these mux
settings through its `twist_mux.ros__parameters` section.

These real/sim profiles are the only built-in controller parameter files.
