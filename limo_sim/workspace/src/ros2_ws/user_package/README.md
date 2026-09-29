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
