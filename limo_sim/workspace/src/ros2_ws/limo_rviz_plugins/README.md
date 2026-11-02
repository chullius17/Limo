# RViz footprint rendering

`limo_rviz_plugins/Polygon` keeps the standard Polygon display's topics,
properties and appearance, but retains only the newest message released by TF
and draws it during the next GUI update. Each display retains at most one
pending footprint; it never queues a history of Qt rendering callbacks.
Outlines render 3 cm above the map plane so costmap tiles do not hide them.
The online real and simulation
profiles use it for **Trajectory Footprint** and **Control Footprint**.

In Foxy 8.2.8, `MessageFilterDisplay` can call `PolygonDisplay::processMessage`
from the TF listener thread when a transform arrives after its polygon. That
thread does not own the rendering context. With NVIDIA 595.99.02 and OGRE
1.12.1 this reproduces a SIGSEGV in `libnvidia-glcore` or GL dispatch while
rendering; disabling GLVND patching alone does not prevent it. This plugin
changes only footprint rendering and does not replace system RViz libraries.
On Linux its library stays mapped until process exit: Foxy's TF buffer can
retain callback destructors after RViz unloads its displays, so unloading the
library earlier can also cause a SIGSEGV during shutdown.

Build from the workspace root, then source the result in the launch terminal:

```bash
colcon build --symlink-install --packages-up-to limo_rviz
source install/setup.bash
ros2 launch user_package desktop_app.launch.py
```

For an isolated regression test, source ROS and the workspace in two terminals.
Use a domain separate from the robot:

```bash
ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix limo_rviz)/share/limo_rviz/config/online_map_real.rviz"
```

```bash
python3 src/ros2_ws/limo_rviz_plugins/test/publish_delayed_footprints.py \
  --domain 175 --duration 120
```

The publisher sends two footprints, three changing maps and TF delayed by
0.4 seconds. RViz must remain responsive and render both footprints for the
whole run. For comparison, copy the RViz profile to `/tmp` and replace
`limo_rviz_plugins/Polygon` with `rviz_default_plugins/Polygon`: this reproduced
the crash within seconds on the affected machine. Neither command publishes
motion commands. Keep the test and robot on separate domains.
