# limo_map

C++ semantic mapping from classified point clouds, with configurable costs for
any input class set. The core and the Qt5 save-map GUI use C++; Python is used
only for ROS launch files and integration tests.

## Inputs and output

| Direction | Default topic / source | Content |
| --- | --- | --- |
| Input | `/limo/cv_package/visual_ptcld/points` | `sensor_msgs/PointCloud2`: `x/y/z` FLOAT32 and `class_id` UINT8. Compatible with `limo_cloud` and `limo_cloud_example`. |
| Input | `/map` | Complete SLAM laser `nav_msgs/OccupancyGrid`, including free and unknown cells. Each message replaces the previous laser layer. |
| Input | TF | Planar, upright cloud-frame → `map` transform at the sensor timestamp. |
| Output | `/limo/map_package/offline/map/semantic_grid` | Semantic costs before laser fusion. |
| Output | `/limo/map_package/offline/map/combined_grid` | Laser obstacles at 100, then observed semantic costs, then remaining laser values. |
| Output | `/limo/map_package/offline/map/class_<ID>_map` | One cost layer per configured input ID: its group cost, 0 in other known groups, -1 for unknown. |

All grids are reliable/transient-local. Geometry follows the latest valid laser
map, including its rotated origin. CV accumulates before that map arrives;
publication and saving wait for it. Points beyond its current extent remain
stored. Input clouds require positive, ordered sensor timestamps and a frame ID.

## Procedure

1. Read finite classified endpoints, ignore unconfigured IDs and apply TF at the
   cloud timestamp. Missing TF is retried for a bounded steady-time interval.
2. Accumulate sparse 32×32 evidence tiles. Each cell receives one normalized
   update per cloud; the unique strongest group above threshold selects its exact
   cost. Ties stay unknown. Evidence is revisable and independent from cost.
3. Render changed maps at the configured rate and fuse the complete laser layer.
   No ray clearing, field-of-view clearing or time decay is applied.

`class_ids` and `class_costs` are parallel arrays: unique IDs in [0,255], costs
in [0,100]. Cost 0 means observed free space; unknown is -1. Optional
`evidence_groups` joins aliases with equal costs; `[]` keeps every class
independent. The supplied profiles reproduce the offline pipeline:

```yaml
class_ids:       [1, 2, 3, 4, 5, 6]
class_costs:     [0, 60, 30, 90, 0, 90]
evidence_groups: [1, 2, 3, 4, 1, 4]
```

The default TF mode corrects classes on revisits; historic CV geometry does not
move retroactively after loop closure. Optional Cartographer mode stores local
submap evidence and follows updated poses. Its build requirements and parameters
are documented in [config/map.yaml](config/map.yaml).

## Run

Build from the ROS workspace; colcon discovers these packages recursively under
`src/ros2_ws/limo extensions`:

```bash
colcon build --symlink-install --packages-select limo_map
source install/setup.bash
ros2 launch limo_map map.launch.py use_sim_time:=true start_gui:=true
```

SLAM, classified cloud and TF must already be running. `start_gui` defaults to
false; `use_sim_time` defaults to false. Use [config/map_real.yaml](config/map_real.yaml)
for the real-robot evidence tuning and save-time median filter:

```bash
ros2 launch limo_map map.launch.py \
  config_file:="$(ros2 pkg prefix limo_map)/share/limo_map/config/map_real.yaml" \
  start_gui:=true
```

The C++ Qt GUI also runs independently on a connected desktop:

```bash
ros2 launch limo_map map_save_gui.launch.py
# Or: ros2 run limo_map map_save_gui
```

It provides SAVE MAP, asynchronous success/error feedback and a live status log.
[config/gui.yaml](config/gui.yaml) documents its service/topic parameters; pass it
with `config_file` to the GUI launch, or `gui_config_file` to the combined launch.
Optional `save_service` / `status_topic` launch arguments override those endpoints.

## Saving and reset

Default `std_srvs/Trigger` endpoints:

```bash
ros2 service call /limo/map_package/offline/map_saver/save_map std_srvs/srv/Trigger '{}'
ros2 service call /limo/map_package/offline/reset_map std_srvs/srv/Trigger '{}'
```

Saving writes `<save_map_name>_complete`, `_laser` and `_cv_obstacle` PGM/YAML
pairs on the mapper machine. Complete/laser files preserve exact raw costs and
unknown cells. The binary CV export has configurable thresholds. The optional
median filters only the saved semantic snapshot. Reset clears CV evidence and
timestamp history while retaining the SLAM grid; reset before rewinding a bag.

See [config/map.yaml](config/map.yaml) for every mapper parameter, actual C++
defaults, memory limits, export rules and pose options. Active YAML entries select
the profile; commented entries show defaults that can be uncommented to customize.
The output prefix preserves the existing save GUI endpoints; generic class layers
replace the old yellow/white/boardwalk topic names. Disable the old mapper when
using these same outputs (`start_mapper:=false` in its launch).

Validation: `colcon test --packages-select limo_map` checks evidence, geometry,
cloud layouts, saving, DDS/TF integration and the C++ Qt GUI with an offscreen
platform. `colcon test-result --verbose` reports the results.
