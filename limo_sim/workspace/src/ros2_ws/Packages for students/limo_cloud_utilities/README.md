# limo_cloud_utilities

A standalone classified-RGBD consumer that combines the `limo_cloud` C++ projector
with Python plugins reproducing the semantic processing of `visual_ptcld`.
The processing algorithms are copied locally into [processing](limo_cloud_utilities/processing).

## Inputs and output

Inputs follow the [limo_cloud contract](../limo_cloud/README.md): aligned depth
(`32FC1` metres or `16UC1` / `mono16` millimetres), a `mono8` / `8UC1` classified
image, RGB CameraInfo and camera-to-output TF. Input labels are `0` invalid,
`1` road, `2` yellow lines and `3` background.

The output is `sensor_msgs/PointCloud2` on
`limo/cv_package/visual_ptcld/points`, with `x`, `y`, `z` and `class_id`, in
`base_link` with `z = 0` by default. Output classes are:

| ID | Meaning |
| --- | --- |
| 1 | Road boundary |
| 2 | Yellow lines |
| 3 | Remaining background |
| 4 | Boardwalk |
| 5 | Interior road |
| 6 | Interior boardwalk |

Additional input IDs pass through by default. Use IDs above `6` to avoid overlap
with the generated classes.

## Procedure

1. **BoundaryMasks:** dilate the background mask and intersect it with road to
   separate boundary road from interior road.
2. **PixelSampling:** select pixel-cell centroids for road and background; retain
   every yellow-line pixel. The C++ core projects the selected pixels into BEV.
3. **Boardwalk:** perform two metric nearest-neighbor passes with `cKDTree` in BEV.
   The first measures background distance from road boundaries and identifies
   boardwalk seeds and interior boardwalk. The second extends boardwalk labels
   near those initial seeds.
4. **RoadBoardwalkOnly / MetricVoxel:** optionally retain only road and boardwalk,
   then reduce points to per-class XY voxel centroids (default voxel size: 2 cm).

Optional debug outputs expose images and the cloud before boardwalk classification.
`cloud.launch.py` starts one consumer node; classification and depth preparation must
already be running upstream.

## Run

```bash
ros2 launch limo_cloud_utilities cloud.launch.py
```

See [config/cloud.yaml](config/cloud.yaml) for parameters and plugin settings.
Optional `profile:=real` and `profile:=sim` load local compatibility settings;
both expect images already cropped to the lower half upstream and disable extra
input classes. The real profile also keeps only road and boardwalk; the sim profile
enables simulation time and debug outputs.

## Full CV integration test

```bash
ros2 launch limo_cloud_utilities real.launch.py
ros2 launch limo_cloud_utilities sim.launch.py
ros2 launch limo_cloud_utilities desktop.launch.py
```

Choose `real` for the robot backend, `sim` for the simulated backend plus RViz,
or `desktop` for the robot's RViz client only. The simulator or camera drivers,
CameraInfo and robot TF must already be available, as for the original CV launch.

These launches reuse `cv_package`'s detector, depth correction, real/sim YAML
profiles and viewer. The backend replaces `visual_ptcld` with
`limo_cloud_utilities_node`, translating semantic settings to plugin parameters;
the original cloud node is disabled. Run this backend in place of the original
CV backend. The generic `limo_cloud` package remains independent of `cv_package`.
Use `config_file:=/path/to/cv_profile.yaml` to test a custom CV profile, or
`profile:=sim` with `desktop.launch.py` for a simulation viewer.
