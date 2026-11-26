# limo_cloud

Efficient C++ projection of already classified RGBD images into a semantic point
cloud. Every class follows the same projection procedure and retains its input ID.

## Inputs and output

| Direction | Default topic / source | Content |
| --- | --- | --- |
| Input | `limo/cv_package/detection/lane_labels/raw` | `sensor_msgs/Image`, `mono8` or `8UC1`: one class ID per pixel. |
| Input | `limo/cv_package/depth_correction/depth_corrected/raw` | `sensor_msgs/Image`: `32FC1` depth in metres, or `16UC1` / `mono16` in millimetres. |
| Input | `/rgb/camera_info` | `sensor_msgs/CameraInfo`: source RGB intrinsics and image dimensions. |
| Input | TF | Camera frame to the configured output frame, at the classified-image timestamp. |
| Output | `limo/cv_package/visual_ptcld/points` | `sensor_msgs/PointCloud2` with `x`, `y`, `z` (`float32`) and `class_id` (`uint8`). |

Depth must already be aligned with the classified image, with matching dimensions.
By default, both images and CameraInfo must share the same nonempty frame ID.
Classification and depth preparation happen upstream; a separate RGB image is
not required. The historical topic names are configurable.

## Procedure

1. Read a classified image and the latest available depth. Skip invalid depth,
   values outside the default 0.1–2.5 m range, and class `0`.
2. Back-project valid pixels using cached camera rays, then apply TF.
3. Pack and publish an unorganized cloud stamped with the classified-image time.
   The default output is in `base_link`, flattened to BEV (`z = 0`);
   `flatten_to_bev: false` retains full transformed XYZ.

The default pipeline performs no voxelization, relabeling or distance processing.
Optional C++ and Python plugins can process images before projection and the cloud
afterwards; see [plugins_example.yaml](config/plugins_example.yaml).

## Run

```bash
ros2 launch limo_cloud cloud.launch.py
```

See [config/cloud.yaml](config/cloud.yaml) for documented parameters, including
topic selection, upstream crop geometry and an optional timestamp-difference limit.
