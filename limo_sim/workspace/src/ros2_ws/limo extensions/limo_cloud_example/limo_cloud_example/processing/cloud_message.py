"""PointCloud2 layout and serialization for semantic BEV points."""

from array import array

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from .cloud_pipeline import CLOUD_DTYPE


CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(
        name='class_id', offset=12, datatype=PointField.UINT8, count=1),
]

DEBUG_CLOUD_DTYPE = np.dtype({
    'names': ('x', 'y', 'z', 'class_id', 'rgb'),
    'formats': ('<f4', '<f4', '<f4', 'u1', '<u4'),
    'offsets': (0, 4, 8, 12, 16),
    'itemsize': 20,
})
DEBUG_CLOUD_FIELDS = CLOUD_FIELDS + [
    PointField(name='rgb', offset=16, datatype=PointField.UINT32, count=1),
]
# Packed RGB equivalents of the pre-classification image's BGR palette:
# invalid, border road, yellow lines, background, unused, interior road.
# Preserve the copied legacy palette and allow additional uint8 input classes.
# Unspecified IDs are gray; this changes only their optional debug appearance.
DEBUG_COLORS = np.full(256, 0x808080, dtype=np.uint32)
DEBUG_COLORS[:6] = [0x000000, 0x0000FF, 0x00FFFF, 0xFFFFFF, 0x000000, 0x000080]


def make_pointcloud2(points, class_ids, header, frame_id, *, include_rgb=False):
    """Serialize BEV labels, optionally adding pre-classification image colors."""
    dtype = DEBUG_CLOUD_DTYPE if include_rgb else CLOUD_DTYPE
    cloud_points = np.empty(len(points), dtype=dtype)
    cloud_points['x'] = points[:, 0]
    cloud_points['y'] = points[:, 1]
    cloud_points['z'] = 0.0
    cloud_points['class_id'] = class_ids
    if include_rgb:
        cloud_points['rgb'] = DEBUG_COLORS[class_ids]

    cloud = PointCloud2()
    cloud.header.stamp = header.stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(cloud_points)
    cloud.fields = DEBUG_CLOUD_FIELDS if include_rgb else CLOUD_FIELDS
    cloud.is_bigendian = False
    cloud.point_step = dtype.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = array('B', cloud_points.tobytes())
    cloud.is_dense = True
    return cloud
