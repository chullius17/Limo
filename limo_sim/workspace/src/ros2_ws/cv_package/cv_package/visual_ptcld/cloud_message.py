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


def make_pointcloud2(points, class_ids, header, frame_id):
    """Serialize 2D points and semantic labels using the public cloud layout."""
    cloud_points = np.empty(len(points), dtype=CLOUD_DTYPE)
    cloud_points['x'] = points[:, 0]
    cloud_points['y'] = points[:, 1]
    cloud_points['z'] = 0.0
    cloud_points['class_id'] = class_ids

    cloud = PointCloud2()
    cloud.header.stamp = header.stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(cloud_points)
    cloud.fields = CLOUD_FIELDS
    cloud.is_bigendian = False
    cloud.point_step = CLOUD_DTYPE.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = array('B', cloud_points.tobytes())
    cloud.is_dense = True
    return cloud
