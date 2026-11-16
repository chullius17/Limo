# Copyright 2026 Giulio Cataldo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Decode pre-inflation observations without rasterizing or sampling points."""

import math
import struct


def laser_points(scan, maximum_range=0.0):
    """Project every valid return; zero maximum uses the sensor's complete range."""
    limit = min(scan.range_max, maximum_range) if maximum_range > 0 else scan.range_max
    points = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= limit:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        points.append((distance * math.cos(angle), distance * math.sin(angle), 0))
    return tuple(points)


def semantic_points(cloud):
    """Read labeled XYZ points, preserving row padding, endianness and all classes."""
    if not cloud.width or not cloud.height:
        return ()
    if (cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) < cloud.height * cloud.row_step):
        raise ValueError('Invalid local pointcloud layout')
    fields = {field.name: field for field in cloud.fields}
    for name, datatype, size in (('x', 7, 4), ('y', 7, 4), ('z', 7, 4), ('class_id', 2, 1)):
        field = fields.get(name)
        if (field is None or field.datatype != datatype or field.count != 1
                or field.offset < 0 or field.offset + size > cloud.point_step):
            raise ValueError('Invalid local pointcloud field: ' + name)
    unpack = struct.Struct('>f' if cloud.is_bigendian else '<f').unpack_from
    data = memoryview(cloud.data).cast('B')
    offsets = [fields[name].offset for name in ('x', 'y', 'z', 'class_id')]
    points = []
    for row in range(cloud.height):
        for column in range(cloud.width):
            base = row * cloud.row_step + column * cloud.point_step
            x, y, z = (unpack(data, base + offset)[0] for offset in offsets[:3])
            label = data[base + offsets[3]]
            if label in (1, 2, 3, 4, 5, 6) and all(math.isfinite(v) for v in (x, y, z)):
                points.append((x, y, label))
    return tuple(points)


def transform_points(points, transform, robot_pose):
    """Compose source-to-view and view-to-robot planar transforms once per layer."""
    tx, ty, yaw = transform
    rx, ry, robot_yaw = robot_pose
    cosine, sine = math.cos(robot_yaw), math.sin(robot_yaw)
    dx, dy = tx - rx, ty - ry
    ox, oy = cosine * dx + sine * dy, -sine * dx + cosine * dy
    cosine, sine = math.cos(yaw - robot_yaw), math.sin(yaw - robot_yaw)
    return tuple((ox + cosine * x - sine * y, oy + sine * x + cosine * y, label)
                 for x, y, label in points)
