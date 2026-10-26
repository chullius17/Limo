"""Check original observations survive decoding and planar alignment."""

import math
import struct
from types import SimpleNamespace
import unittest

from limo_controller.mpc_preview_sources import laser_points, semantic_points, transform_points


class PreviewSourcesTest(unittest.TestCase):
    """Cover finite laser returns and padded, labeled semantic pointclouds."""

    def test_laser_preserves_valid_returns_and_only_applies_explicit_range_limit(self):
        scan = SimpleNamespace(
            ranges=(0.1, 0.2, 1.0, math.inf, math.nan, 3.0, 8.0, 8.1),
            range_min=0.2, range_max=8.0, angle_min=0.0, angle_increment=math.pi / 4)
        points = laser_points(scan)
        self.assertEqual(len(points), 4)
        self.assertAlmostEqual(points[0][0], 0.2 / math.sqrt(2))
        self.assertAlmostEqual(points[0][1], 0.2 / math.sqrt(2))
        self.assertAlmostEqual(points[1][1], 1.0)
        self.assertEqual(len(laser_points(scan, 2.5)), 2)

    def test_semantic_decoder_preserves_all_classes_endianness_and_row_padding(self):
        fields = [SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)
                  for name, offset, datatype in (
                      ('class_id', 0, 2), ('x', 4, 7), ('y', 8, 7), ('z', 12, 7))]
        for big_endian in (False, True):
            data = bytearray(128)
            for index in range(6):
                offset = index // 3 * 64 + index % 3 * 20
                data[offset] = index + 1
                struct.pack_into('>fff' if big_endian else '<fff', data, offset + 4,
                                 float(index), -float(index), 0.0)
            cloud = SimpleNamespace(
                width=3, height=2, point_step=20, row_step=64,
                is_bigendian=big_endian, data=data, fields=fields)
            self.assertEqual(semantic_points(cloud),
                             tuple((float(i), -float(i), i + 1) for i in range(6)))
            cloud.data = data[:-1]
            with self.assertRaises(ValueError):
                semantic_points(cloud)

    def test_planar_alignment_preserves_labels_and_compensates_robot_pose(self):
        points = transform_points(((1.0, 0.5, 2),), (2.0, -1.0, math.pi / 2),
                                  (1.0, 2.0, math.pi / 2))
        self.assertEqual(points[0][2], 2)
        self.assertAlmostEqual(points[0][0], -2.0)
        self.assertAlmostEqual(points[0][1], -0.5)


if __name__ == '__main__':
    unittest.main()
