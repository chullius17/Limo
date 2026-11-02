#!/usr/bin/env python3
# Copyright 2026 Giulio Cataldo
# SPDX-License-Identifier: Apache-2.0
"""Exercise RViz footprint rendering with transforms arriving after messages."""

import argparse
from collections import deque
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, required=True,
                        help='Isolated ROS domain, different from the robot (0-232).')
    parser.add_argument('--duration', type=float, default=120.0)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or args.duration <= 0:
        parser.error('domain must be 0-232 and duration must be positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    import rclpy
    from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import TransformBroadcaster

    rclpy.init(args=[])
    node = rclpy.create_node('delayed_footprint_regression')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    maps = [node.create_publisher(OccupancyGrid, topic, qos) for topic in
            ['/map', '/global_costmap/costmap',
             '/limo/map_package/online/local_costmap']]
    footprints = [node.create_publisher(PolygonStamped, topic, 1) for topic in
                  ['/global_costmap/published_footprint',
                   '/local_costmap/published_footprint']]
    broadcaster = TransformBroadcaster(node)
    pending = deque()
    count = 0
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            transforms = []
            for child in ['odom', 'base_link', 'laser_frame']:
                transform = TransformStamped()
                transform.header.frame_id = 'map'
                transform.header.stamp = stamp
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            pending.append((time.monotonic(), transforms))
            while pending and time.monotonic() - pending[0][0] > 0.4:
                broadcaster.sendTransform(pending.popleft()[1])
            for index, publisher in enumerate(footprints):
                polygon = PolygonStamped()
                polygon.header.frame_id = 'odom'
                polygon.header.stamp = stamp
                polygon.polygon.points = [
                    Point32(x=x, y=y, z=0.0) for x, y in
                    [(0.3, 0.2), (0.3, -0.2), (-0.3, -0.2), (-0.3, 0.2)]]
                # Separate the outlines so that both are visible in the test.
                for point in polygon.polygon.points:
                    point.x += index * 0.8
                publisher.publish(polygon)
            for index, publisher in enumerate(maps):
                grid = OccupancyGrid()
                grid.header.frame_id = ['map', 'odom', 'base_link'][index]
                grid.header.stamp = stamp
                grid.info.resolution = 0.05
                grid.info.width = 100 + (count // 30 % 2) * 5
                grid.info.height = 100
                grid.info.origin.orientation.w = 1.0
                grid.data = [100 if (cell + count) % 37 == 0 else 0
                             for cell in range(grid.info.width * grid.info.height)]
                publisher.publish(grid)
            count += 1
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('Published {} updates with delayed TF on domain {}.'.format(count, args.domain))


if __name__ == '__main__':
    main()
