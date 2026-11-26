"""Run the complete example with a synthetic laser/TF/clock on an isolated domain."""

import math
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def main():
    rclpy.init()
    node = rclpy.create_node('mapping_example_fake_robot')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    broadcaster = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    laser_pose = TransformStamped()
    laser_pose.header.frame_id, laser_pose.child_frame_id = 'base_link', 'test_mapping_laser'
    laser_pose.transform.rotation.w = 1.0
    static.sendTransform(laser_pose)
    pose = TransformStamped()
    pose.header.frame_id, pose.child_frame_id = 'odom', 'base_link'
    pose.transform.rotation.w = 1.0
    scan = LaserScan()
    scan.header.frame_id = laser_pose.child_frame_id
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 2 * math.pi / 359
    scan.range_min, scan.range_max, scan.scan_time = 0.1, 8.0, 0.1
    scan.ranges = [2.0] * 360
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received = []
    sub = node.create_subscription(OccupancyGrid,
                                  '/limo/map_package/offline/map/combined_grid', received.append, qos)
    save = node.create_client(Trigger, '/limo/map_package/offline/map_saver/save_map')
    with tempfile.TemporaryFile(mode='w+') as logs:
        process = subprocess.Popen(
            ['ros2', 'launch', 'limo_map', 'map.launch.py', 'start_rviz:=false'],
            stdout=logs, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 22
            names = set()
            while time.monotonic() < deadline and process.poll() is None:
                stamp = node.get_clock().now().to_msg()
                clock_pub.publish(Clock(clock=stamp))
                pose.header.stamp = scan.header.stamp = stamp
                broadcaster.sendTransform(pose)
                scan_pub.publish(scan)
                rclpy.spin_once(node, timeout_sec=0.05)
                names = {name for name, namespace in node.get_node_names_and_namespaces()}
                required = {'lane_node', 'depth_correction', 'limo_cloud_example',
                            'slam_toolbox', 'limo_map', 'limo_map_save_gui'}
                if required <= names and received and save.service_is_ready():
                    break
            assert process.poll() is None, 'Complete example exited unexpectedly'
            assert required <= names, 'Missing mapping components: ' + str(required - names)
            assert received, 'SLAM did not produce a map that the mapper could publish'
            assert received[-1].header.frame_id == 'map'
            assert node.count_publishers('/limo/cv_package/visual_ptcld/points') == 1, \
                'The example must start exactly one classified cloud producer'
            assert 'visual_ptcld' not in names, 'Legacy cloud stage must remain disabled'
            assert save.service_is_ready(), 'Save GUI endpoint is unavailable'
            print('Complete example: CV, single cloud producer, SLAM, map TF, mapper and C++ GUI passed')
        except Exception:
            logs.seek(0)
            print(logs.read())
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    assert sub
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
