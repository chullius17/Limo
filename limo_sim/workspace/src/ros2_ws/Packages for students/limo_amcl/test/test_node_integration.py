"""Exercise actual C++ localization and Nav2 map/lifecycle servers off the live graph."""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[1]


class RunningAmcl(unittest.TestCase):
    def test_maps_lifecycle_custom_classes_fusion_and_laser_fallback(self):
        with tempfile.TemporaryDirectory(prefix='limo_amcl_integration_') as directory:
            folder = Path(directory)
            # Square room with free interior. Separate server files exercise every map input.
            image = bytes(0 if x in (0, 39) or y in (0, 39) else 254
                          for y in range(40) for x in range(40))
            (folder / 'room.pgm').write_bytes(b'P5\n40 40\n255\n' + image)
            for name in ['complete', 'laser', 'cv_obstacle']:
                (folder / ('limo_map_' + name + '.yaml')).write_text(yaml.safe_dump({
                    'image': 'room.pgm', 'resolution': 0.1, 'origin': [-2.0, -2.0, 0.0],
                    'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
                }))
            config = yaml.safe_load((ROOT / 'config/amcl.yaml').read_text())
            config['amcl']['ros__parameters'].update({
                'set_initial_pose': True, 'initial_pose.x': 0.0, 'initial_pose.y': 0.0,
                'initial_pose.z': 0.0, 'initial_pose.yaw': 0.0,
                'min_particles': 100, 'max_particles': 100,
                'cv_road_class_ids': [7], 'cv_obstacle_class_ids': [42],
                'cv_min_points': 1.0, 'cv_min_non_road_voxels': 1,
                'cv_quality_gate_enabled': False,
                'cv_cloud_topic': '/test/limo_amcl/classified',
            })
            (folder / 'params.yaml').write_text(yaml.safe_dump(config))
            log_path = folder / 'launch.log'
            rclpy.init()
            node = rclpy.create_node('limo_amcl_integration_observer')
            self.node = node
            self.clock_pub = node.create_publisher(Clock, '/clock', 10)
            received = {'pose': [], 'cv': [], 'laser': [], 'tf': [], 'maps': set()}
            retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            subscriptions = [
                node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                         lambda m: received['pose'].append(m), retained),
                node.create_subscription(DiagnosticArray, '/amcl/cv_diagnostics',
                                         lambda m: received['cv'].append(m), 100),
                node.create_subscription(DiagnosticArray, '/amcl/laser_diagnostics',
                                         lambda m: received['laser'].append(m), 100),
                node.create_subscription(TFMessage, '/tf', lambda m: received['tf'].extend(m.transforms), 100),
            ]
            for topic in ['/map', '/limo/map_package/online/maps/laser_map',
                          '/limo/map_package/online/maps/cv_obstacle']:
                subscriptions.append(node.create_subscription(
                    OccupancyGrid, topic, lambda m, topic=topic: received['maps'].add(topic), retained))
            scan_pub = node.create_publisher(LaserScan, '/scan', 10)
            cloud_pub = node.create_publisher(PointCloud2, '/test/limo_amcl/classified', 10)
            broadcaster = StaticTransformBroadcaster(node)
            transforms = []
            for parent, child in [('odom', 'base_link'), ('base_link', 'test_laser')]:
                transform = TransformStamped()
                transform.header.stamp = node.get_clock().now().to_msg()
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            broadcaster.sendTransform(transforms)
            force = node.create_client(Empty, '/request_nomotion_update')
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    'ros2', 'launch', 'limo_amcl', 'amcl.launch.py',
                    'start_rviz:=false',
                    'map_directory:=' + directory, 'config_file:=' + str(folder / 'params.yaml'),
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    self.wait(lambda: len(received['maps']) == 3 and force.service_is_ready(), 18, process)
                    for name in ['limo_amcl_complete_map_server', 'limo_amcl_laser_map_server',
                                 'limo_amcl_cv_map_server', 'amcl']:
                        client = node.create_client(GetState, '/' + name + '/get_state')
                        self.wait(client.service_is_ready, 3, process)
                        future = client.call_async(GetState.Request())
                        self.wait(future.done, 3, process)
                        self.assertEqual(future.result().current_state.id, 3, name)
                    required = {'lane_node', 'depth_correction', 'limo_cloud_utilities'}
                    self.wait(lambda: required <= {name for name, namespace in
                              node.get_node_names_and_namespaces()}, 10, process)
                    self.assertEqual(node.count_publishers('/limo/cv_package/visual_ptcld/points'), 1)
                    self.assertNotIn('visual_ptcld', {name for name, namespace in
                                     node.get_node_names_and_namespaces()})
                    helper = node.create_client(GetParameters, '/amcl_rclcpp_node/get_parameters')
                    self.wait(helper.service_is_ready, 3, process)
                    request = GetParameters.Request(names=['use_sim_time'])
                    future = helper.call_async(request)
                    self.wait(future.done, 3, process)
                    self.assertTrue(future.result().values[0].bool_value,
                                    'The Foxy TF helper must follow the simulation clock')
                    self.wait(lambda: scan_pub.get_subscription_count() > 0 and
                              cloud_pub.get_subscription_count() > 0, 4, process)

                    def publish(with_cloud):
                        stamp = node.get_clock().now().to_msg()
                        if with_cloud:
                            cloud = PointCloud2()
                            cloud.header.frame_id = 'base_link'
                            cloud.header.stamp = stamp
                            cloud.height, cloud.width = 1, 2
                            cloud.point_step, cloud.row_step = 13, 26
                            cloud.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                                            for name, offset, kind in [
                                                ('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('class_id', 12, 2)]]
                            cloud.data = struct.pack('<fffBfffB', 1.9, 0.0, 0.0, 42, 0.5, 0.0, 0.0, 7)
                            cloud_pub.publish(cloud)
                            self.spin(0.04)
                        if force.service_is_ready():
                            future = force.call_async(Empty.Request())
                            self.wait(future.done, 2, process)
                        scan = LaserScan()
                        scan.header.frame_id, scan.header.stamp = 'test_laser', stamp
                        scan.angle_min, scan.angle_max = -math.pi, math.pi
                        scan.angle_increment = math.pi / 90
                        scan.range_min, scan.range_max = 0.05, 8.0
                        scan.ranges = [1.9] * 181
                        scan_pub.publish(scan)
                        self.spin(0.15)

                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and not any(
                            m.status[0].message == 'accepted' for m in received['cv']):
                        publish(True)
                    self.assertTrue(any(m.status[0].message == 'accepted' for m in received['cv']),
                                    'No accepted CV update with custom class IDs')
                    accepted = next(m.status[0] for m in received['cv'] if m.status[0].message == 'accepted')
                    values = {v.key: v.value for v in accepted.values}
                    self.assertEqual(int(values['used_non_road_points']), 1)
                    self.assertEqual(int(values['used_road_points']), 1)
                    self.wait(lambda: bool(received['pose']) and any(
                        t.header.frame_id == 'map' and t.child_frame_id == 'odom' for t in received['tf']),
                        3, process)
                    self.assertTrue(math.isfinite(received['pose'][-1].pose.pose.position.x))
                    self.assertTrue(any(int(v.value) > 0 for m in received['laser']
                                        for s in m.status for v in s.values if v.key == 'used_points'))
                    self.spin(0.3)  # Expire the original cloud instead of silently refreshing its time.
                    previous = len(received['pose'])
                    deadline = time.monotonic() + 4
                    while time.monotonic() < deadline and not any(
                            m.status[0].message == 'no_synchronized_cloud' for m in received['cv']):
                        publish(False)
                    self.assertTrue(any(m.status[0].message == 'no_synchronized_cloud' for m in received['cv']))
                    self.wait(lambda: len(received['pose']) > previous, 3, process)
                    self.assertIsNone(process.poll(), 'Localization exited during fallback')
                except BaseException:
                    log.flush()
                    print(log_path.read_text(), flush=True)
                    raise
                finally:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
                    node.destroy_node()
                    rclpy.shutdown()

    def wait(self, condition, timeout, process):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.assertIsNone(process.poll(), 'Launch process exited')
            self.clock_pub.publish(Clock(clock=self.node.get_clock().now().to_msg()))
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertTrue(condition(), 'Timeout waiting for localization stack')

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.clock_pub.publish(Clock(clock=self.node.get_clock().now().to_msg()))
            rclpy.spin_once(self.node, timeout_sec=0.01)


if __name__ == '__main__':
    unittest.main()
