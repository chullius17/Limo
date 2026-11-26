"""Exercise actual DDS image subscriptions, TF, YAML profiles and plugin loading."""

from array import array
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import StaticTransformBroadcaster

from limo_cloud import POINT_DTYPE


def check(binary, plugin_overlay, serial):
    root = Path(__file__).resolve().parents[1]
    namespace = '/limo_cloud_test_{}'.format(serial)
    args = [binary, '--ros-args', '--params-file', str(root / 'config/cloud.yaml')]
    if plugin_overlay:
        args.extend(['--params-file', str(root / 'config/plugins_example.yaml')])
    args.extend(['-p', 'enable_telemetry:=false', '-p', 'input_crop_y_min:=0.0'])
    for key, suffix in [('depth_topic', 'depth'), ('classified_image_topic', 'labels'),
                        ('camera_info_topic', 'info'), ('pointcloud_topic', 'points')]:
        args.extend(['-p', '{}:={}/{}'.format(key, namespace, suffix)])
    node = rclpy.create_node('limo_cloud_integration_{}'.format(serial))
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    depth_pub = node.create_publisher(Image, namespace + '/depth', sensor_qos)
    labels_pub = node.create_publisher(Image, namespace + '/labels', 1)
    info_pub = node.create_publisher(CameraInfo, namespace + '/info', sensor_qos)
    received = []
    subscription = node.create_subscription(PointCloud2, namespace + '/points', received.append, 1)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'test_rgb_optical'
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 0.2
    transform.transform.translation.y = -0.1
    info = CameraInfo()
    info.header.frame_id = transform.child_frame_id
    info.width, info.height = 3, 2
    info.k = [2.0, 0.0, 1.0, 0.0, 2.0, 0.5, 0.0, 0.0, 1.0]
    depth = Image()
    depth.header.frame_id = transform.child_frame_id
    depth.width, depth.height, depth.step = 3, 2, 12
    depth.encoding = '32FC1'
    depth.data = array('B', np.full((2, 3), 1.25, dtype='<f4').tobytes())
    labels = Image()
    labels.header.frame_id = transform.child_frame_id
    labels.width, labels.height, labels.step = 3, 2, 3
    labels.encoding = 'mono8'
    labels.data = [1, 3, 7, 0, 2, 255]
    with tempfile.TemporaryFile(mode='w+') as output:
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 12
            while not received and time.monotonic() < deadline and process.poll() is None:
                stamp = node.get_clock().now().to_msg()
                depth.header.stamp = labels.header.stamp = info.header.stamp = stamp
                transform.header.stamp = stamp
                broadcaster.sendTransform(transform)
                info_pub.publish(info)
                depth_pub.publish(depth)
                rclpy.spin_once(node, timeout_sec=0.02)
                labels_pub.publish(labels)
                rclpy.spin_once(node, timeout_sec=0.05)
            output.seek(0)
            logs = output.read()
            assert received, 'No point cloud received:\n' + logs
            cloud = received[-1]
            assert cloud.header.frame_id == 'base_link'
            assert cloud.point_step == 16 and cloud.row_step == 16 * cloud.width
            actual = np.frombuffer(cloud.data, dtype=POINT_DTYPE)
            expected_classes = [1, 3] if plugin_overlay else [1, 3, 7, 2, 255]
            assert actual['class_id'].tolist() == expected_classes, actual
            assert np.all(actual['z'] == 0)
            np.testing.assert_allclose(actual['x'][:2], [-0.425, 0.2], atol=1e-6)
            np.testing.assert_allclose(actual['y'][:2], [-0.4125, -0.4125], atol=1e-6)
            assert 'LIMO cloud ready' in logs
            if not plugin_overlay:
                # The embedded interpreter must remain dormant on the default path.
                maps = Path('/proc/{}/maps'.format(process.pid)).read_text()
                assert 'numpy/core/' not in maps, 'NumPy loaded with no Python plugins'
            print('DDS/TF/YAML integration OK ({})'.format('Python + C++' if plugin_overlay else 'no plugins'))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_subscription(subscription)
            node.destroy_node()
        assert process.returncode == 0, process.returncode


if __name__ == '__main__':
    rclpy.init()
    try:
        check(sys.argv[1], False, 0)
        check(sys.argv[1], True, 1)
    finally:
        rclpy.shutdown()
