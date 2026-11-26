"""Compare real C++/Python/DDS outputs with the original visual_ptcld pipeline."""

from array import array
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from tf2_ros import StaticTransformBroadcaster
import yaml

from cv_package.visual_ptcld.cloud_message import DEBUG_CLOUD_DTYPE
from helpers import INTRINSICS, legacy_output, scene, settings, transform
from limo_cloud import POINT_DTYPE


@pytest.mark.parametrize('profile,encoding,debug', [
    ('real', '32FC1', False), ('sim', '32FC1', False),
    ('sim', '16UC1', False), ('sim', '32FC1', True),
])
def test_cpp_projection_and_python_plugins_match_legacy_over_dds(
        profile, encoding, debug, monkeypatch, tmp_path):
    # These tests also publish the legacy debug topics: keep their DDS domain
    # local and separate from any running robot or simulator, including in colcon.
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '194')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    original, parameters = settings(profile, roi_y_min=.17, roi_y_max=.93)
    parameters['use_sim_time'] = False
    parameters['enable_telemetry'] = False
    parameters['debug.enable_debug_publications'] = debug
    if debug and 'debug' not in parameters['plugins']:
        parameters['plugins'].insert(2, 'debug')
    labels_array, depth_array = scene(37)
    if encoding == '16UC1':
        depth_array = (np.nan_to_num(depth_array, nan=0, posinf=0).clip(0, 60) * 1000).astype(np.uint16)
    expected_debug = {} if debug else None
    expected = legacy_output(labels_array, depth_array, original, expected_debug)
    suffix = '{}_{}_{}'.format(profile, encoding.lower(), int(debug))
    prefix = '/limo_cloud_utilities_test_' + suffix
    for key, topic in (('classified_image_topic', 'labels'), ('depth_topic', 'depth'),
                       ('camera_info_topic', 'info'), ('pointcloud_topic', 'points')):
        parameters[key] = prefix + '/' + topic
    binary = Path(get_package_prefix('limo_cloud_utilities')) / 'lib/limo_cloud_utilities/limo_cloud_utilities_node'
    assert binary.is_file(), 'Build limo_cloud_utilities before running the DDS equivalence test'
    # Exercise the dedicated executable's default Python plugin registration.
    parameters.pop('plugins')
    parameters = {key: value for key, value in parameters.items() if not key.endswith('.type')}
    context = Context()
    rclpy.init(args=[], context=context)
    node = Node('example_equivalence_' + suffix, context=context, use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    depth_pub = node.create_publisher(Image, prefix + '/depth', qos)
    labels_pub = node.create_publisher(Image, prefix + '/labels', 1)
    info_pub = node.create_publisher(CameraInfo, prefix + '/info', qos)
    received, debug_received = [], {key: [] for key in ('before', 'debug', 'lines', 'filter')}
    node.create_subscription(PointCloud2, prefix + '/points', received.append, 1)
    if debug:
        root = '/limo/cv_package/visual_ptcld/'
        for key, topic in (('before', 'points_before_kdtree'), ('debug', 'curb_points_debug/compressed'),
                           ('lines', 'lines_and_curbs/compressed'), ('filter', 'blue_filter_debug/compressed')):
            node.create_subscription(PointCloud2 if key == 'before' else CompressedImage,
                                     root + topic, debug_received[key].append, 1)
    static_tf = StaticTransformBroadcaster(node)
    tf = transform()
    tf.header.stamp.sec = 42
    info = CameraInfo()
    info.header.frame_id = tf.child_frame_id
    info.header.stamp.sec = 42
    fx, fy, cx, cy, iw, ih = INTRINSICS
    info.width, info.height = iw, ih
    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    height, width = labels_array.shape
    labels = Image()
    labels.header = info.header
    labels.height, labels.width, labels.step = height, width, width
    labels.encoding = 'mono8'
    labels.data = array('B', labels_array.tobytes())
    depth = Image()
    depth.header = info.header
    depth.height, depth.width = height, width
    depth.step = width * depth_array.itemsize
    depth.encoding = encoding
    depth.data = array('B', depth_array.tobytes())
    with tempfile.TemporaryDirectory(prefix='limo_cloud_utilities_') as temporary:
        config = Path(temporary) / 'parameters.yaml'
        config.write_text(yaml.safe_dump({'limo_cloud_utilities': {'ros__parameters': parameters}}))
        with (Path(temporary) / 'node.log').open('w+') as logs:
            environment = dict(os.environ, ROS_LOG_DIR=temporary, OPENBLAS_NUM_THREADS='1',
                               OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', BLIS_NUM_THREADS='1')
            # The original pipeline is an oracle in the parent test process only.
            # The standalone child must find and run its local algorithm copies.
            for key in ('PYTHONPATH', 'AMENT_PREFIX_PATH'):
                environment[key] = ':'.join(part for part in environment.get(key, '').split(':')
                                            if 'cv_package' not in part)
            process = subprocess.Popen(
                [str(binary), '--ros-args', '--params-file', str(config)],
                stdout=logs, stderr=subprocess.STDOUT, env=environment)
            try:
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline and process.poll() is None:
                    static_tf.sendTransform(tf)
                    info_pub.publish(info)
                    depth_pub.publish(depth)
                    executor.spin_once(timeout_sec=.02)
                    labels_pub.publish(labels)
                    executor.spin_once(timeout_sec=.03)
                    if received and (not debug or all(debug_received.values())):
                        break
                logs.seek(0)
                text = logs.read()
                assert received, 'No output from composed node:\n' + text
                actual = received[-1]
                assert actual.header == expected.header
                assert actual.fields == expected.fields
                assert (actual.width, actual.height, actual.point_step, actual.row_step,
                        actual.is_bigendian, actual.is_dense) == (
                            expected.width, expected.height, expected.point_step, expected.row_step,
                            expected.is_bigendian, expected.is_dense)
                output = np.frombuffer(actual.data, dtype=POINT_DTYPE)
                reference = np.frombuffer(expected.data, dtype=POINT_DTYPE)
                for name in POINT_DTYPE.names:
                    np.testing.assert_array_equal(output[name], reference[name], err_msg=name)
                if debug:
                    assert all(debug_received.values()), 'Missing compatibility debug topics:\n' + text
                    before = debug_received['before'][-1]
                    expected_before = expected_debug['before'][0]
                    assert before.header == expected_before.header
                    assert before.fields == expected_before.fields
                    a = np.frombuffer(before.data, dtype=DEBUG_CLOUD_DTYPE)
                    b = np.frombuffer(expected_before.data, dtype=DEBUG_CLOUD_DTYPE)
                    for name in DEBUG_CLOUD_DTYPE.names:
                        np.testing.assert_array_equal(a[name], b[name])
                    for key in ('debug', 'lines', 'filter'):
                        a, b = debug_received[key][-1], expected_debug[key][0]
                        assert a.header == b.header
                        assert a.format == b.format
                        assert bytes(a.data) == bytes(b.data), key
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                executor.remove_node(node)
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
            assert process.returncode == 0, process.returncode
