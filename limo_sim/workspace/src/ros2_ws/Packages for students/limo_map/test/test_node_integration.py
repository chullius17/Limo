"""Exercise the real C++ node over DDS: TF, custom classes, laser updates and services."""

from array import array
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


def main(binary):
    rclpy.init()
    node = rclpy.create_node('limo_map_integration')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    prefix = '/limo_map_integration'
    clouds = node.create_publisher(PointCloud2, prefix + '/points', 1)
    laser_pub = node.create_publisher(OccupancyGrid, prefix + '/laser', qos)
    combined, semantic, layer = [], [], []
    subscriptions = [
        node.create_subscription(OccupancyGrid, prefix + '/map/' + name, result.append, qos)
        for name, result in [('combined_grid', combined), ('semantic_grid', semantic),
                             ('class_42_map', layer)]]
    save = node.create_client(Trigger, prefix + '/map_saver/save_map')
    reset = node.create_client(Trigger, prefix + '/reset_map')
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'map', 'limo_map_test_sensor'
    tf.transform.translation.x = 1.0
    tf.transform.rotation.w = 1.0
    tf.header.stamp = node.get_clock().now().to_msg()
    broadcaster.sendTransform(tf)

    def wait_for(condition, timeout=8):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
        assert condition(), 'Timed out waiting for mapping output/service'

    def call(client):
        wait_for(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        wait_for(future.done)
        return future.result()

    cloud = PointCloud2()
    cloud.header.frame_id = tf.child_frame_id
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 3, 16, 48
    cloud.fields = [PointField(name=name, offset=offset, datatype=dtype, count=1)
                    for name, offset, dtype in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                                ('class_id', 12, 2)]]
    cloud.data = array('B', b''.join(struct.pack('<fffBxxx', x, 0.1, 0.0, label)
                                    for x, label in [(0.1, 42), (1.1, 7), (2.1, 200)]))
    laser = OccupancyGrid()
    laser.header.frame_id = 'map'
    laser.info.resolution, laser.info.width, laser.info.height = 1.0, 5, 1
    laser.info.origin.orientation.w = 1.0
    laser.data = [-1, 100, 0, 0, -1]
    with tempfile.TemporaryDirectory(prefix='limo_map_integration_') as directory, \
            tempfile.TemporaryFile(mode='w+') as output:
        profile = Path(__file__).resolve().parents[1] / 'config/map.yaml'
        args = [binary, '--ros-args', '--params-file', str(profile)]
        for key, value in {
            'cloud_topic': prefix + '/points', 'reference_map_topic': prefix + '/laser',
            'output_prefix': prefix, 'class_ids': '[7, 42, 200]',
            'class_costs': '[0, 55, 100]', 'evidence_groups': '[7, 42, 200]',
            'resolution': '1.0', 'save_directory': directory,
        }.items():
            # Foxy applies node-specific YAML after wildcard CLI parameters.
            args += ['-p', 'limo_map:{}:={}'.format(key, value)]
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: save.service_is_ready() or process.poll() is not None)
            assert process.poll() is None, 'Mapper exited at startup'
            assert not call(save).success, 'Save must wait for the first laser map'
            wait_for(lambda: clouds.get_subscription_count() > 0)
            # Integrate one cloud before /map exists. Publish repeatedly with its
            # same timestamp to exercise deduplication without increasing evidence.
            cloud.header.stamp = node.get_clock().now().to_msg()
            clouds.publish(cloud)
            laser_pub.publish(laser)
            wait_for(lambda: bool(semantic) and bool(combined) and bool(layer)
                     and list(semantic[-1].data) == [-1, 55, 0, 100, -1]
                     and list(combined[-1].data) == [-1, 100, 0, 100, -1]
                     and list(layer[-1].data) == [-1, 55, 0, 0, -1])
            assert list(combined[-1].data) == [-1, 100, 0, 100, -1]
            assert list(layer[-1].data) == [-1, 55, 0, 0, -1]
            # Contradictory duplicate observations cannot clear the cell.
            duplicate = PointCloud2()
            duplicate.header, duplicate.fields = cloud.header, cloud.fields
            duplicate.height, duplicate.width = 1, 1
            duplicate.point_step, duplicate.row_step = 16, 16
            duplicate.data = array('B', struct.pack('<fffBxxx', 0.1, 0.1, 0.0, 7))
            for _ in range(10):
                clouds.publish(duplicate)
                rclpy.spin_once(node, timeout_sec=0.03)
            laser.data = [-1, 0, 0, 0, -1]
            laser_pub.publish(laser)
            wait_for(lambda: bool(combined) and list(combined[-1].data) == [-1, 55, 0, 100, -1])
            assert call(save).success
            assert len(list(Path(directory).iterdir())) == 6
            raw = (Path(directory) / 'limo_map_complete.pgm').read_bytes()
            assert list(raw[-5:]) == [255, 55, 0, 100, 255]
            assert call(reset).success
            wait_for(lambda: list(semantic[-1].data) == [-1] * 5
                     and list(combined[-1].data) == list(laser.data))
            assert list(combined[-1].data) == list(laser.data)
            # Reset permits the very same bag timestamp to be integrated again.
            clouds.publish(cloud)
            wait_for(lambda: list(semantic[-1].data) == [-1, 55, 0, 100, -1])
            # Invalid laser data must leave the last valid reference untouched.
            laser.data = [101] * 5
            laser_pub.publish(laser)
            assert call(save).success
            print('DDS/TF/custom classes/laser replacement/deduplication/reset/save passed')
        except Exception:
            output.seek(0)
            print(output.read(), file=sys.stderr)
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    assert subscriptions
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1])
