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

"""Exercise the real ROS message-to-compressed-image path with a paused ROS clock."""

from io import BytesIO
import os
import struct
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('limo_interfaces.msg')
from geometry_msgs.msg import (  # noqa: E402
    Point32, PolygonStamped, Pose2D, PoseStamped, TransformStamped,
)
from nav2_msgs.msg import Costmap  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from PIL import Image  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2, PointField  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from limo_interfaces.msg import MpcCandidate, MpcDebug  # noqa: E402
from limo_controller.mpc_preview import MpcPreview  # noqa: E402
from test_mpc_preview_render import sample_frame  # noqa: E402


@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    """Keep synthetic TF and the paused clock separate from an active simulation."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '192' if os.environ.get('ROS_DOMAIN_ID') == '191'
                       else '191')


def debug_message(frame):
    """Encode a renderer fixture using the actual controller telemetry interface."""
    msg = MpcDebug()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = 1234
    msg.base_frame = 'base_link'
    msg.robot_pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    msg.footprint.points = [Point32(x=x, y=y) for x, y in frame.footprint]
    msg.costmap.header = msg.header
    meta = msg.costmap.metadata
    meta.size_x, meta.size_y = frame.costmap_size
    meta.resolution = frame.costmap_resolution
    meta.origin.position.x, meta.origin.position.y, _ = frame.costmap_origin
    meta.origin.orientation.w = 1.0
    msg.costmap.data = list(frame.costmap_data)
    for name in ('wheelbase', 'rear_axle_to_base', 'initial_velocity', 'initial_steering',
                 'command_velocity', 'command_steering', 'model_dt', 'time_steps',
                 'generated_count', 'selected_id'):
        setattr(msg, name, getattr(frame, name))
    msg.candidates = [
        MpcCandidate(candidate_id=c.candidate_id, family=c.family,
                     score_status=c.score_status, total_cost=c.cost,
                     poses=[Pose2D(x=x, y=y, theta=yaw) for x, y, yaw in c.poses])
        for c in frame.candidates
    ]
    return msg


def test_ros_snapshot_becomes_png_and_stale_paths_disappear_without_clock_updates():
    """Publish typed telemetry, decode the PNG and verify its steady-clock timeout."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'debug_topic:=/test_mpc_preview/debug',
        '-p', 'image_topic:=/test_mpc_preview/image/compressed',
        '-p', 'publish_rate:=20.0',
        '-p', 'stale_timeout:=0.20',
        '-p', 'view_mode:=costmap',
        '-p', 'image_width:=640', '-p', 'image_height:=640',
        '-p', 'pixels_per_meter:=100.0', '-p', 'use_sim_time:=true',
    ])
    preview = MpcPreview()
    transport = Node('preview_transport_test')
    executor = SingleThreadedExecutor()
    executor.add_node(preview)
    executor.add_node(transport)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    images = []
    # Match Foxy's rqt_image_view; a best-effort image writer cannot serve it.
    image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    subscription = transport.create_subscription(
        CompressedImage, '/test_mpc_preview/image/compressed', images.append, image_qos)
    publisher = transport.create_publisher(MpcDebug, '/test_mpc_preview/debug', qos)
    msg = debug_message(sample_frame())
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not any(
                image.header.stamp.sec == 1234 for image in images):
            publisher.publish(msg)
            executor.spin_once(timeout_sec=0.02)
        fresh = next(image for image in images if image.header.stamp.sec == 1234)
        assert fresh.header.frame_id == 'base_link'
        assert fresh.format == 'png'
        decoded = Image.open(BytesIO(bytes(fresh.data)))
        assert decoded.size == (640, 640)
        assert decoded.getpixel((260, 220)) == (0, 180, 65)
        # No /clock is ever published: ROS time stays at zero.
        assert preview.get_clock().now().nanoseconds == 0
        images.clear()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not any(
                image.header.stamp.sec == 0 for image in images):
            executor.spin_once(timeout_sec=0.02)
        stale = next(image for image in images if image.header.stamp.sec == 0)
        assert Image.open(BytesIO(bytes(stale.data))).getpixel((260, 220)) != (0, 180, 65)
        assert subscription is not None
    finally:
        executor.remove_node(preview)
        executor.remove_node(transport)
        preview.destroy_node()
        transport.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_source_view_works_before_start_without_any_master_costmap_or_mpc_snapshot():
    """Project the complete laser and labeled local cloud with independent TF."""
    rclpy.init(args=[
        '--ros-args', '-p', 'debug_topic:=/test_mpc_sources/debug',
        '-p', 'image_topic:=/test_mpc_sources/image/compressed',
        '-p', 'laser_topic:=/test_mpc_sources/scan',
        '-p', 'semantic_points_topic:=/test_mpc_sources/cloud',
        '-p', 'publish_rate:=20.0', '-p', 'view_mode:=sources',
        '-p', 'image_width:=640', '-p', 'image_height:=640',
        '-p', 'pixels_per_meter:=100.0', '-p', 'use_sim_time:=true',
    ])
    transport = Node('preview_sources_test')
    broadcaster = StaticTransformBroadcaster(transport)
    base = TransformStamped()
    base.header.frame_id, base.child_frame_id = 'odom', 'base_link'
    base.transform.rotation.w = 1.0
    sensor = TransformStamped()
    sensor.header.frame_id, sensor.child_frame_id = 'base_link', 'test_laser'
    sensor.transform.translation.x = 0.25
    sensor.transform.rotation.w = 1.0
    broadcaster.sendTransform([base, sensor])
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_publisher = transport.create_publisher(PointCloud2, '/test_mpc_sources/cloud', durable)
    scan_publisher = transport.create_publisher(LaserScan, '/test_mpc_sources/scan', 1)
    cloud = PointCloud2()
    cloud.header.frame_id = 'base_link'
    cloud.width, cloud.height, cloud.point_step, cloud.row_step = 3, 1, 16, 48
    cloud.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                    for name, offset, datatype in (
                        ('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('class_id', 12, 2))]
    cloud.data = b''.join(struct.pack('<fffB3x', x, y, 0.0, label)
                          for x, y, label in ((0.7, 0.6, 2), (0.8, -0.7, 4), (0.6, -0.5, 1)))
    cloud_publisher.publish(cloud)
    scan = LaserScan()
    scan.header.frame_id = 'test_laser'
    scan.angle_min, scan.angle_increment = 0.0, 3.141592653589793 / 4
    scan.range_min, scan.range_max = 0.2, 8.0
    scan.ranges = [1.0, float('inf'), 2.0]
    preview = MpcPreview()
    executor = SingleThreadedExecutor()
    executor.add_node(preview)
    executor.add_node(transport)
    images = []
    image_subscription = transport.create_subscription(
        CompressedImage, '/test_mpc_sources/image/compressed', images.append, 1)

    def wait_for_image(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            scan_publisher.publish(scan)
            executor.spin_once(timeout_sec=0.01)
            while images:
                image = Image.open(BytesIO(bytes(images.pop(0).data)))
                if predicate(image):
                    return
        pytest.fail('Source points were not rendered')

    try:
        wait_for_image(lambda image:
                       image.getpixel((320, 195)) == preview.options.laser_color
                       and image.getpixel((260, 250)) == preview.options.semantic_color
                       and image.getpixel((390, 240)) == preview.options.semantic_color
                       and image.getpixel((370, 260)) not in (
                           preview.options.semantic_color, preview.options.laser_color))
        assert preview.frame is None and preview.costmap is None
        assert len(preview.laser[1]) == 2
        assert len(preview.semantic[1]) == 2
        assert all(label not in (1, 5) for _, _, label in preview.semantic[1])
        cloud.width, cloud.height, cloud.row_step, cloud.data = 0, 0, 0, b''
        cloud_publisher.publish(cloud)
        wait_for_image(lambda image:
                       image.getpixel((320, 195)) == preview.options.laser_color
                       and image.getpixel((260, 250)) != preview.options.semantic_color)
        assert image_subscription is not None
    finally:
        executor.remove_node(preview)
        executor.remove_node(transport)
        preview.destroy_node()
        transport.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_latched_live_map_and_path_survive_an_mpc_pause_and_keep_updating():
    """Show inflation before START, through telemetry expiry and after map changes."""
    rclpy.init(args=[
        '--ros-args', '-p', 'debug_topic:=/test_mpc_context/debug',
        '-p', 'image_topic:=/test_mpc_context/image/compressed',
        '-p', 'global_path_topic:=/test_mpc_context/path',
        '-p', 'costmap_topic:=/test_mpc_context/costmap_raw',
        '-p', 'footprint_topic:=/test_mpc_context/published_footprint',
        '-p', 'publish_rate:=20.0', '-p', 'stale_timeout:=0.20',
        '-p', 'image_width:=640', '-p', 'image_height:=640',
        '-p', 'pixels_per_meter:=100.0', '-p', 'use_sim_time:=true',
        '-p', 'costmap_as_points:=false',
        '-p', 'view_mode:=costmap',
    ])
    transport = Node('preview_context_test')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_publisher = transport.create_publisher(Costmap, '/test_mpc_context/costmap_raw', qos)
    footprint_publisher = transport.create_publisher(
        PolygonStamped, '/test_mpc_context/published_footprint', 1)
    path_publisher = transport.create_publisher(Path, '/test_mpc_context/path', qos)
    broadcaster = StaticTransformBroadcaster(transport)
    odom = TransformStamped()
    odom.header.frame_id, odom.child_frame_id = 'map', 'odom'
    odom.transform.translation.x, odom.transform.translation.y = 5.0, -3.0
    odom.transform.rotation.w = 1.0
    base = TransformStamped()
    base.header.frame_id, base.child_frame_id = 'odom', 'base_link'
    base.transform.translation.x, base.transform.translation.y = 1.0, 2.0
    base.transform.rotation.z = base.transform.rotation.w = 2 ** -0.5
    broadcaster.sendTransform([odom, base])
    footprint = PolygonStamped()
    footprint.header.frame_id = 'odom'
    body_polygon = ((-0.171, -0.120), (-0.171, 0.120),
                    (0.171, 0.120), (0.171, -0.120))
    footprint.polygon.points = [Point32(x=1.0 - y, y=2.0 + x) for x, y in body_polygon]
    costmap = Costmap()
    costmap.header.frame_id = 'odom'
    meta = costmap.metadata
    meta.size_x = meta.size_y = 100
    meta.resolution = 0.05
    meta.origin.position.x, meta.origin.position.y = -1.5, -0.5
    meta.origin.orientation.w = 1.0
    costs = bytearray(10000)
    costs[66 * 100 + 62] = 180
    costmap.data = list(costs)
    path = Path()
    path.header.frame_id = 'map'
    for x, y in ((5.4, -0.2), (5.4, 0.2)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        path.poses.append(pose)
    # Publish once before the viewer exists: late subscribers must recover both.
    map_publisher.publish(costmap)
    path_publisher.publish(path)
    preview = MpcPreview()
    executor = SingleThreadedExecutor()
    executor.add_node(preview)
    executor.add_node(transport)
    images = []
    image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    subscription = transport.create_subscription(
        CompressedImage, '/test_mpc_context/image/compressed', images.append, image_qos)
    debug_publisher = transport.create_publisher(
        MpcDebug, '/test_mpc_context/debug',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def wait_for_image(predicate, publish=None):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if publish is not None:
                publish()
            executor.spin_once(timeout_sec=0.01)
            while images:
                msg = images.pop(0)
                decoded = Image.open(BytesIO(bytes(msg.data)))
                if predicate(msg, decoded):
                    return decoded
        pytest.fail('Expected live-context image was not published')

    try:
        inflation = tuple(round(a + 180 / 252 * (b - a)) for a, b in zip(
            preview.options.free_color, preview.options.inflated_color))
        path_color = preview.options.global_path_color
        wait_for_image(lambda _, image: image.getpixel((260, 200)) == path_color
                       and image.getpixel((280, 200)) == (0, 180, 65)
                       and image.getpixel((383, 238)) == inflation
                       and image.getpixel((310, 303)) == (20, 30, 45),
                       lambda: footprint_publisher.publish(footprint))
        assert preview.frame is None
        for actual, expected in zip(preview.footprint, body_polygon):
            assert actual == pytest.approx(expected, abs=1e-6)
        # Active MPC still uses its own coherent snapshot.
        snapshot = debug_message(sample_frame())
        wait_for_image(lambda msg, image: msg.header.stamp.sec == 1234
                       and image.getpixel((272, 240)) == preview.options.selected_color,
                       lambda: debug_publisher.publish(snapshot))
        # Without /clock or new MPC snapshots, context remains after predictions expire.
        wait_for_image(lambda msg, image: msg.header.stamp.sec == 0
                       and image.getpixel((260, 200)) == path_color
                       and image.getpixel((383, 238)) == inflation
                       and image.getpixel((272, 240)) != preview.options.selected_color
                       and image.getpixel((280, 200)) == (0, 180, 65))
        costs[66 * 100 + 62] = 254
        costmap.data = list(costs)
        map_publisher.publish(costmap)
        wait_for_image(lambda _, image: image.getpixel((383, 238)) == preview.options.lethal_color
                       and image.getpixel((260, 200)) == path_color)
        # A robot pose change must move both the path and the stationary obstacle.
        base.transform.translation.x = 1.5
        broadcaster.sendTransform([odom, base])
        wait_for_image(lambda _, image: image.getpixel((210, 200)) == path_color
                       and image.getpixel((230, 200)) == (0, 180, 65)
                       and image.getpixel((333, 238)) == preview.options.lethal_color)
        path_publisher.publish(Path())
        wait_for_image(lambda _, image: image.getpixel((210, 200)) != path_color
                       and image.getpixel((230, 200)) != (0, 180, 65)
                       and image.getpixel((333, 238)) == preview.options.lethal_color)
        assert preview.get_clock().now().nanoseconds == 0
        assert subscription is not None
    finally:
        executor.remove_node(preview)
        executor.remove_node(transport)
        preview.destroy_node()
        transport.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
