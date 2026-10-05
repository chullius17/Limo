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
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('limo_interfaces.msg')
from geometry_msgs.msg import Point32, Pose2D  # noqa: E402
from PIL import Image  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

from limo_interfaces.msg import MpcCandidate, MpcDebug  # noqa: E402
from limo_controller.mpc_preview import MpcPreview  # noqa: E402
from test_mpc_preview_render import sample_frame  # noqa: E402


def test_ros_snapshot_becomes_png_and_stale_paths_disappear_without_clock_updates():
    """Publish typed telemetry, decode the PNG and verify its steady-clock timeout."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'debug_topic:=/test_mpc_preview/debug',
        '-p', 'image_topic:=/test_mpc_preview/image/compressed',
        '-p', 'publish_rate:=20.0',
        '-p', 'stale_timeout:=0.20',
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
    subscription = transport.create_subscription(
        CompressedImage, '/test_mpc_preview/image/compressed', images.append, qos)
    publisher = transport.create_publisher(MpcDebug, '/test_mpc_preview/debug', qos)
    frame = sample_frame()
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
