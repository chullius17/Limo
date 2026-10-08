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

"""Publish a compressed, robot-centered image of simulated MPC candidates."""

from dataclasses import fields
from io import BytesIO
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from limo_interfaces.msg import MpcDebug
from limo_controller.mpc_preview_render import (
    Candidate, DEFAULT_COLORS, FAMILIES, PreviewFrame, RenderOptions, render_preview,
)


class MpcPreview(Node):
    """Render bounded MPC telemetry outside the real-time controller loop."""

    def __init__(self):
        super().__init__('mpc_preview')
        defaults = {
            'debug_topic': '/limo/control/mpc_debug',
            'image_topic': '/limo/control/mpc_preview/image/compressed',
            'publish_rate': 4.0, 'stale_timeout': 1.0,
            'image_format': 'png', 'png_compression': 3, 'jpeg_quality': 85,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        options = RenderOptions()
        for item in fields(options):
            if item.name == 'colors':
                continue
            value = getattr(options, item.name)
            self.declare_parameter(item.name, list(value) if isinstance(value, tuple) else value)
            selected = self.get_parameter(item.name).value
            setattr(options, item.name, tuple(selected) if isinstance(value, tuple) else selected)
        colors = []
        for name, color in zip(FAMILIES, DEFAULT_COLORS):
            key = 'colors.' + name
            self.declare_parameter(key, list(color))
            colors.append(tuple(self.get_parameter(key).value))
        options.colors = tuple(colors)
        options.validate()
        self.options = options
        self.image_format = self.get_parameter('image_format').value.lower()
        self.png_compression = self.get_parameter('png_compression').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        rate = self.get_parameter('publish_rate').value
        self.stale_timeout = self.get_parameter('stale_timeout').value
        if (self.image_format not in ('png', 'jpeg') or not 0 <= self.png_compression <= 9
                or not 1 <= self.jpeg_quality <= 100
                or not math.isfinite(rate) or not 0 < rate <= 100
                or not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0):
            raise ValueError('Invalid image format, compression, rate or stale timeout')
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Foxy's rqt_image_view requests reliable delivery for image transport.
        # Reliable image output also serves best-effort viewers; telemetry input
        # keeps its existing best-effort contract with the controller.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            CompressedImage, self.get_parameter('image_topic').value, image_qos)
        self.subscription = self.create_subscription(
            MpcDebug, self.get_parameter('debug_topic').value, self._snapshot, input_qos)
        self.frame = None
        self.header = None
        self.base_frame = 'base_link'
        self.received_at = None
        self.last_warning = 0.0
        self.render_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate, self._publish, clock=self.render_clock)
        self.get_logger().info('MPC preview publishes ' + self.get_parameter('image_topic').value)

    def _snapshot(self, msg):
        meta = msg.costmap.metadata
        q = meta.origin.orientation
        origin_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
        self.frame = PreviewFrame(
            robot_pose=(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta),
            footprint=tuple((p.x, p.y) for p in msg.footprint.points),
            costmap_size=(meta.size_x, meta.size_y),
            costmap_resolution=meta.resolution,
            costmap_origin=(meta.origin.position.x, meta.origin.position.y, origin_yaw),
            costmap_data=bytes(msg.costmap.data),
            wheelbase=msg.wheelbase, rear_axle_to_base=msg.rear_axle_to_base,
            initial_velocity=msg.initial_velocity, initial_steering=msg.initial_steering,
            command_velocity=msg.command_velocity, command_steering=msg.command_steering,
            model_dt=msg.model_dt, time_steps=msg.time_steps,
            generated_count=msg.generated_count, selected_id=msg.selected_id,
            candidates=tuple(Candidate(c.candidate_id, c.family, c.score_status, c.total_cost,
                                       tuple((p.x, p.y, p.theta) for p in c.poses))
                             for c in msg.candidates),
        )
        self.header = msg.header
        self.base_frame = msg.base_frame
        self.received_at = time.monotonic()

    def _publish(self):
        if self.publisher.get_subscription_count() == 0:
            return
        fresh = (self.received_at is not None
                 and time.monotonic() - self.received_at <= self.stale_timeout)
        status = ('Snapshot MPC scaduto: controllo fermo o dati assenti'
                  if self.received_at is not None else 'In attesa di uno snapshot MPC')
        try:
            image = render_preview(self.frame if fresh else None, self.options, status)
            output = BytesIO()
            if self.image_format == 'png':
                image.save(output, format='PNG', compress_level=self.png_compression)
            else:
                image.save(output, format='JPEG', quality=self.jpeg_quality)
            msg = CompressedImage()
            msg.header.stamp = self.header.stamp if fresh else self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.format = self.image_format
            msg.data = output.getvalue()
            self.publisher.publish(msg)
        except (ValueError, OSError, OverflowError) as error:
            now = time.monotonic()
            if now - self.last_warning > 2.0:
                self.get_logger().warning('MPC preview rendering failed: ' + str(error))
                self.last_warning = now


def main(args=None):
    """Run the preview until shutdown."""
    rclpy.init(args=args)
    node = MpcPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
