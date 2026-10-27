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

"""Publish live planning context and optional MPC predictions around the robot."""

from dataclasses import fields
from io import BytesIO
import math
import time

from geometry_msgs.msg import PolygonStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from limo_interfaces.msg import MpcDebug
from limo_controller.mpc_preview_render import (
    Candidate, DEFAULT_COLORS, FAMILIES, PreviewBackground, PreviewFrame, PreviewSources,
    RenderOptions,
    render_preview, world_to_base,
)
from limo_controller.mpc_preview_sources import laser_points, semantic_points, transform_points


class MpcPreview(Node):
    """Render bounded MPC telemetry outside the real-time controller loop."""

    def __init__(self):
        super().__init__('mpc_preview')
        defaults = {
            'debug_topic': '/limo/control/mpc_debug',
            'global_path_topic': '/limo/planning/path',
            'costmap_topic': '/local_costmap/costmap_raw',
            'footprint_topic': '/local_costmap/published_footprint',
            'view_mode': 'costmap',
            'laser_topic': '/scan',
            'semantic_points_topic': '/limo/map_package/online/local_ctrl_map/points',
            'fixed_frame': 'odom',
            'laser_max_range': 0.0,
            'source_stale_timeout': 1.0,
            'base_frame': 'base_link',
            'image_topic': '/limo/control/mpc_preview/image/compressed',
            'publish_rate': 10.0, 'stale_timeout': 1.0,
            'costmap_stale_timeout': 2.0,
            'image_format': 'png', 'png_compression': 1, 'jpeg_quality': 85,
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
        self.costmap_stale_timeout = self.get_parameter('costmap_stale_timeout').value
        self.view_mode = self.get_parameter('view_mode').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.laser_max_range = self.get_parameter('laser_max_range').value
        self.source_stale_timeout = self.get_parameter('source_stale_timeout').value
        if (self.view_mode not in ('sources', 'costmap') or not self.fixed_frame
                or not math.isfinite(self.laser_max_range) or self.laser_max_range < 0
                or not math.isfinite(self.source_stale_timeout) or self.source_stale_timeout <= 0):
            raise ValueError('Invalid source view settings')
        if (self.image_format not in ('png', 'jpeg') or not 0 <= self.png_compression <= 9
                or not 1 <= self.jpeg_quality <= 100
                or not math.isfinite(rate) or not 0 < rate <= 100
                or not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0
                or not math.isfinite(self.costmap_stale_timeout)
                or self.costmap_stale_timeout <= 0):
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
        context_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_subscription = self.create_subscription(
            Costmap, self.get_parameter('costmap_topic').value, self._costmap, context_qos)
        self.footprint_subscription = self.create_subscription(
            PolygonStamped, self.get_parameter('footprint_topic').value,
            self._footprint, input_qos)
        self.path_subscription = self.create_subscription(
            Path, self.get_parameter('global_path_topic').value, self._path, context_qos)
        self.laser_subscription = self.create_subscription(
            LaserScan, self.get_parameter('laser_topic').value, self._laser, input_qos)
        self.semantic_subscription = self.create_subscription(
            PointCloud2, self.get_parameter('semantic_points_topic').value,
            self._semantic, context_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frame = None
        self.header = None
        self.base_frame = self.get_parameter('base_frame').value
        self.costmap = None
        self.costmap_received_at = None
        self.footprint_message = None
        self.footprint = ()
        self.global_path = None
        self.laser = None
        self.semantic = None
        self.received_at = None
        self.last_warning = 0.0
        self.render_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate, self._publish, clock=self.render_clock)
        self.get_logger().info('MPC preview publishes ' + self.get_parameter('image_topic').value)

    @staticmethod
    def _yaw(q):
        """Extract the planar rotation shared by maps, poses and transforms."""
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def _costmap(self, msg):
        """Cache complete Nav2 costs, including inflation, independently of MPC."""
        meta = msg.metadata
        if (not msg.header.frame_id or meta.size_x <= 0 or meta.size_y <= 0
                or len(msg.data) != meta.size_x * meta.size_y
                or not math.isfinite(meta.resolution) or meta.resolution <= 0):
            return
        self.costmap = msg
        self.costmap_received_at = time.monotonic()

    def _path(self, msg):
        """Retain a planned path until it is replaced or explicitly cleared."""
        self.global_path = msg if msg.poses and msg.header.frame_id else None

    def _footprint(self, msg):
        """Use Nav2's published, padded collision polygon before MPC starts."""
        if (msg.header.frame_id and len(msg.polygon.points) >= 3
                and all(math.isfinite(p.x) and math.isfinite(p.y) for p in msg.polygon.points)):
            self.footprint_message = msg

    def _footprint_in_base(self):
        """Recover body geometry once at the polygon's timestamp, then retain it."""
        msg = self.footprint_message
        if msg is not None:
            try:
                transform = (0.0, 0.0, 0.0)
                if msg.header.frame_id != self.base_frame:
                    tf = self.tf_buffer.lookup_transform(
                        self.base_frame, msg.header.frame_id,
                        Time.from_msg(msg.header.stamp)).transform
                    transform = (tf.translation.x, tf.translation.y, self._yaw(tf.rotation))
                x, y, yaw = transform
                cosine, sine = math.cos(yaw), math.sin(yaw)
                self.footprint = tuple(
                    (x + cosine * p.x - sine * p.y, y + sine * p.x + cosine * p.y)
                    for p in msg.polygon.points)
                self.footprint_message = None
            except TransformException:
                pass  # Retry after TF arrives; keep the last valid body geometry.
        return self.footprint or (self.frame.footprint if self.frame is not None else ())

    def _laser(self, msg):
        """Keep all valid endpoints; distant returns may lie outside the viewport."""
        if msg.header.frame_id:
            self.laser = (msg.header, laser_points(msg, self.laser_max_range), time.monotonic())

    def _semantic(self, msg):
        """Keep live and persistent local-map points before rasterization or inflation."""
        if not msg.header.frame_id:
            return
        try:
            points = tuple(point for point in semantic_points(msg) if point[2] not in (1, 5))
            self.semantic = (msg.header, points, time.monotonic())
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def _sources_in_view(self, target_frame, robot_pose, stamp):
        """Compensate source timestamps through odometry, including while paused."""
        layers, status = {}, []
        for name, source in (('laser', self.laser), ('semantic', self.semantic)):
            label = 'laser' if name == 'laser' else 'semantic points'
            layers[name] = ()
            if source is None:
                status.append(label + ': waiting')
                continue
            header, points, received_at = source
            if time.monotonic() - received_at > self.source_stale_timeout:
                status.append(label + ': stale data')
            if not points:
                continue
            try:
                if header.frame_id == target_frame == self.fixed_frame:
                    transform = (0.0, 0.0, 0.0)
                else:
                    tf = self.tf_buffer.lookup_transform_full(
                        target_frame, stamp, header.frame_id, Time.from_msg(header.stamp),
                        self.fixed_frame).transform
                    transform = (tf.translation.x, tf.translation.y, self._yaw(tf.rotation))
                layers[name] = transform_points(points, transform, robot_pose)
            except TransformException:
                status.append('TF ' + label + ': waiting')
        sources = PreviewSources(
            laser=tuple((x, y) for x, y, _ in layers['laser']),
            semantic=layers['semantic'])  # Preserve each original point's class.
        return sources, status

    def _background(self):
        """Align the latest published local map to the current robot pose."""
        if self.costmap is None:
            return None
        msg, meta = self.costmap, self.costmap.metadata
        if msg.header.frame_id == self.base_frame:
            pose = (0.0, 0.0, 0.0)
        else:
            transform = self.tf_buffer.lookup_transform(
                msg.header.frame_id, self.base_frame, Time()).transform
            pose = (transform.translation.x, transform.translation.y,
                    self._yaw(transform.rotation))
        return PreviewBackground(
            robot_pose=pose, costmap_size=(meta.size_x, meta.size_y),
            costmap_resolution=meta.resolution,
            costmap_origin=(meta.origin.position.x, meta.origin.position.y,
                            self._yaw(meta.origin.orientation)),
            costmap_data=bytes(msg.data),
            footprint=self._footprint_in_base())

    def _path_in_view(self, target_frame, robot_pose, stamp):
        """Transform the persistent global reference into the displayed robot frame."""
        if self.global_path is None:
            return ()
        path = self.global_path
        translation, yaw = (0.0, 0.0), 0.0
        if path.header.frame_id != target_frame:
            transform = self.tf_buffer.lookup_transform(
                target_frame, path.header.frame_id, stamp).transform
            translation = (transform.translation.x, transform.translation.y)
            yaw = self._yaw(transform.rotation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        points = []
        for pose in path.poses:
            if pose.header.frame_id and pose.header.frame_id != path.header.frame_id:
                return ()
            x, y = pose.pose.position.x, pose.pose.position.y
            if not math.isfinite(x) or not math.isfinite(y):
                return ()
            points.append(world_to_base((
                translation[0] + cosine * x - sine * y,
                translation[1] + sine * x + cosine * y), robot_pose))
        return tuple(points)

    def _snapshot(self, msg):
        meta = msg.costmap.metadata
        q = meta.origin.orientation
        origin_yaw = self._yaw(q)
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
        frame = self.frame if fresh else None
        background = None
        status = '' if fresh else (
            'MPC: stale data' if self.received_at is not None else 'MPC: waiting')
        target_frame, robot_pose, stamp = self.base_frame, (0.0, 0.0, 0.0), Time()
        if fresh:
            target_frame, robot_pose = self.header.frame_id, frame.robot_pose
            stamp = Time.from_msg(self.header.stamp)
        elif self.view_mode == 'costmap':
            try:
                background = self._background()
            except TransformException:
                status += ' | Costmap TF unavailable'
            if background is not None:
                target_frame, robot_pose = self.costmap.header.frame_id, background.robot_pose
                age = time.monotonic() - self.costmap_received_at
                status += (' | stale costmap' if age > self.costmap_stale_timeout
                           else ' | local costmap')
            elif self.costmap is None:
                status += ' | waiting for costmap'
        sources = None
        if self.view_mode == 'sources':
            sources, source_status = self._sources_in_view(target_frame, robot_pose, stamp)
            status = ' | '.join(filter(None, (status, *source_status)))
        try:
            global_path = self._path_in_view(target_frame, robot_pose, stamp)
        except TransformException:
            global_path = ()
            status += ' | Path TF unavailable'
        try:
            image = render_preview(frame, self.options, status, background, global_path, sources)
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
