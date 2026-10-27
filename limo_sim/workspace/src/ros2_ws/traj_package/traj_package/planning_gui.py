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

"""Display the planner's global costmap and the current search transcript."""

import ast
from html import escape
import math
import signal
import sys
import time

from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from PyQt5.QtCore import QByteArray, QBuffer, QIODevice, QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QImageWriter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QGraphicsItem, QGraphicsScene, QGraphicsView, QLabel, QPlainTextEdit,
    QSplitter, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_yaw(quaternion):
    """Read the planar heading used by map origins and robot transforms."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def transform_point(point, pose):
    """Apply a planar rigid transform to a point."""
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (x + cosine * point[0] - sine * point[1],
            y + sine * point[0] + cosine * point[1])


def world_to_pixel(point, grid):
    """Project map coordinates onto the flipped image, including origin yaw."""
    origin = grid.info.origin
    yaw = quaternion_yaw(origin.orientation)
    dx, dy = point[0] - origin.position.x, point[1] - origin.position.y
    return ((math.cos(yaw) * dx + math.sin(yaw) * dy) / grid.info.resolution,
            grid.info.height - (-math.sin(yaw) * dx + math.cos(yaw) * dy)
            / grid.info.resolution)


def png_bytes(image):
    """Encode the complete annotated view as a lossless compressed PNG."""
    data = QByteArray()
    buffer = QBuffer(data)
    buffer.open(QIODevice.WriteOnly)
    writer = QImageWriter(buffer, b'png')
    if not writer.write(image):
        raise ValueError('PNG compression failed: {}'.format(writer.errorString()))
    buffer.close()
    return bytes(data)


def costmap_image(grid):
    """Color OccupancyGrid values and put positive map Y toward the top."""
    width, height = grid.info.width, grid.info.height
    if (width <= 0 or height <= 0 or len(grid.data) != width * height
            or not math.isfinite(grid.info.resolution) or grid.info.resolution <= 0.0):
        raise ValueError('Invalid costmap dimensions or resolution')
    values = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    if np.any((values < -1) | (values > 100)):
        raise ValueError('Costmap values outside the range [-1, 100]')

    palette = np.empty((102, 3), dtype=np.uint8)
    palette[0] = (245, 245, 245)  # Free.
    level = np.linspace(0.0, 1.0, 98)
    palette[1:99, 0] = 255
    palette[1:99, 1] = np.rint(235 * (1.0 - level)).astype(np.uint8)
    palette[1:99, 2] = 60
    palette[99] = (190, 50, 210)  # Inscribed obstacle.
    palette[100] = (20, 20, 20)  # Lethal obstacle.
    palette[101] = (125, 135, 145)  # Unknown (-1).
    pixels = np.ascontiguousarray(palette[values][::-1])
    # Own the storage after the temporary numpy array leaves this function.
    return QImage(pixels.data, width, height, pixels.strides[0], QImage.Format_RGB888).copy()


class PlanningGuiNode(Node):
    """Keep only the latest costmap for the Qt refresh loop."""

    def __init__(self, **kwargs):
        super().__init__('planning_gui', **kwargs)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('footprint_topic', '/global_costmap/published_footprint')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter(
            'footprint', '[[-0.161, -0.110], [-0.161, 0.110], [0.161, 0.110], [0.161, -0.110]]')
        self.declare_parameter('footprint_padding', 0.0)
        self.declare_parameter('axis_length', 0.45)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('adjusted_goal_topic', '/adjusted_goal_pose')
        self.declare_parameter('adjusted_start_topic', '/adjusted_start_pose')
        self.declare_parameter('planning_path_topic', '/limo/planning/path')
        self.declare_parameter('planning_log_topic', '/limo/planning/log')
        self.declare_parameter('image_topic', '/limo/planning/preview/image/compressed')
        self.declare_parameter('image_publish_rate', 5.0)
        self.image_publish_rate = self.get_parameter('image_publish_rate').value
        if not math.isfinite(self.image_publish_rate) or not 0.0 < self.image_publish_rate <= 20.0:
            raise ValueError('image_publish_rate must be between 0 and 20 Hz')
        self.requested_goal = None
        self.adjusted_goal = None
        self.adjusted_start = None
        self.planned_path = None
        self.planning_log = None
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.axis_length = self.get_parameter('axis_length').value
        padding = self.get_parameter('footprint_padding').value
        footprint = ast.literal_eval(self.get_parameter('footprint').value)
        if (len(footprint) < 3 or not math.isfinite(padding) or padding < 0.0
                or not math.isfinite(self.axis_length) or self.axis_length <= 0.0
                or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in footprint)):
            raise ValueError('Invalid footprint, padding or axis length')
        # Match Nav2 padFootprint: extend each signed coordinate by padding.
        self.footprint = tuple(tuple(v + (padding if v > 0 else -padding if v < 0 else 0.0)
                                     for v in point) for point in footprint)
        self.footprint_source = 'configuration'
        self.footprint_message = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_grid = None
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self._receive, qos)
        self.create_subscription(
            PolygonStamped, self.get_parameter('footprint_topic').value,
            self._receive_footprint, 1)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self._receive_goal, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('adjusted_goal_topic').value,
            self._receive_adjusted_goal, qos)
        self.create_subscription(
            PoseStamped, self.get_parameter('adjusted_start_topic').value,
            self._receive_adjusted_start, qos)
        self.create_subscription(
            Path, self.get_parameter('planning_path_topic').value, self._receive_path, qos)
        self.create_subscription(
            String, self.get_parameter('planning_log_topic').value, self._receive_log, qos)
        self.image_publisher = self.create_publisher(
            CompressedImage, self.get_parameter('image_topic').value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def _receive(self, grid):
        self.latest_grid = grid

    def _receive_log(self, msg):
        self.planning_log = msg.data

    def _receive_footprint(self, msg):
        if (msg.header.frame_id and len(msg.polygon.points) >= 3
                and all(math.isfinite(p.x) and math.isfinite(p.y) for p in msg.polygon.points)):
            self.footprint_message = msg

    @staticmethod
    def _valid_goal(msg):
        q = msg.pose.orientation
        return (bool(msg.header.frame_id)
                and all(math.isfinite(v) for v in (
                    msg.pose.position.x, msg.pose.position.y, q.x, q.y, q.z, q.w))
                and q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-12)

    def _receive_goal(self, msg):
        if self._valid_goal(msg):
            self.requested_goal = msg
            self.planned_path = None
            self.adjusted_goal = None
            self.adjusted_start = None

    def _receive_adjusted_goal(self, msg):
        if self._valid_goal(msg):
            self.adjusted_goal = msg

    def _receive_adjusted_start(self, msg):
        if self._valid_goal(msg):
            self.adjusted_start = msg

    def _receive_path(self, msg):
        self.planned_path = msg if msg.poses and msg.header.frame_id else None

    def planned_goal(self):
        """Show an accepted goal only when it belongs to a validated path."""
        return self._planned_pose(self.adjusted_goal)

    def planned_start(self):
        """Show the selected planning start, independent of the live robot pose."""
        return self._planned_pose(self.adjusted_start)

    def _planned_pose(self, pose):
        if self.planned_path is None or pose is None:
            return None
        path_stamp = Time.from_msg(self.planned_path.header.stamp).nanoseconds
        pose_stamp = Time.from_msg(pose.header.stamp).nanoseconds
        if pose_stamp < path_stamp:
            return None  # A retained pose from an earlier plan must not turn blue.
        if self.requested_goal is not None:
            request_stamp = Time.from_msg(self.requested_goal.header.stamp).nanoseconds
            if path_stamp < request_stamp:
                return None
        return pose

    def goal_overlay(self, msg, target_frame):
        """Place the selected or accepted pose in the displayed costmap frame."""
        if msg is None:
            return None
        pose = (msg.pose.position.x, msg.pose.position.y, quaternion_yaw(msg.pose.orientation))
        if msg.header.frame_id == target_frame:
            return pose
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
            yaw = quaternion_yaw(tf.rotation)
            x, y = transform_point(pose, (tf.translation.x, tf.translation.y, yaw))
            return x, y, pose[2] + yaw
        except TransformException:
            return None

    def publish_image(self, image, frame_id):
        """Publish only a compressed image, bounded to 960 by 720 pixels."""
        if image.width() > 960 or image.height() > 720:
            image = image.scaled(960, 720, Qt.KeepAspectRatio, Qt.FastTransformation)
        msg = CompressedImage()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'png'
        msg.data = png_bytes(image)
        self.image_publisher.publish(msg)

    def robot_overlay(self, target_frame):
        """Recover Nav2 body geometry at its stamp and place it using live TF."""
        msg = self.footprint_message
        if msg is not None:
            try:
                pose = (0.0, 0.0, 0.0)
                if msg.header.frame_id != self.base_frame:
                    tf = self.tf_buffer.lookup_transform(
                        self.base_frame, msg.header.frame_id,
                        Time.from_msg(msg.header.stamp)).transform
                    pose = (tf.translation.x, tf.translation.y, quaternion_yaw(tf.rotation))
                self.footprint = tuple(transform_point((p.x, p.y), pose)
                                       for p in msg.polygon.points)
                self.footprint_source = 'Nav2'
                self.footprint_message = None
            except TransformException:
                pass  # Retry when timestamped TF arrives; retain the body geometry.
        try:
            pose = (0.0, 0.0, 0.0)
            if target_frame != self.base_frame:
                tf = self.tf_buffer.lookup_transform(
                    target_frame, self.base_frame, Time()).transform
                pose = (tf.translation.x, tf.translation.y, quaternion_yaw(tf.rotation))
            return pose, self.footprint
        except TransformException:
            return None


class CostmapView(QGraphicsView):
    """Fit the entire map to the available area without changing its aspect."""

    def __init__(self):
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setBackgroundBrush(QColor('#20262e'))
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setMinimumSize(240, 180)
        self.map_item = None
        self.robot_items = []
        self.goal_items = []

    def set_goals(self, grid, requested, planned, start=None):
        """Draw the green requested goal and blue planning endpoints with yaw."""
        for item in self.goal_items:
            self.scene().removeItem(item)
        self.goal_items.clear()
        # Different lengths preserve both colors when the two poses coincide.
        for pose, color, name, length, width, offset in [
            (requested, '#00a63c', 'Selected goal', 0.65, 4.0, -0.25),
            (planned, '#1565ff', 'Planned goal', 0.45, 2.5, 0.25),
            (start, '#1565ff', 'Planned start', 0.45, 2.5, -0.65),
        ]:
            if pose is None:
                continue

            def pixel(point):
                return QPointF(*world_to_pixel(transform_point(point, pose), grid))

            arrow = QPainterPath(pixel((0.0, 0.0)))
            arrow.lineTo(pixel((length, 0.0)))
            for side in (-0.45, 0.45):
                arrow.moveTo(pixel((length, 0.0)))
                arrow.lineTo(pixel((length - 0.14 * math.cos(side), 0.14 * math.sin(side))))
            for line_color, line_width in [('#ffffff', width + 2), (color, width)]:
                pen = QPen(QColor(line_color))
                pen.setWidthF(line_width)
                pen.setCosmetic(True)
                item = self.scene().addPath(arrow, pen)
                item.setZValue(4)
                self.goal_items.append(item)
            label = self.scene().addText('')
            label.setFont(QFont('Sans Serif', 9))
            label.document().setDocumentMargin(1)
            label.setHtml('<span style="background-color: white; color: {}">{}</span>'
                          .format(color, name))
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations)
            label.setPos(pixel((0.0, offset)))
            label.setZValue(5)
            self.goal_items.append(label)

    def clear_robot(self):
        """Hide geometry when the robot cannot be located in the displayed map."""
        for item in self.robot_items:
            self.scene().removeItem(item)
        self.robot_items.clear()

    def set_robot(self, grid, pose, footprint, base_frame, axis_length):
        """Draw the footprint and red X / green Y axes like the MPC viewer."""
        self.clear_robot()

        def pixel(point):
            return QPointF(*world_to_pixel(transform_point(point, pose), grid))

        def path_item(path, color, width):
            pen = QPen(QColor(color))
            pen.setWidthF(width)
            pen.setCosmetic(True)
            item = self.scene().addPath(path, pen)
            item.setZValue(2)
            self.robot_items.append(item)

        def label(text, point, color):
            item = self.scene().addText('')
            item.setFont(QFont('Sans Serif', 9))
            item.document().setDocumentMargin(1)
            item.setHtml('<span style="background-color: white; color: {}">{}</span>'
                         .format(color, escape(text)))
            item.setFlag(QGraphicsItem.ItemIgnoresTransformations)
            item.setPos(pixel(point))
            item.setZValue(3)
            self.robot_items.append(item)

        polygon = QPainterPath(pixel(footprint[0]))
        for point in footprint[1:]:
            polygon.lineTo(pixel(point))
        polygon.closeSubpath()
        path_item(polygon, '#ffffff', 7)
        path_item(polygon, '#141e2d', 3)

        for angle, color, name in [(0.0, '#c32323', '+x'), (math.pi / 2, '#238237', '+y')]:
            end = (axis_length * math.cos(angle), axis_length * math.sin(angle))
            arrow = QPainterPath(pixel((0.0, 0.0)))
            arrow.lineTo(pixel(end))
            for side in (-0.45, 0.45):
                arrow.moveTo(pixel(end))
                arrow.lineTo(pixel((end[0] - 0.08 * math.cos(angle + side),
                                    end[1] - 0.08 * math.sin(angle + side))))
            path_item(arrow, '#ffffff', 4)
            path_item(arrow, color, 2)
            label(name, (end[0] * 1.1, end[1] * 1.1), color)
        label(base_frame, (0.0, -0.35), '#141e2d')

    def set_image(self, image):
        """Replace the map while preserving discrete cell colors."""
        pixmap = QPixmap.fromImage(image)
        if self.map_item is None:
            self.map_item = self.scene().addPixmap(pixmap)
        else:
            self.map_item.setPixmap(pixmap)
        bounds = self.map_item.boundingRect()
        margin = 0.025 * max(bounds.width(), bounds.height())
        self.scene().setSceneRect(bounds.adjusted(-margin, -margin, margin, margin))
        self._fit_map()

    def _fit_map(self):
        if self.map_item is not None:
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):  # noqa: N802 - Qt virtual method.
        """Refit the map after resizing the window or splitter."""
        super().resizeEvent(event)
        self._fit_map()


class PlanningWindow(QWidget):
    """Show the global planning map above the planner attempt log."""

    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('LIMO — Planning')
        self.resize(1000, 820)
        self.node = node
        self.grid = None
        self.last_image_publish = 0.0
        self.status = QLabel('Waiting for the planner costmap…')
        self.status.setWordWrap(True)
        self.robot_status = QLabel('Waiting for {} pose'.format(node.base_frame))
        self.robot_status.setWordWrap(True)
        topic = QLabel('Planner costmap: {}'.format(node.costmap_topic))
        topic.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.map_view = CostmapView()
        legend = QLabel(
            'White: free  |  Yellow → red: increasing cost  |  '
            'Purple: inscribed obstacle  |  Black: obstacle  |  Gray: unknown')
        legend.setWordWrap(True)
        goal_legend = QLabel(
            'Green arrow: selected goal  |  Blue arrows: planned goal and start')
        goal_legend.setWordWrap(True)

        map_panel = QWidget()
        map_layout = QVBoxLayout(map_panel)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.addWidget(topic)
        map_layout.addWidget(self.status)
        map_layout.addWidget(self.robot_status)
        map_layout.addWidget(self.map_view, 1)
        map_layout.addWidget(legend)
        map_layout.addWidget(goal_legend)

        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setFont(QFont('Monospace', 10))
        self.terminal.setStyleSheet(
            'QPlainTextEdit { background: #101419; color: #d6e1eb; border: 0; }')
        log_panel = QWidget()
        log_layout = QVBoxLayout(log_panel)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_layout.addWidget(QLabel('PLANNING LOG — attempts and results'))
        log_layout.addWidget(self.terminal)

        self.splitter = QSplitter(Qt.Vertical)
        self.splitter.addWidget(map_panel)
        self.splitter.addWidget(log_panel)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setStretchFactor(0, 4)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([600, 160])
        layout = QVBoxLayout(self)
        layout.addWidget(self.splitter)

        # ROS and Qt share one thread: callbacks only replace the latest grid.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def refresh(self):
        """Drain ROS callbacks within a budget, then update map and live pose."""
        if not rclpy.ok():
            self.close()
            return
        # TF contains separate wheel, odometry and localization messages. One
        # callback per render tick can drop the slower map->odom transform.
        deadline = time.monotonic() + 0.005
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if time.monotonic() >= deadline:
                break
        if (self.node.planning_log is not None
                and self.node.planning_log != self.terminal.toPlainText()):
            scrollbar = self.terminal.verticalScrollBar()
            position = scrollbar.value()
            follow = position >= scrollbar.maximum() - 1
            self.terminal.setPlainText(self.node.planning_log)
            scrollbar.setValue(scrollbar.maximum() if follow else position)
        grid = self.node.latest_grid
        self.node.latest_grid = None
        if grid is not None:
            try:
                image = costmap_image(grid)
            except ValueError as error:
                self.status.setText('Invalid costmap: {}'.format(error))
            else:
                self.grid = grid
                self.map_view.set_image(image)
                self.status.setText(
                    'Frame: {}  |  {} × {} cells  |  {:.3f} m/cell  |  {:.2f} × {:.2f} m'
                    .format(grid.header.frame_id or '—', grid.info.width, grid.info.height,
                            grid.info.resolution, grid.info.width * grid.info.resolution,
                            grid.info.height * grid.info.resolution))
        if self.grid is not None:
            overlay = self.node.robot_overlay(self.grid.header.frame_id)
            if overlay is None:
                self.map_view.clear_robot()
                self.robot_status.setText('TF unavailable: {} → {}'.format(
                    self.node.base_frame, self.grid.header.frame_id))
            else:
                pose, footprint = overlay
                self.map_view.set_robot(
                    self.grid, pose, footprint, self.node.base_frame, self.node.axis_length)
                self.robot_status.setText(
                    '{}: x={:.2f} m, y={:.2f} m, yaw={:.1f}°  |  '
                    'Footprint: {}  |  Axes: +x red, +y green'.format(
                        self.node.base_frame, pose[0], pose[1], math.degrees(pose[2]),
                        self.node.footprint_source))
            requested = self.node.goal_overlay(
                self.node.requested_goal, self.grid.header.frame_id)
            planned = self.node.goal_overlay(self.node.planned_goal(), self.grid.header.frame_id)
            start = self.node.goal_overlay(self.node.planned_start(), self.grid.header.frame_id)
            self.map_view.set_goals(self.grid, requested, planned, start)
            now = time.monotonic()
            if now - self.last_image_publish >= 1.0 / self.node.image_publish_rate:
                self.last_image_publish = now
                self.node.publish_image(
                    self.map_view.viewport().grab().toImage(), self.grid.header.frame_id)

    def closeEvent(self, event):  # noqa: N802 - Qt virtual method.
        """Stop polling when the window closes."""
        self.timer.stop()
        super().closeEvent(event)


def main(args=None):
    """Run the passive planning viewer until the window or launch closes."""
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = PlanningGuiNode()
    window = PlanningWindow(node)

    def request_shutdown(_signum, _frame):
        window.close()
        app.quit()

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    window.show()
    try:
        app.exec_()
    finally:
        window.timer.stop()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
