"""Shadow-free circuit viewer with optional live Gazebo model poses."""

import argparse
import math
import os
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

from PyQt5.QtCore import QPointF, QProcess, Qt, QTimer
from PyQt5.QtGui import QColor, QImageReader, QPainter, QPen, QPixmap, QPolygonF, QTransform
from PyQt5.QtWidgets import (
    QApplication, QGraphicsItem, QGraphicsScene, QGraphicsView, QMainWindow, QPushButton,
)

from user_package.circuit_scene import (
    Pose2D, load_scene_sdf, load_world_scene, pose_from_link_states, world_sdf_from_response,
)


# Physical LIMO polygon in base_link, matching traj_sim.yaml without costmap padding.
LIMO_FOOTPRINT = ((-0.161, -0.110), (-0.161, 0.110),
                  (0.161, 0.110), (0.161, -0.110))


def default_world():
    """Prefer installed simulation assets, with a source-tree offline fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return (Path(get_package_share_directory('custom_start'))
                / 'worlds/limo_obstacle_world.world')
    except (ImportError, LookupError):
        for parent in Path(__file__).resolve().parents:
            candidate = parent / 'custom_start/worlds/limo_obstacle_world.world'
            if candidate.is_file():
                return candidate
    raise FileNotFoundError('Specify --world with the Gazebo world file')


class CircuitView(QGraphicsView):
    """Orthographic rendering in world metres, with zoom and pan."""

    def __init__(self, circuit):
        super().__init__()
        self.circuit = circuit
        self.graphics = QGraphicsScene(self)
        self.setScene(self.graphics)
        self.setBackgroundBrush(QColor('#f2f2f2'))
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setFrameShape(QGraphicsView.NoFrame)
        reader = QImageReader(str(circuit.ground.texture))
        size = reader.size()
        if max(size.width(), size.height()) > 4096:
            size.scale(4096, 4096, Qt.KeepAspectRatio)
            reader.setScaledSize(size)
        image = reader.read()
        if image.isNull():
            raise ValueError('Cannot read circuit texture: ' + reader.errorString())
        self.background = self.graphics.addPixmap(QPixmap.fromImage(image))
        self.background.setZValue(-10)
        self.base_link_items = []
        self.robot_footprint = None
        self.obstacles = []
        self.follow_fit = True
        self.draw_poses(None)
        self.bounds = self.circuit_bounds()
        self.graphics.setSceneRect(self.bounds)

    def draw_poses(self, poses):
        """Draw exact known footprints, retaining the last ground if it disappears."""
        ground = self.circuit.ground
        pose = None if poses is None else poses.get(ground.model_name)
        a, b, c = ground.texture_corners(pose)
        width = self.background.pixmap().width()
        height = self.background.pixmap().height()
        self.background.setTransform(QTransform(
            (b[0] - a[0]) / width, -(b[1] - a[1]) / width,
            (c[0] - a[0]) / height, -(c[1] - a[1]) / height, a[0], -a[1]))
        for item in self.obstacles:
            self.graphics.removeItem(item)
        self.obstacles.clear()
        pen = QPen(QColor('#737373'))
        pen.setWidth(0)  # One screen pixel, without enlarging metric bounds.
        pen.setCosmetic(True)
        for name, points, color in self.circuit.footprints(poses):
            item = self.graphics.addPolygon(
                QPolygonF([QPointF(x, -y) for x, y in points]), pen, QColor(color))
            item.setToolTip(name)
            item.setZValue(1)
            self.obstacles.append(item)

    def draw_base_link(self, pose, stale=False):
        """Show the physical footprint, link origin and axes at the Gazebo pose."""
        for item in self.base_link_items:
            self.graphics.removeItem(item)
        self.base_link_items.clear()
        self.robot_footprint = None
        if pose is None:
            return
        points = [pose.point(*point) for point in LIMO_FOOTPRINT]
        pen = QPen(QColor('#1976d2'))
        pen.setWidth(2)
        pen.setCosmetic(True)
        self.robot_footprint = self.graphics.addPolygon(
            QPolygonF([QPointF(px, -py) for px, py in points]),
            pen, QColor(25, 118, 210, 65))
        self.base_link_items.append(self.robot_footprint)
        x, y = pose.x, -pose.y
        for local, color in (((0.35, 0.0), '#e53935'), ((0.0, 0.35), '#43a047')):
            tip = pose.point(*local)
            tx, ty = tip[0], -tip[1]
            pen = QPen(QColor(color))
            pen.setWidth(3)
            pen.setCosmetic(True)
            axis = self.graphics.addLine(x, y, tx, ty, pen)
            angle = math.atan2(ty - y, tx - x)
            head = [(tx, ty)] + [
                (tx - 0.085 * math.cos(angle + delta),
                 ty - 0.085 * math.sin(angle + delta)) for delta in (-0.45, 0.45)]
            arrow = self.graphics.addPolygon(
                QPolygonF([QPointF(*point) for point in head]), QPen(Qt.NoPen), QColor(color))
            self.base_link_items.extend((axis, arrow))
        pen = QPen(QColor('white'))
        pen.setWidth(2)
        pen.setCosmetic(True)
        origin = self.graphics.addEllipse(x - 0.055, y - 0.055, 0.11, 0.11,
                                          pen, QColor('#1976d2'))
        label = self.graphics.addSimpleText('base_link')
        label.setBrush(QColor('#1565c0'))
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        label.setPos(x + 0.08, y + 0.08)
        self.base_link_items.extend((origin, label))
        for item in self.base_link_items:
            item.setZValue(10)
            item.setOpacity(0.4 if stale else 1.0)
            item.setToolTip('base_link — last received pose' if stale
                            else 'base_link — Gazebo world pose; red: +X, green: +Y')
        self.robot_footprint.setZValue(9)
        self.robot_footprint.setToolTip(
            'Robot footprint — 0.322 × 0.220 m; ' +
            ('last received pose' if stale else 'Gazebo world pose'))

    def fit_circuit(self):
        """Restore a complete view of the circuit and current obstacle extents."""
        self.follow_fit = True
        self.bounds = self.circuit_bounds()
        self.graphics.setSceneRect(self.bounds)
        self.fitInView(self.bounds, Qt.KeepAspectRatio)

    def circuit_bounds(self):
        """Fit the circuit and obstacles without letting the robot move the camera."""
        bounds = self.background.sceneBoundingRect()
        for item in self.obstacles:
            bounds = bounds.united(item.sceneBoundingRect())
        return bounds.adjusted(-0.15, -0.15, 0.15, 0.15)

    def resizeEvent(self, event):
        """Keep the whole circuit visible until the user zooms."""
        super().resizeEvent(event)
        if self.follow_fit and hasattr(self, 'bounds'):
            self.fitInView(self.bounds, Qt.KeepAspectRatio)

    def wheelEvent(self, event):
        """Zoom around the pointer, without changing the world geometry."""
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        scale = self.transform().m11() * factor
        if 5 < scale < 5000:
            self.follow_fit = False
            self.scale(factor, factor)
        event.accept()


class CircuitWindow(QMainWindow):
    """Show circuit geometry and the robot frame on the Qt event loop."""

    def __init__(self, circuit, node=None, topic='/gazebo/model_states', refresh_hz=20.0,
                 world_path=None, model_paths=()):
        super().__init__()
        self.node = node
        self.topic = topic
        self.world_path = Path(world_path) if world_path is not None else default_world()
        self.model_paths = tuple(model_paths)
        self.query = None
        self.notice = ''
        self.notice_until = 0.0
        self.ignore_poses_before = 0.0
        self.query_timeout = QTimer(self)
        self.query_timeout.setSingleShot(True)
        self.query_timeout.timeout.connect(self._query_timed_out)
        self.setWindowTitle('Gazebo circuit — top view')
        self.resize(1200, 820)
        self.view = CircuitView(circuit)
        self.setCentralWidget(self.view)
        self.refresh_button = QPushButton('Refresh obstacles')
        self.refresh_button.clicked.connect(self.refresh_obstacles)
        self.refresh_button.setToolTip(
            'Read new obstacle geometry from Gazebo' if node is not None
            else 'Reload obstacle geometry from the world file (offline)')
        self.statusBar().addPermanentWidget(self.refresh_button)
        fit = QPushButton('Fit view')
        fit.clicked.connect(self.view.fit_circuit)
        self.statusBar().addPermanentWidget(fit)
        self.last_revision = -1
        self.last_base_link = None
        self.statusBar().showMessage('Static world view — Gazebo connection disabled')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(1, round(1000 / refresh_hz)))
        QTimer.singleShot(0, self.view.fit_circuit)

    def refresh(self):
        """Consume the newest sample without blocking painting or input."""
        if self.node is None:
            return
        import rclpy
        if not rclpy.ok():
            self.close()
            return
        # Two independent streams: do not starve link updates behind model states.
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.0)
        if (self.node.revision != self.last_revision and self.node.received_at is not None
                and self.node.received_at >= self.ignore_poses_before):
            self.view.draw_poses(self.node.poses)
            self.last_revision = self.node.revision
        self._update_base_link()
        if self.node.received_at is None:
            message = 'Waiting for {} — initial positions from world file'.format(self.topic)
        else:
            age = time.monotonic() - self.node.received_at
            if age > 1.5:
                message = 'No Gazebo updates for {:.1f} s — last received positions'.format(age)
            else:
                message = 'Gazebo connected — live obstacle positions'
            unknown = self.view.circuit.unknown_models(
                self.node.poses, excluded=('limo', 'robot', 'ground_plane',
                                           self.node.robot_link.split('::', 1)[0]))
            if unknown:
                message += ' | Click Refresh obstacles to load: ' + ', '.join(unknown)
        if self.node.base_link_received_at is None:
            message += ' | Waiting for ' + self.node.robot_link
        elif self.node.base_link_pose is None:
            message += ' | Robot link not present: ' + self.node.robot_link
        elif time.monotonic() - self.node.base_link_received_at > 1.5:
            message += ' | No recent base_link updates'
        if self.query is not None:
            message = 'Refreshing obstacles from Gazebo…'
        elif time.monotonic() < self.notice_until:
            message = self.notice
        self.statusBar().showMessage(message)

    def _update_base_link(self):
        """Update the frame even if the obstacle snapshot has not changed."""
        age = (math.inf if self.node.base_link_received_at is None else
               time.monotonic() - self.node.base_link_received_at)
        state = (self.node.base_link_pose, age > 1.5)
        if state != self.last_base_link:
            self.view.draw_base_link(*state)
            self.last_base_link = state

    def _show_notice(self, message):
        self.notice = message
        self.notice_until = time.monotonic() + 8.0
        self.statusBar().showMessage(message)

    def refresh_obstacles(self):
        """Request a read-only world snapshot without blocking the Qt event loop."""
        if self.query is not None:
            return
        if self.node is None:
            try:
                self._apply_world(self.world_path.read_text(encoding='utf-8'))
            except (OSError, ValueError, ET.ParseError) as exc:
                self._show_notice('Could not refresh obstacles: ' + str(exc))
            return
        self.refresh_button.setEnabled(False)
        self._show_notice('Refreshing obstacles from Gazebo…')
        query = QProcess(self)
        self.query = query
        query.finished.connect(lambda code, status: self._query_finished(query, code, status))
        query.errorOccurred.connect(lambda error: self._query_error(query, error))
        # Classic Gazebo returns a GzString containing the complete current SDF.
        # This request reads the world; it does not pause, save or modify it.
        query.start('gz', ['topic', '--world-name', self.view.circuit.world_name,
                           '--request', 'world_sdf'])
        self.query_timeout.start(15000)

    def _query_error(self, query, error):
        if query is self.query and error == QProcess.FailedToStart:
            self._finish_query(query)
            self._show_notice('Could not refresh obstacles: Gazebo Classic "gz" command not found')

    def _query_timed_out(self):
        query = self.query
        if query is None:
            return
        self.query = None
        self.refresh_button.setEnabled(True)
        query.kill()
        query.finished.connect(query.deleteLater)
        self._show_notice('Could not refresh obstacles: Gazebo did not respond within 15 seconds')

    def _finish_query(self, query):
        self.query_timeout.stop()
        self.query = None
        self.refresh_button.setEnabled(True)
        query.deleteLater()

    def _query_finished(self, query, code, status):
        if query is not self.query:
            return
        output = bytes(query.readAllStandardOutput()).decode('utf-8', errors='replace')
        self._finish_query(query)
        try:
            if code != 0 or status != QProcess.NormalExit:
                raise ValueError('Gazebo query failed')
            self._apply_world(world_sdf_from_response(output))
        except (ValueError, SyntaxError, UnicodeError, OSError, ET.ParseError) as exc:
            self._show_notice('Could not refresh obstacles: ' + str(exc))

    def _apply_world(self, sdf):
        """Replace geometry atomically and keep the original circuit texture."""
        old = self.view.circuit
        scene = load_scene_sdf(sdf, self.world_path, self.model_paths, ground=old.ground)
        if scene.world_name != old.world_name:
            raise ValueError('The snapshot belongs to a different Gazebo world')
        self.view.circuit = scene
        # In a paused simulation ModelStates can predate newly inserted models.
        # Display snapshot poses until a new message is actually received.
        self.ignore_poses_before = time.monotonic()
        self.last_revision = -1
        self.view.draw_poses(None)
        message = 'Obstacles refreshed — {} models loaded'.format(len(scene.models))
        if scene.warnings:
            message += ' | Some visuals could not be loaded (see details)'
        self.refresh_button.setToolTip('\n'.join(scene.warnings) or
                                       'Read new obstacle geometry from Gazebo')
        self._show_notice(message)

    def closeEvent(self, event):
        """Stop only this viewer's query process when the window closes."""
        self.timer.stop()
        self.query_timeout.stop()
        query, self.query = self.query, None
        if query is not None:
            query.kill()
            query.waitForFinished(250)
            query.deleteLater()
        super().closeEvent(event)


def create_gazebo_node(topic, link_states_topic='/gazebo/link_states',
                       robot_link='limo::base_link'):
    """Subscribe to model states; never send commands to the simulation."""
    from gazebo_msgs.msg import LinkStates, ModelStates
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class GazeboSceneNode(Node):
        """Store one complete, validated model-pose snapshot."""

        def __init__(self):
            super().__init__('circuit_experiment')
            self.poses = None
            self.revision = 0
            self.received_at = None
            self.robot_link = robot_link
            self.base_link_pose = None
            self.base_link_received_at = None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.subscription = self.create_subscription(
                ModelStates, topic, self.receive, qos)
            self.link_subscription = self.create_subscription(
                LinkStates, link_states_topic, self.receive_links, qos)

        def receive_links(self, msg):
            """Read the exact scoped base link, independently of the model origin."""
            try:
                pose = pose_from_link_states(msg, self.robot_link)
            except ValueError:
                return
            self.base_link_pose = pose
            self.base_link_received_at = time.monotonic()

        def receive(self, msg):
            """Reject malformed snapshots; model removals follow each full list."""
            if len(msg.name) != len(msg.pose) or len(set(msg.name)) != len(msg.name):
                return
            try:
                poses = {name: Pose2D.from_message(pose)
                         for name, pose in zip(msg.name, msg.pose)}
            except ValueError:
                return
            if poses != self.poses:
                self.poses = poses
                self.revision += 1
            self.received_at = time.monotonic()

    return GazeboSceneNode()


def main(args=None):
    """Run live in ROS, or render a static preview without a ROS installation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, help='The same .world loaded by Gazebo')
    parser.add_argument('--model-states-topic', default='/gazebo/model_states')
    parser.add_argument('--link-states-topic', default='/gazebo/link_states')
    parser.add_argument('--robot-link', default='limo::base_link',
                        help='Exact scoped Gazebo link name for the robot frame')
    parser.add_argument('--refresh-hz', type=float, default=20.0)
    parser.add_argument('--offline', action='store_true', help='Show initial SDF poses only')
    parser.add_argument('--screenshot', type=Path, help='Save a PNG preview and exit')
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.refresh_hz) or not 0 < options.refresh_hz <= 120:
        parser.error('--refresh-hz must be in (0, 120]')
    world_path = options.world or default_world()
    model_paths = [p for p in os.environ.get('GAZEBO_MODEL_PATH', '').split(os.pathsep) if p]
    circuit = load_world_scene(world_path, model_paths=model_paths)
    app = QApplication([sys.argv[0]])
    node = None
    if not options.offline:
        import rclpy
        rclpy.init(args=ros_args)
        node = create_gazebo_node(options.model_states_topic, options.link_states_topic,
                                  options.robot_link)
        for warning in circuit.warnings:
            node.get_logger().warning(warning)
    window = CircuitWindow(circuit, node, options.model_states_topic, options.refresh_hz,
                           world_path=world_path, model_paths=model_paths)
    window.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    if options.screenshot:
        def save():
            options.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(options.screenshot)):
                raise OSError('Cannot write screenshot')
            app.quit()
        QTimer.singleShot(300, save)
    try:
        return app.exec_()
    finally:
        window.close()
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
