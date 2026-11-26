"""Desktop control experiment: global reference and measured-speed footprint trail."""

from collections import deque
from datetime import datetime
import math
from pathlib import Path
import signal
import sys
import time

from geometry_msgs.msg import PoseStamped
from limo_interfaces.msg import MpcDebug
from matplotlib import cm, colors, patheffects
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Patch, Polygon
from matplotlib.transforms import Affine2D
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from user_package.control.control_experiment_model import (
    ControlRecording, footprint_vertices, map_pose, quaternion_yaw,
)
from user_package.map_geometry import MapSnapshot
from user_package.control.control_telemetry import (
    ControlTelemetry, reference_distance, scan_minimum,
)


class ControlExperimentNode(Node):
    """Acquire timestamped observations; never publish control commands or TF."""

    def __init__(self, **kwargs):
        super().__init__('control_experiment', **kwargs)
        defaults = {
            'map_topic': '/map', 'path_topic': '/limo/planning/path',
            'goal_topic': '/goal_pose',
            'scan_topic': '/scan', 'debug_topic': '/limo/control/mpc_debug',
            'control_status_topic': '/limo/control/status',
            'control_paused_topic': '/limo/control/paused',
            'odom_topic': '/odometry/filtered', 'base_frame': 'base_link',
            'footprint': [-0.161, -0.110, -0.161, 0.110, 0.161, 0.110, 0.161, -0.110],
            'speed_scale_max': 0.5, 'sample_period': 0.1, 'max_samples': 50000,
            'refresh_hz': 4.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        if (not math.isfinite(self.settings['speed_scale_max'])
                or self.settings['speed_scale_max'] <= 0
                or not math.isfinite(self.settings['refresh_hz'])
                or not 0 < self.settings['refresh_hz'] <= 30):
            raise ValueError('Invalid speed scale or display refresh rate')
        self.run = ControlRecording(
            np.asarray(self.settings['footprint']).reshape(-1, 2),
            self.settings['sample_period'], self.settings['max_samples'])
        self.telemetry = ControlTelemetry(self.settings['max_samples'])
        self.execution_requested = self.control_paused = False
        self.control_status = 'Waiting for control status'
        self.guidance_stamp = None
        self.live_map = None
        self.reference = ()
        self.path_message = None
        self.goal_message = self.run_goal = None
        self.latest = self.last_arrival = None
        self.last_odom_stamp = None
        self.pending = deque()
        self.dropped = 0
        self.issue = ''
        self.revision = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, self.settings['map_topic'], self.receive_map, retained)
        self.create_subscription(RosPath, self.settings['path_topic'], self.receive_path, retained)
        self.create_subscription(
            PoseStamped, self.settings['goal_topic'], self.receive_goal, 10)
        self.create_subscription(
            Odometry, self.settings['odom_topic'], self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.settings['scan_topic'], self.receive_scan, qos_profile_sensor_data)
        self.create_subscription(
            MpcDebug, self.settings['debug_topic'], self.receive_debug, qos_profile_sensor_data)
        self.create_subscription(
            String, self.settings['control_status_topic'], self.receive_status, retained)
        self.create_subscription(
            Bool, self.settings['control_paused_topic'], self.receive_paused, retained)
        self.create_timer(0.02, self.drain_pending)

    def ros_seconds(self):
        """Use the same ROS clock as simulation and the controller."""
        return self.get_clock().now().nanoseconds*1e-9

    def receive_status(self, message):
        """Follow executor state rather than treating pause cancellation as abort."""
        self.control_status = message.data
        state = message.data.partition(':')[0].strip().upper()
        if state in ('ACTIVE', 'WAITING', 'PAUSED'):
            self.execution_requested = True
            self.receive_paused(Bool(data=state == 'PAUSED'))
            self.maybe_start_recording()
        elif state in ('SUCCESS', 'ERROR', 'ABORT', 'ABORTED', 'IDLE', 'READY'):
            if self.execution_requested or state in ('SUCCESS', 'ERROR', 'ABORT', 'ABORTED'):
                result = 'SUCCESS' if state == 'SUCCESS' else 'ERROR' if state == 'ERROR' else 'ABORTED'
                self.stop_capture(result)
            self.execution_requested = False
            self.control_paused = False

    def receive_paused(self, message):
        """Pause measurements and mark both plots without issuing robot commands."""
        self.control_paused = message.data
        if self.run.recording:
            if message.data:
                self.telemetry.pause(self.ros_seconds())
                self.run.paused = self.run.break_next = True
                self.run.reason = 'PAUSED'
            else:
                self.telemetry.resume(self.ros_seconds())
                self.run.paused = False
                self.run.reason = 'Recording — reference path frozen at start'

    def maybe_start_recording(self):
        """Arm automatically on START, waiting for fresh map-frame observations."""
        if (self.execution_requested and not self.control_paused and self.ready()
                and not self.run.recording and not self.run.samples):
            self.start_recording()

    def stop_capture(self, reason='STOPPED'):
        """Freeze the map trail and both plots as one experiment."""
        if self.run.recording:
            self.run.stop(reason)
            if self.run.samples:
                pose = self.run.samples[-1]
                self.telemetry.append('distance', pose.stamp, reference_distance(
                    pose.x, pose.y, self.run.reference))
        self.telemetry.stop(self.ros_seconds(), reason)

    def receive_scan(self, message):
        """Record the nearest valid laser return at the scan acquisition time."""
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if stamp > 0:
            self.telemetry.append('scan', stamp, scan_minimum(message))
        if self.run.recording and not self.telemetry.recording:
            self.stop_capture(self.telemetry.status)

    def receive_debug(self, message):
        """Shade only cycles with an actual Dijkstra/SMAC avoidance reference."""
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now = self.ros_seconds()
        if stamp <= 0 or not 0 <= now-stamp <= 1.0:
            return
        self.guidance_stamp = stamp
        active = message.guidance_backend.upper() in ('DIJKSTRA', 'SMAC') and bool(
            message.guidance_path)
        self.telemetry.set_avoidance(active, stamp)

    def receive_map(self, message):
        """Keep a running experiment's original map even when live maps change."""
        try:
            grid = MapSnapshot.from_message(message)
        except ValueError as error:
            self.issue = str(error)
            return
        if self.live_map is not None and self.live_map.frame != grid.frame:
            if self.run.recording:
                self.stop_capture('Stopped: complete map frame changed')
            self.latest = self.last_arrival = None
            self.pending.clear()
        self.live_map = grid
        self.revision += 1
        if self.path_message is not None:
            self.receive_path(self.path_message)

    def receive_path(self, message):
        """Display the live plan before recording; freeze its exact points at start."""
        self.path_message = message
        if not message.poses:
            self.reference = ()
            self.revision += 1
            return
        if self.live_map is None:
            return
        try:
            if message.header.frame_id.lstrip('/') != self.live_map.frame:
                raise ValueError('Global path frame does not match complete map frame')
            points = []
            for pose in message.poses:
                if (pose.header.frame_id
                        and pose.header.frame_id.lstrip('/') != self.live_map.frame):
                    raise ValueError('Global path contains a different pose frame')
                points.append((pose.pose.position.x, pose.pose.position.y))
            if len(points) < 2 or not np.all(np.isfinite(points)):
                raise ValueError('Global path needs at least two finite positions')
            self.reference = tuple(points)
            self.issue = ''
            self.revision += 1
        except ValueError as error:
            self.reference = ()
            self.issue = str(error)

    def receive_odom(self, message):
        """Queue briefly for matching TF instead of substituting the latest transform."""
        stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
        if stamp == 0:
            self.issue = 'Odometry timestamp is zero'
            return
        if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
            if stamp < self.last_odom_stamp-1000000000:
                if self.run.recording:
                    self.stop_capture('Stopped: odometry clock moved backwards')
                self.pending.clear()
                self.latest = self.last_arrival = None
            else:
                self.dropped += 1
                return
        self.last_odom_stamp = stamp
        if len(self.pending) >= 100:
            self.pending.popleft()
            self.dropped += 1
        self.pending.append((message, time.monotonic()))
        self.drain_pending()

    def receive_goal(self, message):
        """Keep the requested goal position and orientation for the map arrow."""
        self.goal_message = message
        self.revision += 1

    def goal_pose(self, grid):
        """Resolve the selected goal in map coordinates, or use the plan endpoint."""
        if grid is None:
            return None
        message = self.goal_message
        if message is None:
            if self.path_message is None or not self.reference:
                return None
            message = PoseStamped()
            message.header = self.path_message.header
            message.pose = self.path_message.poses[-1].pose
        try:
            x, y = message.pose.position.x, message.pose.position.y
            yaw = quaternion_yaw(message.pose.orientation)
            if not message.header.frame_id:
                return None
            if message.header.frame_id.lstrip('/') != grid.frame:
                tf = self.tf_buffer.lookup_transform(
                    grid.frame, message.header.frame_id,
                    Time.from_msg(message.header.stamp)).transform
                angle = quaternion_yaw(tf.rotation)
                c, s = math.cos(angle), math.sin(angle)
                x, y = tf.translation.x+c*x-s*y, tf.translation.y+s*x+c*y
                yaw += angle
            if all(math.isfinite(value) for value in (x, y, yaw)):
                return x, y, yaw
        except (TransformException, ValueError):
            pass
        return None

    def drain_pending(self):
        """Retry TF non-blockingly on the Qt/ROS thread, preserving sample order."""
        if self.guidance_stamp is not None and self.ros_seconds()-self.guidance_stamp > 1.0:
            self.telemetry.set_avoidance(False, self.guidance_stamp+1.0)
            self.guidance_stamp = None
        if self.live_map is None:
            return
        for _ in range(min(20, len(self.pending))):
            message, arrival = self.pending[0]
            try:
                transform = (0.0, 0.0, 0.0)
                if message.header.frame_id.lstrip('/') != self.live_map.frame:
                    stamped = self.tf_buffer.lookup_transform(
                        self.live_map.frame, message.header.frame_id,
                        Time.from_msg(message.header.stamp))
                    t, q = stamped.transform.translation, stamped.transform.rotation
                    transform = (t.x, t.y, quaternion_yaw(q))
                pose = map_pose(message, transform, self.settings['base_frame'])
            except TransformException:
                if time.monotonic()-arrival < 0.5:
                    self.issue = 'Waiting for map transform at the odometry timestamp'
                    return
                self.pending.popleft()
                self.dropped += 1
                continue
            except ValueError as error:
                self.pending.popleft()
                self.dropped += 1
                self.issue = str(error)
                continue
            self.pending.popleft()
            self.latest = pose
            self.last_arrival = time.monotonic()
            self.maybe_start_recording()
            if self.run.append(pose):
                self.telemetry.append('distance', pose.stamp, reference_distance(
                    pose.x, pose.y, self.run.reference))
            if not self.run.recording:
                self.telemetry.stop(pose.stamp, self.run.reason)
            self.issue = ''
            self.revision += 1

    def ready(self):
        """A fresh measured pose and global reference are required to record."""
        return (self.live_map is not None and len(self.reference) >= 2
                and self.latest is not None and self.last_arrival is not None
                and time.monotonic()-self.last_arrival <= 0.5)

    def start_recording(self):
        """Start this observer only; robot execution remains controlled separately."""
        if not self.ready():
            raise ValueError('Waiting for complete map, global path and fresh map-frame odometry')
        self.run.start(self.live_map, self.reference)
        self.run_goal = self.goal_pose(self.live_map)
        self.telemetry.start(self.latest.stamp)
        self.run.paused = self.control_paused
        self.run.append(self.latest)
        self.telemetry.append('distance', self.latest.stamp, reference_distance(
            self.latest.x, self.latest.y, self.run.reference))
        if self.control_paused:
            self.telemetry.pause(self.ros_seconds())


class ControlExperimentWindow(QWidget):
    """Record, inspect and export the experiment's first output as a PNG image."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Control Experiment')
        self.resize(1180, 980)
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.start_button = QPushButton('Start recording')
        self.start_button.clicked.connect(self.start_recording)
        self.stop_button = QPushButton('Stop recording')
        self.stop_button.clicked.connect(self.stop_recording)
        self.clear_button = QPushButton('Clear run')
        self.clear_button.clicked.connect(self.clear_run)
        fit = QPushButton('Fit complete map')
        fit.clicked.connect(self.fit_map)
        self.save_button = QPushButton('Save image…')
        self.save_button.clicked.connect(self.save_image)
        for button in (
                self.start_button, self.stop_button, self.clear_button, fit, self.save_button):
            controls.addWidget(button)
        controls.addStretch()
        layout.addLayout(controls)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.figure = Figure(figsize=(10, 8), constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        layout.addWidget(self.canvas, 1)
        panels = self.figure.add_gridspec(3, 1, height_ratios=(3, 1, 1))
        map_panels = panels[0, 0].subgridspec(1, 2, width_ratios=(1, 0.035))
        self.axes = self.figure.add_subplot(map_panels[0, 0])
        colorbar_axes = self.figure.add_subplot(map_panels[0, 1])
        self.plot_axes = tuple(self.figure.add_subplot(panels[i+1, 0]) for i in range(2))
        self.plot_lines = []
        for axes, label, color in zip(
                self.plot_axes, ('Distance [m]', 'Range [m]'),
                ('#2864a0', '#287d5c')):
            line, = axes.plot([], [], color=color, linewidth=1.5)
            self.plot_lines.append(line)
            axes.set_ylabel(label)
            axes.set_xlabel('Active execution time [s]')
            axes.grid(True, alpha=0.2)
        self.plot_axes[0].set_xlabel('')
        self.plot_axes[0].tick_params(labelbottom=False)
        self.plot_axes[0].legend(handles=[
            Patch(facecolor='red', alpha=0.15, label='Dijkstra / SMAC active'),
            Line2D([], [], color='black', label='Pause')], loc='upper right', fontsize=8)
        self.plot_marks = []
        self.telemetry_key = None
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.set_facecolor('#e1e5eb')
        self.axes.set_title('Map and goal')
        self.goal_arrow = FancyArrowPatch(
            (0, 0), (0, 0), arrowstyle='-|>', mutation_scale=20,
            color='#d04432', linewidth=2.5, zorder=8, visible=False)
        self.axes.add_patch(self.goal_arrow)
        self.cmap = cm.get_cmap('rainbow')
        self.norm = colors.Normalize(0, node.settings['speed_scale_max'], clip=True)
        scale = cm.ScalarMappable(norm=self.norm, cmap=self.cmap)
        self.colorbar = self.figure.colorbar(scale, cax=colorbar_axes, extend='max')
        self.colorbar.set_label('Measured planar speed |v| [m/s]')
        self.reference_line, = self.axes.plot([], [], '--', color='black', linewidth=1.7, zorder=5)
        self.reference_line.set_path_effects([
            patheffects.Stroke(linewidth=3.5, foreground='white'), patheffects.Normal()])
        self.latest_polygon = Polygon(
            np.zeros((3, 2)), closed=True, fill=False,
            edgecolor='black', linewidth=1.8, zorder=6, visible=False)
        self.axes.add_patch(self.latest_polygon)
        legend = [Line2D([], [], color='black', linestyle='--', label='Global reference'),
                  Line2D([], [], color='#a330b0', label='Estimated executed trajectory'),
                  Patch(facecolor='#a330b0', alpha=0.4, label='Robot footprint, colored by speed'),
                  Patch(facecolor='none', edgecolor='black', label='Latest displayed footprint'),
                  Line2D([], [], color='#d04432', marker='>', label='Goal')]
        self.axes.legend(handles=legend, loc='upper left', fontsize=8)
        self.details = QLabel()
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.saved = QLabel('Save image exports the map and both time plots as a 300 DPI PNG.')
        self.saved.setWordWrap(True)
        layout.addWidget(self.saved)
        self.image = self.trail = self.polygons = None
        self.map_snapshot = self.render_key = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(round(1000/node.settings['refresh_hz']))
        self.refresh()

    def start_recording(self):
        """Capture a fresh reference and the first measured pose."""
        try:
            self.node.start_recording()
        except ValueError as error:
            self.node.issue = str(error)
        self.refresh()

    def stop_recording(self):
        """Freeze the output while the robot may continue publishing observations."""
        self.node.stop_capture()
        self.refresh()

    def clear_run(self):
        """Explicitly clear the local experiment, retaining live input subscriptions."""
        self.node.run.clear()
        self.node.telemetry.clear()
        self.telemetry_key = None
        self.render_key = None
        self.refresh()

    def fit_map(self):
        """Restore the complete map without following the robot during capture."""
        if self.map_snapshot is not None:
            x0, x1, y0, y1 = self.map_snapshot.bounds()
            pad = 0.03*max(x1-x0, y1-y0)
            self.axes.set_xlim(x0-pad, x1+pad)
            self.axes.set_ylim(y0-pad, y1+pad)
            self.canvas.draw_idle()

    def save_image(self):
        """Ask for the location of the first experiment output."""
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save control experiment image',
            'control_experiment_{}.png'.format(datetime.now().strftime('%Y%m%d_%H%M%S')),
            'PNG image (*.png)')
        if path:
            try:
                self.export_image(path)
            except (OSError, ValueError) as error:
                QMessageBox.critical(self, 'Image export failed', str(error))

    def export_image(self, path):
        """Export the currently inspected view, with its legend and speed scale."""
        if not self.node.run.samples:
            raise ValueError('Record robot motion before exporting the image')
        path = Path(path).with_suffix('.png')
        self.refresh()
        self.figure.savefig(str(path), dpi=300, facecolor='white')
        self.saved.setText('Saved image: {}'.format(path))
        return path

    def refresh(self):
        """Render batched polygons/segments at a bounded desktop refresh rate."""
        node, run = self.node, self.node.run
        self.start_button.setEnabled(node.ready() and not run.recording and not run.samples)
        self.stop_button.setEnabled(run.recording)
        self.clear_button.setEnabled(not run.recording and bool(run.samples))
        self.save_button.setEnabled(bool(run.samples))
        waiting = ''
        if run.recording and (node.last_arrival is None
                              or time.monotonic()-node.last_arrival > 0.5):
            waiting = ' | Waiting for fresh timestamped odometry'
        elif not run.samples and not node.ready():
            waiting = ' | Waiting for map, global path or fresh timestamped pose'
        reason = ('Recording stopped' if run.reason in ('SUCCESS', 'ABORT', 'ABORTED', 'ERROR')
                  else run.reason)
        self.status.setText('{} | Samples: {} | Dropped input: {}{}{}'.format(
            reason, len(run.samples), node.dropped, waiting,
            ' | '+node.issue if node.issue else ''))
        self.render_telemetry()
        key = (run.revision, node.revision if run.map is None else 0)
        if key == self.render_key:
            return
        self.render_key = key
        grid = run.map or node.live_map
        reference = run.reference if run.map is not None else node.reference
        goal = node.run_goal if run.map is not None else node.goal_pose(grid)
        self.goal_arrow.set_visible(goal is not None)
        if goal is not None:
            x, y, yaw = goal
            x0, x1, y0, y1 = grid.bounds()
            length = 0.06*max(x1-x0, y1-y0)
            self.goal_arrow.set_positions(
                (x, y), (x+length*math.cos(yaw), y+length*math.sin(yaw)))
        if grid is not None and grid is not self.map_snapshot:
            fit = self.map_snapshot is None or grid.frame != self.map_snapshot.frame
            if self.image is not None:
                self.image.remove()
            h, w = grid.values.shape
            x, y, yaw = grid.origin
            self.image = self.axes.imshow(
                grid.image(), origin='lower', interpolation='nearest', zorder=0,
                extent=(0, w*grid.resolution, 0, h*grid.resolution),
                transform=Affine2D().rotate(yaw).translate(x, y)+self.axes.transData)
            self.map_snapshot = grid
            self.axes.set_xlabel('{} X [m]'.format(grid.frame))
            self.axes.set_ylabel('{} Y [m]'.format(grid.frame))
            if fit:
                self.fit_map()
        self.reference_line.set_data(
            [p[0] for p in reference], [p[1] for p in reference])
        for artist in (self.trail, self.polygons):
            if artist is not None:
                artist.remove()
        self.trail = self.polygons = None
        if run.samples:
            rgba = self.cmap(self.norm([p.speed for p in run.samples]))
            fill = rgba.copy()
            fill[:, 3] = 0.15
            self.polygons = PolyCollection(
                [footprint_vertices(p, run.footprint) for p in run.samples],
                facecolors=fill, edgecolors=rgba, linewidths=0.6, zorder=2)
            self.axes.add_collection(self.polygons, autolim=False)
            segments, speeds = [], []
            for a, b in zip(run.samples, run.samples[1:]):
                if a.segment == b.segment:
                    segments.append([(a.x, a.y), (b.x, b.y)])
                    speeds.append((a.speed+b.speed)/2)
            self.trail = LineCollection(segments, colors=self.cmap(self.norm(speeds)),
                                        linewidths=2.2, zorder=3)
            self.axes.add_collection(self.trail, autolim=False)
        latest = run.samples[-1] if run.samples else node.latest
        self.latest_polygon.set_visible(latest is not None)
        if latest is not None:
            self.latest_polygon.set_xy(footprint_vertices(latest, run.footprint))
        duration = run.samples[-1].stamp-run.samples[0].stamp if len(run.samples) > 1 else 0
        self.details.setText(
            'Pose: EKF odometry transformed to map at each sample timestamp | '
            'Color: measured speed magnitude (forward and reverse share the scale) | '
            'Duration: {:.1f} s | Reference is frozen during recording'.format(duration))
        self.canvas.draw_idle()

    def render_telemetry(self):
        """Plot acquisition-time data, avoidance intervals and black pause markers."""
        telemetry = self.node.telemetry
        if self.telemetry_key == telemetry.revision:
            return
        self.telemetry_key = telemetry.revision
        for artist in self.plot_marks:
            artist.remove()
        self.plot_marks = []
        for axes, line, values, title in zip(
                self.plot_axes, self.plot_lines, (telemetry.distance, telemetry.scan),
                ('Distance to global reference', 'Nearest laser return')):
            line.set_data([p[0] for p in values], [p[1] for p in values])
            axes.set_title(title + (' — PAUSED' if telemetry.status == 'PAUSED' else ''), loc='left')
            axes.set_xlim(0, max(1.0, telemetry.end*1.03))
            axes.relim()
            axes.autoscale_view(scalex=False)
            axes.set_ylim(0, max(0.1, axes.get_ylim()[1]))
            for elapsed in telemetry.pauses:
                self.plot_marks.append(axes.axvline(elapsed, color='black', linewidth=1.8))
        for start, end in telemetry.avoidance:
            self.plot_marks.append(self.plot_axes[0].axvspan(
                start, telemetry.end if end is None else end, color='red', alpha=0.15))
        self.canvas.draw_idle()


def main(args=None):
    """Run the read-only experiment client on the desktop Qt event loop."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = ControlExperimentNode()
    window = ControlExperimentWindow(node)
    spin_timer = QTimer()

    def spin_ros():
        # Drain high-rate TF/odometry without giving a callback exclusive use of Qt.
        deadline = time.perf_counter()+0.003
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0)
            if time.perf_counter() >= deadline:
                break

    spin_timer.timeout.connect(spin_ros)
    spin_timer.start(20)
    previous = {sig: signal.signal(sig, lambda *_: app.quit())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    window.show()
    try:
        app.exec_()
    finally:
        spin_timer.stop()
        window.timer.stop()
        window.close()
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
