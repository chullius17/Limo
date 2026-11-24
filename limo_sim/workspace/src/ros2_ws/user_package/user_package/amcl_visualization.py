"""Desktop-only complete-map view of AMCL pose and covariance over time."""

import math
import signal
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib import cm, colors
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.collections import EllipseCollection
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Patch
from matplotlib.transforms import Affine2D
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from user_package.amcl_history import LocalizationHistory, MapSnapshot
from user_package.amcl_stats_model import pose_record


class AmclVisualizationNode(Node):
    """Read retained map/AMCL messages without starting localization or control."""

    def __init__(self, **kwargs):
        super().__init__('amcl_visualization', **kwargs)
        defaults = {'map_topic': '/map', 'pose_topic': '/amcl_pose',
                    'max_samples': 5000, 'sample_period': 0.25, 'refresh_hz': 4.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        rate = self.settings['refresh_hz']
        if not math.isfinite(rate) or not 0 < rate <= 30:
            raise ValueError('refresh_hz must be in (0, 30]')
        self.history = LocalizationHistory(
            self.settings['max_samples'], self.settings['sample_period'])
        self.pending_pose = None
        self.issue = ''
        self.last_arrival = None
        # Both publishers in the robot application retain their latest sample.
        # A desktop opened after localization has started also sees a stopped robot.
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, self.settings['map_topic'], self.receive_map, retained)
        self.create_subscription(
            PoseWithCovarianceStamped, self.settings['pose_topic'], self.receive_pose, retained)

    def receive_map(self, message):
        """Replace the full map atomically, preserving the last valid map on error."""
        try:
            self.history.set_map(MapSnapshot.from_message(message))
        except ValueError as error:
            self.issue = str(error)
            return
        self.issue = ''
        if self.pending_pose is not None:
            self._append_pose(self.pending_pose)

    def receive_pose(self, message):
        """Validate the published covariance, not a covariance inferred from motion."""
        try:
            row = pose_record(message)
        except (ValueError, TypeError, OverflowError) as error:
            self.issue = str(error)
            return
        self.pending_pose = row
        self._append_pose(row)

    def _append_pose(self, row):
        try:
            if self.history.append(row):
                self.last_arrival = time.monotonic()
                self.issue = ''
        except ValueError as error:
            self.issue = str(error)


class AmclVisualizationWindow(QWidget):
    """Keep the complete map visible and overlay metric covariance circles."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('AMCL Localization History')
        self.resize(1150, 850)
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        fit = QPushButton('Fit complete map')
        fit.clicked.connect(self.fit_map)
        clear = QPushButton('Clear history')
        clear.clicked.connect(self.clear_history)
        controls.addWidget(fit)
        controls.addWidget(clear)
        controls.addStretch()
        self.status = QLabel('Waiting for complete map and AMCL pose')
        self.status.setWordWrap(True)
        controls.addWidget(self.status, 1)
        layout.addLayout(controls)
        self.figure = Figure(figsize=(10, 7), constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        layout.addWidget(self.canvas, 1)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.set_title('AMCL position and XY uncertainty on the complete robot map')
        self.axes.set_xlabel('Map X [m]')
        self.axes.set_ylabel('Map Y [m]')
        self.axes.set_facecolor('#e1e5eb')
        self.colormap = cm.get_cmap('rainbow')
        self.color_scale = cm.ScalarMappable(norm=colors.Normalize(0, 1), cmap=self.colormap)
        self.colorbar = self.figure.colorbar(self.color_scale, ax=self.axes, pad=0.025)
        self.colorbar.set_label('Elapsed AMCL time [s] — oldest to newest')
        legend = [Patch(facecolor='white', edgecolor='grey', label='Free'),
                  Patch(facecolor='black', label='Occupied / high map cost'),
                  Patch(facecolor='#8c96a3', label='Unknown'),
                  Line2D([], [], color='#a330b0', marker='o', markerfacecolor='none',
                         linestyle='none', label='95% XY covariance bound'),
                  Line2D([], [], color='black', marker='+', linestyle='none',
                         label='Latest pose and heading')]
        self.axes.legend(handles=legend, loc='upper center', bbox_to_anchor=(0.5, -0.10),
                         ncol=3, fontsize=9)
        self.details = QLabel('Circle radius = 2.448 × largest XY standard deviation [m]. '
                              'Encloses the 95% Gaussian covariance ellipse; '
                              'heading uncertainty is reported separately.')
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.pose_details = QLabel('No AMCL pose received')
        self.pose_details.setWordWrap(True)
        layout.addWidget(self.pose_details)
        self.image = self.circles = None
        self.map_snapshot = None
        self.revision = -1
        self.centers, = self.axes.plot([], [], '.', markersize=2, zorder=3)
        self.latest_marker, = self.axes.plot(
            [], [], '+', color='black', markersize=12, markeredgewidth=2, zorder=5)
        self.heading = FancyArrowPatch(
            (0, 0), (0, 0), arrowstyle='-|>', mutation_scale=14,
            linewidth=1.8, color='black', zorder=5, visible=False)
        self.axes.add_patch(self.heading)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(1, round(1000/node.settings['refresh_hz'])))
        self.refresh()

    def fit_map(self):
        """Fit the complete map rather than following the current robot pose."""
        if self.map_snapshot is None:
            return
        x0, x1, y0, y1 = self.map_snapshot.bounds()
        margin = max(x1-x0, y1-y0)*0.03
        self.axes.set_xlim(x0-margin, x1+margin)
        self.axes.set_ylim(y0-margin, y1+margin)
        self.canvas.draw_idle()

    def clear_history(self):
        """Clear only this desktop's history; leave AMCL and the robot untouched."""
        self.node.history.clear()
        self.node.pending_pose = None
        self.node.last_arrival = None
        self.refresh()

    def refresh(self):
        """Redraw only when data change, retaining manual pan and zoom."""
        history = self.node.history
        if self.node.issue:
            self.status.setText('Input rejected: ' + self.node.issue)
        elif history.map is None:
            self.status.setText('Waiting for complete map: ' + self.node.settings['map_topic'])
        elif history.latest is None:
            self.status.setText('Waiting for AMCL pose: ' + self.node.settings['pose_topic'])
        else:
            age = time.monotonic() - self.node.last_arrival
            self.status.setText('Pose age: {:.1f} s | Frame: {} | Circles: {}/{}'.format(
                age, history.map.frame, len(history.samples), self.node.settings['max_samples']))
        if self.revision == history.revision:
            return
        self.revision = history.revision
        grid = history.map
        if grid is not None and grid is not self.map_snapshot:
            first_map = self.map_snapshot is None or grid.frame != self.map_snapshot.frame
            if self.image is not None:
                self.image.remove()
            height, width = grid.values.shape
            x, y, yaw = grid.origin
            self.image = self.axes.imshow(
                grid.image(), origin='lower', interpolation='nearest', zorder=0,
                extent=(0, width*grid.resolution, 0, height*grid.resolution),
                transform=Affine2D().rotate(yaw).translate(x, y) + self.axes.transData)
            self.map_snapshot = grid
            self.axes.set_xlabel('{} X [m]'.format(grid.frame))
            self.axes.set_ylabel('{} Y [m]'.format(grid.frame))
            if first_map:
                self.fit_map()
        if self.circles is not None:
            self.circles.remove()
            self.circles = None
        samples = list(history.samples)
        if samples:
            elapsed = np.array([p.elapsed for p in samples])
            norm = colors.Normalize(elapsed[0], max(elapsed[0]+1.0, elapsed[-1]))
            rgba = self.colormap(norm(elapsed))
            fill = rgba.copy()
            fill[:, 3] = 0.07
            rgba[:, 3] = 0.75
            positions = np.array([(p.x, p.y) for p in samples])
            diameters = np.array([2*p.radius for p in samples])
            self.circles = EllipseCollection(
                diameters, diameters, np.zeros(len(samples)), units='xy',
                offsets=positions, transOffset=self.axes.transData,
                facecolors=fill, edgecolors=rgba, linewidths=0.8, zorder=2)
            self.axes.add_collection(self.circles, autolim=False)
            self.centers.set_data(positions[:, 0], positions[:, 1])
            # Covariance circles carry individual time colors; pose means stay subtle.
            self.centers.set_color('#515964')
            self.color_scale.set_norm(norm)
            self.colorbar.update_normal(self.color_scale)
        else:
            self.centers.set_data([], [])
            self.color_scale.set_norm(colors.Normalize(0, 1))
            self.colorbar.update_normal(self.color_scale)
        pose = history.latest
        self.heading.set_visible(pose is not None)
        if pose is not None:
            self.latest_marker.set_data([pose.x], [pose.y])
            self.heading.set_positions((pose.x, pose.y),
                                       (pose.x+0.35*math.cos(pose.yaw),
                                        pose.y+0.35*math.sin(pose.yaw)))
            self.pose_details.setText(
                'Latest pose: X {:.3f} m | Y {:.3f} m | Heading {:.1f}° | '
                'XY bound radius {:.3f} m | Yaw σ {:.1f}° | Clock resets: {}'.format(
                    pose.x, pose.y, math.degrees(pose.yaw), pose.radius,
                    math.degrees(pose.yaw_std), history.clock_resets))
        else:
            self.latest_marker.set_data([], [])
            self.pose_details.setText('No AMCL pose received')
        self.canvas.draw_idle()


def main(args=None):
    """Spin ROS on the Qt thread so no GUI state is shared across threads."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = AmclVisualizationNode()
    window = AmclVisualizationWindow(node)
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
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
