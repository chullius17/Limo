# Copyright 2026 Giulio Cataldo
# Licensed under the Apache License, Version 2.0 (the "License");
# you may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0

"""Native Qt MPC inspector using ROS telemetry and editable vector overlays."""

import math
import signal
import sys
import time

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPainterPath, QPainterPathStroker
from PyQt5.QtGui import QPen, QPixmap, QTransform
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QGraphicsScene, QGraphicsView,
    QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QPushButton, QSplitter, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)
import rclpy

from limo_controller.mpc_preview.node import MpcPreview
from limo_controller.mpc_preview.render import LABELS, jet_color, point_cost_color, world_to_base


SCALE = 100.0


def scene_point(point):
    """Use the same forward-up, left-left convention as the MPC image."""
    return QPointF(-point[1] * SCALE, -point[0] * SCALE)


def polyline(points, closed=False):
    """Build a native path in robot-relative meters."""
    path = QPainterPath()
    if points:
        path.moveTo(scene_point(points[0]))
        for point in points[1:]:
            path.lineTo(scene_point(point))
        if closed:
            path.closeSubpath()
    return path


class MpcView(QGraphicsView):
    """Keep map cells as a raster and every planning overlay as Qt geometry."""

    def __init__(self, options):
        super().__init__()
        self.options = options
        self.setScene(QGraphicsScene(self))
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QColor(*options.background_color))
        self.setMinimumSize(380, 380)
        self.setSceneRect(QRectF(-600, -600, 1200, 1200))
        self.layers = {}
        self.inspected_id = None
        self.snapshot = None
        self.auto_fit = True

    def wheelEvent(self, event):  # noqa: N802
        """Zoom around the cursor without disturbing the incoming scene."""
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        scale = self.transform().m11() * factor
        if 0.08 <= scale <= 40:
            self.scale(factor, factor)
            self.auto_fit = False
        event.accept()

    def mousePressEvent(self, event):  # noqa: N802
        """Retain a manually panned viewport until the user requests centering."""
        self.auto_fit = False
        super().mousePressEvent(event)

    def resizeEvent(self, event):  # noqa: N802
        """Fit the initial viewport while retaining a manually chosen zoom."""
        super().resizeEvent(event)
        if self.auto_fit:
            self.center_robot()

    def center_robot(self):
        """Restore the configured metric viewing area around base_link."""
        options = self.options
        width = options.image_width / options.pixels_per_meter * SCALE
        height = options.image_height / options.pixels_per_meter * SCALE
        self.fitInView(QRectF(-width / 2, -height / 2, width, height), Qt.KeepAspectRatio)
        self.centerOn(0, 0)
        self.auto_fit = True

    def line(self, path, color, width=2, dashed=False, z=2):
        """Add a vector overlay with a constant screen-space stroke."""
        pen = QPen(QColor(*color) if isinstance(color, tuple) else QColor(color))
        pen.setWidthF(width)
        pen.setCosmetic(True)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        if dashed:
            pen.setStyle(Qt.DashLine)
        item = self.scene().addPath(path, pen)
        item.setZValue(z)
        return item

    def outline(self, path, radius, color, width=2, filled=False):
        """Outline the union of round capsules, preserving the clear interior."""
        stroker = QPainterPathStroker()
        stroker.setWidth(2 * radius * SCALE)
        stroker.setCapStyle(Qt.RoundCap)
        stroker.setJoinStyle(Qt.RoundJoin)
        outline = stroker.createStroke(path)
        if filled:
            outline = outline.united(path)
        self.line(outline.simplified(), color, width)

    def map_cells(self, view):
        """Place the original costmap cells using its full SE2 origin transform."""
        width, height = view.costmap_size
        if (width <= 0 or height <= 0 or len(view.costmap_data) != width * height
                or not math.isfinite(view.costmap_resolution) or view.costmap_resolution <= 0):
            raise ValueError('Invalid costmap dimensions, resolution or data')
        image = QImage(view.costmap_data, width, height, width, QImage.Format_Indexed8).copy()
        options = self.options
        palette = []
        for cost in range(256):
            if cost == 255:
                color = options.unknown_color
            elif cost == 254:
                color = options.lethal_color
            elif options.costmap_colormap == 'jet':
                color = jet_color(cost)
            elif cost == 253:
                color = options.critical_color
            else:
                color = tuple(round(a + cost / 252 * (b - a))
                              for a, b in zip(options.free_color, options.inflated_color))
            palette.append(QColor(*color).rgb())
        image.setColorTable(palette)
        item = self.scene().addPixmap(QPixmap.fromImage(image))
        item.setTransformationMode(Qt.FastTransformation)
        ox, oy, yaw = view.costmap_origin
        origin = scene_point(world_to_base((ox, oy), view.robot_pose))
        angle = yaw - view.robot_pose[2]
        s, c = math.sin(angle), math.cos(angle)
        unit = view.costmap_resolution * SCALE
        item.setTransform(QTransform(-s * unit, -c * unit, -c * unit, s * unit,
                                     origin.x(), origin.y()))
        item.setZValue(-2)

    def update_scene(self, snapshot, layers=None):
        """Replace the latest snapshot without resetting pan, zoom or selection."""
        self.snapshot = snapshot
        if layers is not None:
            self.layers = layers
        frame, _, background, global_path, sources = snapshot
        view = frame if frame is not None else background
        options = self.options
        self.scene().clear()
        if sources is None and view is not None and self.layers.get('map', True):
            self.map_cells(view)
        spacing = options.grid_spacing * SCALE
        for index in range(-20, 21):
            coordinate = index * spacing
            grid = QPainterPath(QPointF(coordinate, -20 * spacing))
            grid.lineTo(coordinate, 20 * spacing)
            grid.moveTo(-20 * spacing, coordinate)
            grid.lineTo(20 * spacing, coordinate)
            self.line(grid, '#d2d5db', 0.5, z=-1)
        if sources is not None and self.layers.get('map', True):
            # Batch source points by color instead of allocating a Qt item per point.
            batches = {}
            for points, color, laser in ((sources.laser, options.laser_color, True),
                                         (sources.semantic, options.semantic_color, False)):
                for point in points:
                    rgb = color
                    if options.source_cost_colors:
                        cost = (options.source_laser_cost if laser else
                                options.source_class_costs[point[2]])
                        rgb = point_cost_color(cost, options)
                    path = batches.setdefault(rgb, QPainterPath())
                    p = scene_point(point)
                    path.moveTo(p)
                    path.lineTo(p + QPointF(0.01, 0.01))
            for color, path in batches.items():
                self.line(path, color, options.costmap_point_size, z=0)
        if global_path and self.layers.get('path', True):
            path = polyline(global_path)
            if options.tube_radius > 0 and self.layers.get('tube', True):
                self.outline(path, options.tube_radius, '#ffffff', options.tube_line_width)
            self.line(path, '#ffffff', 6)
            self.line(path, options.global_path_color, 3)
            end = scene_point(global_path[-1])
            r = options.goal_circle_radius * SCALE
            if r > 0:
                goal = QPainterPath()
                goal.addEllipse(end, r, r)
                self.line(goal, options.global_path_color, 2)
        if frame is not None:
            self.draw_candidates(frame)
            if self.layers.get('guidance', True):
                reference = tuple(world_to_base(p, frame.robot_pose) for p in frame.guidance_path)
                self.line(polyline(reference), '#ffffff', 7, z=4)
                self.line(polyline(reference), options.guidance_path_color, 3, z=4)
                for pose, color, radius in (
                        (frame.path_lookahead, options.path_lookahead_color, 0.10),
                        (frame.guidance_lookahead, options.guidance_lookahead_color, 0.07),
                        (frame.guidance_candidate_target, options.guidance_target_color, 0.14),
                        (frame.guidance_target, options.guidance_target_color, 0.04)):
                    if pose is not None:
                        marker = QPainterPath()
                        marker.addEllipse(scene_point(world_to_base(pose, frame.robot_pose)),
                                          radius * SCALE, radius * SCALE)
                        self.line(marker, color, 3, z=5)
        footprint = (view.footprint if view is not None else ())
        if footprint and self.layers.get('robot', True):
            polygon = polyline(footprint, closed=True)
            if options.trigger_margin > 0 or options.trigger_front_extension > 0:
                front = max(p[0] for p in footprint)
                extended = tuple((x + options.trigger_front_extension
                                  if abs(x - front) < 1e-6 else x, y) for x, y in footprint)
                region = polyline(extended, closed=True)
                if options.trigger_margin > 0:
                    self.outline(region, options.trigger_margin,
                                 options.trigger_region_color, filled=True)
                else:
                    self.line(region, options.trigger_region_color)
            self.line(polygon, '#ffffff', 7, z=6)
            self.line(polygon, '#141e2d', 3, z=6)
        for endpoint, color in (((options.axis_length, 0), '#c32323'),
                                ((0, options.axis_length), '#238237')):
            self.line(polyline(((0, 0), endpoint)), color, 3, z=7)

    def draw_candidates(self, frame):
        """Render candidates with rejection state and selectable inspection."""
        options = self.options
        for candidate in frame.candidates:
            selected = candidate.candidate_id == frame.selected_id
            inspected = candidate.candidate_id == self.inspected_id
            if not selected and not inspected:
                if not self.layers.get('candidates', True):
                    continue
                if candidate.score_status == 2 and not self.layers.get('rejected', True):
                    continue
                if candidate.score_status == 0 and not self.layers.get('pruned', False):
                    continue
            if not 0 <= candidate.family < len(options.colors):
                continue
            points = tuple(world_to_base(p, frame.robot_pose) for p in candidate.poses)
            path = polyline(points)
            color = options.selected_color if selected else options.colors[candidate.family]
            width, z = (options.selected_line_width, 5) if selected else (options.line_width, 3)
            if selected or inspected:
                self.line(path, '#ffffff', width + 4, z=5)
            item = self.line(path, '#00d5ff' if inspected else color,
                             width + 1 if inspected else width,
                             candidate.score_status != 1, z=6 if inspected else z)
            if not selected and not inspected:
                item.setOpacity(options.trajectory_alpha)
            item.setToolTip('ID {} | {} | cost {:.3f}'.format(
                candidate.candidate_id, LABELS[candidate.family], candidate.cost))


class MpcWindow(QWidget):
    """Inspect live local planning with native controls and a candidate table."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('LIMO — MPC')
        self.resize(1250, 900)
        self.view = MpcView(node.options)
        self.status = QLabel('MPC: waiting for telemetry')
        self.status.setWordWrap(True)
        self.metrics = QLabel('Waiting for controller data…')
        self.metrics.setWordWrap(True)
        self.checks = {}
        self.last_frame = None
        self.last_draw = 0.0
        self.frozen = False
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['ID', 'Family', 'State', 'Cost'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(False)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self.inspect_candidate)
        self.mode = QComboBox()
        self.mode.addItem('Costmap MPC', 'costmap')
        self.mode.addItem('Laser + semantic points', 'sources')
        self.mode.setCurrentIndex(0 if node.view_mode == 'costmap' else 1)
        self.mode.currentIndexChanged.connect(self.change_mode)
        center = QPushButton('Center robot')
        center.clicked.connect(self.view.center_robot)
        freeze = QPushButton('Freeze view')
        freeze.setCheckable(True)
        freeze.toggled.connect(self.set_frozen)
        save = QPushButton('Save PNG…')
        save.clicked.connect(self.save_snapshot)
        toolbar = QHBoxLayout()
        for widget in (self.mode, center, freeze, save):
            toolbar.addWidget(widget)
        toolbar.addStretch()
        panel = QWidget()
        side = QVBoxLayout(panel)
        side.addWidget(self.metrics)
        for key, label, default in (
                ('map', 'Map / source points', True), ('path', 'Global path', True),
                ('tube', 'Path tube', True), ('guidance', 'Avoidance reference / targets', True),
                ('robot', 'Footprint / trigger region', True),
                ('candidates', 'Candidate trajectories', True),
                ('rejected', 'Rejected (dashed)', node.options.show_rejected),
                ('pruned', 'Cost pruned (dashed)', node.options.show_cost_pruned)):
            check = QCheckBox(label)
            check.setChecked(default)
            check.toggled.connect(self.redraw)
            self.checks[key] = check
            side.addWidget(check)
        families = QGridLayout()
        for index, (label, rgb) in enumerate(zip(LABELS, node.options.colors)):
            legend = QLabel('━  ' + label)
            legend.setStyleSheet('color: rgb({}, {}, {});'.format(*rgb))
            families.addWidget(legend, index // 2, index % 2)
        side.addLayout(families)
        side.addWidget(QLabel('Select a candidate to highlight its trajectory:'))
        side.addWidget(self.table, 1)
        side.addWidget(QLabel('Grid: {:g} m · Wheel: zoom · Drag: pan\n'
                              'Freeze affects this view only.'.format(node.options.grid_spacing)))
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.view)
        splitter.addWidget(panel)
        splitter.setSizes([880, 350])
        layout = QVBoxLayout(self)
        layout.addLayout(toolbar)
        layout.addWidget(self.status)
        layout.addWidget(splitter, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(20)
        self.view.center_robot()

    def change_mode(self):
        """Switch source layers without changing the controller's configuration."""
        self.node.view_mode = self.mode.currentData()
        if not self.frozen:
            self.last_draw = 0.0

    def set_frozen(self, frozen):
        """Continue receiving ROS while preserving the displayed snapshot."""
        self.frozen = frozen
        if frozen:
            self.status.setText('FROZEN — ' + self.status.text())
        else:
            self.last_draw = 0.0

    def redraw(self):
        """Apply display filters to the exact displayed controller cycle."""
        if self.view.snapshot is not None:
            self.view.update_scene(self.view.snapshot,
                                   {key: check.isChecked() for key, check in self.checks.items()})

    def inspect_candidate(self):
        """Highlight the selected row without changing the MPC winner."""
        row = self.table.currentRow()
        item = self.table.item(row, 0) if row >= 0 else None
        self.view.inspected_id = int(item.text()) if item is not None else None
        self.redraw()

    def save_snapshot(self):
        """Export the current visible viewport when explicitly requested."""
        filename, _ = QFileDialog.getSaveFileName(self, 'Save MPC view', 'mpc.png', 'PNG (*.png)')
        if filename and not self.view.viewport().grab().save(filename, 'PNG'):
            self.status.setText('Cannot save PNG: ' + filename)

    def update_table(self, frame):
        """Keep candidate IDs stable while the controller updates costs."""
        if frame is self.last_frame:
            return
        self.last_frame = frame
        candidates = frame.candidates if frame is not None else ()
        inspected = self.view.inspected_id
        self.table.blockSignals(True)
        self.table.setRowCount(len(candidates))
        row_to_select = None
        for row, candidate in enumerate(candidates):
            family = (LABELS[candidate.family] if 0 <= candidate.family < len(LABELS)
                      else str(candidate.family))
            state = {0: 'Cost pruned', 1: 'Scored', 2: 'Rejected'}.get(
                candidate.score_status, 'Unknown')
            if candidate.candidate_id == frame.selected_id:
                state = 'Selected'
            for column, text in enumerate((str(candidate.candidate_id), family,
                                           state, '{:.3f}'.format(candidate.cost))):
                item = QTableWidgetItem(text)
                if candidate.candidate_id == frame.selected_id:
                    item.setForeground(QColor(*self.node.options.selected_color))
                self.table.setItem(row, column, item)
            if candidate.candidate_id == inspected:
                row_to_select = row
        if row_to_select is not None:
            self.table.selectRow(row_to_select)
        else:
            self.table.clearSelection()
            self.table.setCurrentCell(-1, -1)
            self.view.inspected_id = None
        self.table.blockSignals(False)

    def refresh(self):
        """Drain ROS with a time budget and redraw at the configured rate."""
        if not rclpy.ok():
            self.close()
            return
        deadline = time.monotonic() + 0.005
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if time.monotonic() >= deadline:
                break
        now = time.monotonic()
        if self.frozen or now - self.last_draw < 1 / self.node.get_parameter('publish_rate').value:
            return
        self.last_draw = now
        try:
            snapshot = self.node.scene_snapshot()
            frame, status, _, _, _ = snapshot
            self.update_table(frame)
            self.view.snapshot = snapshot
            self.redraw()
            self.status.setText(status or 'MPC: live telemetry')
            if frame is None:
                self.metrics.setText('Waiting for fresh MPC telemetry.\nMap and path stay live.')
            else:
                self.metrics.setText(
                    'Command: {:.3f} m/s · steering {:.1f}°\n'
                    'Initial: {:.3f} m/s · steering {:.1f}°\n'
                    'Horizon: {} × {:.3f} s\n'
                    'Candidates: {} shown / {} generated\n'
                    'Selected: {}\nAvoidance: {}'.format(
                        frame.command_velocity, math.degrees(frame.command_steering),
                        frame.initial_velocity, math.degrees(frame.initial_steering),
                        frame.time_steps, frame.model_dt, len(frame.candidates),
                        frame.generated_count, frame.selected_id,
                        frame.guidance_backend or 'inactive'))
        except (ValueError, OverflowError) as error:
            self.status.setText('Invalid MPC data: ' + str(error))

    def closeEvent(self, event):  # noqa: N802
        """Stop the refresh loop before ROS resources are destroyed."""
        self.timer.stop()
        super().closeEvent(event)


def main(args=None):
    """Run the standalone GUI without a compressed image publisher."""
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = MpcPreview(gui=True)
    window = MpcWindow(node)

    def shutdown(_signum, _frame):
        window.close()
        app.quit()

    handlers = {sig: signal.signal(sig, shutdown) for sig in (signal.SIGINT, signal.SIGTERM)}
    window.show()
    try:
        app.exec_()
    finally:
        window.timer.stop()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
