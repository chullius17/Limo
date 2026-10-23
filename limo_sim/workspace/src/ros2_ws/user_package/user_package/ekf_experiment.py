"""ROS 2 / Qt viewer for simultaneous raw and EKF odometry experiments."""

import csv
from dataclasses import asdict, fields
from datetime import datetime
import math
from pathlib import Path
import signal
import sys
import threading
import time

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from user_package.ekf_experiment_model import (
    Difference, ExperimentHistory, Sample, compare,
)


class ExperimentNode(Node):
    """Collect both topics without publishing commands, odometry, or TF."""

    def __init__(self):
        super().__init__('ekf_experiment')
        defaults = {
            'raw_topic': '/odom', 'filtered_topic': '/odometry/filtered',
            'history_seconds': 120.0, 'max_samples': 20000,
            'max_sync_gap': 0.2, 'refresh_hz': 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for name in ('history_seconds', 'max_sync_gap', 'refresh_hz'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError('{} must be finite and positive'.format(name))
        if self.settings['max_samples'] < 2:
            raise ValueError('max_samples must be at least 2')
        if self.settings['raw_topic'] == self.settings['filtered_topic']:
            raise ValueError('Choose two different odometry topics')
        self.lock = threading.Lock()
        self.history = ExperimentHistory(
            self.settings['history_seconds'], self.settings['max_samples'])
        self.arrivals = {}
        self.rejected = 0
        self.subscriptions_ = [self.create_subscription(
            Odometry, self.settings[name + '_topic'],
            lambda msg, stream=name: self.receive(stream, msg),
            qos_profile_sensor_data) for name in ('raw', 'filtered')]

    def receive(self, name, msg):
        """Store valid data with its message timestamp, not its receipt time."""
        try:
            sample = Sample.from_odometry(msg)
        except ValueError:
            with self.lock:
                self.rejected += 1
            return
        with self.lock:
            resets = self.history.reset_count
            self.history.append(name, sample)
            if resets != self.history.reset_count:
                self.arrivals.clear()
            self.arrivals[name] = time.monotonic()

    def snapshot(self):
        """Return a consistent copy for plotting and export."""
        with self.lock:
            return (self.history.snapshot(), self.history.origin,
                    dict(self.arrivals), self.rejected, self.history.reset_count)

    def reset(self):
        """Start a new local observation window without resetting the robot."""
        with self.lock:
            self.history.clear()
            self.arrivals.clear()
            self.rejected = 0


def _write_csv(path, row_type, rows):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=[f.name for f in fields(row_type)])
        writer.writeheader()
        writer.writerows(asdict(row) for row in rows)


def export_snapshot(directory, figure, streams, differences):
    """Save exactly the displayed window; timestamps are absolute ROS seconds."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    for name, rows in streams.items():
        _write_csv(directory / (name + '.csv'), Sample, rows)
    _write_csv(directory / 'differences.csv', Difference, differences)
    figure.savefig(str(directory / 'odometry.png'), dpi=150)


class ExperimentWindow(QMainWindow):
    """Show trajectories, dynamics, and synchronized differences in six plots."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.paused = False
        self.displayed = None
        self.setWindowTitle('Esperimento EKF — confronto odometrie')
        self.resize(1400, 900)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        settings = node.settings
        title = QLabel('Odometria: {}    |    EKF: {}'.format(
            settings['raw_topic'], settings['filtered_topic']))
        title.setWordWrap(True)
        title.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout.addWidget(title)
        self.status = QLabel('In attesa delle due odometrie…')
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout.addWidget(self.status)
        controls = QHBoxLayout()
        self.pause_button = QPushButton('Pausa grafici')
        self.pause_button.clicked.connect(self.toggle_pause)
        controls.addWidget(self.pause_button)
        reset_button = QPushButton('Nuova prova / reset grafici')
        reset_button.clicked.connect(self.reset)
        controls.addWidget(reset_button)
        self.export_button = QPushButton('Esporta CSV + PNG')
        self.export_button.clicked.connect(self.export)
        self.export_button.setEnabled(False)
        controls.addWidget(self.export_button)
        controls.addStretch()
        layout.addLayout(controls)
        self.figure = Figure(constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        layout.addWidget(self.canvas, 1)
        self.axes = self.figure.subplots(3, 2).flatten()
        labels = [
            ('Traiettoria XY', 'x [m]', 'y [m]'),
            ('Orientamento', 'tempo ROS dalla partenza [s]', 'yaw [°]'),
            ('Velocità longitudinale', 'tempo [s]', 'vx [m/s]'),
            ('Velocità angolare', 'tempo [s]', 'wz [°/s]'),
            ('Distanza tra le stime sincronizzate', 'tempo [s]', 'distanza [m]'),
            ('Differenza yaw: EKF − odometria', 'tempo [s]', 'Δyaw [°]'),
        ]
        self.lines = {}
        for i, (ax, (title, xlabel, ylabel)) in enumerate(zip(self.axes, labels)):
            ax.set(title=title, xlabel=xlabel, ylabel=ylabel)
            ax.grid(True, alpha=0.3)
            if i < 4:
                for name, label, color in (
                        ('raw', 'Odometria', '#d97706'),
                        ('filtered', 'EKF', '#2563eb')):
                    self.lines[i, name], = ax.plot([], [], label=label, color=color)
                ax.legend(loc='upper right')
            else:
                self.lines[i, 'difference'], = ax.plot([], [], color='#7c3aed')
        self.axes[0].set_aspect('equal', adjustable='box')
        footer = QLabel(
            'Le differenze misurano il disaccordo tra stime, non l’errore reale. '
            'CSV/PNG contengono solo la finestra conservata (massimo {:.0f} s / {} campioni).'
            .format(settings['history_seconds'], settings['max_samples']))
        footer.setWordWrap(True)
        footer.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout.addWidget(footer)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(20, round(1000 / settings['refresh_hz'])))

    def toggle_pause(self):
        """Freeze rendering while subscriptions continue collecting data."""
        self.paused = not self.paused
        self.pause_button.setText('Riprendi grafici' if self.paused else 'Pausa grafici')
        if not self.paused:
            self.refresh()

    def reset(self):
        """Discard the local experiment and resume rendering."""
        self.node.reset()
        self.paused = False
        self.pause_button.setText('Pausa grafici')
        self.refresh()

    def refresh(self):
        """Interpolate only across short intervals and compatible frame IDs."""
        if self.paused:
            return
        streams, origin, arrivals, rejected, resets = self.node.snapshot()
        differences = compare(
            streams['raw'], streams['filtered'], self.node.settings['max_sync_gap'])
        self.displayed = (streams, differences)
        self.export_button.setEnabled(any(streams.values()))
        origin = origin if origin is not None else 0.0
        for name, samples in streams.items():
            # Limit rendering cost; synchronization and CSV retain every sample.
            stride = max(1, math.ceil(len(samples) / 2500))
            visible = samples[::stride]
            if samples and visible[-1] is not samples[-1]:
                visible.append(samples[-1])
            times = [s.stamp - origin for s in visible]
            self.lines[0, name].set_data([s.x for s in visible], [s.y for s in visible])
            yaw = [math.degrees(s.yaw) for s in visible]
            # Avoid drawing a line across the +/-180 degree discontinuity.
            for i in range(len(yaw) - 1, 0, -1):
                if abs(yaw[i] - yaw[i - 1]) > 180:
                    yaw[i] = float('nan')
            self.lines[1, name].set_data(times, yaw)
            self.lines[2, name].set_data(times, [s.vx for s in visible])
            self.lines[3, name].set_data(times, [math.degrees(s.wz) for s in visible])
        times = [d.stamp - origin for d in differences]
        self.lines[4, 'difference'].set_data(times, [d.distance for d in differences])
        self.lines[5, 'difference'].set_data(times, [math.degrees(d.dyaw) for d in differences])
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
        status = []
        for name, title in (('raw', 'Odometria'), ('filtered', 'EKF')):
            age = time.monotonic() - arrivals[name] if name in arrivals else None
            state = 'in attesa' if age is None else '{:.1f} s dall’ultimo messaggio'.format(age)
            status.append('{}: {} campioni, {}'.format(title, len(streams[name]), state))
        if differences:
            rms = math.sqrt(sum(d.distance**2 for d in differences) / len(differences))
            yaw_rms = math.degrees(math.sqrt(
                sum(d.dyaw**2 for d in differences) / len(differences)))
            status.append('{} coppie | RMS distanza {:.3f} m | RMS Δyaw {:.2f}°'.format(
                len(differences), rms, yaw_rms))
        elif all(streams.values()):
            status.append('Nessuna coppia: verificare timestamp, frame e max_sync_gap')
        if all(streams.values()):
            raw, filtered = (streams[name][-1] for name in ('raw', 'filtered'))
            if (raw.frame, raw.child) != (filtered.frame, filtered.child):
                status.append('Frame diversi: traiettorie non confrontabili senza trasformazione')
        if rejected or resets:
            status.append('Campioni invalidi: {} | reset temporali: {}'.format(rejected, resets))
        self.status.setText('    |    '.join(status))
        self.canvas.draw_idle()

    def export(self):
        """Let the operator choose the parent directory for a unique export."""
        if self.displayed is None:
            return
        parent = QFileDialog.getExistingDirectory(self, 'Cartella per esportare la prova')
        if not parent:
            return
        directory = Path(parent) / datetime.now().strftime('ekf_%Y%m%d_%H%M%S_%f')
        try:
            export_snapshot(directory, self.figure, *self.displayed)
        except (OSError, ValueError) as error:
            QMessageBox.critical(self, 'Esportazione fallita', str(error))
            return
        QMessageBox.information(self, 'Prova esportata', str(directory))


def main(args=None):
    """Run ROS acquisition on a worker and all Qt operations on the main thread."""
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = None
    worker = None
    stop = threading.Event()
    previous_handlers = {}
    signal_timer = QTimer()
    try:
        node = ExperimentNode()
        window = ExperimentWindow(node)

        def spin():
            while not stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

        worker = threading.Thread(target=spin, daemon=True)
        worker.start()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: app.quit())
        signal_timer.timeout.connect(lambda: None)
        signal_timer.start(50)
        window.show()
        app.exec_()
    finally:
        signal_timer.stop()
        stop.set()
        if worker is not None:
            worker.join(timeout=2.0)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
