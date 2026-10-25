"""Qt/ROS viewer for actual CV fusion counts and AMCL pose covariance."""

from collections import Counter
import math
from pathlib import Path
import signal
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from online_map_package.experiments.amcl_records import (
    ExperimentSession, cv_records, pose_record,
)


def default_results_directory():
    """Prefer the source package, also when installed with symlink-install."""
    source = Path(__file__).resolve().parents[2]
    if (source / 'package.xml').is_file():
        return str(source / 'results')
    return str(Path(get_package_share_directory('online_map_package')) / 'results')


class ExperimentNode(Node):
    """Journal both AMCL topics without altering localization or robot motion."""

    def __init__(self, **kwargs):
        super().__init__('amcl_experiment', **kwargs)
        defaults = {
            'pose_topic': '/amcl_pose', 'diagnostics_topic': '/amcl/cv_diagnostics',
            'results_directory': default_results_directory(),
            'history_seconds': 120.0, 'max_samples': 20000, 'refresh_hz': 5.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key in ('history_seconds', 'refresh_hz'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError('{} must be finite and positive'.format(key))
        if self.settings['max_samples'] < 2:
            raise ValueError('max_samples must be at least 2')
        self.lock = threading.Lock()
        self.rejected = Counter()
        self.arrivals = {}
        self.failure = ''
        self.session = ExperimentSession(
            Path(self.settings['results_directory']).expanduser(), self.settings)
        try:
            self.subscriptions_ = [
                self.create_subscription(
                    PoseWithCovarianceStamped, self.settings['pose_topic'],
                    self.receive_pose, qos_profile_sensor_data),
                self.create_subscription(
                    DiagnosticArray, self.settings['diagnostics_topic'],
                    self.receive_cv, 100),
            ]
        except Exception:
            self.session.close()
            raise
        self.get_logger().info('Registrazione AMCL: {}'.format(self.session.directory))

    def _receive(self, key, parser, message):
        try:
            records = parser(message)
        except (ValueError, KeyError, TypeError, OverflowError):
            with self.lock:
                self.rejected[key] += 1
            return
        with self.lock:
            if self.failure or self.session.closed:
                return
            for row in records:
                try:
                    if self.session.append(key, row):
                        self.arrivals[key] = time.monotonic()
                except OSError as error:
                    self.failure = str(error)
                    self.get_logger().error('Errore registrazione: {}'.format(error))
                    break

    def receive_pose(self, message):
        """Keep the published covariance, without estimating it from the path."""
        self._receive('pose', lambda msg: [pose_record(msg)], message)

    def receive_cv(self, message):
        """Record exact counts after AMCL class filtering and voxelization."""
        self._receive('cv', cv_records, message)

    def snapshot(self):
        """Return a consistent view for the GUI thread."""
        with self.lock:
            return (self.session.snapshot(), dict(self.arrivals),
                    dict(self.session.counts), dict(self.rejected), self.failure)


class ExperimentWindow(QMainWindow):
    """Display six live plots and save the complete session on close."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.paused = False
        self.finalized = False
        self.displayed = None
        self.setWindowTitle('Esperimento AMCL — punti CV e covarianza')
        self.resize(1400, 950)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        for text in (
                'CV: {}    |    Posa: {}'.format(
                    node.settings['diagnostics_topic'], node.settings['pose_topic']),
                'Salvataggio automatico: {}'.format(node.session.directory)):
            label = QLabel(text)
            label.setWordWrap(True)
            label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
            layout.addWidget(label)
        self.status = QLabel('In attesa di AMCL…')
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout.addWidget(self.status)
        controls = QHBoxLayout()
        self.pause_button = QPushButton('Pausa grafici')
        self.pause_button.clicked.connect(self.toggle_pause)
        controls.addWidget(self.pause_button)
        save_button = QPushButton('Salva grafici adesso')
        save_button.clicked.connect(self.save_now)
        controls.addWidget(save_button)
        controls.addStretch()
        layout.addLayout(controls)
        self.figure = Figure(constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        layout.addWidget(self.canvas, 1)
        self.axes = self.figure.subplots(3, 2).flatten()
        self.input_axis = self.axes[0].twinx()
        self.input_axis.set_ylabel('punti cloud in ingresso', color='#6b7280')
        labels = [
            ('Punti CV per aggiornamento AMCL', 'tempo acquisizione [s]', 'punti'),
            ('Punti effettivamente usati', 'tempo acquisizione [s]', 'punti'),
            ('Varianze della posizione', 'tempo acquisizione [s]', 'varianza [m²]'),
            ('Covarianza XY', 'tempo acquisizione [s]', 'covarianza [m²]'),
            ('Varianza orientamento', 'tempo acquisizione [s]', 'varianza yaw [rad²]'),
            ('Traiettoria AMCL', 'x [m]', 'y [m]'),
        ]
        specs = [
            [('input_points', 'Cloud in ingresso', '#9ca3af'),
             ('candidate_points', 'Candidati voxelizzati', '#d97706'),
             ('used_points', 'Usati nella fusione', '#2563eb')],
            [('used_road_points', 'Strada', '#2563eb'),
             ('used_non_road_points', 'Linee / boardwalk', '#a21caf')],
            [('cov_0_0', 'Var(x)', '#2563eb'), ('cov_1_1', 'Var(y)', '#d97706')],
            [('cov_0_1', 'Cov(x,y)', '#7c3aed')],
            [('cov_5_5', 'Var(yaw)', '#059669')],
            [('trajectory', 'Posa stimata', '#2563eb')],
        ]
        self.lines = {}
        for axis, label, series in zip(self.axes, labels, specs):
            axis.set(title=label[0], xlabel=label[1], ylabel=label[2])
            axis.grid(True, alpha=0.3)
            for key, title, color in series:
                target = self.input_axis if key == 'input_points' else axis
                self.lines[key], = target.plot([], [], label=title, color=color)
            axis.legend(handles=[self.lines[key] for key, _, _ in series], loc='upper right')
        self.axes[5].set_aspect('equal', adjustable='box')
        footer = QLabel(
            'La covarianza misura l’incertezza stimata da AMCL, non l’errore reale. '
            'La finestra mostra gli ultimi {:.0f} s; i CSV e il PNG finale conservano '
            'l’intera prova. Le pause fermano soltanto i grafici.'
            .format(node.settings['history_seconds']))
        footer.setWordWrap(True)
        layout.addWidget(footer)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(20, round(1000 / node.settings['refresh_hz'])))

    def toggle_pause(self):
        """Freeze rendering while continuing to write every sample."""
        self.paused = not self.paused
        self.pause_button.setText('Riprendi grafici' if self.paused else 'Pausa grafici')
        if not self.paused:
            self.refresh()

    def render(self, streams):
        """Plot a bounded preview without changing recorded samples."""
        self.displayed = streams
        for stream, records in streams.items():
            stride = max(1, math.ceil(len(records) / 2500))
            rows = records[::stride]
            if records and rows[-1] is not records[-1]:
                rows.append(records[-1])
            times = [row['elapsed_s'] for row in rows]
            keys = (['input_points', 'candidate_points', 'used_points',
                     'used_road_points', 'used_non_road_points'] if stream == 'cv' else
                    ['cov_0_0', 'cov_1_1', 'cov_0_1', 'cov_5_5'])
            for key in keys:
                self.lines[key].set_data(times, [row[key] for row in rows])
            if stream == 'pose':
                xs, ys, previous = [], [], None
                for row in rows:
                    identity = row['segment'], row['frame_id']
                    if previous is not None and previous != identity:
                        xs.append(float('nan'))
                        ys.append(float('nan'))
                    xs.append(row['x'])
                    ys.append(row['y'])
                    previous = identity
                self.lines['trajectory'].set_data(xs, ys)
        for axis in list(self.axes) + [self.input_axis]:
            axis.relim()
            axis.autoscale_view()
        self.canvas.draw_idle()

    def refresh(self):
        """Show counts, rejection reasons and the last reported uncertainty."""
        if self.paused or self.finalized:
            return
        streams, arrivals, counts, rejected, failure = self.node.snapshot()
        self.render(streams)
        parts = []
        for key, label in (('cv', 'CV'), ('pose', 'Posa')):
            age = time.monotonic() - arrivals[key] if key in arrivals else None
            state = 'in attesa' if age is None else 'ultimo dato {:.1f} s fa'.format(age)
            parts.append('{}: {} campioni, {}'.format(label, counts.get(key, 0), state))
        if streams['cv']:
            latest = streams['cv'][-1]
            parts.append('CV usati {} / candidati {} | {}'.format(
                latest['used_points'], latest['candidate_points'], latest['reason']))
            parts.append('Fusioni accettate: {}'.format(counts.get('accepted', 0)))
        if streams['pose']:
            pose = streams['pose'][-1]
            parts.append('σx={:.3f} m, σy={:.3f} m, σyaw={:.2f}°'.format(
                math.sqrt(pose['cov_0_0']), math.sqrt(pose['cov_1_1']),
                math.degrees(math.sqrt(pose['cov_5_5']))))
        if rejected:
            parts.append('Campioni invalidi: {}'.format(sum(rejected.values())))
        if failure:
            parts.append('ERRORE SALVATAGGIO: ' + failure)
        self.status.setText('    |    '.join(parts))

    def save(self, complete=False):
        """Export all-session plots; CSV recording continues for manual saves."""
        with self.node.lock:
            session = self.node.session
            if complete:
                session.close()
            streams = session.read_plot_data()
            session.write_metadata(
                self.node.rejected, complete=False, acquisition_error=self.node.failure)
        self.render(streams)
        path = session.directory / 'amcl_experiment.png'
        temporary = path.with_suffix('.tmp.png')
        self.figure.savefig(str(temporary), dpi=150)
        temporary.replace(path)
        with self.node.lock:
            session.write_metadata(
                self.node.rejected, complete=complete, acquisition_error=self.node.failure)

    def save_now(self):
        """Save a full-session snapshot without opening a directory dialog."""
        try:
            self.save()
        except (OSError, ValueError) as error:
            QMessageBox.critical(self, 'Salvataggio fallito', str(error))

    def finish(self):
        """Save once on window close, Ctrl+C, or orderly ROS shutdown."""
        if self.finalized:
            return
        self.timer.stop()
        self.save(complete=True)
        self.finalized = True
        self.node.get_logger().info('Risultati salvati: {}'.format(self.node.session.directory))

    def closeEvent(self, event):
        """Keep the window open if automatic export needs to be retried."""
        try:
            self.finish()
        except (OSError, ValueError) as error:
            QMessageBox.critical(self, 'Salvataggio fallito', str(error))
            event.ignore()
            return
        event.accept()


def main(args=None):
    """Keep Qt on the main thread and acquire ROS messages on a worker."""
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = window = worker = None
    stop = threading.Event()
    handlers = {}
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
            handlers[signum] = signal.signal(signum, lambda *_: window.close())
        signal_timer.timeout.connect(lambda: None if rclpy.ok() else window.close())
        signal_timer.start(50)
        window.show()
        app.exec_()
    finally:
        signal_timer.stop()
        stop.set()
        if worker is not None:
            worker.join(timeout=2.0)
        try:
            if window is not None:
                window.finish()
        finally:
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
            if node is not None:
                node.session.close()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
