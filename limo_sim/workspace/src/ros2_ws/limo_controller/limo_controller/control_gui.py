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

"""Small Qt GUI used to start, pause and abort Nav2 path control."""

import math
import signal
import sys
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
import rclpy
from PyQt5.QtCore import Qt, QTime, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from limo_interfaces.msg import MpcDebug


class GuiSignals(QWidget):
    """Thread-safe signals from ROS callbacks to the Qt event loop."""

    request_finished = pyqtSignal(str, bool, bool, str)
    status_received = pyqtSignal(str)
    active_received = pyqtSignal(bool)
    paused_received = pyqtSignal(bool)
    diagnostic_received = pyqtSignal(str)
    mpc_received = pyqtSignal(str, float)
    preview_received = pyqtSignal(bytes, float)


class ControlGuiNode(Node):
    """ROS service client controlled by the control window."""

    def __init__(self, signals):
        super().__init__('control_gui')
        self.signals = signals
        self.declare_parameter('mpc_debug_topic', '/limo/control/mpc_debug')
        self.declare_parameter(
            'mpc_image_topic', '/limo/control/mpc_preview/image/compressed')
        self.declare_parameter('mpc_stale_timeout', 1.0)
        self.mpc_stale_timeout = self.get_parameter('mpc_stale_timeout').value
        if not math.isfinite(self.mpc_stale_timeout) or self.mpc_stale_timeout <= 0:
            raise ValueError('mpc_stale_timeout must be positive and finite')
        telemetry_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            MpcDebug, self.get_parameter('mpc_debug_topic').value,
            self._mpc_callback, telemetry_qos)
        self.create_subscription(
            CompressedImage, self.get_parameter('mpc_image_topic').value,
            lambda msg: self.signals.preview_received.emit(
                bytes(msg.data), time.monotonic()), telemetry_qos)
        self.start_abort_client = self.create_client(
            SetBool,
            '/limo/control/set_active',
        )
        self.pause_resume_client = self.create_client(
            SetBool,
            '/limo/control/set_enabled',
        )
        self._service_availability = None
        self._last_cmd_vel_log = 0.0
        self._last_cmd_vel = None
        self._last_follow_status = None
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            '/limo/control/status',
            lambda msg: self.signals.status_received.emit(msg.data),
            state_qos,
        )
        self.create_subscription(
            String,
            '/limo/planning/status',
            lambda msg: self.signals.diagnostic_received.emit(msg.data),
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/limo/control/active',
            lambda msg: self.signals.active_received.emit(msg.data),
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/limo/control/paused',
            lambda msg: self.signals.paused_received.emit(msg.data),
            state_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',
            self._follow_path_status_callback,
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
        )
        self.create_subscription(
            String,
            '/limo/control/cmd_vel_source',
            lambda msg: self.signals.diagnostic_received.emit(
                f'CMD_VEL SOURCE: {msg.data}'),
            state_qos,
        )
        self.create_timer(1.0, self._report_service_availability)

    def _mpc_callback(self, msg):
        winner = next((candidate for candidate in msg.candidates
                       if candidate.candidate_id == msg.selected_id), None)
        selected = ('nessuno' if msg.selected_id < 0 else str(msg.selected_id))
        cost = f'{winner.total_cost:.3f}' if winner is not None else 'n/d'
        velocity = f'{msg.command_velocity:.3f}' if msg.selected_id >= 0 else 'n/d'
        steering = f'{math.degrees(msg.command_steering):.1f}°' if msg.selected_id >= 0 else 'n/d'
        summary = (
            f'Campioni generati: {msg.generated_count} | '
            f'in telemetria: {len(msg.candidates)} | scelto: {selected} | costo: {cost}\n'
            f'Orizzonte: {msg.time_steps * msg.model_dt:.2f} s '
            f'({msg.time_steps} passi, dt={msg.model_dt:.3f} s)\n'
            f'Velocità: {msg.initial_velocity:.3f} → {velocity} m/s | '
            f'Sterzo: {math.degrees(msg.initial_steering):.1f}° '
            f'→ {steering}'
        )
        self.signals.mpc_received.emit(summary, time.monotonic())

    def _diagnostic(self, message):
        self.get_logger().info(message)
        self.signals.diagnostic_received.emit(message)

    def _report_service_availability(self):
        availability = (
            self.start_abort_client.service_is_ready(),
            self.pause_resume_client.service_is_ready(),
        )
        if availability == self._service_availability:
            return
        self._service_availability = availability
        self._diagnostic(
            'SERVICES: set_active={} set_enabled={}'.format(
                'ready' if availability[0] else 'missing',
                'ready' if availability[1] else 'missing',
            )
        )

    def _follow_path_status_callback(self, msg):
        labels = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        if not msg.status_list:
            return
        signature = tuple(item.status for item in msg.status_list)
        if signature == self._last_follow_status:
            return
        self._last_follow_status = signature
        summary = ', '.join(
            labels.get(item.status, str(item.status))
            for item in msg.status_list
        )
        self._diagnostic(f'FOLLOW_PATH ACTION: {summary}')

    def _cmd_vel_callback(self, msg):
        now = time.monotonic()
        command = (round(msg.linear.x, 3), round(msg.angular.z, 3))
        if command == self._last_cmd_vel and now - self._last_cmd_vel_log < 1.0:
            return
        if now - self._last_cmd_vel_log < 0.25:
            return
        self._last_cmd_vel = command
        self._last_cmd_vel_log = now
        self._diagnostic(
            f'CMD_VEL: linear.x={msg.linear.x:.3f} m/s '
            f'angular.z={msg.angular.z:.3f} rad/s'
        )

    def request_active(self, active):
        """Request start or abort without blocking the Qt event loop."""
        return self._call_service(
            'active',
            self.start_abort_client,
            active,
        )

    def request_enabled(self, enabled):
        """Request resume or pause without blocking the Qt event loop."""
        return self._call_service(
            'enabled',
            self.pause_resume_client,
            enabled,
        )

    def _call_service(self, kind, client, requested_state):
        operation = {
            ('active', True): 'START',
            ('active', False): 'ABORT',
            ('enabled', True): 'RESUME',
            ('enabled', False): 'PAUSE',
        }[(kind, requested_state)]
        self._diagnostic(
            f'REQUEST: {operation} via {client.srv_name}'
        )
        if not client.service_is_ready():
            message = f"Service '{client.srv_name}' is not available"
            self.get_logger().error(message)
            self.signals.request_finished.emit(
                kind,
                False,
                requested_state,
                message,
            )
            return False

        request = SetBool.Request()
        request.data = requested_state
        future = client.call_async(request)
        future.add_done_callback(
            lambda result: self._request_done(
                result,
                kind,
                requested_state,
            )
        )
        return True

    def _request_done(self, future, kind, requested_state):
        try:
            response = future.result()
            success = bool(response.success)
            message = response.message
        except Exception as exc:
            success = False
            message = f'Control request failed: {exc}'
        log = self.get_logger().info if success else self.get_logger().error
        log(message)
        self.signals.diagnostic_received.emit(
            'RESPONSE: {} success={} message={}'.format(
                kind, success, message)
        )
        self.signals.request_finished.emit(
            kind,
            success,
            requested_state,
            message,
        )


class MpcImageView(QLabel):
    """Keep the source image and fit it to the available space without distortion."""

    def __init__(self):
        super().__init__('In attesa dell’anteprima MPC')
        self.source = QPixmap()
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.setMinimumHeight(220)
        self.setStyleSheet('background-color: #101418; color: #d7e0e7;')

    def show_image(self, pixmap):
        """Display a decoded image at the current widget size."""
        self.source = pixmap
        self._fit_image()

    def show_status(self, message):
        """Replace the image with a waiting, stale or decoding status."""
        self.source = QPixmap()
        self.clear()
        self.setText(message)

    def _fit_image(self):
        if not self.source.isNull():
            self.setPixmap(self.source.scaled(
                self.contentsRect().size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        """Rescale the original image when the layout changes."""
        super().resizeEvent(event)
        self._fit_image()


class ControlWindow(QWidget):
    """Window containing pause/resume and start/abort controls."""

    def __init__(self, node, signals):
        super().__init__()
        self.node = node
        self.control_active = False
        self.control_paused = False
        self.control_requested = False
        self.path_ready = False
        self.last_status = 'waiting for status'
        self.last_mpc_received = None
        self.last_preview_received = None

        self.setWindowTitle('LIMO Control')
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        self.setMinimumWidth(360)
        self.resize(580, 820)

        layout = QVBoxLayout(self)
        self.state_label = QLabel('Control state: waiting for status')
        self.state_label.setStyleSheet('font-weight: bold;')
        self.state_label.setWordWrap(True)
        layout.addWidget(self.state_label)

        self.pause_button = QPushButton('PAUSE CONTROL')
        self.pause_button.setEnabled(False)
        self.pause_button.clicked.connect(self._toggle_pause)
        layout.addWidget(self.pause_button)

        self.start_button = QPushButton('START CONTROL')
        self.start_button.clicked.connect(self._toggle_active)
        layout.addWidget(self.start_button)

        terminal_label = QLabel('CONTROL TERMINAL')
        terminal_label.setStyleSheet('font-weight: bold;')
        layout.addWidget(terminal_label)

        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setMinimumHeight(180)
        self.terminal.document().setMaximumBlockCount(1000)
        self.terminal.setStyleSheet(
            'background-color: #101418; color: #d7e0e7; '
            'font-family: monospace; font-size: 12px; padding: 6px;'
        )
        layout.addWidget(self.terminal)

        telemetry_label = QLabel('TELEMETRIA MPC')
        telemetry_label.setStyleSheet('font-weight: bold;')
        layout.addWidget(telemetry_label)
        self.mpc_state_label = QLabel('In attesa di dati MPC (abilitare MPC.Debug.enabled)')
        self.mpc_state_label.setWordWrap(True)
        layout.addWidget(self.mpc_state_label)
        self.mpc_summary = QLabel('Nessuno snapshot ricevuto')
        self.mpc_summary.setWordWrap(True)
        self.mpc_summary.setStyleSheet('font-family: monospace; font-size: 12px;')
        layout.addWidget(self.mpc_summary)
        self.mpc_image = MpcImageView()
        layout.addWidget(self.mpc_image, 2)
        self.mpc_image.hide()

        signals.request_finished.connect(self._request_finished)
        signals.status_received.connect(self._set_status)
        signals.active_received.connect(self._set_active)
        signals.paused_received.connect(self._set_paused)
        signals.diagnostic_received.connect(self._append_status)
        signals.mpc_received.connect(self._set_mpc)
        signals.preview_received.connect(self._set_preview)
        self.mpc_timer = QTimer(self)
        self.mpc_timer.timeout.connect(self._check_mpc_freshness)
        self.mpc_timer.start(250)
        self._refresh_buttons()
        self._append_status('GUI ready; waiting for path executor status')

    @pyqtSlot(str, float)
    def _set_mpc(self, summary, received_at):
        self.last_mpc_received = received_at
        self.mpc_summary.setText(summary)
        self.mpc_state_label.setText('Telemetria MPC attiva')
        self._check_mpc_freshness()

    @pyqtSlot(bytes, float)
    def _set_preview(self, data, received_at):
        pixmap = QPixmap()
        if not pixmap.loadFromData(data):
            self.last_preview_received = None
            self.mpc_image.show_status('Anteprima MPC non decodificabile')
            return
        self.last_preview_received = received_at
        self.mpc_image.show()
        self.mpc_image.show_image(pixmap)
        self._check_mpc_freshness()

    def _check_mpc_freshness(self):
        now = time.monotonic()
        timeout = self.node.mpc_stale_timeout
        if self.last_mpc_received is not None and now - self.last_mpc_received > timeout:
            self.mpc_state_label.setText('Telemetria MPC scaduta: controllo fermo o dati assenti')
            self.mpc_summary.setText('Nessuno snapshot recente')
        if (self.last_preview_received is not None
                and now - self.last_preview_received > timeout):
            self.last_preview_received = None
            self.mpc_image.show_status('Anteprima MPC scaduta: nessuna immagine recente')

    @staticmethod
    def _button_style(color):
        return (
            f'background-color: {color}; color: white; font-weight: bold; '
            'font-size: 16px; padding: 12px; border-radius: 5px;'
        )

    @pyqtSlot()
    def _toggle_active(self):
        requested_state = not self.control_requested
        self._append_status(
            'BUTTON: {} CONTROL pressed'.format(
                'START' if requested_state else 'ABORT')
        )
        self.start_button.setEnabled(False)
        if not self.node.request_active(requested_state):
            self.start_button.setEnabled(True)

    @pyqtSlot()
    def _toggle_pause(self):
        requested_state = self.control_paused
        self.pause_button.setEnabled(False)
        if not self.node.request_enabled(requested_state):
            self.pause_button.setEnabled(True)

    @pyqtSlot(str, bool, bool, str)
    def _request_finished(self, kind, success, requested_state, message):
        if success and kind == 'active':
            self.control_requested = requested_state
            if requested_state:
                QTimer.singleShot(3000, self._check_controller_started)
            if not requested_state:
                self.control_active = False
                self.control_paused = False
        elif success and kind == 'enabled':
            self.control_paused = not requested_state
        self._append_status(message)
        self._refresh_buttons()

    @pyqtSlot(str)
    def _set_status(self, status):
        self.last_status = status
        self.state_label.setText(f'Control state: {status}')
        if status.startswith('READY:'):
            self.path_ready = True
            self.control_requested = False
        elif status.startswith('IDLE:'):
            self.path_ready = False
            self.control_requested = False
        elif status.startswith('ERROR:'):
            self.control_requested = False
        elif status.startswith(('WAITING:', 'STARTING:', 'ACTIVE:', 'PAUSING:',
                                'PAUSED:', 'RESUMING:', 'ABORTING:')):
            self.control_requested = not status.startswith('ABORTING:')
        color = '#2e7d32'
        if status.startswith(('WAITING:', 'PAUSED:', 'PAUSING:')):
            color = '#ef6c00'
        elif status.startswith(('ERROR:', 'ABORTING:')):
            color = '#c62828'
        self.state_label.setStyleSheet(
            f'color: {color}; font-weight: bold;'
        )
        self._append_status(status)
        self._refresh_buttons()

    def _check_controller_started(self):
        if self.last_status.startswith(('WAITING:', 'STARTING:')):
            self._append_status(
                'WARNING: FollowPath has not reached ACTIVE after 3 seconds'
            )

    @pyqtSlot(bool)
    def _set_active(self, active):
        self.control_active = active
        self.control_requested = active
        self._refresh_buttons()

    @pyqtSlot(bool)
    def _set_paused(self, paused):
        self.control_paused = paused
        if paused:
            self.control_requested = True
        self._refresh_buttons()

    def _refresh_buttons(self):
        if self.control_paused:
            self.pause_button.setText('RESUME CONTROL')
            self.pause_button.setStyleSheet(self._button_style('#2e7d32'))
        else:
            self.pause_button.setText('PAUSE CONTROL')
            self.pause_button.setStyleSheet(self._button_style('#ef6c00'))
        self.pause_button.setEnabled(self.control_requested)

        if self.control_requested:
            self.start_button.setText('ABORT CONTROL')
            self.start_button.setStyleSheet(self._button_style('#c62828'))
        else:
            self.start_button.setText('START CONTROL')
            self.start_button.setStyleSheet(self._button_style('#0056b3'))
        self.start_button.setEnabled(
            self.control_requested or self.path_ready
        )

    @pyqtSlot(str)
    def _append_status(self, message):
        timestamp = QTime.currentTime().toString('HH:mm:ss')
        self.terminal.append(f'{timestamp}  {message}')


def main(args=None):
    """Run the Qt control GUI and its ROS client node."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    shutdown_requested = threading.Event()

    def request_shutdown(_signum=None, _frame=None):
        if shutdown_requested.is_set():
            return
        shutdown_requested.set()
        app.quit()

    previous_sigint_handler = signal.signal(signal.SIGINT, request_shutdown)
    previous_sigterm_handler = signal.signal(signal.SIGTERM, request_shutdown)
    signal_dispatch_timer = QTimer()
    signal_dispatch_timer.timeout.connect(lambda: None)
    signal_dispatch_timer.start(20)

    signals = GuiSignals()
    node = ControlGuiNode(signals)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    window = ControlWindow(node, signals)
    window.show()
    window.raise_()
    window.activateWindow()
    try:
        app.exec_()
    finally:
        signal_dispatch_timer.stop()
        signal.signal(signal.SIGINT, previous_sigint_handler)
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
