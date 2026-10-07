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

"""Verify the Qt panel receives ROS telemetry and expires data with a paused clock."""

from io import BytesIO
import os
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
pytest.importorskip('limo_interfaces.msg')
from PIL import Image  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

from limo_interfaces.msg import MpcCandidate, MpcDebug  # noqa: E402
from limo_controller.control_gui import (  # noqa: E402
    ControlGuiNode, ControlWindow, GuiSignals)


@pytest.fixture
def gui():
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    signals = GuiSignals()
    node = ControlGuiNode(signals)
    node.set_parameters([Parameter('use_sim_time', value=True)])
    window = ControlWindow(node, signals)
    window.show()
    app.processEvents()
    try:
        yield app, node, window, signals
    finally:
        window.mpc_timer.stop()
        window.close()
        node.destroy_node()
        rclpy.shutdown()


def image_bytes():
    output = BytesIO()
    Image.new('RGB', (320, 160), (0, 180, 65)).save(output, format='PNG')
    return output.getvalue()


def test_numeric_telemetry_works_without_preview_images(gui):
    _, node, window, _ = gui
    msg = MpcDebug()
    msg.generated_count = 128
    msg.selected_id = 7
    msg.candidates = [MpcCandidate(candidate_id=7, total_cost=3.4)]
    node._mpc_callback(msg)
    assert window.mpc_state_label.text() == 'Telemetria MPC attiva'
    assert 'generati: 128' in window.mpc_summary.text()
    assert 'scelto: 7 | costo: 3.400' in window.mpc_summary.text()
    assert window.mpc_image.isHidden()
    assert window.last_preview_received is None


def test_gui_receives_ros_snapshot_and_compressed_image_below_control_terminal(gui):
    app, node, window, _ = gui
    transport = Node('gui_telemetry_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(transport)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    debug = transport.create_publisher(MpcDebug, '/limo/control/mpc_debug', qos)
    preview = transport.create_publisher(
        CompressedImage, '/limo/control/mpc_preview/image/compressed', qos)
    msg = MpcDebug()
    msg.generated_count = 64
    msg.time_steps = 25
    msg.model_dt = 0.1
    msg.initial_velocity = 0.2
    msg.command_velocity = 0.3
    msg.initial_steering = 0.1
    msg.command_steering = 0.2
    msg.selected_id = 7
    msg.candidates = [MpcCandidate(candidate_id=7, total_cost=3.4)]
    image = CompressedImage(format='png', data=image_bytes())
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and (
                window.last_mpc_received is None or window.last_preview_received is None):
            debug.publish(msg)
            preview.publish(image)
            executor.spin_once(timeout_sec=0.02)
            app.processEvents()
        assert window.mpc_state_label.text() == 'Telemetria MPC attiva'
        assert 'generati: 64' in window.mpc_summary.text()
        assert 'scelto: 7 | costo: 3.400' in window.mpc_summary.text()
        assert '2.50 s' in window.mpc_summary.text()
        assert '0.200 → 0.300 m/s' in window.mpc_summary.text()
        assert '5.7° → 11.5°' in window.mpc_summary.text()
        assert window.mpc_image.isVisible()
        assert window.mpc_image.y() > window.terminal.y() + window.terminal.height()
        window.resize(720, 900)
        app.processEvents()
        pixmap = window.mpc_image.pixmap()
        assert not pixmap.isNull()
        assert abs(pixmap.width() / pixmap.height() - 2.0) < 0.02
        assert pixmap.width() <= window.mpc_image.width()
        assert pixmap.height() <= window.mpc_image.height()
        window._set_status('READY: path available')
        assert window.start_button.isEnabled()
        assert window.start_button.text() == 'START CONTROL'
    finally:
        executor.remove_node(node)
        executor.remove_node(transport)
        transport.destroy_node()
        executor.shutdown()


def test_stale_telemetry_and_images_clear_without_advancing_ros_clock(gui, monkeypatch):
    _, node, window, signals = gui
    received = time.monotonic()
    signals.mpc_received.emit('previous snapshot', received)
    signals.preview_received.emit(image_bytes(), received)
    assert not window.mpc_image.source.isNull()
    assert node.get_clock().now().nanoseconds == 0
    monkeypatch.setattr(time, 'monotonic', lambda: received + node.mpc_stale_timeout + 0.1)
    window._check_mpc_freshness()
    assert 'scaduta' in window.mpc_state_label.text()
    assert 'previous snapshot' not in window.mpc_summary.text()
    assert window.mpc_image.source.isNull()
    assert 'scaduta' in window.mpc_image.text()
    assert node.get_clock().now().nanoseconds == 0
    signals.mpc_received.emit('new snapshot', time.monotonic())
    signals.preview_received.emit(image_bytes(), time.monotonic())
    assert window.mpc_summary.text() == 'new snapshot'
    assert window.mpc_state_label.text() == 'Telemetria MPC attiva'
    assert not window.mpc_image.source.isNull()


def test_corrupt_preview_clears_previous_image(gui):
    _, _, window, signals = gui
    signals.preview_received.emit(image_bytes(), time.monotonic())
    signals.preview_received.emit(b'invalid image', time.monotonic())
    assert window.mpc_image.source.isNull()
    assert window.last_preview_received is None
    assert 'non decodificabile' in window.mpc_image.text()
