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

"""Verify control state and service requests after removing the MPC panel."""

import os
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
from PyQt5.QtWidgets import QApplication, QLabel  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from limo_controller.control_gui import (  # noqa: E402
    ControlGuiNode, ControlWindow, GuiSignals)


@pytest.fixture
def gui():
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    signals = GuiSignals()
    node = ControlGuiNode(signals)
    window = ControlWindow(node, signals)
    window.show()
    app.processEvents()
    try:
        yield app, node, window, signals
    finally:
        window.close()
        node.destroy_node()
        rclpy.shutdown()


def spin_until(app, executor, predicate):
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline and not predicate():
        executor.spin_once(timeout_sec=0.02)
        app.processEvents()
    assert predicate()


def test_gui_keeps_control_terminal_without_mpc_panel_or_subscriptions(gui):
    _, node, window, _ = gui
    labels = [label.text() for label in window.findChildren(QLabel)]
    assert 'CONTROL TERMINAL' in labels
    assert all('MPC' not in label for label in labels)
    assert all('/mpc_' not in subscription.topic_name for subscription in node.subscriptions)
    assert window.terminal.isVisible()
    assert not window.start_button.isEnabled()


def test_gui_receives_ros_state_and_sends_start_pause_resume_abort(gui):
    app, node, window, _ = gui
    transport = Node('gui_control_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(transport)
    requests = []

    def service(kind):
        def callback(request, response):
            requests.append((kind, request.data))
            response.success = True
            response.message = 'test accepted'
            return response
        return callback

    transport.create_service(SetBool, '/limo/control/set_active', service('active'))
    transport.create_service(SetBool, '/limo/control/set_enabled', service('enabled'))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = transport.create_publisher(String, '/limo/control/status', qos)
    try:
        spin_until(app, executor, lambda: status.get_subscription_count() > 0
                   and node.start_abort_client.service_is_ready()
                   and node.pause_resume_client.service_is_ready())
        status.publish(String(data='READY: path available'))
        spin_until(app, executor, window.start_button.isEnabled)
        assert 'READY: path available' in window.terminal.toPlainText()
        window.start_button.click()
        spin_until(app, executor, lambda: window.start_button.text() == 'ABORT CONTROL')
        window.pause_button.click()
        spin_until(app, executor, lambda: window.pause_button.text() == 'RESUME CONTROL')
        window.pause_button.click()
        spin_until(app, executor, lambda: window.pause_button.text() == 'PAUSE CONTROL')
        window.start_button.click()
        spin_until(app, executor, lambda: window.start_button.text() == 'START CONTROL')
        assert requests == [('active', True), ('enabled', False),
                            ('enabled', True), ('active', False)]
    finally:
        executor.remove_node(node)
        executor.remove_node(transport)
        transport.destroy_node()
        executor.shutdown()
