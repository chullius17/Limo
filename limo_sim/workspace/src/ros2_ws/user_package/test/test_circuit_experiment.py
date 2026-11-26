"""Exercise real Qt geometry transforms and live obstacle rendering."""

import math
import os
from pathlib import Path
import sys
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('PyQt5')
from PyQt5.QtCore import QPointF  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from user_package.control.circuit_experiment import CircuitView, CircuitWindow  # noqa: E402
from user_package.control.circuit_scene import load_world_scene, Pose2D  # noqa: E402

WORLD = Path(__file__).resolve().parents[3] / 'custom_start/worlds/limo_obstacle_world.world'


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def test_texture_world_alignment_and_dynamic_obstacles(app):
    circuit = load_world_scene(WORLD)
    view = CircuitView(circuit)
    view.resize(1200, 800)
    view.show()
    view.fit_circuit()
    app.processEvents()
    image = view.background.pixmap()
    a, b, c = circuit.ground.texture_corners()
    for pixel, world in zip(((0, 0), (image.width(), 0), (0, image.height())), (a, b, c)):
        point = view.background.mapToScene(QPointF(*pixel))
        assert (point.x(), point.y()) == pytest.approx((world[0], -world[1]))
    assert len(view.obstacles) == 6  # Four walls plus two boxes, no robot.
    view.draw_poses({'unit_box': Pose2D(2, 1, math.pi / 2)})
    assert len(view.obstacles) == 1
    shape = view.obstacles[0]
    assert shape.toolTip() == 'unit_box'
    assert shape.boundingRect().center().x() == pytest.approx(2)
    assert shape.boundingRect().center().y() == pytest.approx(-1)
    first = shape.polygon()[0]
    assert (first.x(), first.y()) == pytest.approx((2.5, -0.5))
    view.draw_poses({})
    assert view.obstacles == []
    view.close()


def test_model_states_update_gui_when_ros_is_available(app):
    rclpy = pytest.importorskip('rclpy')
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Pose
    from user_package.control.circuit_experiment import create_gazebo_node
    rclpy.init()
    receiver = create_gazebo_node('/circuit_test/model_states')
    sender = rclpy.create_node('circuit_test_sender')
    publisher = sender.create_publisher(ModelStates, '/circuit_test/model_states', 1)
    window = CircuitWindow(load_world_scene(WORLD), receiver)
    window.show()
    try:
        # Wait for discovery; the bounded deadline catches a broken ROS/Qt bridge.
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(sender, timeout_sec=0.01)
            app.processEvents()
        pose = Pose()
        pose.position.x, pose.position.y, pose.orientation.w = 3.0, 2.0, 1.0
        message = ModelStates()
        message.name, message.pose = ['unit_box'], [pose]
        deadline = time.monotonic() + 3
        while receiver.poses is None and time.monotonic() < deadline:
            publisher.publish(message)
            app.processEvents()
            window.refresh()
            rclpy.spin_once(sender, timeout_sec=0.01)
        assert receiver.poses == {'unit_box': Pose2D(3, 2, 0)}
        assert len(window.view.obstacles) == 1
        assert window.view.obstacles[0].boundingRect().center().x() == pytest.approx(3)
        assert 'connected' in window.statusBar().currentMessage()
    finally:
        window.timer.stop()
        window.close()
        receiver.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()


def _wait_query(app, window):
    deadline = time.monotonic() + 3
    while window.query is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert window.query is None, 'Refresh process did not finish'


def test_refresh_button_loads_unsaved_obstacle_without_ros(app, tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace
    import xml.etree.ElementTree as ET
    from PyQt5.QtTest import QTest
    from PyQt5.QtCore import Qt
    sdf = ET.parse(str(WORLD))
    world = sdf.getroot().find('world')
    ET.SubElement(world, 'model', name='limo').append(ET.fromstring(
        '<pose>0 0 0 0.2 0.3 0</pose>'))
    world.append(ET.fromstring('''<model name="inserted_live"><pose>3 2 0 0 0 0</pose>
        <link name="link"><visual name="v"><geometry><cylinder><radius>0.4</radius>
        <length>1</length></cylinder></geometry></visual></link></model>'''))
    response = 'data: ' + json.dumps(ET.tostring(sdf.getroot(), encoding='unicode'))
    fake_gz = tmp_path / 'gz'
    fake_gz.write_text('#!' + sys.executable + '\nimport sys, time\n'
                       'assert sys.argv[1:] == ["topic", "--world-name", "default", '
                       '"--request", "world_sdf"]\n'
                       'time.sleep(0.1)\nprint(' + repr(response) + ')\n')
    fake_gz.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path))
    node = SimpleNamespace(poses={'unit_box': Pose2D()}, revision=0, received_at=0.0)
    window = CircuitWindow(load_world_scene(WORLD), node, world_path=WORLD)
    window.timer.stop()  # Exercise native Gazebo query without importing ROS.
    window.show()
    app.processEvents()
    original_ground = window.view.circuit.ground
    QTest.mouseClick(window.refresh_button, Qt.LeftButton)
    assert not window.refresh_button.isEnabled()
    _wait_query(app, window)
    assert window.refresh_button.isEnabled()
    assert window.windowTitle() == 'Gazebo circuit — top view'
    assert 'Obstacles refreshed' in window.statusBar().currentMessage()
    assert 'inserted_live' in window.view.circuit.models
    assert 'limo' not in window.view.circuit.models
    assert window.view.circuit.ground is original_ground
    item = next(i for i in window.view.obstacles if i.toolTip() == 'inserted_live')
    assert item.boundingRect().center().x() == pytest.approx(3)
    assert item.boundingRect().center().y() == pytest.approx(-2)
    # Snapshot is displayed even when the cached ModelStates predates insertion.
    assert window.ignore_poses_before > node.received_at
    window.close()


def test_refresh_failure_and_timeout_keep_existing_geometry(app, tmp_path, monkeypatch):
    from types import SimpleNamespace
    fake_gz = tmp_path / 'gz'
    fake_gz.write_text('#!' + sys.executable + '\nprint("not a world response")\n')
    fake_gz.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path))
    original = load_world_scene(WORLD)
    window = CircuitWindow(original, SimpleNamespace(), world_path=WORLD)
    window.timer.stop()
    window.refresh_obstacles()
    _wait_query(app, window)
    assert window.view.circuit is original
    assert 'Could not refresh obstacles' in window.statusBar().currentMessage()
    assert window.refresh_button.isEnabled()
    fake_gz.write_text('#!' + sys.executable + '\nimport time\ntime.sleep(10)\n')
    window.refresh_obstacles()
    process = window.query
    window._query_timed_out()
    assert window.view.circuit is original
    assert window.refresh_button.isEnabled()
    assert '15 seconds' in window.statusBar().currentMessage()
    process.waitForFinished(500)
    window.close()


def test_base_link_axes_match_world_heading_and_survive_obstacle_refresh(app):
    from types import SimpleNamespace
    from PyQt5.QtWidgets import QGraphicsLineItem
    node = SimpleNamespace(base_link_pose=Pose2D(2, 1, math.pi / 2),
                           base_link_received_at=time.monotonic())
    window = CircuitWindow(load_world_scene(WORLD), node, world_path=WORLD)
    window.timer.stop()
    window._update_base_link()
    items = window.view.base_link_items
    axes = {i.pen().color().name(): i.line() for i in items if isinstance(i, QGraphicsLineItem)}
    x_axis = axes['#e53935']
    y_axis = axes['#43a047']
    assert (x_axis.x1(), x_axis.y1()) == pytest.approx((2, -1))
    assert (x_axis.x2(), x_axis.y2()) == pytest.approx((2, -1.35))
    assert (y_axis.x2(), y_axis.y2()) == pytest.approx((1.65, -1))
    # At 90 degrees the 32.2 cm length points along world +Y (screen -Y).
    footprint = window.view.robot_footprint
    expected = ((2.110, -0.839), (1.890, -0.839),
                (1.890, -1.161), (2.110, -1.161))
    for point, xy in zip(footprint.polygon(), expected):
        assert (point.x(), point.y()) == pytest.approx(xy)
    assert footprint.brush().color().alpha() < 255
    assert 1 < footprint.zValue() < items[-1].zValue()
    marker_ids = [id(i) for i in items]
    window._apply_world(WORLD.read_text())
    assert [id(i) for i in window.view.base_link_items] == marker_ids
    assert all(i.zValue() > 1 for i in window.view.base_link_items)
    # No new obstacle revision is needed to move the robot marker.
    node.base_link_pose = Pose2D(-1, -2, 0)
    window._update_base_link()
    axis = next(i.line() for i in window.view.base_link_items if isinstance(i, QGraphicsLineItem))
    assert (axis.x1(), axis.y1()) == pytest.approx((-1, 2))
    body = window.view.robot_footprint.polygon().boundingRect()
    assert (body.center().x(), body.center().y()) == pytest.approx((-1, 2))
    assert (body.width(), body.height()) == pytest.approx((0.322, 0.220))
    # Old poses are visibly faded; explicit disappearance removes the marker.
    node.base_link_received_at = time.monotonic() - 5
    window._update_base_link()
    assert all(i.opacity() == pytest.approx(0.4) for i in window.view.base_link_items)
    node.base_link_pose = None
    window._update_base_link()
    assert not window.view.base_link_items
    assert window.view.robot_footprint is None
    window.close()


def test_base_link_is_not_invented_and_does_not_change_camera_fit(app):
    view = CircuitView(load_world_scene(WORLD))
    assert not view.base_link_items
    assert view.robot_footprint is None
    original = view.circuit_bounds()
    view.draw_base_link(Pose2D(100, 100, 0))
    assert view.circuit_bounds() == original
    view.fit_circuit()
    assert view.bounds == original
    view.close()


def test_link_updates_independently_when_ros_is_available(app):
    rclpy = pytest.importorskip('rclpy')
    from gazebo_msgs.msg import LinkStates
    from geometry_msgs.msg import Pose
    from user_package.control.circuit_experiment import create_gazebo_node
    rclpy.init()
    receiver = create_gazebo_node('/base_test/models', '/base_test/links')
    sender = rclpy.create_node('base_test_sender')
    pub = sender.create_publisher(LinkStates, '/base_test/links', 1)
    window = CircuitWindow(load_world_scene(WORLD), receiver, world_path=WORLD)
    try:
        deadline = time.monotonic() + 5
        pose = Pose()
        pose.position.x, pose.position.y, pose.orientation.w = 2.0, 3.0, 1.0
        msg = LinkStates()
        msg.name, msg.pose = ['limo::base_link'], [pose]
        while receiver.base_link_pose is None and time.monotonic() < deadline:
            pub.publish(msg)
            window.refresh()
            app.processEvents()
            rclpy.spin_once(sender, timeout_sec=0.01)
        assert receiver.poses is None  # ModelStates has not published anything.
        assert receiver.base_link_pose == Pose2D(2, 3, 0)
        assert window.view.base_link_items
        msg.name, msg.pose = [], []
        deadline = time.monotonic() + 3
        while receiver.base_link_pose is not None and time.monotonic() < deadline:
            pub.publish(msg)
            window.refresh()
            app.processEvents()
            rclpy.spin_once(sender, timeout_sec=0.01)
        assert not window.view.base_link_items
    finally:
        window.close()
        receiver.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()
