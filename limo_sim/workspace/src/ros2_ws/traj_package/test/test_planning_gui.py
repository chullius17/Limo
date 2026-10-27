"""Check map orientation, retained ROS delivery and the passive Qt panels."""

import math
import os
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
import numpy as np  # noqa: E402
from PyQt5.QtGui import QImage  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from traj_package.planning_gui import (  # noqa: E402
    CostmapView, PlanningGuiNode, PlanningWindow, costmap_image, world_to_pixel)


def grid(width=3, height=2):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height = width, height
    msg.info.resolution = 0.05
    msg.info.origin.orientation.w = 1.0
    msg.data = [0, 1, 50, 98, 99, -1][:width * height]
    return msg


def test_image_puts_map_y_up_and_preserves_cells_and_owned_pixels():
    msg = grid()
    image = costmap_image(msg)
    msg.data = [100] * 6
    assert (image.width(), image.height()) == (3, 2)
    assert image.pixelColor(0, 1).getRgb()[:3] == (245, 245, 245)
    assert image.pixelColor(0, 0).getRgb()[:3] == (255, 0, 60)
    assert image.pixelColor(1, 0).getRgb()[:3] == (190, 50, 210)
    assert image.pixelColor(2, 0).getRgb()[:3] == (125, 135, 145)
    lethal = grid()
    lethal.data[0] = 100
    assert costmap_image(lethal).pixelColor(0, 1).getRgb()[:3] == (20, 20, 20)


@pytest.mark.parametrize('case', ['empty', 'size', 'resolution', 'nan', 'values'])
def test_invalid_grids_are_rejected(case):
    msg = grid()
    if case == 'empty':
        msg.info.width = 0
    elif case == 'size':
        msg.data = [0]
    elif case == 'resolution':
        msg.info.resolution = 0.0
    elif case == 'nan':
        msg.info.resolution = float('nan')
    else:
        msg.data[0] = -2
    with pytest.raises(ValueError):
        costmap_image(msg)


def transform(x, y, yaw, stamp=1):
    msg = TransformStamped()
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_link'
    msg.header.stamp.sec = stamp
    msg.transform.translation.x = float(x)
    msg.transform.translation.y = float(y)
    msg.transform.rotation.z = math.sin(yaw / 2)
    msg.transform.rotation.w = math.cos(yaw / 2)
    return msg


def goal(x=0.5, y=1.0, yaw=0.0, stamp=1):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = stamp
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def path(stamp):
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = stamp
    msg.poses = [goal(stamp=stamp)]
    return msg


@pytest.fixture
def ros_app():
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=['--ros-args', '-p', 'costmap_topic:=/test/planning_gui/costmap'])
    try:
        yield app
    finally:
        rclpy.shutdown()


def test_footprint_and_axes_follow_robot_and_rotated_map_origin(ros_app):
    msg = grid()
    msg.info.width = msg.info.height = 20
    msg.data = [0] * 400
    msg.info.resolution = 0.1
    msg.info.origin.position.x, msg.info.origin.position.y = 10.0, -3.0
    msg.info.origin.orientation.z = math.sin(math.pi / 4)
    msg.info.origin.orientation.w = math.cos(math.pi / 4)
    assert world_to_pixel((9.5, -2), msg) == pytest.approx((10, 15))
    view = CostmapView()
    footprint = ((-0.2, -0.1), (-0.2, 0.1), (0.2, 0.1), (0.2, -0.1))
    try:
        view.set_image(costmap_image(msg))
        view.set_robot(msg, (9.5, -2, math.pi), footprint, 'base_link', 0.45)
        paths = [item for item in view.robot_items if hasattr(item, 'path')]
        border = next(item for item in paths if item.pen().color().name() == '#141e2d')
        first = border.path().elementAt(0)
        assert (first.x, first.y) == pytest.approx((11, 17))
        for color, expected in [('#c32323', (10, 10.5)), ('#238237', (5.5, 15))]:
            axis = next(item for item in paths if item.pen().color().name() == color)
            end = axis.path().elementAt(1)
            assert (end.x, end.y) == pytest.approx(expected)
    finally:
        view.close()


def test_published_footprint_uses_its_stamp_then_follows_latest_robot_pose(ros_app):
    node = PlanningGuiNode(parameter_overrides=[Parameter('footprint_padding', value=0.03)])
    try:
        node.tf_buffer.set_transform(transform(2, 3, math.pi / 2), 'test')
        node.tf_buffer.set_transform(transform(4, 5, 0, stamp=2), 'test')
        polygon = PolygonStamped()
        polygon.header.frame_id = 'map'
        polygon.header.stamp.sec = 1
        # This is the already padded Nav2 polygon at the OLD robot pose.
        polygon.polygon.points = [Point32(x=x, y=y) for x, y in [
            (2.1, 2.8), (1.9, 2.8), (1.9, 3.2), (2.1, 3.2)]]
        node._receive_footprint(polygon)
        pose, footprint = node.robot_overlay('map')
        assert pose == pytest.approx((4, 5, 0))
        expected = ((-0.2, -0.1), (-0.2, 0.1), (0.2, 0.1), (0.2, -0.1))
        for point, corner in zip(footprint, expected):
            assert point == pytest.approx(corner, abs=1e-6)
        assert node.footprint_source == 'Nav2'
        node.tf_buffer.clear()
        assert node.robot_overlay('map') is None
    finally:
        node.destroy_node()


def test_configured_footprint_padding_is_used_before_nav2_publishes(ros_app):
    node = PlanningGuiNode(parameter_overrides=[Parameter('footprint_padding', value=0.01)])
    try:
        _, footprint = node.robot_overlay('base_link')
        assert footprint[0] == pytest.approx((-0.171, -0.120))
        assert footprint[2] == pytest.approx((0.171, 0.120))
        assert node.footprint_source == 'configurazione'
    finally:
        node.destroy_node()


def test_blue_goal_requires_current_valid_path_and_survives_topic_delivery_order(ros_app):
    node = PlanningGuiNode()
    try:
        selected = goal(stamp=1)
        adjusted = goal(x=0.7, yaw=math.pi / 2, stamp=3)
        node._receive_goal(selected)
        node._receive_adjusted_goal(adjusted)
        assert node.planned_goal() is None
        node._receive_path(path(2))
        assert node.planned_goal() is adjusted
        node._receive_goal(goal(stamp=10))
        assert node.planned_goal() is None
        node._receive_adjusted_goal(adjusted)  # Old retained result.
        node._receive_path(path(2))
        assert node.planned_goal() is None
        node._receive_path(Path())
        assert node.planned_goal() is None
        node._receive_path(path(11))  # New path arrives before its adjusted pose.
        assert node.planned_goal() is None
        current = goal(x=0.8, stamp=12)
        node._receive_adjusted_goal(current)
        assert node.planned_goal() is current
        node._receive_path(Path())  # Failed or invalidated planning clears blue.
        assert node.planned_goal() is None
        assert node.requested_goal.header.stamp.sec == 10
    finally:
        node.destroy_node()


def test_goal_pose_is_transformed_at_its_timestamp(ros_app):
    node = PlanningGuiNode()
    try:
        node.tf_buffer.set_transform(transform(2, 3, math.pi / 2), 'test')
        msg = goal(x=1.0, y=0.0, yaw=math.pi / 2)
        msg.header.frame_id = 'base_link'
        assert node.goal_overlay(msg, 'map') == pytest.approx((2, 4, math.pi))
        assert node.goal_overlay(msg, 'missing_frame') is None
    finally:
        node.destroy_node()


def test_planning_start_is_retained_for_its_path_and_cleared_on_replanning(ros_app):
    node = PlanningGuiNode()
    try:
        selected_start = goal(x=0.6, y=0.4, yaw=math.pi / 2, stamp=2)
        node._receive_adjusted_start(selected_start)
        assert node.planned_start() is None
        node._receive_path(path(2))
        assert node.planned_start() is selected_start
        assert node.goal_overlay(node.planned_start(), 'map') == pytest.approx(
            (0.6, 0.4, math.pi / 2))
        node.tf_buffer.set_transform(transform(3, 4, 0), 'test')
        assert node.planned_start() is selected_start  # Planning start stays fixed.
        node._receive_goal(goal(stamp=10))
        assert node.planned_start() is None
        node._receive_adjusted_start(selected_start)
        node._receive_path(path(11))
        assert node.planned_start() is None  # Old retained start cannot match.
        current = goal(x=0.8, stamp=11)
        node._receive_adjusted_start(current)
        assert node.planned_start() is current
        node._receive_path(Path())
        assert node.planned_start() is None
    finally:
        node.destroy_node()


def test_green_and_blue_arrows_show_yaw_and_remain_distinct_when_coincident(ros_app):
    msg = grid()
    msg.info.width = msg.info.height = 20
    msg.info.resolution = 0.1
    msg.data = [0] * 400
    view = CostmapView()
    try:
        view.set_image(costmap_image(msg))
        view.set_goals(msg, (0.5, 1.0, math.pi / 2), (1.0, 1.0, 0))

        def arrow(color):
            return next(item.path() for item in view.goal_items
                        if hasattr(item, 'pen') and item.pen().color().name() == color)

        for color, expected in [('#00a63c', (5, 3.5)), ('#1565ff', (14.5, 10))]:
            tip = arrow(color).elementAt(1)
            assert (tip.x, tip.y) == pytest.approx(expected)
        view.set_goals(msg, (1, 1, 0), (1, 1, 0))
        green, blue = arrow('#00a63c'), arrow('#1565ff')
        assert green.elementAt(0).x == blue.elementAt(0).x
        assert green.elementAt(1).x > blue.elementAt(1).x
        view.set_goals(msg, None, (1, 1, 0), start=(0.5, 0.5, math.pi / 2))
        blue_arrows = [item.path() for item in view.goal_items
                       if hasattr(item, 'pen') and item.pen().color().name() == '#1565ff']
        assert len(blue_arrows) == 2
        tip = blue_arrows[1].elementAt(1)
        assert (tip.x, tip.y) == pytest.approx((5, 10.5))
        labels = [item.toPlainText() for item in view.goal_items
                  if hasattr(item, 'toPlainText')]
        assert labels == ['Goal pianificato', 'Start pianificato']
    finally:
        view.close()


def test_ros_goal_overlays_are_included_in_compressed_png_output(ros_app):
    app = ros_app
    topics = {
        'goal_topic': '/test/planning_gui/goal',
        'adjusted_goal_topic': '/test/planning_gui/adjusted_goal',
        'adjusted_start_topic': '/test/planning_gui/adjusted_start',
        'planning_path_topic': '/test/planning_gui/path',
        'image_topic': '/test/planning_gui/image/compressed',
    }
    node = PlanningGuiNode(parameter_overrides=[
        Parameter(name, value=value) for name, value in topics.items()])
    transport = rclpy.create_node('planning_gui_goals_test')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    requested_pub = transport.create_publisher(PoseStamped, topics['goal_topic'], 10)
    adjusted_pub = transport.create_publisher(PoseStamped, topics['adjusted_goal_topic'], qos)
    start_pub = transport.create_publisher(PoseStamped, topics['adjusted_start_topic'], qos)
    path_pub = transport.create_publisher(Path, topics['planning_path_topic'], qos)
    frames = []
    transport.create_subscription(CompressedImage, topics['image_topic'], frames.append, 1)
    window = PlanningWindow(node)
    window.show()

    def until(predicate):
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not predicate():
            app.processEvents()
            rclpy.spin_once(transport, timeout_sec=0.0)
            time.sleep(0.01)
        assert predicate()

    try:
        background = grid()
        background.info.width = background.info.height = 40
        background.info.resolution = 0.1
        background.data = [0] * 1600
        node._receive(background)
        until(lambda: requested_pub.get_subscription_count() > 0)
        requested_pub.publish(goal(x=1.0, y=2.0, stamp=1))
        until(lambda: node.requested_goal is not None)
        adjusted_pub.publish(goal(x=2.0, y=2.0, yaw=math.pi / 2, stamp=3))
        start_pub.publish(goal(x=1.0, y=1.0, yaw=math.pi / 2, stamp=2))
        path_pub.publish(path(2))
        until(lambda: node.planned_goal() is not None and node.planned_start() is not None
              and len(window.map_view.goal_items) == 9)
        frames.clear()
        until(lambda: bool(frames))
        frame = frames[-1]
        assert frame.format == 'png'
        assert frame.header.frame_id == 'map'
        assert bytes(frame.data).startswith(b'\x89PNG\r\n\x1a\n')
        image = QImage.fromData(bytes(frame.data)).convertToFormat(QImage.Format_RGB888)
        assert not image.isNull()
        assert image.width() <= 960 and image.height() <= 720
        assert len(frame.data) < image.width() * image.height()
        bits = image.bits()
        bits.setsize(image.byteCount())
        rows = np.frombuffer(bits, dtype=np.uint8).reshape(image.height(), image.bytesPerLine())
        pixels = rows[:, :image.width() * 3].reshape(image.height(), image.width(), 3)
        for color in [(0, 166, 60), (21, 101, 255)]:
            assert np.any(np.all(pixels == color, axis=2))
        # Both blue arrows must be included in the actual compressed frame.
        start_pixel = window.map_view.mapFromScene(*world_to_pixel((1.0, 1.2), background))
        viewport_width = window.map_view.viewport().width()
        viewport_height = window.map_view.viewport().height()
        u = round(start_pixel.x() * image.width() / viewport_width)
        v = round(start_pixel.y() * image.height() / viewport_height)
        patch = pixels[v - 2:v + 3, u - 2:u + 3]
        assert np.any(np.all(patch == (21, 101, 255), axis=2))
        path_pub.publish(Path())
        until(lambda: len(window.map_view.goal_items) == 3)
        assert window.terminal.toPlainText() == ''
    finally:
        window.close()
        node.destroy_node()
        transport.destroy_node()


def test_log_panel_receives_full_retained_transcript_without_a_costmap(ros_app):
    app = ros_app
    transport = rclpy.create_node('planning_gui_log_test')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = transport.create_publisher(String, '/test/planning_gui/log', qos)
    transcript = (
        'NUOVA RICERCA #1\nTENTATIVO 1/2\n'
        '  Start: x=+1.000 m, y=+0.000 m, yaw=+90.0° [frame=map]\n'
        '  Arrivo: x=+2.000 m, y=+0.000 m, yaw=+0.0° [frame=map]\n'
        '  FALLITO: Virtual start or goal is occupied\n'
        'TENTATIVO 2/2\n  SUCCESSO: percorso con 25 pose.\n')
    publisher.publish(String(data=transcript))  # Publish before the GUI joins.
    node = PlanningGuiNode(parameter_overrides=[Parameter(
        'planning_log_topic', value='/test/planning_gui/log')])
    window = PlanningWindow(node)
    window.show()

    def until(text):
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and window.terminal.toPlainText() != text:
            app.processEvents()
            time.sleep(0.01)
        assert window.terminal.toPlainText() == text

    try:
        until(transcript)
        assert window.grid is None
        assert window.terminal.isReadOnly()
        publisher.publish(String(data=transcript))
        app.processEvents()
        assert window.terminal.toPlainText().count('TENTATIVO 1/2') == 1
        current = 'NUOVA RICERCA #2\nTENTATIVO 1/1\n  Calcolo in corso…\n'
        publisher.publish(String(data=current))
        until(current)
        assert 'RICERCA #1' not in window.terminal.toPlainText()
    finally:
        window.close()
        node.destroy_node()
        transport.destroy_node()


def test_late_gui_receives_retained_costmap_and_keeps_log_empty(ros_app):
    app = ros_app
    transport = rclpy.create_node('planning_gui_test_publisher')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = transport.create_publisher(OccupancyGrid, '/test/planning_gui/costmap', qos)
    publisher.publish(grid())
    node = PlanningGuiNode(parameter_overrides=[Parameter(
        'footprint_topic', value='/test/planning_gui/footprint')])
    broadcaster = TransformBroadcaster(transport)
    footprint_pub = transport.create_publisher(PolygonStamped, '/test/planning_gui/footprint', 1)
    window = PlanningWindow(node)
    window.show()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and window.map_view.map_item is None:
            app.processEvents()
            time.sleep(0.01)
        assert window.map_view.map_item is not None
        assert 'Frame: map' in window.status.text()
        assert '3 × 2 celle' in window.status.text()
        assert node.costmap_topic == '/test/planning_gui/costmap'
        assert window.terminal.isReadOnly()
        assert window.terminal.toPlainText() == ''
        assert window.terminal.isVisible()
        assert window.map_view.geometry().height() > window.terminal.geometry().height()
        for width, height in [(600, 950), (1200, 650)]:
            window.resize(width, height)
            app.processEvents()
            view_transform = window.map_view.transform()
            assert view_transform.m11() == pytest.approx(view_transform.m22())
            image = window.map_view.map_item.pixmap().toImage()
            assert image.pixelColor(0, 1).getRgb()[:3] == (245, 245, 245)
        publisher.publish(grid(2, 3))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and '2 × 3 celle' not in window.status.text():
            app.processEvents()
            time.sleep(0.01)
        assert '2 × 3 celle' in window.status.text()
        assert window.map_view.map_item.pixmap().width() == 2
        polygon = PolygonStamped()
        polygon.header.frame_id = 'base_link'
        polygon.polygon.points = [Point32(x=x, y=y) for x, y in [
            (-0.2, -0.1), (-0.2, 0.1), (0.2, 0.1), (0.2, -0.1)]]
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not window.map_view.robot_items:
            broadcaster.sendTransform(transform(0.05, 0.05, math.pi / 2))
            footprint_pub.publish(polygon)
            app.processEvents()
            time.sleep(0.01)
        assert window.map_view.robot_items
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and node.footprint_source != 'Nav2':
            footprint_pub.publish(polygon)
            app.processEvents()
            time.sleep(0.01)
        assert 'Footprint: Nav2' in window.robot_status.text()
        # Update TF without publishing another costmap: the overlay must move.
        broadcaster.sendTransform(transform(0.10, 0.05, 0, stamp=2))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and 'x=0.10' not in window.robot_status.text():
            app.processEvents()
            time.sleep(0.01)
        assert 'x=0.10' in window.robot_status.text()
        window.grid.header.frame_id = 'missing_map'
        window.refresh()
        assert not window.map_view.robot_items
        assert 'TF non disponibile' in window.robot_status.text()
        assert window.terminal.toPlainText() == ''
    finally:
        window.close()
        app.processEvents()
        assert not window.timer.isActive()
        node.destroy_node()
        transport.destroy_node()
