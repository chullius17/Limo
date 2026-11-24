"""Check metric uncertainty, frame safety, retained ROS delivery and Qt rendering."""

import math
import os
import time

import numpy as np
import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from user_package.amcl_history import LocalizationHistory, MapSnapshot, XY_95_SCALE  # noqa: E402
from user_package.amcl_stats_model import pose_record  # noqa: E402
from user_package.amcl_visualization import (  # noqa: E402
    AmclVisualizationNode, AmclVisualizationWindow,
)


def pose(stamp=1.0, frame='map'):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    message.pose.pose.position.x = 1.5
    message.pose.pose.position.y = -0.25
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.04
    message.pose.covariance[7] = 0.09
    message.pose.covariance[1] = message.pose.covariance[6] = 0.03
    message.pose.covariance[35] = 0.01
    return message


def grid(frame='map'):
    message = OccupancyGrid()
    message.header.frame_id = frame
    message.info.width = 3
    message.info.height = 2
    message.info.resolution = 0.5
    message.info.origin.position.x = 2.0
    message.info.origin.position.y = -1.0
    message.info.origin.orientation.z = math.sin(math.pi/4)
    message.info.origin.orientation.w = math.cos(math.pi/4)
    message.data = [-1, 0, 30, 60, 90, 100]
    return message


def history(**kwargs):
    result = LocalizationHistory(**kwargs)
    result.set_map(MapSnapshot.from_message(grid()))
    return result


def test_correlated_xy_ellipse_has_two_principal_axes_in_metres():
    model = history()
    model.append(pose_record(pose()))
    maximum_eigenvalue = 0.065 + math.sqrt(0.025**2 + 0.03**2)
    minimum_eigenvalue = 0.065 - math.sqrt(0.025**2 + 0.03**2)
    assert model.latest.semi_major == pytest.approx(XY_95_SCALE*math.sqrt(maximum_eigenvalue))
    assert model.latest.semi_minor == pytest.approx(XY_95_SCALE*math.sqrt(minimum_eigenvalue))
    assert model.latest.semi_major > XY_95_SCALE*0.3  # Include correlation.
    assert model.latest.semi_minor < XY_95_SCALE*0.2
    assert model.latest.yaw_std == pytest.approx(0.1)
    assert model.latest.elapsed == 0


@pytest.mark.parametrize('angle,major_std,minor_std', [
    (0, 0.3, 0.1), (30, 0.3, 0.1), (-40, 0.3, 0.1), (90, 0.3, 0.1),
    (45, 0.3, 0), (0, 0.2, 0.2), (0, 0, 0),
])
def test_ellipse_reconstructs_rotated_covariance_including_degenerate_cases(
        angle, major_std, minor_std):
    theta = math.radians(angle)
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    covariance = rotation @ np.diag([major_std**2, minor_std**2]) @ rotation.T
    message = pose()
    message.pose.covariance[0] = covariance[0, 0]
    message.pose.covariance[1] = covariance[0, 1]
    message.pose.covariance[6] = covariance[1, 0]
    message.pose.covariance[7] = covariance[1, 1]
    # The uncertainty axes follow map covariance, independently of robot heading.
    message.pose.pose.orientation.z = math.sin(0.6)
    message.pose.pose.orientation.w = math.cos(0.6)
    model = history()
    model.append(pose_record(message))
    sample = model.latest
    assert sample.semi_major == pytest.approx(XY_95_SCALE*major_std)
    assert sample.semi_minor == pytest.approx(XY_95_SCALE*minor_std, abs=1e-8)
    if major_std != minor_std:
        assert sample.ellipse_yaw == pytest.approx(theta % math.pi)
    c, s = math.cos(sample.ellipse_yaw), math.sin(sample.ellipse_yaw)
    axes = np.array([[c*sample.semi_major, -s*sample.semi_minor],
                     [s*sample.semi_major, c*sample.semi_minor]])
    np.testing.assert_allclose(axes @ axes.T, XY_95_SCALE**2 * covariance, atol=1e-12)


def test_full_map_origin_rotation_bounds_and_unknown_pixels():
    snapshot = MapSnapshot.from_message(grid())
    assert snapshot.bounds() == pytest.approx((1, 2, -1, 0.5))
    assert np.allclose(snapshot.image()[0, 0], (0.55, 0.59, 0.64))
    assert np.allclose(snapshot.image()[0, 1], 1)
    assert np.allclose(snapshot.image()[1, 2], 0)
    assert not snapshot.values.flags.writeable


def test_history_is_bounded_sampled_and_keeps_latest_pose():
    model = history(max_samples=3, sample_period=0.25)
    for stamp in (1, 1.1, 1.25, 1.5, 1.75, 2, 2.1):
        model.append(pose_record(pose(stamp)))
    assert [sample.elapsed for sample in model.samples] == pytest.approx([0.5, 0.75, 1.0])
    assert model.latest.elapsed == pytest.approx(1.1)
    assert not model.append(pose_record(pose(2.05)))
    model.append(pose_record(pose(0)))
    assert model.clock_resets == 1
    assert len(model.samples) == 1 and model.latest.elapsed == 0
    model.clear()
    assert model.map is not None and not model.samples and model.latest is None


def test_frame_mismatch_and_invalid_covariance_cannot_corrupt_history():
    model = history()
    with pytest.raises(ValueError, match='frame'):
        model.append(pose_record(pose(frame='odom')))
    bad = pose()
    bad.pose.covariance[1] = bad.pose.covariance[6] = 1.0
    with pytest.raises(ValueError, match='positive semidefinite'):
        model.append(pose_record(bad))
    bad.pose.covariance[6] = 0.0
    with pytest.raises(ValueError, match='symmetric'):
        model.append(pose_record(bad))
    assert model.latest is None and not model.samples
    model.append(pose_record(pose()))
    model.set_map(MapSnapshot.from_message(grid(frame='another_map')))
    assert not model.samples and model.latest is None


@pytest.mark.parametrize('kind', ['data', 'resolution', 'origin'])
def test_invalid_map_is_rejected(kind):
    message = grid()
    if kind == 'data':
        message.data = [0]
    elif kind == 'resolution':
        message.info.resolution = float('nan')
    else:
        message.info.origin.orientation.w = message.info.origin.orientation.z = 0.0
    with pytest.raises(ValueError):
        MapSnapshot.from_message(message)


def test_retained_ros_inputs_render_colored_ellipses_and_preserve_manual_view():
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    peer = Node('amcl_visualization_test_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_publisher = peer.create_publisher(OccupancyGrid, '/map', qos)
    pose_publisher = peer.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', qos)
    # Late desktop discovery must recover both inputs with no further robot motion.
    map_publisher.publish(grid())
    pose_publisher.publish(pose())
    node = AmclVisualizationNode()
    window = AmclVisualizationWindow(node)
    executor.add_node(node)
    window.show()
    try:
        deadline = time.monotonic()+5
        while node.history.latest is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            app.processEvents()
        assert node.history.latest is not None
        window.refresh()
        x_limits, y_limits = window.axes.get_xlim(), window.axes.get_ylim()
        assert x_limits[0] < 1 and x_limits[1] > 2
        assert y_limits[0] < -1 and y_limits[1] > 0.5
        window.axes.set_xlim(-4, 4)
        window.axes.set_ylim(-3, 3)
        node.receive_pose(pose(2))
        node.receive_pose(pose(3))
        window.refresh()
        window.canvas.draw()
        assert window.axes.get_xlim() == (-4, 4)
        assert window.axes.get_ylim() == (-3, 3)
        assert len(window.ellipses.get_offsets()) == 3
        # Check actual rendered axes, including diameters, rotation and metric units.
        shape = window.ellipses.get_transforms()[0, :2, :2]
        expected_covariance = np.array([[0.04, 0.03], [0.03, 0.09]])
        np.testing.assert_allclose(shape @ shape.T,
                                   XY_95_SCALE**2 * expected_covariance, atol=1e-12)
        np.testing.assert_allclose(
            window.ellipses.get_transform().get_matrix()[:2, :2],
            window.axes.transData.get_affine().get_matrix()[:2, :2])
        edge_colors = window.ellipses.get_edgecolors()
        assert not np.allclose(edge_colors[0], edge_colors[-1])
        assert '95% XY semi-axes' in window.pose_details.text()
        assert 'Ellipse angle' in window.pose_details.text()
        assert 'Elapsed AMCL time' in window.colorbar.ax.get_ylabel()
        node.receive_map(OccupancyGrid())
        window.refresh()
        assert 'Input rejected' in window.status.text()
        assert window.map_snapshot is not None
        node.receive_map(grid())
        window.refresh()
        assert 'Input rejected' not in window.status.text()
        window.clear_history()
        assert not node.history.samples and window.ellipses is None
        assert len(window.latest_marker.get_xdata()) == 0
        assert window.map_snapshot is not None
    finally:
        window.timer.stop()
        window.close()
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
