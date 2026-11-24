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


def test_circle_encloses_correlated_xy_ellipse_in_metres():
    model = history()
    model.append(pose_record(pose()))
    maximum_eigenvalue = 0.065 + math.sqrt(0.025**2 + 0.03**2)
    assert model.latest.radius == pytest.approx(XY_95_SCALE*math.sqrt(maximum_eigenvalue))
    assert model.latest.radius > XY_95_SCALE*0.3  # Correlation must not be discarded.
    assert model.latest.yaw_std == pytest.approx(0.1)
    assert model.latest.elapsed == 0


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


def test_retained_ros_inputs_render_colored_circles_and_preserve_manual_view():
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
        assert len(window.circles.get_offsets()) == 3
        edge_colors = window.circles.get_edgecolors()
        assert not np.allclose(edge_colors[0], edge_colors[-1])
        assert 'XY bound radius' in window.pose_details.text()
        assert 'Elapsed AMCL time' in window.colorbar.ax.get_ylabel()
        node.receive_map(OccupancyGrid())
        window.refresh()
        assert 'Input rejected' in window.status.text()
        assert window.map_snapshot is not None
        node.receive_map(grid())
        window.refresh()
        assert 'Input rejected' not in window.status.text()
        window.clear_history()
        assert not node.history.samples and window.circles is None
        assert len(window.latest_marker.get_xdata()) == 0
        assert window.map_snapshot is not None
    finally:
        window.timer.stop()
        window.close()
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
