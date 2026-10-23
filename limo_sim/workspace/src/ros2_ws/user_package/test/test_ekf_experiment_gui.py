"""Exercise the real Qt viewer, ROS subscriptions, reset, and exported artifacts."""

import csv
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
pytest.importorskip('matplotlib')
from PyQt5.QtWidgets import QApplication  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from user_package.ekf_experiment import (  # noqa: E402
    ExperimentNode, ExperimentWindow, export_snapshot)


@pytest.fixture
def viewer():
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    node = ExperimentNode()
    window = ExperimentWindow(node)
    window.timer.stop()
    yield node, window, app
    window.close()
    node.destroy_node()
    rclpy.shutdown()


def odom(stamp, x=0., frame='odom'):
    msg = Odometry()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = frame
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.
    msg.pose.pose.position.x = x
    return msg


def test_live_ros_topics_reach_viewer_and_exports(viewer, tmp_path):
    node, window, app = viewer
    publisher = rclpy.create_node('ekf_experiment_test_publisher')
    raw = publisher.create_publisher(Odometry, '/odom', 10)
    filtered = publisher.create_publisher(Odometry, '/odometry/filtered', 10)
    try:
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            raw.publish(odom(1, 1.))
            filtered.publish(odom(1, 1.5))
            rclpy.spin_once(node, timeout_sec=.02)
            rclpy.spin_once(node, timeout_sec=.02)
            if all(node.snapshot()[0].values()):
                break
        window.refresh()
        app.processEvents()
        streams, differences = window.displayed
        assert differences[0].distance == pytest.approx(.5)
        assert list(window.lines[0, 'filtered'].get_xdata()) == [1.5]
        directory = tmp_path / 'export'
        export_snapshot(directory, window.figure, streams, differences)
        assert (directory / 'odometry.png').stat().st_size > 1000
        with (directory / 'differences.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        assert float(rows[0]['distance']) == pytest.approx(.5)
        assert (directory / 'raw.csv').is_file()
        assert (directory / 'filtered.csv').is_file()
    finally:
        publisher.destroy_node()


def test_pause_continues_acquisition_and_reset_clears_both(viewer):
    node, window, _ = viewer
    node.receive('raw', odom(1))
    node.receive('filtered', odom(1))
    window.refresh()
    window.toggle_pause()
    node.receive('raw', odom(2))
    window.refresh()
    assert len(window.displayed[0]['raw']) == 1
    assert len(node.snapshot()[0]['raw']) == 2
    window.toggle_pause()
    assert len(window.displayed[0]['raw']) == 2
    window.reset()
    assert not any(window.displayed[0].values())
    assert not window.export_button.isEnabled()


def test_frame_mismatch_and_invalid_quaternion_are_visible(viewer):
    node, window, _ = viewer
    invalid = odom(1)
    invalid.pose.pose.orientation.w = 0.
    node.receive('raw', invalid)
    node.receive('raw', odom(1))
    node.receive('filtered', odom(1, frame='map'))
    window.refresh()
    assert window.displayed[1] == []
    assert 'Frame diversi' in window.status.text()
    assert 'Campioni invalidi: 1' in window.status.text()
