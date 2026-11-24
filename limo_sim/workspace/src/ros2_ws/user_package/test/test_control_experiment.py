"""Check measured speed, timestamped transforms, run isolation and PNG output."""

import importlib.util
import math
import os
from pathlib import Path
import time

import numpy as np
import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
import rclpy  # noqa: E402

from user_package.control_experiment import (  # noqa: E402
    ControlExperimentNode, ControlExperimentWindow,
)
from user_package.control_experiment_model import (  # noqa: E402
    ControlRecording, TrackPose, footprint_vertices, map_pose,
)
from user_package.map_geometry import MapSnapshot  # noqa: E402


FOOTPRINT = ((-0.161, -0.110), (-0.161, 0.110), (0.161, 0.110), (0.161, -0.110))
REFERENCE = ((1.0, 1.0), (2.0, 1.0), (3.0, 2.0))
PACKAGE = Path(__file__).resolve().parents[1]


def grid(frame='map'):
    message = OccupancyGrid()
    message.header.frame_id = frame
    message.info.width = message.info.height = 10
    message.info.resolution = 0.5
    message.info.origin.orientation.w = 1.0
    message.data = [0]*100
    return message


def path(points=REFERENCE, frame='map'):
    message = RosPath()
    message.header.frame_id = frame
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def odometry(stamp=10.0, x=1.0, vx=0.3, vy=0.0, frame='odom'):
    message = Odometry()
    message.header.frame_id = frame
    message.child_frame_id = 'base_link'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = 1.0
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x, message.twist.twist.linear.y = float(vx), float(vy)
    return message


def transform(stamp=10, x=0.0, yaw=0.0):
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = 'map', 'odom'
    message.header.stamp.sec = stamp
    message.transform.translation.x = x
    message.transform.rotation.z = math.sin(yaw/2)
    message.transform.rotation.w = math.cos(yaw/2)
    return message


def test_measured_reverse_speed_and_metric_rotated_footprint():
    forward = map_pose(odometry(vx=0.3, vy=0.4), (2, -1, math.pi/2), 'base_link')
    reverse = map_pose(odometry(vx=-0.3, vy=-0.4), (2, -1, math.pi/2), 'base_link')
    assert forward.speed == reverse.speed == pytest.approx(0.5)
    assert (forward.x, forward.y) == pytest.approx((1, 0))
    points = np.array(footprint_vertices(forward, FOOTPRINT))
    assert np.ptp(points, axis=0) == pytest.approx((0.220, 0.322))
    bad = odometry()
    bad.child_frame_id = 'different_body_frame'
    with pytest.raises(ValueError, match='child frame'):
        map_pose(bad, (0, 0, 0), 'base_link')
    bad = odometry(vx=float('nan'))
    with pytest.raises(ValueError, match='Non-finite'):
        map_pose(bad, (0, 0, 0), 'base_link')


def recording(**kwargs):
    result = ControlRecording(FOOTPRINT, **kwargs)
    result.start(MapSnapshot.from_message(grid()), REFERENCE)
    return result


def test_recording_sampling_final_pose_gaps_and_clock_reset():
    run = recording()
    run.append(TrackPose(10, 1, 1, 0, 0.2))
    assert not run.append(TrackPose(10.05, 1.01, 1, 0, 0.2))
    run.stop()
    assert len(run.samples) == 2 and run.samples[-1].stamp == 10.05
    assert not run.append(TrackPose(11, 2, 1, 0, 0.3))
    with pytest.raises(ValueError, match='Clear'):
        run.start(run.map, REFERENCE)
    run.clear()
    run.start(MapSnapshot.from_message(grid()), REFERENCE)
    for sample in (TrackPose(10, 1, 1, 0, 0.2), TrackPose(10.1, 1.02, 1, 0, 0.2),
                   TrackPose(11, 1.03, 1, 0, 0.2), TrackPose(11.1, 3, 1, 0, 0.2)):
        run.append(sample)
    assert [p.segment for p in run.samples] == [0, 0, 1, 2]
    run.append(TrackPose(1, 0, 0, 0, 0.0))
    assert not run.recording and 'clock' in run.reason
    assert len(run.samples) == 4


def test_capacity_stops_instead_of_erasing_early_trajectory():
    run = recording(max_samples=3)
    for i in range(5):
        run.append(TrackPose(10+i*0.1, i*0.02, 1, 0, 0.2))
    assert not run.recording and 'limit' in run.reason
    assert len(run.samples) == 3 and run.samples[0].stamp == 10


def test_wall_clock_timestamp_precision_does_not_halve_sampling_rate():
    run = recording()
    for i in range(4):
        run.append(TrackPose(1795000000.0+i*0.1, 1+i*0.01, 1, 0, 0.1))
    assert len(run.samples) == 4


@pytest.fixture
def client():
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    node = ControlExperimentNode()
    try:
        yield app, node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_missing_tf_waits_for_exact_timestamp_instead_of_using_latest(client):
    _, node = client
    node.receive_map(grid())
    node.tf_buffer.set_transform(transform(10, x=0.0), 'test')
    node.tf_buffer.set_transform(transform(11, x=2.0), 'test')
    # A fresh query for timestamp 10 still uses x=0, not the latest x=2.
    node.receive_odom(odometry(stamp=10))
    assert node.latest.x == pytest.approx(1.0)
    node.receive_odom(odometry(stamp=12))
    assert node.latest.stamp == 10 and len(node.pending) == 1
    node.tf_buffer.set_transform(transform(12, x=3.0), 'test')
    node.drain_pending()
    assert not node.pending and node.latest.x == pytest.approx(4.0)


def test_unavailable_tf_is_dropped_after_bounded_wait(client):
    _, node = client
    node.receive_map(grid())
    message = odometry()
    node.receive_odom(message)
    node.pending[0] = (message, time.monotonic()-1)
    node.drain_pending()
    assert not node.pending and node.dropped == 1 and node.latest is None


def test_window_freezes_reference_and_exports_actual_run(tmp_path, client):
    app, node = client
    node.receive_map(grid())
    node.receive_path(path())
    node.receive_odom(odometry(frame='map'))
    assert node.ready()
    node.start_recording()
    node.receive_odom(odometry(10.2, x=1.04, vx=-0.2, frame='map'))
    node.receive_odom(odometry(10.4, x=1.08, vx=0.4, frame='map'))
    node.receive_path(path(((2, 3), (3, 4))))
    assert node.run.reference == REFERENCE
    window = ControlExperimentWindow(node)
    window.show()
    try:
        app.processEvents()
        window.axes.set_xlim(-1, 6)
        window.stop_recording()
        window.canvas.draw()
        assert window.axes.get_xlim() == (-1, 6)
        assert len(window.polygons.get_paths()) == 3
        assert len(window.trail.get_segments()) == 2
        assert tuple(window.reference_line.get_xdata()) == (1, 2, 3)
        assert 'Measured planar speed' in window.colorbar.ax.get_ylabel()
        original = window.latest_polygon.get_xy().copy()
        node.receive_odom(odometry(11, x=4, frame='map'))
        window.refresh()
        assert np.array_equal(window.latest_polygon.get_xy(), original)
        exported = window.export_image(tmp_path/'control_run.png')
        assert exported.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
        assert exported.stat().st_size > 10000
        window.clear_run()
        assert not node.run.samples and window.polygons is None
    finally:
        window.timer.stop()
        window.close()


def test_map_frame_change_stops_recording_but_keeps_original_output(client):
    _, node = client
    node.receive_map(grid())
    node.receive_path(path())
    node.receive_odom(odometry(frame='map'))
    node.start_recording()
    node.receive_map(grid('new_map'))
    assert not node.run.recording and 'frame changed' in node.run.reason
    assert node.run.map.frame == 'map' and node.run.reference == REFERENCE


def test_clock_rollback_stops_before_waiting_for_a_missing_transform(client):
    _, node = client
    node.receive_map(grid())
    node.receive_path(path())
    node.receive_odom(odometry(frame='map'))
    node.start_recording()
    # The old map transform may be absent after a clock jump: stop immediately.
    node.receive_odom(odometry(stamp=1, frame='odom'))
    assert not node.run.recording and 'clock' in node.run.reason
    assert len(node.run.samples) == 1


def test_launch_shares_real_controller_footprint_and_measurement_sources(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'control_experiment_launch_test', PACKAGE/'launch/control_experiment.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda name: str(PACKAGE.parent/name))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    node, = module._launch_experiment(context)
    parameters = node['parameters'][0]
    assert parameters['use_sim_time'] is False
    assert parameters['map_topic'] == '/map'
    assert parameters['path_topic'] == '/limo/planning/path'
    assert parameters['odom_topic'] == '/odometry/filtered'
    assert parameters['footprint'] == list(np.asarray(FOOTPRINT).ravel())
    assert parameters['base_frame'] == 'base_link'
    assert parameters['speed_scale_max'] > 0
