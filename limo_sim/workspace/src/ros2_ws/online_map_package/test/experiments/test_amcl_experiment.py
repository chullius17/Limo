"""Verify actual counts, complete journals, GUI acquisition and automatic export."""

import csv
import json
from pathlib import Path
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from online_map_package.experiments.amcl_experiment import (  # noqa: E402
    ExperimentNode, ExperimentWindow, default_results_directory,
)
from online_map_package.experiments.amcl_records import (  # noqa: E402
    ExperimentSession, cv_records, pose_record,
)


def pose(stamp=1, variance=0.04):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.orientation.w = 1.
    msg.pose.covariance[0] = variance
    msg.pose.covariance[7] = 0.09
    msg.pose.covariance[35] = 0.01
    msg.pose.covariance[1] = msg.pose.covariance[6] = -0.005
    return msg


def diagnostics(stamp=1, accepted=True):
    msg = DiagnosticArray()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'map'
    status = DiagnosticStatus()
    status.name = 'amcl/cv_fusion'
    status.message = 'accepted' if accepted else 'quality_gate: uninformative'
    values = {
        'accepted': str(accepted).lower(), 'reused': 'false',
        'input_points': 1000, 'candidate_points': 50,
        'used_points': 50 if accepted else 0,
        'used_road_points': 30 if accepted else 0,
        'used_non_road_points': 20 if accepted else 0,
        'particles': 300, 'evaluations': 15000, 'cloud_stamp': stamp,
        'sync_error': 0.02,
    }
    status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
    msg.status = [status]
    return msg


def test_records_preserve_covariance_and_actual_counts():
    row = pose_record(pose())
    assert row['cov_0_1'] == row['cov_1_0'] == -0.005
    assert len([key for key in row if key.startswith('cov_')]) == 36
    accepted, = cv_records(diagnostics())
    rejected, = cv_records(diagnostics(accepted=False))
    assert accepted['used_points'] == 50
    assert accepted['input_points'] == 1000
    assert rejected['used_points'] == 0
    assert rejected['candidate_points'] == 50
    assert rejected['evaluations'] == 15000


@pytest.mark.parametrize('value', [float('nan'), -1.])
def test_invalid_variance_is_rejected(value):
    with pytest.raises(ValueError):
        pose_record(pose(variance=value))


def test_csv_keeps_all_samples_when_window_expires_and_clock_resets(tmp_path):
    clock = [0.]
    session = ExperimentSession(
        tmp_path, {'max_samples': 2, 'history_seconds': 1.}, clock=lambda: clock[0])
    for index, stamp in enumerate((10, 11, 1, 2)):
        clock[0] = float(index)
        session.append('pose', pose_record(pose(stamp)))
    assert len(session.snapshot()['pose']) == 2
    session.close()
    rows = list(csv.DictReader((session.directory / 'pose.csv').open()))
    assert len(rows) == 4
    assert [int(row['segment']) for row in rows] == [0, 0, 1, 1]
    assert [float(row['elapsed_s']) for row in rows] == [0., 1., 2., 3.]
    full = session.read_plot_data()
    assert len(full['pose']) == 4
    assert full['pose'][-1]['stamp_s'] == 2.


@pytest.fixture
def viewer(tmp_path):
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    node = ExperimentNode(parameter_overrides=[
        Parameter('results_directory', value=str(tmp_path))])
    window = ExperimentWindow(node)
    window.timer.stop()
    yield node, window, app
    window.close()
    node.destroy_node()
    rclpy.shutdown()


def test_pause_keeps_acquiring_and_close_saves_every_sample(viewer):
    node, window, app = viewer
    node.receive_pose(pose())
    node.receive_cv(diagnostics())
    window.refresh()
    window.toggle_pause()
    node.receive_pose(pose(2))
    node.receive_cv(diagnostics(2, accepted=False))
    window.refresh()
    assert len(window.displayed['pose']) == 1
    assert len(node.snapshot()[0]['pose']) == 2
    window.show()
    window.close()
    app.processEvents()
    directory = node.session.directory
    assert window.finalized
    assert (directory / 'amcl_experiment.png').stat().st_size > 1000
    metadata = json.loads((directory / 'metadata.json').read_text())
    assert metadata['complete'] is True
    assert metadata['samples']['accepted'] == 1
    assert len(list(csv.DictReader((directory / 'pose.csv').open()))) == 2
    assert len(list(csv.DictReader((directory / 'cv.csv').open()))) == 2
    assert list(window.lines['used_points'].get_ydata()) == [50, 0]
    assert list(window.lines['cov_0_0'].get_ydata()) == [0.04, 0.04]


def test_live_ros_subscriptions_and_invalid_pose(viewer):
    node, window, _ = viewer
    publisher = rclpy.create_node('amcl_experiment_test_publisher')
    cv_pub = publisher.create_publisher(DiagnosticArray, '/amcl/cv_diagnostics', 10)
    pose_pub = publisher.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
    try:
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            cv_pub.publish(diagnostics())
            pose_pub.publish(pose())
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(node, timeout_sec=0.02)
            if all(node.snapshot()[0].values()):
                break
        assert all(node.snapshot()[0].values())
        node.receive_pose(pose(variance=float('nan')))
        window.refresh()
        assert 'Campioni invalidi: 1' in window.status.text()
        assert 'CV usati 50 / candidati 50' in window.status.text()
    finally:
        publisher.destroy_node()


def test_default_results_are_inside_source_package():
    source = Path(__file__).resolve().parents[2]
    assert Path(default_results_directory()) == source / 'results'
