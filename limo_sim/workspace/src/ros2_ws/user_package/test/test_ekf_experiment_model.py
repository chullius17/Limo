"""Check temporal alignment, angular wrap, frame safety and bounded histories."""

from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from user_package.ekf_experiment_model import ExperimentHistory, Sample, compare


def sample(stamp, **overrides):
    return replace(Sample(stamp, stamp, 0., 0., 1., 0.), **overrides)


def test_interpolation_uses_message_time_and_shortest_yaw_arc():
    raw = [sample(1., yaw=math.pi)]
    filtered = [sample(0., yaw=math.radians(179)),
                sample(2., yaw=math.radians(-179))]
    result = compare(raw, filtered, 2.1)
    assert len(result) == 1
    assert result[0].distance == pytest.approx(0.)
    assert result[0].dyaw == pytest.approx(0., abs=1e-12)


def test_signed_differences_and_distance():
    result = compare([sample(1.)], [sample(
        1., x=4., y=4., yaw=.1, vx=2., wz=.2)], .2)[0]
    assert (result.dx, result.dy, result.distance) == (3., 4., 5.)
    assert result.dyaw == pytest.approx(.1)
    assert result.dvx == 1.
    assert result.dwz == .2


def test_no_extrapolation_or_interpolation_across_missing_data():
    raw = [sample(0.), sample(1.), sample(3.)]
    assert compare(raw, [sample(.5), sample(2.)], .2) == []


@pytest.mark.parametrize('override', [{'frame': 'map'}, {'child': 'other'}, {'frame': ''}])
def test_different_or_missing_frames_cannot_be_compared(override):
    assert compare([sample(1.)], [sample(1., **override)], .2) == []


def test_interpolation_cannot_cross_frame_changes():
    assert compare([sample(1.)], [sample(.9), sample(1.1, frame='map')], .3) == []


def test_history_time_window_memory_limit_duplicates_and_out_of_order():
    history = ExperimentHistory(history_seconds=2., max_samples=3)
    for stamp in (1., 2., 3., 4.):
        history.append('raw', sample(stamp))
    history.append('filtered', sample(4.))
    history.append('raw', sample(4., x=99.))
    history.append('raw', sample(3.9))
    assert [s.stamp for s in history.snapshot()['raw']] == [2., 3., 4.]
    assert history.snapshot()['raw'][-1].x == 99.
    history.append('filtered', sample(7.))
    assert history.snapshot()['raw'] == []


def test_backwards_clock_jump_resets_both_streams_and_origin():
    history = ExperimentHistory()
    history.append('raw', sample(10.))
    history.append('filtered', sample(10.))
    history.append('raw', sample(0.))
    assert history.origin == 0.
    assert history.reset_count == 1
    assert history.snapshot()['filtered'] == []
    history.append('filtered', sample(0.))
    assert len(compare(**history.snapshot(), max_gap=.2)) == 1


def test_quaternion_normalization_and_invalid_data():
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=500000000),
                               frame_id='odom'), child_frame_id='base_link',
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1., y=2.),
            orientation=SimpleNamespace(x=0., y=0., z=2., w=2.))),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=1.), angular=SimpleNamespace(z=0.))))
    result = Sample.from_odometry(msg)
    assert result.stamp == 1.5
    assert result.yaw == pytest.approx(math.pi/2)
    msg.pose.pose.orientation.z = 0.
    msg.pose.pose.orientation.w = 0.
    with pytest.raises(ValueError):
        Sample.from_odometry(msg)
