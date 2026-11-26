"""Check geometric distance, valid laser returns and the paused acquisition clock."""

import math
from types import SimpleNamespace

import pytest

from user_package.control.control_telemetry import (
    ControlTelemetry, reference_distance, scan_minimum,
)


def test_distance_uses_segment_projection_and_clamps_to_endpoints():
    reference = ((0, 0), (0, 0), (10, 0), (10, 10))
    assert reference_distance(5, 2, reference) == pytest.approx(2)
    assert reference_distance(12, 5, reference) == pytest.approx(2)
    assert reference_distance(-3, -4, reference) == pytest.approx(5)


def test_scan_minimum_never_turns_invalid_or_missing_returns_into_zero():
    scan = SimpleNamespace(range_min=0.1, range_max=10.0,
                           ranges=(float('nan'), float('inf'), -1, 0, 0.05, 12))
    assert math.isnan(scan_minimum(scan))
    scan.ranges += (0.3, 1.0)
    assert scan_minimum(scan) == pytest.approx(0.3)


def test_pause_excludes_time_and_breaks_lines_without_replaying_old_samples():
    history = ControlTelemetry()
    history.start(10)
    history.append('distance', 10, 1)
    history.append('scan', 10, 2)
    history.set_avoidance(True, 10.1)
    history.pause(10.2)
    history.pause(10.3)
    assert not history.append('distance', 11, 100)
    assert len(history.pauses) == 1
    assert math.isnan(history.distance[-1][1])
    assert history.avoidance == [[pytest.approx(0.1), pytest.approx(0.2)]]
    history.resume(20.2)
    assert not history.append('scan', 11, 100)  # Delayed sample from the pause.
    assert history.append('scan', 20.4, 0.5)
    assert history.scan[-1][0] == pytest.approx(0.4)
    history.stop(20.5, 'SUCCESS')
    end, revision = history.end, history.revision
    history.set_avoidance(True, 21)
    history.pause(21)
    assert not history.append('scan', 21, 0.1)
    assert (history.end, history.revision) == (end, revision)


def test_full_history_stops_without_erasing_the_beginning():
    history = ControlTelemetry(capacity=2)
    history.start(10)
    history.append('scan', 10, 2)
    history.append('scan', 11, 1)
    assert not history.append('scan', 12, 0.5)
    assert history.scan == [(0, 2), (1, 1)]
    assert history.status == 'SAMPLE LIMIT'
