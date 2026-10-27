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

"""Check continuous measured arrival, clock discontinuities and missing feedback."""

import math

import pytest

from limo_controller.arrival import ArrivalDwell


def test_success_requires_the_full_continuous_stop_including_the_circle_boundary():
    monitor = ArrivalDwell()
    for index in range(20):
        assert not monitor.update(index / 10, 0.20, 0.01, -0.05)
    assert monitor.update(2.0, 0.20, 0.01, -0.05)


@pytest.mark.parametrize('distance,linear,angular', [
    (0.201, 0.0, 0.0), (0.1, 0.011, 0.0), (0.1, 0.0, -0.051),
    (math.nan, 0.0, 0.0), (0.1, math.inf, 0.0),
])
def test_leaving_the_circle_or_moving_restarts_the_entire_dwell(distance, linear, angular):
    monitor = ArrivalDwell()
    for index in range(11):
        assert not monitor.update(index / 10, 0.1, 0.0, 0.0)
    assert not monitor.update(1.1, distance, linear, angular)
    for index in range(12, 32):
        assert not monitor.update(index / 10, 0.1, 0.0, 0.0)
    assert monitor.update(3.2, 0.1, 0.0, 0.0)


def test_missing_samples_repeated_timestamps_and_clock_rewind_do_not_count_as_stopped_time():
    monitor = ArrivalDwell()
    for index in range(11):
        assert not monitor.update(index / 10, 0.1, 0.0, 0.0)
    for _ in range(50):
        assert not monitor.update(1.0, 0.1, 0.0, 0.0)
    assert not monitor.update(4.0, 0.1, 0.0, 0.0)  # Gap: restart instead of completing.
    assert monitor.started_at == 4.0
    assert not monitor.update(0.0, 0.1, 0.0, 0.0)  # New clock epoch.
    for index in range(1, 20):
        assert not monitor.update(index / 10, 0.1, 0.0, 0.0)
    assert monitor.update(2.0, 0.1, 0.0, 0.0)


def test_custom_radius_duration_and_speed_thresholds_are_used():
    monitor = ArrivalDwell(radius=0.35, duration=1.0, linear_limit=0.005,
                           angular_limit=0.02)
    for index in range(10):
        assert not monitor.update(index / 10, 0.30, 0.004, 0.01)
    assert monitor.update(1.0, 0.30, 0.004, 0.01)


@pytest.mark.parametrize('values', [
    {'radius': 0.0}, {'duration': -1.0}, {'duration': math.inf},
    {'sample_timeout': 0.0}, {'linear_limit': math.nan}, {'angular_limit': -0.01},
])
def test_invalid_completion_parameters_are_rejected(values):
    with pytest.raises(ValueError):
        ArrivalDwell(**values)
