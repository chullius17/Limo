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

"""Exercise fail-closed chassis checks independently of ROS discovery."""

from types import SimpleNamespace

import pytest

from limo_controller.chassis_safety import ChassisSafety


def status(**overrides):
    values = dict(vehicle_state=0, control_mode=1, error_code=0, motion_mode=2)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_simulation_does_not_require_chassis_feedback():
    assert ChassisSafety().reason(now=10.0) is None


def test_real_requires_fresh_feedback_but_preserves_raw_motion_mode_override():
    guard = ChassisSafety(enabled=True)
    assert 'No chassis status' in guard.reason(now=10.0)
    guard.update(status(), now=10.0)
    assert guard.reason(now=10.5) is None
    assert 'stale' in guard.reason(now=11.01)
    assert 'stale' in guard.reason(now=9.0)


@pytest.mark.parametrize('override,reason', [
    ({'control_mode': 2}, 'control_mode=2 (required 1)'),
    ({'vehicle_state': 1}, 'vehicle_state=1'),
    ({'error_code': 4}, 'error_code=4'),
])
def test_bad_chassis_state_blocks_control(override, reason):
    guard = ChassisSafety(enabled=True)
    guard.update(status(**override), now=10.0)
    assert reason in guard.reason(now=10.1)


@pytest.mark.parametrize('timeout', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_timeout_is_rejected(timeout):
    with pytest.raises(ValueError, match='timeout'):
        ChassisSafety(timeout=timeout)


@pytest.mark.parametrize('mode', [-1, 256, True, 1.0])
def test_invalid_command_mode_is_rejected(mode):
    with pytest.raises(ValueError, match='uint8'):
        ChassisSafety(commanded_mode=mode)
