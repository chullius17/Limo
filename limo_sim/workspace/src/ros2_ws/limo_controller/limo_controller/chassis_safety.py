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

"""Fail-closed chassis checks without changing the robot's control mode."""

import math
import time


class ChassisSafety:
    """Require fresh, healthy feedback in command mode on physical robots."""

    def __init__(self, enabled=False, commanded_mode=1, timeout=1.0):
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('chassis_status_timeout must be finite and positive')
        if (not isinstance(commanded_mode, int) or isinstance(commanded_mode, bool)
                or not 0 <= commanded_mode <= 255):
            raise ValueError('commanded_control_mode must be a uint8')
        self.enabled = enabled
        self.commanded_mode = commanded_mode
        self.timeout = timeout
        self.status = None
        self.received_at = None

    def update(self, status, now=None):
        """Record chassis feedback using wall time, not a simulation clock."""
        self.status = status
        self.received_at = time.monotonic() if now is None else now

    def reason(self, now=None):
        """Return a blocking reason or None; never infer mechanical drive mode."""
        if not self.enabled:
            return None
        if self.status is None:
            return 'No chassis status received on /limo_status; START blocked'
        now = time.monotonic() if now is None else now
        if not 0.0 <= now - self.received_at <= self.timeout:
            return 'Chassis status is stale; control stopped, request START again'
        if self.status.vehicle_state != 0 or self.status.error_code != 0:
            return ('Chassis not healthy: vehicle_state={} error_code={}; '
                    'control blocked').format(
                        self.status.vehicle_state, self.status.error_code)
        if self.status.control_mode != self.commanded_mode:
            return ('Chassis control_mode={} (required {}); release phone/remote '
                    'control and restore command mode before START').format(
                        self.status.control_mode, self.commanded_mode)
        return None
