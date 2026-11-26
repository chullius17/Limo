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

"""Recognize a continuous measured stop inside the final goal circle."""

import math


class ArrivalDwell:
    """Accumulate dwell only across fresh, consecutive qualifying samples."""

    def __init__(self, radius=0.20, duration=2.0, linear_limit=0.01,
                 angular_limit=0.05, sample_timeout=0.5, yaw_tolerance=math.radians(20.0)):
        for name, value in (('radius', radius), ('duration', duration),
                            ('sample_timeout', sample_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        for name, value in (('linear_limit', linear_limit), ('angular_limit', angular_limit)):
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + ' must be finite and nonnegative')
        if not math.isfinite(yaw_tolerance) or not 0 <= yaw_tolerance <= math.pi:
            raise ValueError('yaw_tolerance must be finite and between 0 and pi radians')
        self.radius = radius
        self.duration = duration
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.sample_timeout = sample_timeout
        self.yaw_tolerance = yaw_tolerance
        self.reset()

    def reset(self):
        """Discard all stopped time collected for the previous condition."""
        self.started_at = None
        self.last_sample_at = None

    def update(self, stamp, distance, linear_speed, angular_speed, yaw_error=0.0):
        """Use measurement time, so a paused clock cannot complete the dwell."""
        if not all(math.isfinite(v) for v in (
                stamp, distance, linear_speed, angular_speed, yaw_error)):
            self.reset()
            return False
        if (self.last_sample_at is not None and
                not 0 <= stamp - self.last_sample_at <= self.sample_timeout):
            self.reset()
        self.last_sample_at = stamp
        yaw_error = abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error)))
        if (not 0 <= distance <= self.radius or yaw_error > self.yaw_tolerance + 1e-12
                or abs(linear_speed) > self.linear_limit
                or abs(angular_speed) > self.angular_limit):
            self.started_at = None
            return False
        if self.started_at is None:
            self.started_at = stamp
        return stamp - self.started_at >= self.duration
