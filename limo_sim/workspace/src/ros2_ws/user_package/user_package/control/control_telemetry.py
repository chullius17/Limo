"""Reference distance and timestamped telemetry for one control execution."""

import math

import numpy as np


def reference_distance(x, y, reference):
    """Measure distance to line segments, including repeated reference vertices."""
    points = np.asarray(reference, dtype=float)
    start, delta = points[:-1], np.diff(points, axis=0)
    squared = np.sum(delta*delta, axis=1)
    fraction = np.divide(np.sum((np.array((x, y))-start)*delta, axis=1), squared,
                         out=np.zeros_like(squared), where=squared > 0)
    closest = start+np.clip(fraction, 0, 1)[:, None]*delta
    return float(np.min(np.linalg.norm(closest-(x, y), axis=1)))


def scan_minimum(message):
    """Ignore non-finite and out-of-range returns; NaN leaves a visible data gap."""
    valid = [value for value in message.ranges if math.isfinite(value)
             and max(0.0, message.range_min) <= value <= message.range_max]
    return min(valid) if valid else float('nan')


class ControlTelemetry:
    """Pause the acquisition clock and retain terminal results until cleared."""

    def __init__(self, capacity=50000):
        self.capacity = capacity
        self.clear()

    def clear(self):
        self.distance = []
        self.scan = []
        self.pauses = []
        self.avoidance = []
        self.origin = None
        self.pause_stamp = None
        self.paused_seconds = 0.0
        self.end = 0.0
        self.status = 'IDLE'
        self.revision = 0

    @property
    def recording(self):
        return self.status in ('RECORDING', 'PAUSED')

    def start(self, stamp):
        self.clear()
        self.origin = stamp
        self.status = 'RECORDING'
        self.revision += 1

    def elapsed(self, stamp):
        if self.origin is None:
            return 0.0
        return max(0.0, (self.pause_stamp if self.pause_stamp is not None else stamp)
                   - self.origin - self.paused_seconds)

    def append(self, stream, stamp, value):
        if self.status != 'RECORDING' or stamp < self.origin:
            return False
        values = getattr(self, stream)
        elapsed = self.elapsed(stamp)
        if values and elapsed <= values[-1][0]:
            return False
        if len(values) >= self.capacity:
            self.stop(stamp, 'SAMPLE LIMIT')
            return False
        values.append((elapsed, value))
        self.end = max(self.end, elapsed)
        self.revision += 1
        return True

    def set_avoidance(self, active, stamp):
        if self.status != 'RECORDING':
            return
        opened = bool(self.avoidance and self.avoidance[-1][1] is None)
        elapsed = max(self.end, self.elapsed(stamp))
        if active and not opened:
            self.avoidance.append([elapsed, None])
            self.revision += 1
        elif not active and opened:
            self.avoidance[-1][1] = elapsed
            self.revision += 1
        self.end = elapsed

    def pause(self, stamp):
        if self.status != 'RECORDING':
            return
        self.set_avoidance(False, stamp)
        self.pause_stamp = max(stamp, self.origin+self.paused_seconds+self.end)
        elapsed = self.elapsed(self.pause_stamp)
        self.pauses.append(elapsed)
        for values in (self.distance, self.scan):
            if values:
                values.append((elapsed, float('nan')))
        self.end = elapsed
        self.status = 'PAUSED'
        self.revision += 1

    def resume(self, stamp):
        if self.status == 'PAUSED':
            self.paused_seconds += max(0.0, stamp-self.pause_stamp)
            self.pause_stamp = None
            self.status = 'RECORDING'
            self.revision += 1

    def stop(self, stamp, status):
        if self.recording:
            if self.status == 'RECORDING':
                self.set_avoidance(False, stamp)
            self.end = max(self.end, self.elapsed(stamp))
            self.status = status
            self.revision += 1
