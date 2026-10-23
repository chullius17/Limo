"""Bounded, timestamp-based comparison of two odometry streams, without ROS."""

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math


def wrap_angle(angle):
    """Return the signed shortest angle in radians."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Sample:
    """One planar odometry sample, expressed in its declared ROS frames."""

    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    wz: float
    frame: str = 'odom'
    child: str = 'base_link'

    @classmethod
    def from_odometry(cls, msg):
        """Extract yaw from a normalized quaternion, rejecting invalid data."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError('Invalid odometry quaternion')
        x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
        sample = cls(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            p.x, p.y, math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)),
            msg.twist.twist.linear.x, msg.twist.twist.angular.z,
            msg.header.frame_id, msg.child_frame_id)
        if not all(math.isfinite(v) for v in (
                sample.stamp, sample.x, sample.y, sample.yaw, sample.vx, sample.wz)):
            raise ValueError('Non-finite odometry data')
        return sample


@dataclass(frozen=True)
class Difference:
    """Filtered minus raw at a common timestamp; this is not ground-truth error."""

    stamp: float
    dx: float
    dy: float
    distance: float
    dyaw: float
    dvx: float
    dwz: float


def compare(raw, filtered, max_gap):
    """Interpolate filtered data at raw timestamps, without extrapolation."""
    stamps = [sample.stamp for sample in filtered]
    result = []
    for sample in raw:
        i = bisect_left(stamps, sample.stamp)
        if i == len(filtered):
            continue
        right = filtered[i]
        if right.stamp == sample.stamp:
            left = right
            weight = 0.0
        elif i == 0:
            continue
        else:
            left = filtered[i - 1]
            gap = right.stamp - left.stamp
            if gap <= 0 or gap > max_gap:
                continue
            weight = (sample.stamp - left.stamp) / gap
        frames = (sample.frame, sample.child)
        if not all(frames) or any(
                (item.frame, item.child) != frames for item in (left, right)):
            continue
        x = left.x + weight * (right.x - left.x)
        y = left.y + weight * (right.y - left.y)
        yaw = left.yaw + weight * wrap_angle(right.yaw - left.yaw)
        vx = left.vx + weight * (right.vx - left.vx)
        wz = left.wz + weight * (right.wz - left.wz)
        dx, dy = x - sample.x, y - sample.y
        result.append(Difference(
            sample.stamp, dx, dy, math.hypot(dx, dy),
            wrap_angle(yaw - sample.yaw), vx - sample.vx, wz - sample.wz))
    return result


class ExperimentHistory:
    """Keep bounded histories and restart cleanly after simulation time resets."""

    def __init__(self, history_seconds=120.0, max_samples=20000):
        self.history_seconds = history_seconds
        self.streams = {name: deque(maxlen=max_samples) for name in ('raw', 'filtered')}
        self.origin = None
        self.reset_count = 0

    def clear(self):
        """Discard both streams and their shared time origin."""
        for stream in self.streams.values():
            stream.clear()
        self.origin = None

    def append(self, name, sample):
        """Ignore small out-of-order packets and clear on large backwards jumps."""
        stream = self.streams[name]
        if stream and sample.stamp < stream[-1].stamp - 1.0:
            self.clear()
            self.reset_count += 1
        elif stream and sample.stamp < stream[-1].stamp:
            return
        if self.origin is None:
            self.origin = sample.stamp
        if stream and sample.stamp == stream[-1].stamp:
            stream[-1] = sample
        else:
            stream.append(sample)
        newest = max(s[-1].stamp for s in self.streams.values() if s)
        for values in self.streams.values():
            while values and values[0].stamp < newest - self.history_seconds:
                values.popleft()

    def snapshot(self):
        """Copy data so GUI work can happen outside the subscription lock."""
        return {name: list(values) for name, values in self.streams.items()}
