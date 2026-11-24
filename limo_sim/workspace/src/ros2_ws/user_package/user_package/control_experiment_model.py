"""Measured motion, metric footprints and one immutable-reference control run."""

from dataclasses import dataclass, replace
import math

import numpy as np


def quaternion_yaw(q):
    """Normalize a quaternion before extracting a finite planar heading."""
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = (value/norm for value in (q.x, q.y, q.z, q.w))
    return math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))


@dataclass(frozen=True)
class TrackPose:
    """Estimated map pose and measured speed from the same odometry message."""

    stamp: float
    x: float
    y: float
    yaw: float
    speed: float
    segment: int = 0


def map_pose(message, transform, base_frame):
    """Apply map<-odometry at the odometry timestamp, without using cmd_vel."""
    if (not message.header.frame_id
            or message.child_frame_id.lstrip('/') != base_frame.lstrip('/')):
        raise ValueError('Odometry child frame must match the footprint base frame')
    p, velocity = message.pose.pose.position, message.twist.twist.linear
    tx, ty, angle = transform
    c, s = math.cos(angle), math.sin(angle)
    result = TrackPose(
        message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
        tx+c*p.x-s*p.y, ty+s*p.x+c*p.y,
        angle+quaternion_yaw(message.pose.pose.orientation),
        math.hypot(velocity.x, velocity.y))
    if not all(math.isfinite(v) for v in (
            result.stamp, result.x, result.y, result.yaw, result.speed)):
        raise ValueError('Non-finite odometry or map transform')
    return result


def footprint_vertices(pose, footprint):
    """Place the controller's physical polygon at an estimated robot pose."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return [(pose.x+c*x-s*y, pose.y+s*x+c*y) for x, y in footprint]


def validate_footprint(flattened):
    """Keep the launch-provided polygon in metres, with no display scaling."""
    points = np.asarray(flattened, dtype=float)
    if points.size < 6 or points.size % 2 or not np.all(np.isfinite(points)):
        raise ValueError('Footprint needs at least three finite XY vertices')
    points = points.reshape(-1, 2)
    area = np.dot(points[:, 0], np.roll(points[:, 1], 1)) - np.dot(
        points[:, 1], np.roll(points[:, 0], 1))
    if abs(area) < 1e-9:
        raise ValueError('Physical footprint must have non-zero area')
    return tuple(map(tuple, points))


class ControlRecording:
    """Freeze map/reference at start; preserve the whole run up to a hard limit."""

    def __init__(self, footprint, sample_period=0.1, max_samples=50000):
        self.footprint = validate_footprint(np.asarray(footprint).ravel())
        if (not math.isfinite(sample_period) or not 0 < sample_period <= 0.5
                or max_samples < 2):
            raise ValueError('Invalid recording sample period or capacity')
        self.sample_period = sample_period
        self.max_samples = max_samples
        self.revision = 0
        self.clear()

    def clear(self):
        """Reset desktop acquisition only, without issuing robot commands."""
        self.samples = []
        self.map = None
        self.reference = ()
        self.recording = False
        self.latest = None
        self.reason = 'Ready for a new recording'
        self.revision += 1

    def start(self, grid, reference):
        """Require a map/reference pair before beginning a fresh experiment."""
        if self.samples or self.recording:
            raise ValueError('Clear the previous run before starting a new recording')
        if grid is None or len(reference) < 2 or not np.all(np.isfinite(reference)):
            raise ValueError('A complete map and global reference path are required')
        self.map, self.reference = grid, tuple(tuple(p) for p in reference)
        self.recording = True
        self.reason = 'Recording — reference path frozen at start'
        self.revision += 1

    def append(self, pose, force=False):
        """Break the drawn trail at data gaps and localization discontinuities."""
        if not self.recording:
            return False
        if self.latest is not None and pose.stamp < self.latest.stamp - 1.0:
            self.stop('Stopped: odometry clock moved backwards')
            return False
        if self.latest is not None and pose.stamp < self.latest.stamp:
            return False
        self.latest = pose
        if self.samples:
            last = self.samples[-1]
            dt = pose.stamp-last.stamp
            # ROS wall-clock seconds have sub-microsecond float quantization here.
            if dt <= 0 or (dt < self.sample_period-1e-6 and not force):
                return False
            yaw_delta = math.atan2(
                math.sin(pose.yaw-last.yaw), math.cos(pose.yaw-last.yaw))
            broken = (dt > 0.5 or math.hypot(pose.x-last.x, pose.y-last.y) > 0.5
                      or abs(yaw_delta) > math.pi/2)
            pose = replace(pose, segment=last.segment+int(broken))
        if len(self.samples) >= self.max_samples:
            self.recording = False
            self.reason = 'Stopped: sample limit reached; full retained run preserved'
            self.revision += 1
            return False
        self.samples.append(pose)
        self.revision += 1
        return True

    def stop(self, reason='Recording stopped'):
        """Include the last received pose and freeze the experiment for export."""
        if self.recording and self.latest is not None:
            self.append(self.latest, force=True)
        self.recording = False
        self.reason = reason
        self.revision += 1
