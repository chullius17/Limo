"""Metric map geometry and bounded AMCL history, independent of ROS and Qt."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


# A circle enclosing the 95% XY Gaussian ellipse, including XY correlation.
XY_95_SCALE = math.sqrt(-2.0 * math.log(0.05))


@dataclass(frozen=True)
class MapSnapshot:
    """Own the complete occupancy grid and its planar origin transform."""

    frame: str
    resolution: float
    origin: tuple
    values: np.ndarray

    @classmethod
    def from_message(cls, message):
        """Reject invalid maps before replacing the displayed raster."""
        info = message.info
        if (info.width <= 0 or info.height <= 0
                or len(message.data) != info.width * info.height
                or not math.isfinite(info.resolution) or info.resolution <= 0
                or not message.header.frame_id):
            raise ValueError('Invalid complete map geometry or frame')
        q = info.origin.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError('Invalid map origin quaternion')
        x, y, z, w = (value / norm for value in (q.x, q.y, q.z, q.w))
        origin = (info.origin.position.x, info.origin.position.y,
                  math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z)))
        if not all(math.isfinite(value) for value in origin):
            raise ValueError('Non-finite map origin')
        values = np.array(message.data, dtype=np.int16).reshape(info.height, info.width)
        if np.any((values < -1) | (values > 100)):
            raise ValueError('Map occupancy must be in [-1, 100]')
        values.flags.writeable = False
        return cls(message.header.frame_id.lstrip('/'), info.resolution, origin, values)

    def bounds(self):
        """Return world bounds, including a rotated and translated grid origin."""
        height, width = self.values.shape
        x, y, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x + c*u - s*v, y + s*u + c*v)
                   for u in (0, width*self.resolution)
                   for v in (0, height*self.resolution)]
        return (min(p[0] for p in corners), max(p[0] for p in corners),
                min(p[1] for p in corners), max(p[1] for p in corners))

    def image(self):
        """Preserve all occupancy levels; distinguish unknown from free space."""
        palette = np.ones((102, 3))
        palette[0] = (0.55, 0.59, 0.64)
        palette[1:] = np.linspace(1.0, 0.0, 101)[:, None]
        return palette[self.values + 1]


@dataclass(frozen=True)
class HistoryPose:
    """One AMCL mean and uncertainty in the complete map's metric frame."""

    x: float
    y: float
    yaw: float
    elapsed: float
    radius: float
    yaw_std: float


class LocalizationHistory:
    """Retain sampled circles while always updating the latest accepted pose."""

    def __init__(self, max_samples=5000, sample_period=0.25):
        if max_samples < 2 or not math.isfinite(sample_period) or sample_period < 0:
            raise ValueError('Invalid history size or sample period')
        self.samples = deque(maxlen=max_samples)
        self.sample_period = sample_period
        self.map = None
        self.revision = 0
        self.clock_resets = 0
        self.clear()

    def clear(self):
        """Start a new color timeline without discarding the complete map."""
        self.samples.clear()
        self.latest = None
        self.first_stamp = self.last_stamp = self.last_sample_stamp = None
        self.revision += 1

    def set_map(self, grid):
        """Never overlay historical coordinates from a different frame."""
        if self.map is not None and self.map.frame != grid.frame:
            self.clear()
        self.map = grid
        self.revision += 1

    def append(self, row):
        """Use AMCL timestamps, and restart colors after a ROS clock rollback."""
        if self.map is None:
            return False
        if row['frame_id'].lstrip('/') != self.map.frame:
            raise ValueError('AMCL pose frame does not match complete map frame')
        xy = np.array([[row['cov_0_0'], row['cov_0_1']],
                       [row['cov_1_0'], row['cov_1_1']]])
        if not np.allclose(xy, xy.T, rtol=1e-6, atol=1e-10):
            raise ValueError('XY covariance is not symmetric')
        eigenvalues = np.linalg.eigvalsh((xy + xy.T) / 2)
        if not np.all(np.isfinite(eigenvalues)) or eigenvalues[0] < -1e-10:
            raise ValueError('XY covariance is not positive semidefinite')
        stamp = row['stamp_s']
        if self.last_stamp is not None:
            if stamp < self.last_stamp - 1.0:
                self.clear()
                self.clock_resets += 1
            elif stamp <= self.last_stamp:
                return False
        if self.first_stamp is None:
            self.first_stamp = stamp
        sample = HistoryPose(
            row['x'], row['y'], row['yaw'], stamp-self.first_stamp,
            XY_95_SCALE*math.sqrt(max(0.0, eigenvalues[-1])),
            math.sqrt(row['cov_5_5']))
        self.latest = sample
        self.last_stamp = stamp
        if (self.last_sample_stamp is None
                or stamp-self.last_sample_stamp >= self.sample_period - 1e-9):
            self.samples.append(sample)
            self.last_sample_stamp = stamp
        self.revision += 1
        return True
