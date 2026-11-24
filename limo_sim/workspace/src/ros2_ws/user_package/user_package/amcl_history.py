"""Metric map geometry and bounded AMCL history, independent of ROS and Qt."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from user_package.map_geometry import MapSnapshot  # noqa: F401 — compatibility import


# A circle enclosing the 95% XY Gaussian ellipse, including XY correlation.
XY_95_SCALE = math.sqrt(-2.0 * math.log(0.05))


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
