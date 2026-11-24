"""Complete robot occupancy-map geometry shared by desktop experiment views."""

from dataclasses import dataclass
import math

import numpy as np


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
