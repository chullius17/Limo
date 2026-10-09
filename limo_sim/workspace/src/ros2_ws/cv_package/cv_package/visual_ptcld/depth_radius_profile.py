"""Discrete metric radii selected by each point's camera optical depth."""

import numpy as np


class DepthRadiusProfile:
    """Use upper-inclusive depth bands and repeat the last supplied radius."""

    def __init__(self, breakpoints, minimum, maximum, propagation):
        self.breakpoints = np.asarray(breakpoints, dtype=np.float64)
        if (self.breakpoints.ndim != 1
                or not np.isfinite(self.breakpoints).all()
                or np.any(self.breakpoints <= 0.0)
                or np.any(np.diff(self.breakpoints) <= 0.0)):
            raise ValueError(
                'radius_depth_breakpoints_m must be finite, positive and strictly increasing')
        band_count = len(self.breakpoints) + 1
        columns = []
        for name, values in (
                ('blue_radius_min_m', minimum),
                ('blue_radius_max_m', maximum),
                ('boardwalk_propagation_radius_m', propagation)):
            values = np.atleast_1d(np.asarray(values, dtype=np.float64))
            if values.ndim != 1 or not 1 <= len(values) <= band_count:
                raise ValueError(
                    f'{name} must have 1 to {band_count} values '
                    '(at most number of depth breakpoints + 1)')
            columns.append(np.pad(values, (0, band_count - len(values)), mode='edge'))
        self.radii = np.stack(columns)
        if (not np.isfinite(self.radii).all()
                or np.any(self.radii < 0.0)
                or np.any(self.radii[0] > self.radii[1])):
            raise ValueError('Depth radii must be finite, non-negative and min <= max in every band')

    @property
    def enabled(self):
        """Without breakpoints the profile uses one constant radius per rule."""
        return bool(self.breakpoints.size)

    def resolve(self, depths):
        """Return three aligned radius arrays without interpolation."""
        if not self.enabled:
            return tuple(self.radii[:, 0])
        depths = np.asarray(depths)
        # Corrected images use float32: equality must use the same representation.
        boundaries = (self.breakpoints.astype(depths.dtype, copy=False)
                      if np.issubdtype(depths.dtype, np.floating) else self.breakpoints)
        levels = np.searchsorted(boundaries, depths, side='left')
        return tuple(self.radii[:, levels])

    def describe(self):
        """Summarize effective bands for startup and periodic diagnostics."""
        bands = []
        lower = 0.0
        for upper, (minimum, maximum, propagation) in zip(
                np.append(self.breakpoints, np.inf), self.radii.T):
            bounds = (f'{lower:.3f} < depth <= {upper:.3f} m'
                      if np.isfinite(upper) else f'depth > {lower:.3f} m')
            bands.append(f'{bounds}: {minimum:.3f}/{maximum:.3f}/{propagation:.3f} m')
            lower = upper
        return 'depth bands (min/max/propagation): ' + '; '.join(bands)
