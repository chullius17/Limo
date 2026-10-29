"""Calibrated depth registration and robust ground geometry, without ROS."""

import numpy as np

try:
    from ._registration_native import project_depth as _native_project_depth
except ImportError:
    _native_project_depth = None


def pixel_rays(k, source_width, source_height, width, height, crop=0.0):
    """Rays at output pixel centres, using the same crop as the RGB labels."""
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or min(source_width, source_height, width, height) <= 0
            or not 0 <= crop < 1):
        raise ValueError('Invalid camera calibration or image geometry')
    start = int(source_height * crop)
    u = (np.arange(width) + .5) * source_width / width - .5
    v = start + (np.arange(height) + .5) * (source_height - start) / height - .5
    u, v = np.meshgrid(u, v)
    return np.stack(((u - k[0, 2]) / k[0, 0],
                     (v - k[1, 2]) / k[1, 1], np.ones_like(u)), axis=-1)


def registration_source_crop(rays, rotation, translation, rgb_k, rgb_size,
                             crop=.5, min_depth=.1, max_depth=5.0):
    """Bound source pixels that can reach the RGB crop at any allowed depth.

    Clip each source ray's depth interval against the RGB viewing frustum.
    This runs only when calibration changes, before any image downsampling.
    A literal lower-half source crop would be wrong for distinct RGB optics.
    """
    directions = rays @ rotation.T
    k = np.asarray(rgb_k).reshape(3, 3)
    width, height = rgb_size
    lower = np.full(rays.shape[:2], min_depth, dtype=np.float64)
    upper = np.full(rays.shape[:2], max_depth, dtype=np.float64)
    constraints = [(directions[..., 2], translation[2] - min_depth),
                   (-directions[..., 2], max_depth - translation[2])]
    for axis, low, high, focal, center in (
            (0, -.5, width - .5, k[0, 0], k[0, 2]),
            (1, int(height * crop) - .5, height - .5, k[1, 1], k[1, 2])):
        low, high = (low - center) / focal, (high - center) / focal
        constraints.extend([
            (directions[..., axis] - low * directions[..., 2],
             translation[axis] - low * translation[2]),
            (high * directions[..., 2] - directions[..., axis],
             high * translation[2] - translation[axis]),
        ])
    for coefficient, offset in constraints:
        boundary = np.zeros_like(lower)
        np.divide(-offset, coefficient, out=boundary, where=coefficient != 0)
        np.maximum(lower, boundary, out=lower, where=coefficient > 0)
        np.minimum(upper, boundary, out=upper, where=coefficient < 0)
        upper[(coefficient == 0) & (offset < 0)] = -np.inf
    rows, cols = np.nonzero((lower <= upper) & np.isfinite(directions).all(axis=2))
    if not len(rows):
        # Keep resize well-defined when the cameras have no shared field of view.
        return slice(0, 1), slice(0, 1)
    return (slice(int(rows.min()), int(rows.max()) + 1),
            slice(int(cols.min()), int(cols.max()) + 1))


def register_depth(depth, rays, rotation, translation, rgb_k, rgb_size,
                   output_size, crop=.5, min_depth=.1, max_depth=5.0):
    """Forward project measured points; keep the nearest RGB Z in each pixel.

    No interpolation across holes or foreground/background boundaries. Input
    and target calibrations describe rectified/zero-distortion images.

    Args:
            depth: Array of shape (H_depth, W_depth) containing optical-axis
                depth (Z) in metres, expressed in the depth camera frame.
                Non-finite values and values outside the allowed range are ignored.
    
            rays: Array of shape (H_depth, W_depth, 3) containing the depth
                camera ray for each input pixel:
                [(u - cx_depth) / fx_depth, (v - cy_depth) / fy_depth, 1].
                These vectors are not unit-length; multiplying by depth gives
                the corresponding 3D point in the depth camera frame.
    
            rotation: Rotation matrix of shape (3, 3) that transforms points
                from the depth optical frame into the RGB optical frame.
    
            translation: Translation vector of shape (3,), in metres, used
                with rotation:
                point_rgb = rotation @ point_depth + translation.
    
            rgb_k: RGB intrinsic calibration matrix, supplied as a flat
                9-element sequence or a (3, 3) array:
                [[fx_rgb, 0, cx_rgb], [0, fy_rgb, cy_rgb], [0, 0, 1]].
                Values are in pixels and refer to the full RGB resolution.
                Both camera calibrations must describe rectified images
                or images with zero distortion.
    
            rgb_size: Tuple (width, height) of the full RGB image, in pixels.
                This is the resolution associated with rgb_k.
    
            output_size: Tuple (width, height) of the output depth image,
                in pixels, representing the retained RGB region.
    
            crop: Fraction of the RGB image height discarded from the top,
                in [0, 1). The first retained row is int(rgb_height * crop).
                For example, 0.5 keeps the lower half; 0.0 keeps the full image.
    
            min_depth: Minimum accepted optical-axis depth in metres,
                inclusive. Applied to both input depth and transformed RGB Z.
    
            max_depth: Maximum accepted optical-axis depth in metres,
                inclusive. Applied to both input depth and transformed RGB Z.
    
        Returns:
            float32 array of shape (output_height, output_width), containing
            RGB optical-axis depth in metres. If multiple points map to the
            same pixel, the smallest RGB Z is retained. Pixels without a
            measurement contain NaN; no interpolation or hole filling is applied.
    """
    width, height = output_size
    rgb_width, rgb_height = rgb_size
    k = np.asarray(rgb_k).reshape(3, 3)
    with np.errstate(invalid='ignore'):
        valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    points = (rays[valid] * depth[valid, None]) @ rotation.T + translation
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
    start = int(rgb_height * crop)
    u = points[:, 0] / points[:, 2] * k[0, 0] + k[0, 2]
    v = points[:, 1] / points[:, 2] * k[1, 1] + k[1, 2]
    u = (u + .5) * width / rgb_width - .5
    v = (v - start + .5) * height / (rgb_height - start) - .5
    visible = ((u >= -.5) & (u < width - .5)
               & (v >= -.5) & (v < height - .5)
               & (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth))
    cols = np.floor(u[visible] + .5).astype(np.int32)
    rows = np.floor(v[visible] + .5).astype(np.int32)
    result = np.full(height * width, np.inf, dtype=np.float32)
    np.minimum.at(result, rows * width + cols, points[visible, 2])
    result[~np.isfinite(result)] = np.nan
    return result.reshape(height, width)


class DepthRegistration:
    """Cache rigid ray rotation and reuse the registration output buffer.

    The returned float32 array belongs to this instance and is overwritten by
    the next call. Copy it when retaining an earlier frame. Depth-dependent
    translation, projection and nearest-Z visibility are evaluated per frame.
    """

    def __init__(self, rays, rotation, translation, rgb_k, rgb_size,
                 output_size, crop=.5, min_depth=.1, max_depth=5.0,
                 use_native=True):
        self.rays = np.ascontiguousarray(np.asarray(rays, dtype=np.float64)
                                         @ np.asarray(rotation, dtype=np.float64).T)
        k = np.asarray(rgb_k, dtype=np.float64).reshape(3, 3)
        width, height = output_size
        rgb_width, rgb_height = rgb_size
        self.parameters = np.array([
            *translation, k[0, 0], k[1, 1], k[0, 2], k[1, 2],
            rgb_width, rgb_height, int(rgb_height * crop), min_depth, max_depth,
        ], dtype=np.float64)
        if (self.rays.ndim != 3 or self.rays.shape[2] != 3
                or not np.isfinite(self.rays).all()
                or not np.isfinite(self.parameters).all()
                or min(width, height, rgb_width, rgb_height) <= 0
                or not 0 <= crop < 1 or not 0 < min_depth < max_depth):
            raise ValueError('Invalid registration geometry')
        self.output = np.empty((height, width), dtype=np.float32)
        self._native = _native_project_depth if use_native else None
        self.backend = 'native' if self._native is not None else 'numpy'
        self._points = None

    def register(self, depth):
        """Register float32/float64 optical Z, including strided source views."""
        if depth.shape != self.rays.shape[:2] or depth.dtype not in (np.float32, np.float64):
            raise ValueError('Depth shape/dtype does not match registration geometry')
        if self._native is not None:
            self._native(depth, self.rays, self.output, self.parameters)
            return self.output
        if self._points is None:
            self._points = np.empty_like(self.rays)
            self._u = np.zeros(depth.shape, dtype=np.float64)
            self._v = np.zeros(depth.shape, dtype=np.float64)
        tx, ty, tz, fx, fy, cx, cy, rw, rh, start, near, far = self.parameters
        height, width = self.output.shape
        with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
            np.multiply(self.rays, depth[..., None], out=self._points)
            np.add(self._points, [tx, ty, tz], out=self._points)
            x, y, z = (self._points[..., i] for i in range(3))
            valid = (np.isfinite(depth) & (depth >= near) & (depth <= far)
                     & np.isfinite(z) & (z >= near) & (z <= far))
            np.divide(x, z, out=self._u, where=valid)
            np.divide(y, z, out=self._v, where=valid)
            self._u *= fx
            self._u += cx
            self._u += .5
            self._u *= width
            self._u /= rw
            self._u -= .5
            self._v *= fy
            self._v += cy
            self._v -= start
            self._v += .5
            self._v *= height
            self._v /= rh - start
            self._v -= .5
            valid &= ((self._u >= -.5) & (self._u < width - .5)
                      & (self._v >= -.5) & (self._v < height - .5))
        cols = np.floor(self._u[valid] + .5).astype(np.intp)
        rows = np.floor(self._v[valid] + .5).astype(np.intp)
        self.output.fill(np.inf)
        np.minimum.at(self.output.ravel(), rows * width + cols, z[valid])
        self.output[~np.isfinite(self.output)] = np.nan
        return self.output

def fit_ground_plane(points, threshold=.008, min_points=150,
                     min_inlier_ratio=.6, max_tilt_deg=20.0):
    """Fit n.p+d=0 in a frame with Z up, rejecting walls and narrow support."""
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < min_points:
        raise ValueError('Not enough measured ground candidates')
    rng = np.random.RandomState(42)
    if len(points) > 2500:
        points = points[rng.choice(len(points), 2500, replace=False)]
    min_z = np.cos(np.deg2rad(max_tilt_deg))
    best = None
    best_count = 0
    for _ in range(96):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        offset = -normal @ sample[0]
        if normal[2] < min_z or not .05 <= offset <= .6:
            continue
        mask = np.abs(points @ normal + offset) <= threshold
        if np.count_nonzero(mask) > best_count:
            best, best_count = mask, np.count_nonzero(mask)
    if best is None or best_count < max(min_points, min_inlier_ratio * len(points)):
        raise ValueError('No sufficiently supported ground plane; walls rejected')
    for _ in range(3):
        selected = points[best]
        if len(selected) < min_points:
            raise ValueError('Ground support lost during refinement')
        center = selected.mean(axis=0)
        _, singular, vectors = np.linalg.svd(selected - center, full_matrices=False)
        normal = vectors[-1]
        if normal[2] < 0:
            normal = -normal
        offset = -normal @ center
        best = np.abs(points @ normal + offset) <= threshold
    if (normal[2] < min_z or not .05 <= offset <= .6
            or np.count_nonzero(best) < max(min_points, min_inlier_ratio * len(points))
            or singular[1] / np.sqrt(len(selected)) < .025):
        raise ValueError('Ground plane is degenerate or has insufficient coverage')
    return np.r_[normal, offset]


def ground_depth(rays, rotation, translation, plane, min_depth=.1, max_depth=5.0):
    """Intersect target-camera rays with a plane expressed in the parent frame."""
    normal, offset = plane[:3], plane[3]
    denominator = rays @ (rotation.T @ normal)
    numerator = -(offset + normal @ translation)
    result = np.full(rays.shape[:2], np.nan, dtype=np.float32)
    visible = denominator < -1e-6
    result[visible] = numerator / denominator[visible]
    with np.errstate(invalid='ignore'):
        result[(result < min_depth) | (result > max_depth)] = np.nan
    return result


def complete_depth(measured, ground):
    """Fill every missing return with the plane, without consulting class labels."""
    result = measured.copy()
    with np.errstate(invalid='ignore'):
        missing = ~np.isfinite(result) | (result <= 0)
        fill = missing & np.isfinite(ground) & (ground > 0)
    result[fill] = ground[fill]
    return result
