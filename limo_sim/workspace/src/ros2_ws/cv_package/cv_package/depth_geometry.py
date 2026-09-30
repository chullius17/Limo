"""Calibrated depth registration and robust ground geometry, without ROS."""

import numpy as np


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


def register_depth(depth, rays, rotation, translation, rgb_k, rgb_size,
                   output_size, crop=.5, min_depth=.1, max_depth=5.0):
    """Forward project measured points; keep the nearest RGB Z in each pixel.

    No interpolation across holes or foreground/background boundaries. Input
    and target calibrations describe rectified/zero-distortion images.
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
