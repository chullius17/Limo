"""Pure geometry and semantic processing for the visual point-cloud node."""

import time

import numpy as np

from .cloud_cpu import transform_xy, voxel_groups


LABEL_INVALID = np.uint8(0)
LABEL_BLUE = np.uint8(1)
LABEL_TURQUOISE = np.uint8(2)
LABEL_BACKGROUND = np.uint8(3)
LABEL_BOARDWALK = np.uint8(4)
LABEL_INTERIOR_BLUE = np.uint8(5)
LABEL_INTERIOR_BOARDWALK = np.uint8(6)

CLOUD_DTYPE = np.dtype({
    'names': ('x', 'y', 'z', 'class_id'),
    'formats': ('<f4', '<f4', '<f4', 'u1'),
    'offsets': (0, 4, 8, 12),
    'itemsize': 16,
})

PUBLISHED_LABELS = (
    ('published_blue_count', LABEL_BLUE),
    ('published_turquoise_count', LABEL_TURQUOISE),
    ('published_background_count', LABEL_BACKGROUND),
    ('published_boardwalk_count', LABEL_BOARDWALK),
    ('published_interior_blue_count', LABEL_INTERIOR_BLUE),
    ('published_interior_boardwalk_count', LABEL_INTERIOR_BOARDWALK),
)


def voxelize_pixels(raw_points, image_width, voxel_size):
    """Replace all points in each image cell with their rounded centroid."""
    if voxel_size == 1 or len(raw_points) == 0:
        return raw_points

    voxel_columns = (image_width + voxel_size - 1) // voxel_size
    voxel_ids = (
        (raw_points[:, 0] // voxel_size) * voxel_columns
        + raw_points[:, 1] // voxel_size
    )
    counts = np.bincount(voxel_ids)
    sum_y = np.bincount(voxel_ids, weights=raw_points[:, 0])
    sum_x = np.bincount(voxel_ids, weights=raw_points[:, 1])
    occupied = np.flatnonzero(counts)

    centroids_y = np.rint(
        sum_y[occupied] / counts[occupied]).astype(np.int32)
    centroids_x = np.rint(
        sum_x[occupied] / counts[occupied]).astype(np.int32)
    return np.column_stack((centroids_y, centroids_x))


def voxelize_metric(points, class_ids, voxel_size):
    """Downsample finite BEV points independently for every semantic class."""
    if len(points) == 0:
        return points, class_ids

    finite = np.isfinite(points).all(axis=1)
    passthrough = ~finite
    reduced_indices = np.flatnonzero(finite)
    if not len(reduced_indices):
        return points, class_ids

    reduced_points = points[reduced_indices]
    first, inverse = voxel_groups(
        reduced_points, class_ids[reduced_indices], voxel_size)
    counts = np.bincount(inverse)
    centroids = np.column_stack((
        np.bincount(inverse, weights=reduced_points[:, 0]) / counts,
        np.bincount(inverse, weights=reduced_points[:, 1]) / counts,
    )).astype(points.dtype, copy=False)

    representatives = reduced_indices[first]
    keep = passthrough.copy()
    keep[representatives] = True
    output_points = points.copy()
    output_points[representatives] = centroids
    return output_points[keep], class_ids[keep]


def quaternion_to_rotation(quaternion):
    """Return the rotation matrix represented by a ROS quaternion."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        raise ValueError('TF contains an invalid zero quaternion')

    scale = 2.0 / norm
    return np.array([
        [1.0 - scale * (y * y + z * z),
         scale * (x * y - z * w),
         scale * (x * z + y * w)],
        [scale * (x * y + z * w),
         1.0 - scale * (x * x + z * z),
         scale * (y * z - x * w)],
        [scale * (x * z - y * w),
         scale * (y * z + x * w),
         1.0 - scale * (x * x + y * y)],
    ], dtype=np.float32)


def build_semantic_cloud(
        point_groups, depth, intrinsics, width, height, input_crop_y_min,
        ray_cache, rotation, translation, cloud_min_depth, cloud_max_depth,
        boardwalk_classifier, blue_radius_min, blue_radius_max,
        boardwalk_propagation_radius, road_boardwalk_only, voxel_size,
        depth_radius_profile=None):
    """Project labeled pixels, classify boardwalk and reduce the outgoing cloud."""
    projection_started_at = time.perf_counter()
    nonempty_groups = [
        (points, label) for points, label in point_groups if len(points)
    ]
    if nonempty_groups:
        pixels = np.concatenate(
            [points for points, _ in nonempty_groups], axis=0)
        class_ids = np.concatenate([
            np.full(len(points), label, dtype=np.uint8)
            for points, label in nonempty_groups
        ])
    else:
        pixels = np.empty((0, 2), dtype=np.int32)
        class_ids = np.empty(0, dtype=np.uint8)

    rows = pixels[:, 0]
    cols = pixels[:, 1]
    z = depth[rows, cols]
    valid = (
        np.isfinite(z)
        & (z >= cloud_min_depth)
        & (z <= cloud_max_depth)
    )
    rows = rows[valid]
    cols = cols[valid]
    z = z[valid].astype(np.float32, copy=False)
    class_ids = class_ids[valid]

    ray_x, ray_y = ray_cache.get(
        intrinsics, width, height, input_crop_y_min)
    x = ray_x[cols] * z
    y = ray_y[rows] * z
    projection_ms = (
        time.perf_counter() - projection_started_at) * 1000.0

    transform_started_at = time.perf_counter()
    bev_points = transform_xy(x, y, z, rotation, translation)
    transform_ms = (
        time.perf_counter() - transform_started_at) * 1000.0

    stats = {}
    if boardwalk_classifier is not None:
        radii = (blue_radius_min, blue_radius_max, boardwalk_propagation_radius)
        if depth_radius_profile is not None and depth_radius_profile.enabled:
            radii = depth_radius_profile.resolve(z)
        stats = boardwalk_classifier.classify(
            bev_points, class_ids,
            *radii,
            LABEL_BLUE, LABEL_BACKGROUND, LABEL_BOARDWALK,
            LABEL_INTERIOR_BOARDWALK,
        )

    if road_boardwalk_only:
        keep = np.isin(class_ids, (
            LABEL_BLUE, LABEL_INTERIOR_BLUE,
            LABEL_BOARDWALK, LABEL_INTERIOR_BOARDWALK,
        ))
        bev_points, class_ids = bev_points[keep], class_ids[keep]

    point_count_before_voxel = len(bev_points)
    voxel_started_at = time.perf_counter()
    bev_points, class_ids = voxelize_metric(
        bev_points, class_ids, voxel_size)
    voxel_ms = (time.perf_counter() - voxel_started_at) * 1000.0

    stats.update({
        key: int(np.count_nonzero(class_ids == label))
        for key, label in PUBLISHED_LABELS
    })
    return (
        bev_points, class_ids, point_count_before_voxel, stats,
        projection_ms, transform_ms, voxel_ms,
    )
