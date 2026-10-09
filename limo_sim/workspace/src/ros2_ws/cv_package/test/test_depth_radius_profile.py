"""Verify discrete depth boundaries and alignment through the cloud pipeline."""

import numpy as np
import pytest

from cv_package.visual_ptcld.boardwalk import BoardwalkClassifier
from cv_package.visual_ptcld.cloud_cpu import RayCache
from cv_package.visual_ptcld.cloud_pipeline import build_semantic_cloud
from cv_package.visual_ptcld.depth_radius_profile import DepthRadiusProfile


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_bands_are_upper_inclusive_without_interpolation(dtype):
    profile = DepthRadiusProfile(
        [0.5, 0.8], [0.07, 0.09, 0.1], [0.25, 0.3, 0.4], [0.15, 0.2, 0.25])
    depths = np.array([0.1, 0.5, 0.50001, 0.65, 0.8, 0.80001, 1.0, 3.0], dtype=dtype)
    minimum, maximum, propagation = profile.resolve(depths)
    np.testing.assert_array_equal(minimum, [0.07]*2 + [0.09]*3 + [0.1]*3)
    np.testing.assert_array_equal(maximum, [0.25]*2 + [0.3]*3 + [0.4]*3)
    np.testing.assert_array_equal(propagation, [0.15]*2 + [0.2]*3 + [0.25]*3)
    assert profile.enabled
    assert '0.500 < depth <= 0.800 m: 0.090/0.300/0.200 m' in profile.describe()
    assert all(values.shape == (0,) for values in profile.resolve(np.empty(0)))


def test_no_breakpoints_preserves_scalar_mode():
    profile = DepthRadiusProfile([], 0.07, 0.25, 0.15)
    assert not profile.enabled
    assert profile.resolve([1.0]) == (0.07, 0.25, 0.15)


def test_short_lists_repeat_last_value_independently():
    profile = DepthRadiusProfile([0.5, 0.8], [0.07, 0.09], [0.25], 0.15)
    minimum, maximum, propagation = profile.resolve([0.25, 0.6, 1.2])
    np.testing.assert_array_equal(minimum, [0.07, 0.09, 0.09])
    np.testing.assert_array_equal(maximum, [0.25, 0.25, 0.25])
    np.testing.assert_array_equal(propagation, [0.15, 0.15, 0.15])


def test_arbitrary_number_of_breakpoints_and_radii():
    profile = DepthRadiusProfile(
        [0.25, 0.5, 0.75, 1.0, 1.25], [0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        [0.2, 0.25], [0.1, 0.15, 0.2])
    minimum, maximum, propagation = profile.resolve([0.1, 0.3, 0.6, 0.9, 1.1, 2])
    np.testing.assert_array_equal(minimum, [0.01, 0.02, 0.03, 0.04, 0.05, 0.06])
    np.testing.assert_array_equal(maximum, [0.2, 0.25, 0.25, 0.25, 0.25, 0.25])
    np.testing.assert_array_equal(propagation, [0.1, 0.15, 0.2, 0.2, 0.2, 0.2])


@pytest.mark.parametrize('depths,minimum,maximum,propagation', [
    ([0.5], [0.1, 0.2, 0.3], [0.4], [0.15]),
    ([0.5], [0.1], [0.2, 0.3, 0.4], [0.15]),
    ([0.5], [0.1], [0.4], [0.1, 0.2, 0.3]),
    ([], [0.1, 0.2], [0.4], [0.15]),
    ([0.5], [], [0.25], [0.15]),
    ([0.5], [0.1], [], [0.15]),
    ([0.5], [0.1], [0.25], []),
    ([[0.5]], [[0.1]], [[0.25]], [[0.15]]),
    ([1, 0], [0.1, 0.1], [0.25, 0.25], [0.15, 0.15]),
    ([1, 1], [0.1, 0.1], [0.25, 0.25], [0.15, 0.15]),
    ([-1], [0.1], [0.25], [0.15]),
    ([np.nan], [0.1], [0.25], [0.15]),
    ([np.inf], [0.1], [0.25], [0.15]),
    ([0], [0.1], [0.25], [0.15]),
    ([0.5], [-0.1], [0.25], [0.15]),
    ([0.5], [0.3], [0.25], [0.15]),
    ([0.5], [0.1, 0.3], [0.25], [0.15]),
    ([0.5], [0.1], [np.inf], [0.15]),
    ([0.5], [0.1], [0.25], [-0.15]),
    ([0.5], [0.1], [0.25], [np.nan]),
])
def test_invalid_tables_are_rejected(depths, minimum, maximum, propagation):
    with pytest.raises(ValueError):
        DepthRadiusProfile(depths, minimum, maximum, propagation)


@pytest.mark.parametrize('profile_enabled', [False, True])
def test_cloud_uses_optical_depth_before_transform_and_after_depth_filter(profile_enabled):
    profile = (DepthRadiusProfile([1.5], [0.1], [0.5, 0.2], [0])
               if profile_enabled else DepthRadiusProfile([], 0.1, 0.5, 0))
    result = build_semantic_cloud(
        point_groups=[(np.array([[0, 0]]), 1),
                      (np.array([[0, 1], [0, 2], [0, 3], [0, 4], [0, 5]]), 3)],
        depth=np.array([[1, np.nan, 2, 0.05, 1, 4]], dtype=np.float32),
        intrinsics=(16.0, 16.0, 0.0, 0.0, 6, 1), width=6, height=1,
        input_crop_y_min=0, ray_cache=RayCache(), rotation=np.eye(3),
        translation=(10, 20), cloud_min_depth=0.1, cloud_max_depth=3,
        boardwalk_classifier=BoardwalkClassifier(), blue_radius_min=0.1,
        blue_radius_max=0.5, boardwalk_propagation_radius=0,
        road_boardwalk_only=False, voxel_size=0.02, depth_radius_profile=profile)
    points, labels, before_voxel, stats = result[:4]
    # Both background points are 0.25 m from road, despite different optical Z.
    expected_points = [[10, 20], [10.25, 20]]
    if profile_enabled:
        expected_points.append([10.25, 20])
    np.testing.assert_allclose(points, expected_points)
    np.testing.assert_array_equal(labels, [1, 6, 4] if profile_enabled else [1, 4])
    if not profile_enabled:
        # Identical coordinates with the same class merge during voxelization.
        assert len(points) == 2
    assert before_voxel == 3
    assert stats['boardwalk_seed_count'] == (1 if profile_enabled else 2)
