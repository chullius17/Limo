"""Verify tilted floors, distinct RGB optics, occlusions and unconditional filling."""

import numpy as np
import pytest

from cv_package.depth_geometry import (
    complete_depth, fit_ground_plane, ground_depth, pixel_rays, register_depth,
)


@pytest.mark.parametrize('tilt', [0.0, 2.0, -8.0])
def test_floor_fit_recovers_height_and_tilt_in_presence_of_wall(tilt):
    rng = np.random.RandomState(9)
    n = np.array([np.sin(np.deg2rad(tilt)), .01, np.cos(np.deg2rad(tilt))])
    n /= np.linalg.norm(n)
    xy = rng.uniform([.4, -.5], [1.8, .5], (1000, 2))
    z = (-.18 - xy @ n[:2]) / n[2] + rng.normal(0, .001, 1000)
    floor = np.column_stack((xy, z))
    wall = np.column_stack((np.full(250, 2.0), rng.uniform(-.5, .5, 250),
                            rng.uniform(-.2, .5, 250)))
    plane = fit_ground_plane(np.vstack((floor, wall)))
    np.testing.assert_allclose(plane[:3], n, atol=.001)
    assert plane[3] == pytest.approx(.18, abs=.001)


def test_wall_and_degenerate_strip_do_not_calibrate_as_floor():
    rng = np.random.RandomState(8)
    wall = np.column_stack((np.ones(1000), rng.uniform(-1, 1, 1000),
                            rng.uniform(-.3, 1, 1000)))
    with pytest.raises(ValueError):
        fit_ground_plane(wall)
    narrow = np.column_stack((np.linspace(.3, 1.5, 1000),
                              rng.normal(0, .0001, 1000), np.full(1000, -.18)))
    with pytest.raises(ValueError):
        fit_ground_plane(narrow)


def test_rgb_registration_uses_extrinsics_not_resizing():
    k = np.array([[100, 0, 4], [0, 100, 3], [0, 0, 1]])
    rays = pixel_rays(k, 9, 7, 9, 7)
    depth = np.full((7, 9), np.nan)
    depth[3, 4] = 1.0
    registered = register_depth(depth, rays, np.eye(3), np.array([.03, 0, 0]),
                                k, (9, 7), (9, 7), crop=0)
    assert registered[3, 7] == 1
    assert np.isnan(registered[3, 4])
    assert np.count_nonzero(np.isfinite(registered)) == 1


def test_registration_z_buffer_preserves_foreground_and_rejects_behind_camera():
    depth = np.array([[2., 1.]])
    rays = np.array([[[0., 0., 1.], [0., 0., 1.]]])
    output = register_depth(depth, rays, np.eye(3), np.zeros(3), np.eye(3),
                            (1, 1), (1, 1), crop=0)
    assert output[0, 0] == 1
    output = register_depth(depth, rays, np.eye(3), np.array([0., 0., -3.]),
                            np.eye(3), (1, 1), (1, 1), crop=0)
    assert np.isnan(output).all()


def test_crop_pixel_centres_and_rgb_z_translation():
    k = np.array([[100, 0, 3.5], [0, 100, 3.5], [0, 0, 1]])
    rays = pixel_rays(k, 8, 8, 4, 2, crop=.5)
    np.testing.assert_allclose(rays[0, 0], [-.03, .01, 1])
    # Identity registration of constant depth produces the same cropped image.
    full = pixel_rays(k, 8, 8, 8, 8)
    output = register_depth(np.ones((8, 8)), full, np.eye(3), np.zeros(3),
                            k, (8, 8), (4, 2), crop=.5)
    np.testing.assert_array_equal(output, np.ones((2, 4)))


def test_ground_lut_satisfies_full_plane_and_rejects_upward_rays():
    k = np.array([[100, 0, 4], [0, 100, 1], [0, 0, 1]])
    rays = pixel_rays(k, 9, 50, 9, 50)
    rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    translation = np.array([.001, .03, .002])
    plane = np.array([.034, .011, 1., .18])
    plane /= np.linalg.norm(plane[:3])
    depth = ground_depth(rays, rotation, translation, plane)
    valid = np.isfinite(depth)
    assert valid.sum() > 100
    points = rays[valid] * depth[valid, None] @ rotation.T + translation
    np.testing.assert_allclose(points @ plane[:3] + plane[3], 0, atol=2e-8)
    assert np.isnan(depth[0]).all()


def test_completion_fills_all_holes_without_labels_and_preserves_measurements():
    measured = np.array([[np.nan, 0, np.inf, -1, .7, 3., np.nan]], dtype=np.float32)
    ground = np.array([[1., 2., 3., 4., 1., 1., np.nan]], dtype=np.float32)
    original = measured.copy()
    actual = complete_depth(measured, ground)
    np.testing.assert_allclose(actual, [[1., 2., 3., 4., .7, 3., np.nan]], equal_nan=True)
    np.testing.assert_allclose(measured, original, equal_nan=True)


def test_invalid_camera_calibration_is_rejected():
    with pytest.raises(ValueError):
        pixel_rays(np.zeros((3, 3)), 640, 400, 320, 120)
