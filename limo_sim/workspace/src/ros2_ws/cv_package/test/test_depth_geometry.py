"""Verify tilted floors, distinct RGB optics, occlusions and unconditional filling."""

import numpy as np
import pytest

from cv_package.depth_correction.depth_geometry import (
    complete_depth, fit_ground_plane, ground_depth, pixel_rays, register_depth,
    registration_source_crop, DepthRegistration,
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


def test_registration_source_crop_identity_keeps_exact_lower_half():
    k = np.array([[100, 0, 3.5], [0, 100, 3.5], [0, 0, 1]])
    rays = pixel_rays(k, 8, 8, 8, 8)
    rows, cols = registration_source_crop(rays, np.eye(3), np.zeros(3), k, (8, 8))
    assert (rows.start, rows.stop, cols.start, cols.stop) == (4, 8, 0, 8)


@pytest.mark.parametrize('translation', [[.03, -.02, .002], [-.2, .1, -.15]])
def test_source_crop_preserves_every_visible_point_across_depth_range(translation):
    k = np.array([[55, 0, 31], [0, 55, 19], [0, 0, 1]])
    rgb_k = np.array([[50, 0, 29], [0, 50, 23], [0, 0, 1]])
    angle = np.deg2rad(8)
    rotation = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])
    translation = np.array(translation)
    rays = pixel_rays(k, 64, 40, 64, 40)
    roi = registration_source_crop(rays, rotation, translation, rgb_k, (64, 48))
    rng = np.random.RandomState(21)
    depths = [np.full((40, 64), d) for d in [.1, .15, .3, 1., 5.]]
    depths.append(rng.uniform(.1, 5., (40, 64)))
    for depth in depths:
        full = register_depth(depth, rays, rotation, translation, rgb_k,
                              (64, 48), (32, 12))
        cropped = register_depth(depth[roi], rays[roi], rotation, translation,
                                 rgb_k, (64, 48), (32, 12))
        np.testing.assert_array_equal(cropped, full)


@pytest.mark.parametrize('native', [False, True])
@pytest.mark.parametrize('dtype', [np.float32, np.float64])
@pytest.mark.parametrize('tz', [.003, -.15])
def test_cached_registration_matches_reference_on_changing_depth(native, dtype, tz):
    k = np.array([[57., 0, 22.3], [0, 61., 15.2], [0, 0, 1]])
    rgb_k = np.array([[52., 0, 21.9], [0, 53., 18.3], [0, 0, 1]])
    rays = pixel_rays(k, 47, 31, 47, 31)
    angle = .07
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])
    translation = np.array([.03, -.012, tz])
    cached = DepthRegistration(rays, rotation, translation, rgb_k, (47, 37),
                               (24, 10), use_native=native)
    if native and cached.backend != 'native':
        pytest.skip('Build the native extension to test its kernel')
    rng = np.random.RandomState(33)
    for frame in range(8):
        padded = rng.uniform(.05, 5.2, (31, 94)).astype(dtype)
        depth = padded[:, ::2] if frame % 2 else padded[:, ::-2]
        depth.flat[:7] = [np.nan, np.inf, -np.inf, 0, -1, .1, 5.]
        depth.setflags(write=False)
        original = depth.copy()
        reference = register_depth(depth, rays, rotation, translation, rgb_k,
                                   (47, 37), (24, 10))
        actual = cached.register(depth)
        np.testing.assert_array_equal(np.isfinite(actual), np.isfinite(reference))
        np.testing.assert_allclose(actual, reference, rtol=1e-7, atol=2e-7, equal_nan=True)
        np.testing.assert_array_equal(depth, original)


@pytest.mark.parametrize('native', [False, True])
def test_cached_z_buffer_keeps_nearest_and_clears_previous_frame(native):
    rays = np.array([[[0., 0., 1.], [0., 0., 1.]]])
    cached = DepthRegistration(rays, np.eye(3), np.zeros(3), np.eye(3),
                               (1, 1), (1, 1), crop=0, use_native=native)
    if native and cached.backend != 'native':
        pytest.skip('Build the native extension to test its kernel')
    depth = np.array([[2., 1.]], dtype=np.float32)
    assert cached.register(depth)[0, 0] == 1
    depth = np.full((1, 2), np.nan, dtype=np.float32)
    assert np.isnan(cached.register(depth)).all()
    with pytest.raises(ValueError):
        cached.register(np.ones((2, 2), dtype=np.float32))


def test_native_kernel_rejects_incompatible_buffers():
    native = pytest.importorskip('cv_package.depth_correction._registration_native')
    rays = np.ones((1, 2, 3), dtype=np.float64)
    output = np.empty((1, 1), dtype=np.float32)
    params = np.array([0, 0, 0, 1, 1, 0, 0, 1, 1, 0, .1, 5], dtype=np.float64)
    with pytest.raises(ValueError):
        native.project_depth(np.ones((1, 2), dtype=np.uint16), rays, output, params)
    with pytest.raises(ValueError):
        native.project_depth(np.ones((1, 3), dtype=np.float32), rays, output, params)


@pytest.mark.parametrize('native', [False, True])
def test_cached_projection_respects_exact_crop_pixel_boundaries(native):
    rays = np.array([[[-.5, 1.5, 1], [3.5, 2, 1], [.5, 1.49, 1],
                      [.5, 3.5, 1], [.5, 1.5, 1], [1.5, 2.5, 1]]])
    cached = DepthRegistration(rays, np.eye(3), np.zeros(3), np.eye(3),
                               (4, 4), (2, 2), use_native=native)
    if native and cached.backend != 'native':
        pytest.skip('Build the native extension to test its kernel')
    actual = cached.register(np.ones((1, 6), dtype=np.float32))
    np.testing.assert_array_equal(actual, [[1, np.nan], [np.nan, 1]])
