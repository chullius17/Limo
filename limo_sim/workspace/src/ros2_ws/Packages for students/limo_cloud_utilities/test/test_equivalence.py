"""Compare the composed Python chain against actual VisualPtcld node methods."""

import numpy as np
import pytest

from helpers import frame_for, legacy_output, plugins, run_plugins, scene, settings
from limo_cloud import POINT_DTYPE
from limo_cloud_utilities.cloud_plugins import Boardwalk


@pytest.mark.parametrize('profile', ['real', 'sim'])
@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
@pytest.mark.parametrize('seed', [3, 19])
def test_complete_plugin_chain_matches_actual_legacy_node(profile, encoding, seed):
    original, parameters = settings(profile, roi_y_min=.17, roi_y_max=.93)
    labels, depth = scene(seed)
    if encoding == '16UC1':
        depth = (np.nan_to_num(depth, nan=0, posinf=0, neginf=0).clip(0, 60) * 1000).astype(np.uint16)
    expected = np.frombuffer(legacy_output(labels, depth, original).data, dtype=POINT_DTYPE)
    actual = run_plugins(labels, depth, parameters).cloud
    np.testing.assert_array_equal(actual['class_id'], expected['class_id'])
    for name in ('x', 'y', 'z'):
        np.testing.assert_array_equal(actual[name], expected[name])


@pytest.mark.parametrize('value', [0, 1, 2, 3, 255])
def test_empty_and_single_class_scenes_match(value):
    original, parameters = settings('sim', enable_boardwalk=False)
    labels = np.full((12, 24), value, np.uint8)
    depth = np.full(labels.shape, 1.1, np.float32)
    expected = np.frombuffer(legacy_output(labels, depth, original).data, dtype=POINT_DTYPE)
    actual = run_plugins(labels, depth, parameters).cloud
    for name in POINT_DTYPE.names:
        np.testing.assert_array_equal(actual[name], expected[name])


def test_sampling_preserves_centroids_outside_masks_and_duplicate_pixels():
    _, parameters = settings('sim', point_voxel_size=3, blue_boundary_kernel_size=3)
    labels = np.zeros((5, 5), dtype=np.uint8)
    labels[0, 0] = labels[0, 2] = labels[2, 0] = labels[2, 2] = 1
    labels[1, 1] = 3
    frame = frame_for(labels, np.ones(labels.shape, np.float32), parameters)
    chain = plugins(parameters)
    for plugin in chain:
        plugin.process_images(frame)
    assert frame.samples['class_id'].tolist() == [1, 3]
    assert frame.samples['row'].tolist() == [1, 1]
    assert frame.samples['col'].tolist() == [1, 1]
    assert labels[1, 1] == 3


def test_two_neighbor_passes_use_exact_bounds_and_frozen_seeds():
    # Road at x=0. Seed at .25; propagation excludes the exact .125 boundary
    # and cannot recurse through the point at .1875 to reach x=.0625.
    cloud = np.zeros(6, dtype=POINT_DTYPE)
    cloud['x'] = [0, .25, .125, .1875, .0625, .375]
    cloud['class_id'] = [1, 3, 3, 3, 3, 3]
    frame = frame_for(np.ones((1, 1), np.uint8), np.ones((1, 1), np.float32), {})
    frame.cloud = cloud
    plugin = Boardwalk()
    plugin.configure({'blue_radius_min_m': .1875, 'blue_radius_max_m': .25,
                      'boardwalk_propagation_radius_m': .125})
    plugin.process_cloud(frame)
    assert cloud['class_id'].tolist() == [1, 4, 3, 4, 3, 6]


@pytest.mark.parametrize('size', [0, -1])
def test_invalid_sampling_configuration_fails_early(size):
    from limo_cloud_utilities.image_plugins import PixelSampling
    with pytest.raises(ValueError):
        PixelSampling().configure({'point_voxel_size': size})
