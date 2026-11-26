"""Adapt the original CV cloud stage's flat parameters to Python plugin names."""


PLUGIN_PARAMETERS = {
    'opencv_num_threads': 'masks.opencv_num_threads',
    'blue_boundary_kernel_size': 'masks.blue_boundary_kernel_size',
    'point_voxel_size': 'sampling.point_voxel_size',
    'enable_debug_publications': 'debug.enable_debug_publications',
    'debug_jpeg_quality': 'debug.debug_jpeg_quality',
    'enable_boardwalk': 'boardwalk.enable_boardwalk',
    'blue_radius_min_m': 'boardwalk.blue_radius_min_m',
    'blue_radius_max_m': 'boardwalk.blue_radius_max_m',
    'boardwalk_propagation_radius_m': 'boardwalk.boardwalk_propagation_radius_m',
    'road_boardwalk_only': 'class_filter.road_boardwalk_only',
    'pointcloud_voxel_size_m': 'voxel.pointcloud_voxel_size_m',
}


def cloud_parameters(parameters):
    """Keep core settings and translate semantic settings without copying profiles."""
    if not isinstance(parameters, dict):
        raise ValueError('Expected a visual_ptcld parameter mapping in the CV profile')
    result = {PLUGIN_PARAMETERS.get(name, name): value
              for name, value in parameters.items()}
    # This compatibility path matches visual_ptcld's treatment of detector IDs.
    # The standalone cloud.launch.py keeps its more general passthrough default.
    result['sampling.passthrough_extra_classes'] = False
    return result
