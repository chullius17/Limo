"""The example launches one consumer node and reads only its own configuration."""

import ast
import importlib.util

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from helpers import PACKAGE, default_parameters, run_plugins, scene
from limo_cloud_utilities.configuration import load_parameters


def launch_node(monkeypatch, **overrides):
    spec = importlib.util.spec_from_file_location('example_launch', PACKAGE / 'launch/cloud.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def package_share(package):
        assert package == 'limo_cloud_utilities', 'Launch requested an external pipeline'
        return str(PACKAGE)

    module.get_package_share_directory = package_share
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    nodes = module._launch(context)
    assert len(nodes) == 1
    node = nodes[0]
    assert (node['package'], node['executable'], node['name']) == (
        'limo_cloud_utilities', 'limo_cloud_utilities_node', 'limo_cloud_utilities')
    return node['parameters'][0]


def test_default_launch_only_consumes_classified_images(monkeypatch):
    parameters = launch_node(monkeypatch)
    assert parameters == {'sampling.point_voxel_size': 5}


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_cloud_only_profiles_are_local(monkeypatch, profile):
    parameters = launch_node(monkeypatch, profile=profile)
    assert parameters['input_crop_y_min'] == .5
    assert parameters['sampling.passthrough_extra_classes'] is False
    if profile == 'real':
        assert parameters['pointcloud_max_depth_m'] == 1.3
        assert parameters['boardwalk.blue_radius_min_m'] == .07
        assert parameters['class_filter.road_boardwalk_only'] is True
    else:
        assert parameters['use_sim_time'] is True
        assert parameters['debug.enable_debug_publications'] is True


def test_explicit_clock_and_telemetry_override_profile(monkeypatch):
    parameters = launch_node(monkeypatch, profile='sim', use_sim_time='false', enable_telemetry='false')
    assert parameters['use_sim_time'] is False
    assert parameters['enable_telemetry'] is False


def test_runtime_algorithms_have_no_cv_package_imports():
    for path in (PACKAGE / 'limo_cloud_utilities').rglob('*.py'):
        tree = ast.parse(path.read_text())
        for item in ast.walk(tree):
            if isinstance(item, ast.ImportFrom):
                assert not (item.module or '').startswith('cv_package'), path
            elif isinstance(item, ast.Import):
                assert all(not name.name.startswith('cv_package') for name in item.names), path


def test_extra_input_classes_are_preserved_by_the_default_plugin_chain():
    labels, depth = scene()
    labels[:] = 7
    labels[0, 0] = 255
    depth[:] = 1
    parameters = default_parameters()
    frame = run_plugins(labels, depth, parameters)
    assert set(frame.cloud['class_id']) == {7, 255}
