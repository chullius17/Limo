"""Check real/sim wiring, shared profile tuning and the visualization-only client."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters, normalize_parameters

from limo_cloud_example.cv_configuration import cloud_parameters
from limo_cloud_example import cv_launch


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent.parent


def package_path(name):
    extension = PACKAGE.parent / name
    return extension if extension.is_dir() else PACKAGES / name


def load_launch(package, name):
    spec = importlib.util.spec_from_file_location('cv_integration_launch',
                                                package_path(package) / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: str(package_path(name))
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def integration(monkeypatch, **overrides):
    module = cv_launch
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(package_path(name)))
    context = context_for(module, **overrides)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    actions = module._launch(context)
    return dict(actions[0].launch_arguments), actions[1:]


def original_pipeline(monkeypatch, arguments):
    module = load_launch('cv_package', 'cv.launch.py')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'OnProcessStart', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'RegisterEventHandler', lambda handler: {'handler': handler})
    actions = module._launch_cv(context_for(module, **arguments))
    nodes = {item['name']: item for item in actions if 'name' in item}
    for item in actions:
        if 'handler' in item:
            for node in item['handler']['on_start']:
                nodes[node['name']] = node
    return nodes


@pytest.mark.parametrize('profile,detector', [
    ('real', 'lane_detector_waterfall'), ('sim', 'lane_detector'),
])
def test_backend_reuses_upstream_and_replaces_only_cloud(monkeypatch, profile, detector):
    arguments, replacements = integration(monkeypatch, profile=profile, mode='backend')
    assert arguments['start_visual_ptcld'] == 'false'
    nodes = original_pipeline(monkeypatch, arguments)
    assert set(nodes) == {'lane_node', 'depth_correction'}
    assert nodes['lane_node']['executable'] == detector
    legacy = original_pipeline(monkeypatch, dict(arguments, start_visual_ptcld='true'))
    assert set(legacy) == {'lane_node', 'depth_correction', 'visual_ptcld'}
    for name in nodes:
        assert nodes[name] == legacy[name]
    assert len(replacements) == 1
    replacement = replacements[0]
    assert (replacement['package'], replacement['executable']) == (
        'limo_cloud_example', 'limo_cloud_example_node')
    params = replacement['parameters'][0]
    assert params == cloud_parameters(legacy['visual_ptcld']['parameters'][0])
    assert params['sampling.point_voxel_size'] == 5
    assert params['sampling.passthrough_extra_classes'] is False
    assert params['input_crop_y_min'] == .5
    assert params['use_sim_time'] is (profile == 'sim')
    if profile == 'real':
        assert params['boardwalk.blue_radius_min_m'] == .07
        assert params['class_filter.road_boardwalk_only'] is True
    else:
        assert params['debug.enable_debug_publications'] is True
    assert evaluate_parameters(LaunchContext(), normalize_parameters([params]))[0] == params


@pytest.mark.parametrize('profile,rviz,decoder', [
    ('real', 'cv_visual_real.rviz', 'cv_rviz_waterfall_overlay_decoder'),
    ('sim', 'cv_visual.rviz', 'cv_rviz_lane_overlay_decoder'),
])
def test_desktop_starts_original_viewer_without_any_backend(monkeypatch, profile, rviz, decoder):
    arguments, replacements = integration(monkeypatch, profile=profile, mode='desktop')
    assert replacements == []
    nodes = original_pipeline(monkeypatch, arguments)
    assert set(nodes) == {'cv_rviz', decoder}
    assert nodes['cv_rviz']['arguments'] == ['-d', str(PACKAGES / 'limo_rviz/config' / rviz)]
    assert nodes['cv_rviz']['parameters'][0]['use_sim_time'] is (profile == 'sim')


def test_sim_viewer_and_clock_override_are_shared(monkeypatch):
    arguments, replacements = integration(monkeypatch, profile='sim', start_rviz='true',
                                          use_sim_time='false', enable_telemetry='false')
    nodes = original_pipeline(monkeypatch, arguments)
    assert set(nodes) == {'lane_node', 'depth_correction', 'cv_rviz',
                          'cv_rviz_lane_overlay_decoder'}
    assert all(node['parameters'][0]['use_sim_time'] is False for node in nodes.values())
    assert replacements[0]['parameters'][0]['use_sim_time'] is False
    assert replacements[0]['parameters'][0]['enable_telemetry'] is False


def test_custom_cv_profile_updates_cpp_plugin_tuning(monkeypatch, tmp_path):
    profile = yaml.safe_load((PACKAGES / 'cv_package/config/cv_real.yaml').read_text())
    profile['visual_ptcld'].update(point_voxel_size=9, blue_radius_max_m=.4)
    filename = tmp_path / 'custom.yaml'
    filename.write_text(yaml.safe_dump(profile))
    arguments, replacements = integration(monkeypatch, config_file=str(filename))
    assert arguments['config_file'] == str(filename)
    params = replacements[0]['parameters'][0]
    assert params['sampling.point_voxel_size'] == 9
    assert params['boardwalk.blue_radius_max_m'] == .4


@pytest.mark.parametrize('filename,expected', [
    ('real.launch.py', {'profile': 'real', 'mode': 'backend', 'start_rviz': ''}),
    ('sim.launch.py', {'profile': 'sim', 'mode': 'profile', 'start_rviz': 'true'}),
    ('desktop.launch.py', {'profile': 'real', 'mode': 'desktop', 'start_rviz': 'true'}),
])
def test_entrypoints_select_the_expected_pipeline(filename, expected):
    module = load_launch('limo_cloud_example', filename)
    context = context_for(module)
    assert {name: context.launch_configurations[name] for name in expected} == expected
