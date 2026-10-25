"""Check online profiles, wrappers and desktop/backend separation."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch_ros.utilities import (  # noqa: E402
    evaluate_parameters,
    normalize_parameters,
)


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent


def load_launch(filename, package=PACKAGE):
    spec = importlib.util.spec_from_file_location(
        'localization_launch', package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, 'get_package_share_directory'):
        module.get_package_share_directory = lambda name: str(PACKAGES / name)
    return module


def online(monkeypatch, profile='sim', **overrides):
    module = load_launch('online_map.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'config_file': str(
            PACKAGE / 'config' / ('mapping_' + profile + '.yaml')),
        **overrides,
    })
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    actions = module._launch_online(context)
    nodes = {}
    includes = []
    for action in actions:
        if isinstance(action, list):
            includes.extend(action)
        elif isinstance(action, dict):
            parameters = evaluate_parameters(
                context, normalize_parameters(action['parameters']))
            action['values'] = {
                key: value for group in parameters
                for key, value in group.items()
            }
            nodes[action['name']] = action
        elif isinstance(action, IncludeLaunchDescription):
            includes.append(action)
    return nodes, includes


def test_simulation_starts_maps_amcl_rviz_and_cv(monkeypatch):
    nodes, includes = online(monkeypatch)
    assert set(nodes) == {
        'complete_map_server', 'laser_map_server', 'cv_map_server',
        'lifecycle_manager_online_maps', 'local_ctrl_map', 'rviz2',
    }
    assert all(node['values']['use_sim_time'] is True
               for node in nodes.values())
    assert len(includes) == 2
    include_arguments = [dict(action.launch_arguments) for action in includes]
    amcl = next(values for values in include_arguments
                if 'cv_voxel_size' in values)
    assert amcl['cv_voxel_size'] == '0.075'
    assert amcl['map_topic'].endswith('/laser_map')
    assert amcl['cv_map_topic'].endswith('/cv_obstacle')
    assert nodes['rviz2']['arguments'][-2:] == ['-f', 'map']
    assert nodes['local_ctrl_map']['values']['base_frame'] == 'base_link'
    assert nodes['local_ctrl_map']['values']['maximum_points'] == 300
    assert nodes['local_ctrl_map']['values']['grid_resolution'] == 0.02
    assert nodes['local_ctrl_map']['values']['inflation_radius'] == 0.0
    assert nodes['local_ctrl_map']['values']['output_cloud_topic'] == (
        '/limo/map_package/online/local_ctrl_map/points')
    assert nodes['local_ctrl_map']['values']['cmd_vel_topic'] == '/cmd_vel'
    assert nodes['local_ctrl_map']['values'][
        'linear_speed_at_max_decay'] == 0.50
    assert nodes['local_ctrl_map']['values'][
        'angular_speed_at_max_decay'] == 1.00
    assert nodes['local_ctrl_map']['values']['yellow_line_cost'] == 60
    assert nodes['local_ctrl_map']['values']['boardwalk_cost'] == 90


def test_real_profile_is_headless_and_starts_cv(monkeypatch):
    nodes, includes = online(monkeypatch, 'real', mode='backend')
    assert set(nodes) == {
        'complete_map_server', 'laser_map_server', 'cv_map_server',
        'lifecycle_manager_online_maps', 'local_ctrl_map',
    }
    assert all(node['values']['use_sim_time'] is False
               for node in nodes.values())
    assert len(includes) == 2
    include_arguments = [dict(action.launch_arguments) for action in includes]
    amcl = next(values for values in include_arguments
                if 'cv_voxel_size' in values)
    assert amcl['use_sim_time'] == 'false'
    assert amcl['cv_enabled'] == 'true'


def test_desktop_online_starts_rviz_and_overlay_decoder(monkeypatch):
    nodes, includes = online(monkeypatch, 'real', mode='desktop')
    assert set(nodes) == {'rviz2', 'online_rviz_waterfall_overlay_decoder'}
    assert nodes['rviz2']['values']['use_sim_time'] is False
    assert nodes['rviz2']['arguments'][-2:] == ['-f', 'map']
    assert includes == []


def test_custom_profile_and_cli_overrides_reach_consumers(
        monkeypatch, tmp_path):
    profile = yaml.safe_load(
        (PACKAGE / 'config' / 'mapping_sim.yaml').read_text())
    profile['map_servers']['name'] = 'custom'
    profile['map_servers']['complete_topic'] = '/test/complete_map'
    profile['map_servers']['laser_topic'] = '/test/laser_map'
    profile['amcl']['cv_voxel_size'] = 0.12
    config = tmp_path / 'profile.yaml'
    config.write_text(yaml.safe_dump(profile))
    nodes, includes = online(
        monkeypatch, config_file=str(config),
        map_directory=str(tmp_path), max_particles='600',
        local_map_maximum_points='123')
    assert nodes['local_ctrl_map']['values']['maximum_points'] == 123
    assert nodes['complete_map_server']['values']['yaml_filename'] == str(
        tmp_path / 'custom_complete.yaml')
    assert ('map', '/test/laser_map') in (
        nodes['laser_map_server']['remappings'])
    amcl = next(dict(action.launch_arguments) for action in includes
                if 'cv_voxel_size' in dict(action.launch_arguments))
    assert amcl['cv_voxel_size'] == '0.12'
    assert amcl['max_particles'] == '600'
    assert amcl['map_topic'] == '/test/laser_map'


@pytest.mark.parametrize('filename,profile,mode', [
    ('online_map_sim.launch.py', 'sim', None),
    ('online_map_real.launch.py', 'real', 'backend'),
    ('desktop_online.launch.py', 'real', 'desktop'),
])
def test_wrappers_select_profile_and_role(filename, profile, mode):
    module = load_launch(filename)
    actions = module.generate_launch_description().entities
    assert len(actions) == 1
    expected = {
        'config_file': str(
            PACKAGE / 'config' / ('mapping_' + profile + '.yaml')),
    }
    if mode is not None:
        expected['mode'] = mode
    assert dict(actions[0].launch_arguments) == expected


@pytest.mark.parametrize('overrides', [
    {'use_sim_time': 'typo'},
    {'mode': 'typo'},
    {'max_particles': 'not-an-integer'},
])
def test_invalid_launch_settings_fail(monkeypatch, overrides):
    with pytest.raises(ValueError):
        online(monkeypatch, **overrides)


def test_amcl_launch_passes_typed_cloud_parameters(monkeypatch):
    module = load_launch(
        'amcl.launch.py', PACKAGES / 'limo_rviz')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({
        'cv_voxel_size': '0.12',
        'cv_enabled': 'false',
        'cv_cloud_topic': '/test/cloud',
        'use_sim_time': 'false',
        'max_particles': '600',
    })
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = {}
    for node in description.entities:
        if isinstance(node, dict):
            parameters = evaluate_parameters(
                context, normalize_parameters(node['parameters']))
            nodes[node['name']] = {
                key: value for group in parameters
                for key, value in group.items()
            }
    params = nodes['amcl']
    assert params['cv_enabled'] is False
    assert params['use_sim_time'] is False
    assert params['cv_voxel_size'] == 0.12
    assert params['cv_cloud_topic'] == '/test/cloud'
    assert params['max_particles'] == 600
    assert params['cv_map_topic'].endswith('/cv_obstacle')
    assert params['map_topic'].endswith('/laser_map')


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_default_cv_selects_matching_profile(monkeypatch, profile):
    _, includes = online(monkeypatch, profile)
    cv = [dict(action.launch_arguments) for action in includes
          if 'config_file' in dict(action.launch_arguments)]
    assert len(cv) == 1
    assert cv[0]['config_file'] == str(
        PACKAGES / 'cv_package' / 'config' / ('cv_' + profile + '.yaml'))


@pytest.mark.parametrize('profile,clock', [('sim', 'false'), ('real', 'true')])
def test_cv_profile_identity_survives_clock_override(monkeypatch, profile, clock):
    _, includes = online(monkeypatch, profile, start_cv='true', use_sim_time=clock)
    args = next(dict(action.launch_arguments) for action in includes
                if 'config_file' in dict(action.launch_arguments))
    assert args['config_file'].endswith('/cv_' + profile + '.yaml')
    assert args['use_sim_time'] == clock
    assert args['mode'] == 'backend'


def test_cv_config_override_reaches_include(monkeypatch):
    _, includes = online(monkeypatch, 'real', start_cv='true', cv_config='/tmp/custom_cv.yaml')
    args = next(dict(action.launch_arguments) for action in includes
                if 'config_file' in dict(action.launch_arguments))
    assert args['config_file'] == '/tmp/custom_cv.yaml'


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_local_exterior_road_settings_leave_amcl_on_the_original_cv_cloud(
        monkeypatch, profile):
    nodes, includes = online(monkeypatch, profile)
    local = nodes['local_ctrl_map']['values']
    assert local['persist_exterior_road'] is (profile == 'real')
    assert local['exterior_road_cost'] == (30 if profile == 'real' else 0)
    amcl = next(dict(action.launch_arguments) for action in includes
                if 'cv_cloud_topic' in dict(action.launch_arguments))
    assert amcl['cv_cloud_topic'] == '/limo/cv_package/visual_ptcld/points'
    assert amcl['cv_enabled'] == 'true'
    assert amcl['cv_weight_factor'] == amcl['laser_weight_factor'] == '1.0'


def ideal(monkeypatch, **overrides):
    module = load_launch('experiments/online_map_ideal_sim.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    return module._launch_ideal(context)


def test_ideal_launch_uses_existing_map_and_cv_only_amcl(monkeypatch):
    actions = ideal(monkeypatch)
    nodes = {action['name']: action for action in actions
             if isinstance(action, dict)}
    assert set(nodes) == {
        'ideal_map_server', 'lifecycle_manager_ideal_map', 'rviz2', 'amcl_experiment',
    }
    server = nodes['ideal_map_server']
    params = server['parameters'][0]
    assert params['use_sim_time'] is True
    assert params['frame_id'] == 'map'
    assert Path(params['yaml_filename']) == (
        PACKAGES / 'offline_map_package' / 'experiments' / 'sim'
        / 'ideal_map' / 'ideal_map.yaml')
    metadata = yaml.safe_load(Path(params['yaml_filename']).read_text())
    assert metadata['mode'] == 'raw'
    assert (Path(params['yaml_filename']).parent / metadata['image']).is_file()
    assert ('map', '/map') in server['remappings']
    assert nodes['lifecycle_manager_ideal_map']['parameters'][0]['node_names'] == [
        'ideal_map_server']
    cv = next(action[0] for action in actions if isinstance(action, list))
    cv_args = dict(cv.launch_arguments)
    assert cv_args['config_file'].endswith('/cv_package/config/cv_sim.yaml')
    assert cv_args['mode'] == 'backend'
    assert cv_args['use_sim_time'] == 'true'
    amcl = next(action for action in actions
                if isinstance(action, IncludeLaunchDescription))
    args = dict(amcl.launch_arguments)
    assert args['map_topic'] == args['cv_map_topic'] == '/map'
    assert args['laser_weight_factor'] == '0.0'
    assert args['cv_weight_factor'] == '1.0'
    assert args['cv_enabled'] == 'true'
    assert args['cv_occupied_threshold'] == '50'
    assert args['cv_cloud_topic'] == '/limo/cv_package/visual_ptcld/points'
    assert args['scan_topic'] == '/scan'
    assert args['tf_broadcast'] == 'true'


def test_ideal_launch_supports_cloud_replay_and_custom_map(monkeypatch, tmp_path):
    # The map server validates the map contents; launch validates the YAML path.
    custom_map = tmp_path / 'custom.yaml'
    custom_map.write_text('image: custom.pgm\n')
    actions = ideal(
        monkeypatch, map_yaml=str(custom_map), start_cv='false',
        start_rviz='false', start_monitor='false', use_sim_time='false')
    assert not any(isinstance(action, list) for action in actions)
    nodes = {action['name']: action for action in actions
             if isinstance(action, dict)}
    assert set(nodes) == {'ideal_map_server', 'lifecycle_manager_ideal_map'}
    assert nodes['ideal_map_server']['parameters'][0]['yaml_filename'] == str(custom_map)
    assert all(node['parameters'][0]['use_sim_time'] is False
               for node in nodes.values())
    amcl = next(action for action in actions
                if isinstance(action, IncludeLaunchDescription))
    assert dict(amcl.launch_arguments)['use_sim_time'] == 'false'
    assert dict(amcl.launch_arguments)['laser_weight_factor'] == '0.0'


def test_ideal_launch_rejects_missing_map(monkeypatch, tmp_path):
    with pytest.raises(ValueError, match='Ideal map YAML not found'):
        ideal(monkeypatch, map_yaml=str(tmp_path / 'missing.yaml'))
