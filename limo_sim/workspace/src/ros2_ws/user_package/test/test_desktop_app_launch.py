"""Verify desktop visualization never launches robot control/processing nodes."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
)


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent


def load_launch(package, filename):
    spec = importlib.util.spec_from_file_location(
        package + '_desktop_test', PACKAGES / package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: str(PACKAGES / name)
    return module


def launch_context(module, arguments):
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def desktop(monkeypatch, **overrides):
    module = load_launch('user_package', 'desktop_app.launch.py')
    context = launch_context(module, overrides)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    return module._launch_desktop(context)


def expanded_viewers(monkeypatch, actions):
    nodes = []
    for action in actions:
        if isinstance(action, dict):
            nodes.append(action)
            continue
        assert len(action) == 1
        include = action[0]
        assert isinstance(include, IncludeLaunchDescription)
        arguments = dict(include.launch_arguments)
        if 'mapping_config' in arguments:
            module = load_launch('user_package', 'control/control_experiment.launch.py')
            context = launch_context(module, arguments)
            monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
            nodes.extend(module._launch_experiment(context))
            continue
        assert arguments['mode'] == 'desktop'
        assert arguments['use_sim_time'] == 'false'
        package = (
            'cv_package' if arguments['config_file'].endswith('cv_real.yaml')
            else 'online_map_package')
        filename = 'cv.launch.py' if package == 'cv_package' else 'online_map.launch.py'
        module = load_launch(package, filename)
        context = launch_context(module, arguments)
        monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
        if package == 'cv_package':
            monkeypatch.setattr(module, 'OnProcessStart', lambda **kwargs: kwargs)
            monkeypatch.setattr(module, 'RegisterEventHandler', lambda handler: handler)
            nodes.extend(module._launch_cv(context))
        else:
            monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
            nodes.extend(module._launch_online(context))
    return nodes


def test_desktop_expands_to_viewers_and_overlay_decoder(monkeypatch):
    nodes = expanded_viewers(monkeypatch, desktop(monkeypatch))
    assert {(node['package'], node['executable']) for node in nodes} == {
        ('rviz2', 'rviz2'), ('limo_controller', 'control_gui'),
        ('image_transport', 'republish'), ('limo_controller', 'mpc_gui'),
        ('user_package', 'amcl_visualization'), ('user_package', 'control_experiment')}
    assert {node['name'] for node in nodes} == {
        'rviz2', 'control_gui', 'online_rviz_waterfall_overlay_decoder', 'mpc_preview',
        'amcl_visualization', 'control_experiment'}
    assert sum(node['executable'] == 'rviz2' for node in nodes) == 1
    assert all(node['parameters'][-1]['use_sim_time'] is False for node in nodes)
    gui = next(node for node in nodes if node['name'] == 'control_gui')
    assert gui['executable'] != 'controller_server'
    preview = next(node for node in nodes if node['name'] == 'mpc_preview')
    assert preview['parameters'][-1]['trigger_margin'] == 0.05
    assert preview['parameters'][-1]['trigger_front_extension'] == 0.0


@pytest.mark.parametrize('geometry', [
    {'trigger_margin': 0.07, 'trigger_front_extension': 0.12}, {}])
def test_desktop_preview_shares_real_controller_geometry(monkeypatch, tmp_path, geometry):
    config = yaml.safe_load(
        (PACKAGES / 'limo_controller/config/control_real.yaml').read_text())
    assert config['controller_server']['ros__parameters']['FollowPath']['MPC'][
        'Debug']['preview_enabled'] is False
    config['path_executor']['ros__parameters']['goal_circle_radius'] = 0.35
    config['local_costmap']['local_costmap']['ros__parameters'][
        'path_distance_layer']['tube_radius'] = 0.75
    guidance = config['controller_server']['ros__parameters']['FollowPath'][
        'MppiPath']['ObstacleGuidance']
    for name in ('trigger_margin', 'trigger_front_extension'):
        guidance.pop(name, None)
    guidance.update(geometry)
    path = tmp_path / 'controller.yaml'
    path.write_text(yaml.safe_dump(config))
    preview_path = tmp_path / 'preview.yaml'
    preview_path.write_text(yaml.safe_dump({'mpc_preview': {'ros__parameters': {
        'use_sim_time': True, 'tube_radius': 9.0, 'goal_circle_radius': 9.0,
        'trigger_margin': 9.0, 'trigger_front_extension': 9.0}}}))
    actions = desktop(monkeypatch, controller_params_file=str(path),
                      mpc_preview_params_file=str(preview_path))
    preview = next(action for action in actions if isinstance(action, dict)
                   and action['name'] == 'mpc_preview')
    expected = {
        'use_sim_time': False, 'goal_circle_radius': 0.35, 'tube_radius': 0.75,
        'trigger_margin': geometry.get('trigger_margin', 0.05),
        'trigger_front_extension': geometry.get('trigger_front_extension', 0.0),
    }
    assert preview['parameters'][-1] == expected
    rewritten = preview['parameters'][0].perform(LaunchContext())
    assert yaml.safe_load(Path(rewritten).read_text())[
        'mpc_preview']['ros__parameters'] == expected


@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('override', ['', 'true', 'false'])
def test_cv_view_requires_explicit_switch_independently_of_backend(
        monkeypatch, tmp_path, enabled, override):
    config = yaml.safe_load(
        (PACKAGES / 'online_map_package/config/mapping_real.yaml').read_text())
    config['launch']['start_cv'] = enabled
    path = tmp_path / 'mapping.yaml'
    path.write_text(yaml.safe_dump(config))
    actions = desktop(
        monkeypatch, mapping_config=str(path), start_cv_rviz=override)
    includes = [dict(action[0].launch_arguments)
                for action in actions if isinstance(action, list)
                and 'config_file' in dict(action[0].launch_arguments)]
    expected = override == 'true'
    assert len(includes) == 1 + int(expected)
    assert includes[0]['config_file'] == str(path)
    if expected:
        assert includes[1]['config_file'] == str(
            PACKAGES / 'cv_package/config/cv_real.yaml')


def test_custom_cv_profile_and_client_switches_are_forwarded(monkeypatch):
    actions = desktop(
        monkeypatch, cv_config='/tmp/custom_cv.yaml',
        start_mapping_rviz='false', start_control_gui='false', start_cv_rviz='true',
        start_mpc_preview='false', start_amcl_visualization='false')
    assert len(actions) == 2
    arguments = dict(actions[0][0].launch_arguments)
    assert arguments['config_file'] == '/tmp/custom_cv.yaml'
    assert arguments['mode'] == 'desktop'


def test_cv_profile_is_resolved_from_mapping_yaml(monkeypatch, tmp_path):
    config = {'launch': {'start_cv': True, 'cv_config': 'custom_cv.yaml'}}
    path = tmp_path / 'mapping.yaml'
    path.write_text(yaml.safe_dump(config))
    actions = desktop(
        monkeypatch, mapping_config=str(path),
        start_mapping_rviz='false', start_control_gui='false', start_cv_rviz='true',
        start_amcl_visualization='false')
    arguments = dict(actions[0][0].launch_arguments)
    assert arguments['config_file'] == str(
        PACKAGES / 'cv_package/config/custom_cv.yaml')


def test_all_desktop_clients_can_be_disabled(monkeypatch):
    assert desktop(
        monkeypatch, start_mapping_rviz='false',
        start_cv_rviz='false', start_control_gui='false', start_mpc_preview='false',
        start_amcl_visualization='false', start_control_experiment='false') == []


@pytest.mark.parametrize('name', [
    'start_mapping_rviz', 'start_cv_rviz', 'start_control_gui', 'start_mpc_preview',
    'start_amcl_visualization', 'start_control_experiment'])
def test_invalid_desktop_switch_is_rejected(monkeypatch, name):
    with pytest.raises(ValueError, match='Expected true or false'):
        desktop(monkeypatch, **{name: 'typo'})


def test_desktop_requires_mapping_launch_settings(monkeypatch, tmp_path):
    path = tmp_path / 'invalid.yaml'
    path.write_text('launch: []\n')
    with pytest.raises(ValueError, match='missing YAML mapping'):
        desktop(monkeypatch, mapping_config=str(path))


def test_viewer_includes_have_separate_launch_scopes():
    module = load_launch('user_package', 'desktop_app.launch.py')
    context = launch_context(module, {
        'start_control_gui': 'false', 'start_cv_rviz': 'true', 'start_mpc_preview': 'false',
        'start_amcl_visualization': 'false'})
    actions = module._launch_desktop(context)
    assert len(actions) == 3
    assert all(isinstance(action, GroupAction) for action in actions)
    for action in actions:
        scoped_actions = action.get_sub_entities()
        assert isinstance(scoped_actions[0], PushLaunchConfigurations)
        assert isinstance(scoped_actions[-1], PopLaunchConfigurations)


def test_desktop_launch_uses_deferred_profile_resolution():
    module = load_launch('user_package', 'desktop_app.launch.py')
    description = module.generate_launch_description()
    assert any(isinstance(action, OpaqueFunction)
               for action in description.entities)


def test_desktop_experiment_shares_custom_profiles_and_can_be_disabled(monkeypatch):
    actions = desktop(monkeypatch, controller_params_file='/tmp/custom_control.yaml',
                      start_mpc_preview='false')
    experiment = next(dict(action[0].launch_arguments) for action in actions
                      if isinstance(action, list)
                      and 'mapping_config' in dict(action[0].launch_arguments))
    assert experiment['controller_params_file'] == '/tmp/custom_control.yaml'
    assert experiment['mapping_config'].endswith('/mapping_real.yaml')
    assert experiment['use_sim_time'] == 'false'
    actions = desktop(monkeypatch, start_control_experiment='false')
    assert all('mapping_config' not in dict(action[0].launch_arguments)
               for action in actions if isinstance(action, list))


@pytest.mark.parametrize('override', ['', '/custom_complete_map'])
def test_amcl_viewer_follows_complete_map_profile(monkeypatch, tmp_path, override):
    profile = yaml.safe_load(
        (PACKAGES / 'online_map_package/config/mapping_real.yaml').read_text())
    profile['map_servers']['complete_topic'] = '/robot/complete_map'
    profile['map_servers']['laser_topic'] = '/robot/laser_only'
    path = tmp_path / 'mapping.yaml'
    path.write_text(yaml.safe_dump(profile))
    nodes = desktop(monkeypatch, mapping_config=str(path), amcl_map_topic=override,
                    amcl_pose_topic='/robot/amcl_pose')
    viewer = next(action for action in nodes if isinstance(action, dict)
                  and action['name'] == 'amcl_visualization')
    assert viewer['parameters'] == [{
        'use_sim_time': False, 'map_topic': override or '/robot/complete_map',
        'pose_topic': '/robot/amcl_pose'}]


def test_standalone_amcl_viewer_uses_same_complete_map_and_wall_clock(monkeypatch):
    module = load_launch('user_package', 'amcl/amcl_visualization.launch.py')
    context = launch_context(module, {})
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    viewer, = module._launch_viewer(context)
    assert viewer['parameters'] == [{
        'use_sim_time': False, 'map_topic': '/map', 'pose_topic': '/amcl_pose',
        'max_samples': 5000, 'sample_period': 0.25, 'refresh_hz': 4.0}]
