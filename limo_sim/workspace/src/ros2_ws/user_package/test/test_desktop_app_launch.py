"""Verify application desktop clients never launch robot processing nodes."""

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
        ('image_transport', 'republish')}
    assert {node['name'] for node in nodes} == {
        'rviz2', 'control_gui', 'online_rviz_waterfall_overlay_decoder'}
    assert sum(node['executable'] == 'rviz2' for node in nodes) == 1
    assert all(node['parameters'][0]['use_sim_time'] is False for node in nodes)
    gui = next(node for node in nodes if node['name'] == 'control_gui')
    assert gui['executable'] != 'controller_server'


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
                for action in actions if isinstance(action, list)]
    expected = override == 'true'
    assert len(includes) == 1 + int(expected)
    assert includes[0]['config_file'] == str(path)
    if expected:
        assert includes[1]['config_file'] == str(
            PACKAGES / 'cv_package/config/cv_real.yaml')


def test_custom_cv_profile_and_client_switches_are_forwarded(monkeypatch):
    actions = desktop(
        monkeypatch, cv_config='/tmp/custom_cv.yaml',
        start_mapping_rviz='false', start_control_gui='false', start_cv_rviz='true')
    assert len(actions) == 1
    arguments = dict(actions[0][0].launch_arguments)
    assert arguments['config_file'] == '/tmp/custom_cv.yaml'
    assert arguments['mode'] == 'desktop'


def test_cv_profile_is_resolved_from_mapping_yaml(monkeypatch, tmp_path):
    config = {'launch': {'start_cv': True, 'cv_config': 'custom_cv.yaml'}}
    path = tmp_path / 'mapping.yaml'
    path.write_text(yaml.safe_dump(config))
    actions = desktop(
        monkeypatch, mapping_config=str(path),
        start_mapping_rviz='false', start_control_gui='false', start_cv_rviz='true')
    arguments = dict(actions[0][0].launch_arguments)
    assert arguments['config_file'] == str(
        PACKAGES / 'cv_package/config/custom_cv.yaml')


def test_all_desktop_clients_can_be_disabled(monkeypatch):
    assert desktop(
        monkeypatch, start_mapping_rviz='false',
        start_cv_rviz='false', start_control_gui='false') == []


@pytest.mark.parametrize('name', [
    'start_mapping_rviz', 'start_cv_rviz', 'start_control_gui'])
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
        'start_control_gui': 'false', 'start_cv_rviz': 'true'})
    actions = module._launch_desktop(context)
    assert len(actions) == 2
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
