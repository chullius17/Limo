"""Verify both application launches compose all LIMO subsystems."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch')
from launch import LaunchContext  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.utilities import (  # noqa: E402
    normalize_to_list_of_substitutions, perform_substitutions)


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent


def load_launch(profile):
    filename = 'limo_app.launch.py' if profile == 'legacy' else 'limo_app_{}.launch.py'.format(profile)
    path = PACKAGE / 'launch' / filename
    spec = importlib.util.spec_from_file_location(
        'limo_app_{}_launch'.format(profile), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compose(monkeypatch, profile, **overrides):
    module = load_launch(profile)
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda name: str(PACKAGES / name))
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    if profile != 'legacy':
        include = next(action for action in description.entities
                       if isinstance(action, IncludeLaunchDescription))
        assert isinstance(include, IncludeLaunchDescription)
        include.launch_description_source.get_launch_description(context)
        assert include.launch_description_source.location.endswith('/launch/limo_app.launch.py')
        context.launch_configurations.update({
            name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
            for name, value in include.launch_arguments})
        module = load_launch('legacy')
        monkeypatch.setattr(
            module, 'get_package_share_directory', lambda name: str(PACKAGES / name))
        description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(
        module, '_include',
        lambda package, launch, arguments=None: {
            'package': package,
            'launch': launch,
            'arguments': arguments or {},
        })
    opaque_action = next(
        action for action in description.entities
        if isinstance(action, OpaqueFunction))
    return opaque_action.execute(context)


def test_sim_launches_map_trajectory_and_controller(monkeypatch):
    actions = compose(monkeypatch, 'sim')
    assert [(item['package'], item['launch']) for item in actions] == [
        ('online_map_package', 'online_map_sim.launch.py'),
        ('traj_package', 'trajectory.launch.py'),
        ('limo_controller', 'control.launch.py'),
    ]
    assert actions[1]['arguments'] == {
        'robot_model': 'sim',
        'planner_params_file': str(PACKAGES / 'traj_package/config/traj_sim.yaml'),
        'map_topic': '/map',
        'use_sim_time': 'true',
    }
    assert actions[2]['arguments']['start_gui'] == 'true'
    assert actions[2]['arguments']['robot_model'] == 'sim'


def test_real_uses_wall_clock_and_robot_feedback(monkeypatch):
    actions = compose(monkeypatch, 'real')
    assert actions[0]['launch'] == 'online_map_real.launch.py'
    assert actions[1]['arguments']['use_sim_time'] == 'false'
    assert actions[2]['arguments']['use_sim_time'] == 'false'
    assert actions[2]['arguments']['start_gui'] == 'false'
    assert actions[2]['arguments']['robot_model'] == 'real'


def test_application_overrides_are_forwarded(monkeypatch):
    actions = compose(
        monkeypatch, 'sim', map_topic='/custom_map',
        start_control_gui='true')
    assert actions[1]['arguments']['map_topic'] == '/custom_map'
    assert actions[2]['arguments']['start_gui'] == 'true'


def test_mpc_preview_options_are_forwarded(monkeypatch):
    actions = compose(monkeypatch, 'sim', start_mpc_preview='false',
                      mpc_preview_params_file='/tmp/custom_preview.yaml')
    assert actions[2]['arguments']['start_mpc_preview'] == 'false'
    assert actions[2]['arguments']['mpc_preview_params_file'] == '/tmp/custom_preview.yaml'
    assert compose(monkeypatch, 'sim')[2]['arguments']['start_mpc_preview'] == ''
    assert compose(monkeypatch, 'real')[2]['arguments']['start_mpc_preview'] == ''
    assert compose(monkeypatch, 'sim', start_mpc_preview='true')[2][
        'arguments']['start_mpc_preview'] == 'true'


@pytest.mark.parametrize('profile', ['sim', 'real'])
@pytest.mark.parametrize('override', ['', 'true', 'false'])
def test_mpc_preview_yaml_resolves_despite_empty_parent_argument(monkeypatch, profile, override):
    from nav2_common.launch import RewrittenYaml

    controller = compose(monkeypatch, profile, start_mpc_preview=override)[2]
    module = load_subsystem(controller['package'], controller['launch'])
    context = launch_context(module, {
        'mpc_preview_params_file': '', **controller['arguments']})
    selected = context.launch_configurations['mpc_preview_params_file']
    assert selected == str(PACKAGES / 'limo_controller/config/mpc_preview_sim.yaml')
    rewritten = RewrittenYaml(
        source_file=selected, root_key='',
        param_rewrites={'use_sim_time': controller['arguments']['use_sim_time']},
        convert_types=True)
    config = yaml.safe_load(Path(rewritten.perform(context)).read_text())
    assert config['mpc_preview']['ros__parameters']['use_sim_time'] is (profile == 'sim')
    module.Node = lambda **kwargs: kwargs
    controller_yaml = RewrittenYaml(
        source_file=context.launch_configurations['controller_params_file'], root_key='',
        param_rewrites={'use_sim_time': controller['arguments']['use_sim_time']},
        convert_types=True)
    preview = module.mpc_preview_node(context, controller_yaml, rewritten)[0]
    expected = profile == 'sim' if override == '' else override == 'true'
    assert preview['condition'].evaluate(context) is expected


def test_invalid_control_gui_setting_fails(monkeypatch):
    with pytest.raises(ValueError):
        compose(monkeypatch, 'sim', start_control_gui='invalid')


def test_invalid_internal_profile_fails():
    module = load_launch('legacy')
    context = LaunchContext()
    context.launch_configurations['profile'] = 'invalid'
    with pytest.raises(ValueError):
        module._launch_app(context)


@pytest.mark.parametrize('profile', ['real', 'sim', 'legacy'])
def test_cv_inherits_config_and_explicit_override_is_forwarded_once(monkeypatch, profile):
    actions = compose(monkeypatch, profile)
    assert 'start_cv' not in actions[0]['arguments']
    actions = compose(monkeypatch, profile, start_cv='true', cv_config='/tmp/custom_cv.yaml')
    assert len(actions) == 3
    assert actions[0]['arguments']['start_cv'] == 'true'
    assert actions[0]['arguments']['cv_config'] == '/tmp/custom_cv.yaml'
    assert all(item['package'] != 'cv_package' for item in actions)
    if profile == 'legacy':
        assert actions[0]['launch'] == 'online_map_sim.launch.py'
        assert actions[0]['arguments']['use_sim_time'] == 'false'


@pytest.mark.parametrize('profile', ['real', 'sim', 'legacy'])
def test_cv_can_be_disabled_when_already_running(monkeypatch, profile):
    actions = compose(monkeypatch, profile, start_cv='false')
    assert actions[0]['arguments']['start_cv'] == 'false'


def test_invalid_cv_switch_fails(monkeypatch):
    with pytest.raises(ValueError):
        compose(monkeypatch, 'real', start_cv='typo')


def load_subsystem(package, filename):
    spec = importlib.util.spec_from_file_location(
        package + '_launch_test', PACKAGES / package / 'launch' / filename)
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


@pytest.mark.parametrize('profile', ['real', 'sim', 'legacy'])
@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('override', ['', 'true', 'false'])
def test_application_cv_follows_yaml_and_connects_to_mapping(
        monkeypatch, tmp_path, profile, enabled, override):
    pytest.importorskip('launch_ros')
    from launch.actions import IncludeLaunchDescription

    mapping_profile = 'sim' if profile == 'legacy' else profile
    config_name = 'mapping_' + mapping_profile + '.yaml'
    config = yaml.safe_load(
        (PACKAGES / 'online_map_package' / 'config' / config_name).read_text())
    config['launch']['start_cv'] = enabled
    config_dir = tmp_path / 'config'
    config_dir.mkdir()
    (config_dir / config_name).write_text(yaml.safe_dump(config))

    application = compose(monkeypatch, profile, start_cv=override)
    wrapper = load_subsystem('online_map_package', application[0]['launch'])
    wrapper.get_package_share_directory = lambda name: str(tmp_path)
    include = wrapper.generate_launch_description().entities[0]
    online = load_subsystem('online_map_package', 'online_map.launch.py')
    context = launch_context(online, {
        **dict(include.launch_arguments), **application[0]['arguments']})
    monkeypatch.setattr(online, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(online, 'GroupAction', lambda actions: actions)
    actions = online._launch_online(context)
    cv_includes = [include for action in actions if isinstance(action, list)
                   for include in action]
    expected_enabled = enabled if override == '' else override == 'true'
    assert len(cv_includes) == int(expected_enabled)
    if not expected_enabled:
        return

    cv_arguments = dict(cv_includes[0].launch_arguments)
    assert cv_arguments['config_file'].endswith('/cv_' + mapping_profile + '.yaml')
    assert cv_arguments['mode'] == 'backend'
    expected_clock = 'true' if profile == 'sim' else 'false'
    assert cv_arguments['use_sim_time'] == expected_clock
    cv = load_subsystem('cv_package', 'cv.launch.py')
    cv_context = launch_context(cv, cv_arguments)
    monkeypatch.setattr(cv, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(cv, 'OnProcessStart', lambda **kwargs: kwargs)
    monkeypatch.setattr(cv, 'RegisterEventHandler', lambda handler: handler)
    pipeline = cv._launch_cv(cv_context)
    lane = next(node for node in pipeline if node.get('name') == 'lane_node')
    expected_detector = 'lane_detector_waterfall' if profile == 'real' else 'lane_detector'
    assert lane['executable'] == expected_detector
    handler = next(action for action in pipeline if 'on_start' in action)
    cloud = handler['on_start'][0]['parameters'][0]
    amcl = next(dict(action.launch_arguments) for action in actions
                if isinstance(action, IncludeLaunchDescription))
    assert amcl['cv_enabled'] == 'true'
    assert '/' + cloud['pointcloud_topic'].lstrip('/') == amcl['cv_cloud_topic']
    local_map = next(action for action in actions
                     if isinstance(action, dict) and action['name'] == 'local_ctrl_map')
    local_parameters = local_map['parameters'][0]
    assert local_parameters.get('input_topic', '/limo/cv_package/visual_ptcld/points') == (
        amcl['cv_cloud_topic'])
    assert cloud['use_sim_time'] is (profile == 'sim')


def test_real_application_rejects_a_local_control_gui(monkeypatch):
    with pytest.raises(ValueError, match='desktop_app.launch.py'):
        compose(monkeypatch, 'real', start_control_gui='true')


def test_real_mapping_is_forced_headless(monkeypatch):
    actions = compose(monkeypatch, 'real')
    assert actions[0]['arguments']['mode'] == 'backend'
    assert actions[0]['arguments']['start_rviz'] == 'false'
    assert actions[0]['arguments']['use_sim_time'] == 'false'


def test_custom_mapping_profile_is_forwarded_to_backend(monkeypatch):
    actions = compose(
        monkeypatch, 'real', mapping_config='/tmp/custom_mapping.yaml',
        start_cv='false')
    assert actions[0]['launch'] == 'online_map.launch.py'
    assert actions[0]['arguments']['config_file'] == '/tmp/custom_mapping.yaml'
    assert actions[0]['arguments']['mode'] == 'backend'
    assert actions[0]['arguments']['start_cv'] == 'false'
    assert actions[2]['arguments']['start_gui'] == 'false'


@pytest.mark.parametrize('profile,expected', [
    ('real', 'real'), ('sim', 'sim'), ('legacy', 'sim')])
def test_application_selects_matching_planner_and_control_models(
        monkeypatch, profile, expected):
    actions = compose(monkeypatch, profile)
    assert actions[1]['arguments']['robot_model'] == expected
    assert actions[2]['arguments']['robot_model'] == expected
    assert actions[1]['arguments']['planner_params_file'].endswith(
        '/traj_' + expected + '.yaml')
    assert actions[2]['arguments']['controller_params_file'].endswith(
        '/control_' + expected + '.yaml')


def test_application_forwards_custom_planner_and_controller_files(monkeypatch):
    actions = compose(
        monkeypatch, 'real', planner_params_file='/tmp/planner.yaml',
        controller_params_file='/tmp/control.yaml')
    assert actions[1]['arguments']['planner_params_file'] == '/tmp/planner.yaml'
    assert actions[2]['arguments']['controller_params_file'] == '/tmp/control.yaml'


@pytest.mark.parametrize('profile,model,clock', [
    ('real', 'real', False), ('sim', 'sim', True), ('legacy', 'sim', False)])
def test_application_resolves_yaml_defaults_despite_empty_parent_arguments(
        monkeypatch, profile, model, clock):
    application = compose(monkeypatch, profile)
    for include in application[1:]:
        module = load_subsystem(include['package'], include['launch'])
        monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
        context = launch_context(module, {
            'planner_params_file': '', 'controller_params_file': '',
            **include['arguments']})
        description = module.generate_launch_description()
        if include['package'] == 'traj_package':
            node = next(action for action in description.entities
                        if isinstance(action, dict) and action['name'] == 'planner_server')
            selected = context.launch_configurations['planner_params_file']
        else:
            action = next(action for action in description.entities
                          if isinstance(action, OpaqueFunction))
            node = action.execute(context)[0]
            selected = context.launch_configurations['controller_params_file']
        assert selected.endswith('_' + model + '.yaml')
        config = yaml.safe_load(Path(node['parameters'][0].perform(context)).read_text())
        assert config[node['name']]['ros__parameters']['use_sim_time'] is clock
