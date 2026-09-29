"""Check model YAML selection, clock overrides, controller geometry and mux."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402


PACKAGE = Path(__file__).resolve().parents[1]


def controller(monkeypatch, **overrides):
    path = PACKAGE / 'launch/control.launch.py'
    spec = importlib.util.spec_from_file_location('control_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: str(PACKAGE)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    action = next(action for action in description.entities
                  if isinstance(action, OpaqueFunction))
    node = action.execute(context)[0]
    assert len(node['parameters']) == 1
    config = yaml.safe_load(Path(node['parameters'][0].perform(context)).read_text())
    return context, config, description


@pytest.mark.parametrize('profile,wheelbase,offset,radius', [
    ('real', 0.20, 0.10, 0.462),
    ('sim', 0.24, 0.12, 0.55),
])
def test_model_yaml_contains_the_effective_ackermann_geometry(
        monkeypatch, profile, wheelbase, offset, radius):
    context, config, _ = controller(monkeypatch, robot_model=profile)
    assert context.launch_configurations['controller_params_file'] == str(
        PACKAGE / 'config' / ('control_' + profile + '.yaml'))
    params = config['controller_server']['ros__parameters']['FollowPath']
    assert params['MPC']['wheelbase'] == wheelbase
    assert params['MPC']['rear_axle_to_base'] == offset
    assert params['AckermannKinematics.min_turning_radius'] == radius
    assert config['controller_server']['ros__parameters']['use_sim_time'] is (
        profile == 'sim')
    assert config['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is (
        profile == 'sim')


@pytest.mark.parametrize('profile,clock', [('sim', 'false'), ('real', 'true')])
def test_clock_override_does_not_change_control_profile(monkeypatch, profile, clock):
    context, config, _ = controller(
        monkeypatch, robot_model=profile, use_sim_time=clock)
    assert context.launch_configurations['controller_params_file'].endswith(
        'control_' + profile + '.yaml')
    assert config['controller_server']['ros__parameters']['use_sim_time'] is (
        clock == 'true')
    assert config['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is (
        clock == 'true')
    assert config['twist_mux']['ros__parameters']['use_sim_time'] is (
        clock == 'true')


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_custom_control_file_preserves_geometry(monkeypatch, tmp_path, profile):
    config = yaml.safe_load((PACKAGE / 'config/control_real.yaml').read_text())
    params = config['controller_server']['ros__parameters']['FollowPath']
    params['MPC']['wheelbase'] = 0.31
    params['MPC']['rear_axle_to_base'] = 0.155
    params['AckermannKinematics.min_turning_radius'] = 0.82
    path = tmp_path / 'custom_control.yaml'
    path.write_text(yaml.safe_dump(config))
    context, result, _ = controller(
        monkeypatch, robot_model=profile, controller_params_file=str(path))
    assert context.launch_configurations['controller_params_file'] == str(path)
    assert result['controller_server']['ros__parameters']['FollowPath'] == params


def test_real_control_defaults_to_wall_clock_without_gui(monkeypatch):
    context, _, description = controller(monkeypatch)
    gui = next(action for action in description.entities
               if isinstance(action, dict) and action.get('name') == 'control_gui')
    assert not gui['condition'].evaluate(context)
    assert context.launch_configurations['use_sim_time'].lower() == 'false'


def test_simulation_retains_its_local_control_gui(monkeypatch):
    context, _, description = controller(monkeypatch, robot_model='sim')
    gui = next(action for action in description.entities
               if isinstance(action, dict) and action.get('name') == 'control_gui')
    assert gui['condition'].evaluate(context)


def test_unknown_model_fails(monkeypatch):
    with pytest.raises(RuntimeError, match='robot_model'):
        controller(monkeypatch, robot_model='differential')


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_mux_uses_selected_control_profile(monkeypatch, profile):
    context, config, description = controller(monkeypatch, robot_model=profile)
    mux = next(action for action in description.entities
               if isinstance(action, dict) and action.get('name') == 'twist_mux')
    assert len(mux['parameters']) == 1
    mux_config = yaml.safe_load(
        Path(mux['parameters'][0].perform(context)).read_text())
    assert mux_config == config
    params = mux_config['twist_mux']['ros__parameters']
    assert params['use_sim_time'] is (profile == 'sim')
    assert params['output_topic'] == '/cmd_vel'
    assert params['publish_rate'] == 50.0
    assert params['topics'] == {
        'autonomy': {
            'topic': '/cmd_vel_autonomy', 'timeout': 0.5, 'priority': 10,
        },
        'teleop': {
            'topic': '/cmd_vel_teleop', 'timeout': 0.5, 'priority': 100,
        },
    }


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_custom_control_file_also_overrides_mux(monkeypatch, tmp_path, profile):
    config = yaml.safe_load((PACKAGE / 'config/control_real.yaml').read_text())
    params = config['twist_mux']['ros__parameters']
    params['output_topic'] = '/test/cmd_vel'
    params['topics']['teleop']['priority'] = 150
    path = tmp_path / 'custom_control_mux.yaml'
    path.write_text(yaml.safe_dump(config))
    context, result, description = controller(
        monkeypatch, robot_model=profile, controller_params_file=str(path))
    mux = next(action for action in description.entities
               if isinstance(action, dict) and action.get('name') == 'twist_mux')
    mux_config = yaml.safe_load(
        Path(mux['parameters'][0].perform(context)).read_text())
    assert mux_config == result
    actual = mux_config['twist_mux']['ros__parameters']
    assert actual['output_topic'] == '/test/cmd_vel'
    assert actual['topics']['teleop']['priority'] == 150
    assert actual['use_sim_time'] is (profile == 'sim')
