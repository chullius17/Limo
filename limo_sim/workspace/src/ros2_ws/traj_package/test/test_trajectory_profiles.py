"""Check trajectory profile selection and planner/bridge YAML parameters."""

import ast
import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent


def trajectory(monkeypatch, **overrides):
    spec = importlib.util.spec_from_file_location(
        'trajectory_launch_test', PACKAGE / 'launch/trajectory.launch.py')
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
    nodes = {action['name']: action for action in description.entities
             if isinstance(action, dict)}
    config = yaml.safe_load(Path(
        nodes['planner_server']['parameters'][0].perform(context)).read_text())
    bridge_config = yaml.safe_load(Path(
        nodes['rviz_goal_bridge']['parameters'][0].perform(context)).read_text())
    assert bridge_config == config
    return context, config


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_planner_and_bridge_use_selected_model_yaml(monkeypatch, profile):
    context, config = trajectory(monkeypatch, robot_model=profile)
    assert context.launch_configurations['planner_params_file'] == str(
        PACKAGE / 'config' / ('traj_' + profile + '.yaml'))
    for name in ('planner_server', 'rviz_goal_bridge'):
        assert config[name]['ros__parameters']['use_sim_time'] is (profile == 'sim')
    assert config['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] is (
        profile == 'sim')
    # Keep the previously configured conservative planner radius for both profiles.
    assert config['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] == 0.7
    assert config['rviz_goal_bridge']['ros__parameters']['max_planning_attempts'] == 48


@pytest.mark.parametrize('profile,clock', [('sim', 'false'), ('real', 'true')])
def test_clock_override_does_not_change_planner_profile(monkeypatch, profile, clock):
    context, config = trajectory(
        monkeypatch, robot_model=profile, use_sim_time=clock)
    assert context.launch_configurations['planner_params_file'].endswith(
        'traj_' + profile + '.yaml')
    for name in ('planner_server', 'rviz_goal_bridge'):
        assert config[name]['ros__parameters']['use_sim_time'] is (clock == 'true')


def test_custom_yaml_controls_planner_bridge_and_map_topic(monkeypatch, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/traj_real.yaml').read_text())
    config['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = 0.9
    config['rviz_goal_bridge']['ros__parameters']['max_planning_attempts'] = 7
    path = tmp_path / 'custom_planner.yaml'
    path.write_text(yaml.safe_dump(config))
    context, result = trajectory(
        monkeypatch, robot_model='sim',
        planner_params_file=str(path), map_topic='/custom/map')
    assert context.launch_configurations['planner_params_file'] == str(path)
    assert result['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] == 0.9
    assert result['rviz_goal_bridge']['ros__parameters']['max_planning_attempts'] == 7
    costmap = result['global_costmap']['global_costmap']['ros__parameters']
    assert costmap['static_layer']['map_topic'] == '/custom/map'
    assert costmap['border_follow_layer']['source_topic'] == '/custom/map'


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_planning_validation_and_control_footprints_agree(profile):
    config = yaml.safe_load(
        (PACKAGE / 'config' / ('traj_' + profile + '.yaml')).read_text())
    control = yaml.safe_load(
        (PACKAGES / 'limo_controller/config' / ('control_' + profile + '.yaml')).read_text())
    global_params = config['global_costmap']['global_costmap']['ros__parameters']
    local_params = control['local_costmap']['local_costmap']['ros__parameters']
    assert global_params['footprint'] == local_params['footprint']
    footprint = ast.literal_eval(global_params['footprint'])
    bridge = config['rviz_goal_bridge']['ros__parameters']
    assert max(x for x, _ in footprint) - min(x for x, _ in footprint) == (
        bridge['footprint_length'])
    assert max(y for _, y in footprint) - min(y for _, y in footprint) == (
        bridge['footprint_width'])


def test_unknown_model_fails(monkeypatch):
    with pytest.raises(RuntimeError, match='robot_model'):
        trajectory(monkeypatch, robot_model='invalid')
