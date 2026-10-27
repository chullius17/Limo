"""Verify the viewer follows the bridge's configured costmap and clock."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402


PACKAGE = Path(__file__).resolve().parents[1]


def compose(monkeypatch, **overrides):
    spec = importlib.util.spec_from_file_location(
        'planning_gui_launch', PACKAGE / 'launch/planning_gui.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGE))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return module._launch_gui(context)[0], context


def test_default_launch_uses_planning_costmap_and_sim_clock(monkeypatch):
    node, context = compose(monkeypatch)
    assert node['executable'] == 'planning_gui'
    params = node['parameters'][0]
    assert params['costmap_topic'] == '/global_costmap/costmap'
    assert params['footprint_topic'] == '/global_costmap/published_footprint'
    assert params['robot_base_frame'] == 'base_link'
    assert params['goal_topic'] == '/goal_pose'
    assert params['adjusted_goal_topic'] == '/adjusted_goal_pose'
    assert params['adjusted_start_topic'] == '/adjusted_start_pose'
    assert params['planning_log_topic'] == '/limo/planning/log'
    assert params['image_publish_rate'].evaluate(context) == 5.0
    assert '-0.110' in params['footprint']
    assert params['use_sim_time'].evaluate(context) is True


def test_custom_bridge_topic_and_wall_clock_are_respected(monkeypatch, tmp_path):
    path = tmp_path / 'planner.yaml'
    path.write_text(yaml.safe_dump({
        'rviz_goal_bridge': {'ros__parameters': {
            'costmap_topic': '/custom/planner_map', 'goal_topic': '/custom/goal',
            'adjusted_goal_topic': '/custom/adjusted_goal',
            'adjusted_start_topic': '/custom/adjusted_start',
            'planning_log_topic': '/custom/planning_log'}}}))
    node, context = compose(monkeypatch, planner_params_file=str(path), use_sim_time='false')
    params = node['parameters'][0]
    assert params['costmap_topic'] == '/custom/planner_map'
    assert params['goal_topic'] == '/custom/goal'
    assert params['adjusted_goal_topic'] == '/custom/adjusted_goal'
    assert params['adjusted_start_topic'] == '/custom/adjusted_start'
    assert params['planning_log_topic'] == '/custom/planning_log'
    assert params['use_sim_time'].evaluate(context) is False


def test_topic_override_can_run_without_a_planner_yaml(monkeypatch):
    node, _ = compose(
        monkeypatch, planner_params_file='/missing/planner.yaml', costmap_topic='/other/map')
    assert node['parameters'][0]['costmap_topic'] == '/other/map'


def test_real_planner_geometry_and_footprint_topic_override(monkeypatch):
    node, _ = compose(
        monkeypatch, planner_params_file=str(PACKAGE / 'config/traj_real.yaml'),
        footprint_topic='/custom/footprint')
    params = node['parameters'][0]
    assert params['footprint_topic'] == '/custom/footprint'
    assert '-0.100' in params['footprint']
    assert params['footprint_padding'] == 0.0
