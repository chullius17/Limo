"""Verify standalone MPC GUI profile clocks and controller geometry."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('model,clock', [('real', False), ('sim', True)])
def test_standalone_gui_uses_profile_clock_geometry_and_no_controller(monkeypatch, model, clock):
    spec = importlib.util.spec_from_file_location(
        'mpc_gui_launch', PACKAGE / 'launch/mpc_gui.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGE))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations['robot_model'] = model
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = module._launch_gui(context)
    assert len(nodes) == 1
    assert nodes[0]['executable'] == 'mpc_gui'
    params = nodes[0]['parameters'][-1]
    assert params['use_sim_time'] is clock
    controller = yaml.safe_load((PACKAGE / 'config' / ('control_' + model + '.yaml')).read_text())
    assert params['tube_radius'] == controller['local_costmap']['local_costmap'][
        'ros__parameters']['path_distance_layer']['tube_radius']
    assert params['goal_circle_radius'] == controller['path_executor'][
        'ros__parameters']['goal_circle_radius']
    rewritten = nodes[0]['parameters'][0].perform(context)
    values = yaml.safe_load(Path(rewritten).read_text())['mpc_preview']['ros__parameters']
    assert values['use_sim_time'] is clock
    assert values['trigger_margin'] == params['trigger_margin']
