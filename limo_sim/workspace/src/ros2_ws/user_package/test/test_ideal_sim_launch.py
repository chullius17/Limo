"""Verify ideal planning stays separate from the live controller costmap."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch')
pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent


def compose(monkeypatch, **overrides):
    path = PACKAGE / 'launch/experiments/limo_app_ideal_sim.launch.py'
    spec = importlib.util.spec_from_file_location('ideal_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda name: str(PACKAGES / name))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, '_include', lambda package, filename, arguments: {
        'package': package, 'launch': filename, 'arguments': arguments})
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return module._launch_experiment(context)


def test_ideal_map_drives_planning_while_control_keeps_live_sensors(monkeypatch):
    mapping, local_map, planner, controller = compose(monkeypatch)
    assert mapping['launch'] == 'experiments/online_map_ideal_sim.launch.py'
    assert Path(mapping['arguments']['map_yaml']) == (
        PACKAGES / 'offline_map_package/experiments/sim/ideal_map/ideal_map.yaml')
    assert Path(mapping['arguments']['laser_map_yaml']) == (
        PACKAGES / 'offline_map_package/experiments/sim/ideal_map/ideal_laser_map.yaml')
    assert mapping['arguments']['start_cv'] == 'true'
    assert mapping['arguments']['start_monitor'] == 'false'
    assert planner['launch'] == 'trajectory.launch.py'
    assert planner['arguments']['map_topic'] == '/map'
    assert controller['launch'] == 'control.launch.py'
    assert local_map['executable'] == 'local_ctrl_map'
    source = yaml.safe_load(
        (PACKAGES / 'online_map_package/config/mapping_sim.yaml').read_text())
    params = local_map['parameters'][0]
    assert all(params[name] == value for name, value in source['local_ctrl_map'].items())
    control = yaml.safe_load(Path(controller['arguments']['controller_params_file']).read_text())
    costmap = control['local_costmap']['local_costmap']['ros__parameters']
    assert costmap['plugins'] == [
        'obstacle_layer', 'local_ctrl_map_layer', 'inflation_layer', 'path_distance_layer']
    assert costmap['obstacle_layer']['scan']['topic'] == '/scan'
    assert costmap['local_ctrl_map_layer']['map_topic'] == (
        '/limo/map_package/online/local_costmap')
    for subsystem in (planner, controller):
        assert subsystem['arguments']['robot_model'] == 'sim'
        assert subsystem['arguments']['use_sim_time'] == 'true'
    assert params['use_sim_time'] is True


def test_existing_cv_and_headless_execution_are_supported(monkeypatch):
    mapping, _, _, controller = compose(
        monkeypatch, start_cv='false', start_rviz='false',
        start_control_gui='false', start_mpc_preview='false')
    assert mapping['arguments']['start_cv'] == 'false'
    assert mapping['arguments']['start_rviz'] == 'false'
    assert controller['arguments']['start_gui'] == 'false'
    assert controller['arguments']['start_mpc_preview'] == 'false'
    assert Path(controller['arguments']['mpc_preview_params_file']).is_file()


def test_custom_map_and_tuning_files_are_forwarded(monkeypatch, tmp_path):
    map_yaml = tmp_path / 'ideal.yaml'
    map_yaml.write_text('image: ideal.pgm\n')
    mapping, _, planner, controller = compose(
        monkeypatch, map_yaml=str(map_yaml), planner_params_file='/tmp/planner.yaml',
        controller_params_file='/tmp/controller.yaml', laser_map_yaml='/tmp/walls.yaml')
    assert mapping['arguments']['map_yaml'] == str(map_yaml)
    assert mapping['arguments']['laser_map_yaml'] == '/tmp/walls.yaml'
    assert planner['arguments']['planner_params_file'] == '/tmp/planner.yaml'
    assert controller['arguments']['controller_params_file'] == '/tmp/controller.yaml'


def test_missing_map_is_rejected(monkeypatch, tmp_path):
    with pytest.raises(ValueError, match='Ideal map YAML not found'):
        compose(monkeypatch, map_yaml=str(tmp_path / 'missing.yaml'))


@pytest.mark.parametrize('name', [
    'start_cv', 'start_rviz', 'start_control_gui', 'start_mpc_preview'])
def test_invalid_switch_is_rejected(monkeypatch, name):
    with pytest.raises(ValueError, match=name + ' must be true or false'):
        compose(monkeypatch, **{name: 'invalid'})
