# Copyright 2026 Giulio Cataldo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    for extra in tuple(description.entities):
        if isinstance(extra, OpaqueFunction) and extra is not action:
            for resolved in extra.execute(context):
                description.add_action(resolved)
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
    assert params['publish_rate'] == (20.0 if profile == 'real' else 50.0)
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


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_executor_uses_selected_profile_and_real_chassis_guard(monkeypatch, profile):
    context, config, description = controller(monkeypatch, robot_model=profile)
    executor = next(action for action in description.entities
                    if isinstance(action, dict) and action.get('name') == 'path_executor')
    assert len(executor['parameters']) == 2
    assert executor['parameters'][1]['require_chassis_status'].evaluate(
        context) is (profile == 'real')
    params = yaml.safe_load(Path(executor['parameters'][0].perform(context)).read_text())
    assert params == config
    guard = params.get('path_executor', {}).get('ros__parameters', {})
    assert guard.get('require_chassis_status', False) is (profile == 'real')
    if profile == 'real':
        assert guard['commanded_control_mode'] == 1
        assert guard['chassis_status_timeout'] == 1.0


def test_custom_real_yaml_does_not_accidentally_disable_chassis_guard(
        monkeypatch, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/control_real.yaml').read_text())
    del config['path_executor']
    path = tmp_path / 'custom_without_guard.yaml'
    path.write_text(yaml.safe_dump(config))
    context, _, description = controller(
        monkeypatch, robot_model='real', controller_params_file=str(path))
    executor = next(action for action in description.entities
                    if isinstance(action, dict) and action.get('name') == 'path_executor')
    assert executor['parameters'][1]['require_chassis_status'].evaluate(context)


def test_nano_budget_preserves_horizon_and_safety_timeouts():
    config = yaml.safe_load((PACKAGE / 'config/control_real.yaml').read_text())
    params = config['controller_server']['ros__parameters']
    mpc = params['FollowPath']['MPC']
    assert params['controller_frequency'] == 10.0
    assert mpc['model_dt'] * params['controller_frequency'] == 1.0
    assert mpc['model_dt'] * mpc['time_steps'] == 4.0
    assert mpc['batch_size'] == 128
    assert mpc['batch_size'] >= 2 + mpc['velocity_samples'] * mpc['curvature_samples']
    timeout = config['twist_mux']['ros__parameters']['topics']['autonomy']['timeout']
    assert 3 * mpc['model_dt'] < timeout == 0.5
    assert params['progress_checker']['movement_time_allowance'] == 10.0


@pytest.mark.parametrize('profile', ['sim', 'real'])
def test_default_launch_enables_telemetry_without_image_renderer(monkeypatch, profile):
    context, config, description = controller(monkeypatch, robot_model=profile)
    debug = config['controller_server']['ros__parameters']['FollowPath']['MPC']['Debug']
    assert debug['enabled'] is True
    preview = next(action for action in description.entities
                   if isinstance(action, dict) and action.get('name') == 'mpc_preview')
    assert not preview['condition'].evaluate(context)


@pytest.mark.parametrize('profile,clock,requested,enabled', [
    ('sim', 'true', 'true', True),
    ('sim', 'false', 'true', True),
    ('sim', 'true', 'false', False),
    ('real', 'false', 'true', True),
    ('real', 'true', 'true', True),
])
def test_mpc_preview_uses_yaml_flag_and_optional_launch_disable(
        monkeypatch, profile, clock, requested, enabled):
    context, _, description = controller(
        monkeypatch, robot_model=profile, use_sim_time=clock, start_mpc_preview=requested)
    preview = next(action for action in description.entities
                   if isinstance(action, dict) and action.get('name') == 'mpc_preview')
    assert preview['condition'].evaluate(context) is enabled
    config = yaml.safe_load(Path(preview['parameters'][0].perform(context)).read_text())
    assert config['mpc_preview']['ros__parameters']['use_sim_time'] is (clock == 'true')


@pytest.mark.parametrize('profile,enabled', [('sim', False), ('real', True)])
def test_custom_yaml_controls_mpc_preview_activation(monkeypatch, tmp_path, profile, enabled):
    config = yaml.safe_load((PACKAGE / 'config' / ('control_' + profile + '.yaml')).read_text())
    debug = config['controller_server']['ros__parameters']['FollowPath']['MPC']['Debug']
    debug['enabled'] = enabled
    path = tmp_path / 'control.yaml'
    path.write_text(yaml.safe_dump(config))
    context, _, description = controller(
        monkeypatch, robot_model=profile, controller_params_file=str(path),
        start_mpc_preview='true')
    preview = next(action for action in description.entities
                   if isinstance(action, dict) and action.get('name') == 'mpc_preview')
    assert preview['condition'].evaluate(context) is enabled


def test_yaml_disabled_preview_cannot_be_enabled_by_launch(monkeypatch, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/control_sim.yaml').read_text())
    config['controller_server']['ros__parameters']['FollowPath']['MPC']['Debug']['enabled'] = False
    path = tmp_path / 'control.yaml'
    path.write_text(yaml.safe_dump(config))
    context, _, description = controller(
        monkeypatch, robot_model='sim', controller_params_file=str(path), start_mpc_preview='true')
    preview = next(action for action in description.entities
                   if isinstance(action, dict) and action.get('name') == 'mpc_preview')
    assert not preview['condition'].evaluate(context)


def test_custom_mpc_preview_file_is_used(monkeypatch, tmp_path):
    path = tmp_path / 'preview.yaml'
    path.write_text(yaml.safe_dump({'mpc_preview': {'ros__parameters': {
        'use_sim_time': True, 'image_width': 720, 'pixels_per_meter': 100.0,
    }}}))
    context, _, description = controller(
        monkeypatch, robot_model='sim', mpc_preview_params_file=str(path))
    preview = next(action for action in description.entities
                   if isinstance(action, dict) and action.get('name') == 'mpc_preview')
    config = yaml.safe_load(Path(preview['parameters'][0].perform(context)).read_text())
    assert config['mpc_preview']['ros__parameters']['image_width'] == 720


def test_gui_follows_custom_telemetry_topics_and_preview_timeout(monkeypatch, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/control_sim.yaml').read_text())
    config['controller_server']['ros__parameters']['FollowPath']['MPC']['Debug'][
        'topic'] = '/custom/mpc_debug'
    control_path = tmp_path / 'control.yaml'
    control_path.write_text(yaml.safe_dump(config))
    preview_path = tmp_path / 'preview.yaml'
    preview_path.write_text(yaml.safe_dump({'mpc_preview': {'ros__parameters': {
        'debug_topic': '/custom/mpc_debug', 'image_topic': '/custom/preview/compressed',
        'stale_timeout': 2.5,
    }}}))
    context, _, description = controller(
        monkeypatch, robot_model='sim', controller_params_file=str(control_path),
        mpc_preview_params_file=str(preview_path))
    gui = next(action for action in description.entities
               if isinstance(action, dict) and action.get('name') == 'control_gui')
    params = gui['parameters'][0]
    assert params['mpc_debug_topic'] == '/custom/mpc_debug'
    assert params['mpc_image_topic'] == '/custom/preview/compressed'
    assert params['mpc_stale_timeout'] == 2.5
    assert params['use_sim_time'].evaluate(context)
