"""Verify a single experiment launch selects clocks without duplicating bringup."""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip('launch')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # noqa: E402


def compose(monkeypatch, **overrides):
    path = Path(__file__).resolve().parents[1] / 'launch/experiments/ekf_experiment.launch.py'
    spec = importlib.util.spec_from_file_location('experiment_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: '/custom_start')
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return module._launch_experiment(context)


@pytest.mark.parametrize('profile, sim_time', [('real', False), ('sim', True)])
def test_common_launch_selects_clock_and_starts_only_viewer(monkeypatch, profile, sim_time):
    actions = compose(monkeypatch, profile=profile)
    assert len(actions) == 1
    assert actions[0]['parameters'][0]['use_sim_time'] is sim_time
    assert actions[0]['executable'] == 'ekf_experiment'


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_optional_bringup_includes_existing_ekf_launch(monkeypatch, profile):
    actions = compose(monkeypatch, profile=profile, start_bringup='true')
    assert len(actions) == 2
    assert isinstance(actions[0], IncludeLaunchDescription)


def test_clock_override_and_custom_topics(monkeypatch):
    params = compose(monkeypatch, profile='real', use_sim_time='true',
                     raw_topic='/bag/odom', filtered_topic='/candidate')[0]['parameters'][0]
    assert params['use_sim_time'] is True
    assert params['raw_topic'] == '/bag/odom'
    assert params['filtered_topic'] == '/candidate'
    with pytest.raises(ValueError):
        compose(monkeypatch, profile='real', start_bringup='true', use_sim_time='true')
