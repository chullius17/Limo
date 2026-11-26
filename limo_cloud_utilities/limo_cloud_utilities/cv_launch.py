"""Reuse the CV pipeline and viewer, replacing its cloud stage with the C++ host."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_cloud_utilities.cv_configuration import cloud_parameters


def _boolean(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    raise ValueError('Expected true or false, got {!r}'.format(value))


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    profile = value('profile')
    if profile not in ('real', 'sim'):
        raise ValueError('profile must be real or sim')
    mode = value('mode')
    if mode not in ('profile', 'backend', 'desktop'):
        raise ValueError('mode must be profile, backend or desktop')

    cv_share = get_package_share_directory('cv_package')
    filename = os.path.expanduser(value('config_file')) or os.path.join(
        cv_share, 'config', 'cv_' + profile + '.yaml')
    with open(filename, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('launch'), dict):
        raise ValueError('Expected a launch mapping in the CV profile')
    clock = value('use_sim_time')
    use_sim_time = _boolean(clock) if clock else _boolean(config['launch']['use_sim_time'])

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cv_share, 'launch', 'cv.launch.py')),
        launch_arguments={
            'config_file': filename,
            'mode': mode,
            'use_sim_time': str(use_sim_time).lower(),
            'start_rviz': value('start_rviz'),
            'start_visual_ptcld': 'false',
        }.items(),
    )]
    if mode != 'desktop':
        parameters = cloud_parameters(config.get('visual_ptcld'))
        parameters['use_sim_time'] = use_sim_time
        if value('enable_telemetry'):
            parameters['enable_telemetry'] = _boolean(value('enable_telemetry'))
        actions.append(Node(
            package='limo_cloud_utilities', executable='limo_cloud_utilities_node',
            name='limo_cloud_utilities', output='screen', emulate_tty=True,
            parameters=[parameters], additional_env={
                'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1', 'BLIS_NUM_THREADS': '1',
            },
        ))
    return actions


def generate_launch_description(profile='real', mode='profile', start_rviz=''):
    """Expose the same override arguments directly on each integration entrypoint."""
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=profile,
                              description='Original CV profile: real or sim.'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='Original CV YAML override; empty selects the profile.'),
        DeclareLaunchArgument('mode', default_value=mode,
                              description='profile: YAML viewer setting; backend: no viewer; '
                                          'desktop: viewer only.'),
        DeclareLaunchArgument('use_sim_time', default_value='',
                              description='Override the clock for every pipeline node and viewer.'),
        DeclareLaunchArgument('start_rviz', default_value=start_rviz,
                              description='Override the original profile viewer setting.'),
        DeclareLaunchArgument('enable_telemetry', default_value='',
                              description='Override C++ cloud timing reports.'),
        OpaqueFunction(function=_launch),
    ])
