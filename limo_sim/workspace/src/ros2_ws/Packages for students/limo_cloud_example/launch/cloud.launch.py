"""Start one classified-RGBD processing node, with optional local tuning."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_cloud_example.configuration import load_parameters


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    profile = value('profile')
    if profile not in ('default', 'real', 'sim'):
        raise ValueError('profile must be default, real or sim')
    parameters = load_parameters(value('config_file'))
    if profile != 'default':
        # These are local cloud-only settings, independent of cv_package.
        directory = os.path.join(get_package_share_directory('limo_cloud_example'), 'config')
        parameters.update(load_parameters(os.path.join(directory, profile + '.yaml')))
    for argument in ('use_sim_time', 'enable_telemetry'):
        override = value(argument)
        if override:
            if override.lower() not in ('true', 'false'):
                raise ValueError(argument + ' must be true or false')
            parameters[argument] = override.lower() == 'true'
    return [Node(
        package='limo_cloud_example', executable='limo_cloud_example_node',
        name='limo_cloud_example', output='screen', emulate_tty=True,
        parameters=[parameters], additional_env={
            'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
            'MKL_NUM_THREADS': '1', 'BLIS_NUM_THREADS': '1',
        },
    )]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('limo_cloud_example'), 'config', 'cloud.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config,
                              description='Local classified-RGBD node parameters and Python plugin overrides.'),
        DeclareLaunchArgument('profile', default_value='default',
                              description='Optional local cloud tuning: default, real or sim.'),
        DeclareLaunchArgument('use_sim_time', default_value='',
                              description='Override the clock: true or false.'),
        DeclareLaunchArgument('enable_telemetry', default_value='',
                              description='Override timing reports: true or false.'),
        OpaqueFunction(function=_launch),
    ])
