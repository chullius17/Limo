"""Launch the physical LIMO's visualization and control clients on the PC."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    """Parse a strict ROS launch boolean."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    raise ValueError('Expected true or false, got {!r}'.format(value))


def _optional_boolean(context, name, default):
    """Use the default when a boolean launch argument is empty."""
    value = LaunchConfiguration(name).perform(context)
    return default if value == '' else _boolean(value)


def _desktop_include(package, filename, arguments):
    """Scope each client so its launch settings cannot affect another client."""
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items(),
    )])


def _launch_desktop(context):
    """Compose only viewers and the remote control service client."""
    mapping_config = os.path.expanduser(
        LaunchConfiguration('mapping_config').perform(context))
    with open(mapping_config, encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or not isinstance(profile.get('launch'), dict):
        raise ValueError(
            '{}: missing YAML mapping \'launch\''.format(mapping_config))
    settings = profile['launch']
    start_cv_rviz = _optional_boolean(
        context, 'start_cv_rviz', _boolean(settings.get('start_cv', False)))
    start_mapping_rviz = _boolean(
        LaunchConfiguration('start_mapping_rviz').perform(context))
    start_control_gui = _boolean(
        LaunchConfiguration('start_control_gui').perform(context))

    actions = []
    if start_mapping_rviz:
        actions.append(_desktop_include(
            'online_map_package', 'online_map.launch.py', {
                'config_file': mapping_config,
                'mode': 'desktop',
                'start_rviz': 'true',
                'use_sim_time': 'false',
            }))
    if start_cv_rviz:
        cv_config = LaunchConfiguration('cv_config').perform(context)
        cv_config = cv_config or settings.get('cv_config', 'cv_real.yaml')
        if not isinstance(cv_config, str) or not cv_config:
            raise ValueError('launch.cv_config must be a non-empty file name')
        cv_config = os.path.expanduser(cv_config)
        if not os.path.isabs(cv_config):
            cv_config = os.path.join(
                get_package_share_directory('cv_package'), 'config', cv_config)
        actions.append(_desktop_include('cv_package', 'cv.launch.py', {
            'config_file': cv_config,
            'mode': 'desktop',
            'start_rviz': 'true',
            'use_sim_time': 'false',
        }))
    if start_control_gui:
        actions.append(Node(
            package='limo_controller', executable='control_gui',
            name='control_gui', output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    return actions


def generate_launch_description():
    """Create the desktop clients for the physical robot application."""
    mapping_config = os.path.join(
        get_package_share_directory('online_map_package'),
        'config', 'mapping_real.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mapping_config', default_value=mapping_config,
            description='Mapping YAML also used by the robot application.'),
        DeclareLaunchArgument(
            'cv_config', default_value='',
            description='CV YAML override; empty uses launch.cv_config from mapping YAML.'),
        DeclareLaunchArgument(
            'start_mapping_rviz', default_value='true',
            description='Open mapping, planning and local-costmap RViz on the PC.'),
        DeclareLaunchArgument(
            'start_cv_rviz', default_value='',
            description='Open CV RViz; empty follows launch.start_cv from mapping YAML.'),
        DeclareLaunchArgument(
            'start_control_gui', default_value='true',
            description='Open the remote start/pause/resume/abort control window.'),
        OpaqueFunction(function=_launch_desktop),
    ])
