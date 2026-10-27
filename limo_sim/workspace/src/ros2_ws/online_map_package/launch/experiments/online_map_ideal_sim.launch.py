"""Localize with CV and laser against ideal circuit and Gazebo wall maps."""

import os
from pathlib import Path

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

from limo_rviz.overlay import lane_overlay_bridge


def _default_map(share, filename='ideal_map.yaml'):
    relative = Path(
        'src/ros2_ws/offline_map_package/experiments/sim/ideal_map') / filename
    for start in (Path(__file__).resolve(), Path(share).resolve()):
        for root in start.parents:
            candidate = root / relative
            if candidate.is_file():
                return str(candidate)
    return ''


def _flag(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError('{} must be true or false'.format(name))
    return value == 'true'


def _launch_ideal(context):
    map_yaml = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('map_yaml').perform(context)))
    if not os.path.isfile(map_yaml):
        raise ValueError(
            'Ideal map YAML not found: {}. Set map_yaml:=/path/to/ideal_map.yaml'
            .format(map_yaml))
    laser_map_yaml = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('laser_map_yaml').perform(context)))
    if not os.path.isfile(laser_map_yaml):
        raise ValueError(
            'Ideal laser map YAML not found: {}. Set laser_map_yaml:=/path/to/laser.yaml'
            .format(laser_map_yaml))
    share = get_package_share_directory('online_map_package')
    with open(os.path.join(share, 'config', 'mapping_sim.yaml'),
              encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    clock = {'use_sim_time': _flag(context, 'use_sim_time')}
    amcl = dict(profile['amcl'])
    # Laser sees physical walls; CV compares only the circuit markings.
    amcl.update(
        **clock, map_topic=profile['map_servers']['laser_topic'],
        cv_map_topic='/map', cv_enabled=True)
    arguments = {
        name: str(value).lower() if isinstance(value, bool) else str(value)
        for name, value in amcl.items()
    }
    actions = [
        Node(
            package='nav2_map_server', executable='map_server',
            name='ideal_laser_map_server', output='screen', parameters=[{
                **clock, 'yaml_filename': laser_map_yaml,
                'frame_id': amcl['global_frame_id'],
            }], remappings=[
                ('map', amcl['map_topic']),
                ('map_metadata', amcl['map_topic'] + '_metadata')]),
        Node(
            package='nav2_map_server', executable='map_server',
            name='ideal_map_server', output='screen', parameters=[{
                **clock, 'yaml_filename': map_yaml,
                'frame_id': amcl['global_frame_id'],
            }], remappings=[('map', '/map'), ('map_metadata', '/map_metadata')]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_ideal_map', output='screen', parameters=[{
                **clock, 'autostart': True,
                'node_names': ['ideal_map_server', 'ideal_laser_map_server'],
            }]),
    ]
    if _flag(context, 'start_cv'):
        cv_share = get_package_share_directory('cv_package')
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                cv_share, 'launch', 'cv.launch.py')),
            launch_arguments={
                'config_file': os.path.join(
                    cv_share, 'config', profile['launch']['cv_config']),
                'mode': 'backend',
                'use_sim_time': str(clock['use_sim_time']).lower(),
                'visual_ptcld_enable_telemetry': 'false',
            }.items(),
        )]))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('limo_rviz'), 'launch', 'amcl.launch.py')),
        launch_arguments=arguments.items(),
    ))
    if _flag(context, 'start_monitor'):
        actions.append(Node(
            package='online_map_package', executable='amcl_experiment',
            name='amcl_experiment', output='screen', parameters=[clock]))
    if _flag(context, 'start_rviz'):
        rviz_config = os.path.join(
            get_package_share_directory('limo_rviz'), 'config',
            profile['launch']['rviz_config'])
        bridge, overlay_remappings = lane_overlay_bridge(rviz_config, 'online_rviz')
        if bridge:
            actions.append(Node(**bridge, parameters=[clock]))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config, '-f', amcl['global_frame_id']],
            remappings=[
                ('/limo/map_package/online/maps/cv_obstacle', '/map'),
            ] + overlay_remappings, parameters=[clock]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('online_map_package')
    return LaunchDescription([
        DeclareLaunchArgument(
            'laser_map_yaml', default_value=_default_map(share, 'ideal_laser_map.yaml'),
            description='Gazebo wall occupancy YAML used for AMCL laser likelihood.'),
        DeclareLaunchArgument(
            'map_yaml', default_value=_default_map(share),
            description='Ideal circuit map YAML; automatically located in the workspace.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo or rosbag clock.'),
        DeclareLaunchArgument(
            'start_cv', default_value='true',
            description='Start CV; set false when replaying recorded CV clouds.'),
        DeclareLaunchArgument(
            'start_rviz', default_value='true',
            description='Open the online localization RViz configuration.'),
        DeclareLaunchArgument(
            'start_monitor', default_value='true',
            description='Open the AMCL experiment graphs and record the session.'),
        OpaqueFunction(function=_launch_ideal),
    ])
