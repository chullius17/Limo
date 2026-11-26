"""Standalone semantic mapper; input cloud, SLAM and TF are supplied upstream."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('limo_map'), 'config', 'map.yaml')
    share = get_package_share_directory('limo_map')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_gui', default_value='false'),
        DeclareLaunchArgument('gui_config_file', default_value=os.path.join(share, 'config', 'gui.yaml')),
        DeclareLaunchArgument('save_service', default_value=''),
        DeclareLaunchArgument('status_topic', default_value=''),
        Node(
            package='limo_map', executable='limo_map_node', name='limo_map',
            output='screen', parameters=[LaunchConfiguration('config_file')],
            arguments=['--ros-args', '-p', [
                'limo_map:use_sim_time:=', LaunchConfiguration('use_sim_time')]],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'map_save_gui.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_gui')),
            launch_arguments={
                'config_file': LaunchConfiguration('gui_config_file'),
                'save_service': LaunchConfiguration('save_service'),
                'status_topic': LaunchConfiguration('status_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
