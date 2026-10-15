"""Launch Gazebo with the obstacle world and the Ackermann LIMO robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    custom_start_share = get_package_share_directory('custom_start')
    default_config = os.path.join(
        custom_start_share, 'config', 'limo_obstacles.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='YAML profile for the Gazebo obstacle world.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    custom_start_share, 'launch', 'limo_circuit.launch.py')),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
            }.items(),
        ),
    ])
