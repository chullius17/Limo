"""Standalone projector; the detector/depth producer are launched separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('limo_cloud'), 'config', 'cloud.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='limo_cloud', executable='limo_cloud_node', name='limo_cloud',
            output='screen', parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
