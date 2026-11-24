"""Open only the physical robot's AMCL history viewer on the desktop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _launch_viewer(context):
    """Use the complete map topic from the same profile as the robot backend."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    with open(os.path.expanduser(value('mapping_config')), encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    return [Node(
        package='user_package', executable='amcl_visualization',
        name='amcl_visualization', output='screen', parameters=[{
            'use_sim_time': False,
            'map_topic': value('map_topic') or profile['map_servers']['complete_topic'],
            'pose_topic': value('pose_topic'),
            'max_samples': int(value('max_samples')),
            'sample_period': float(value('sample_period')),
            'refresh_hz': float(value('refresh_hz')),
        }],
    )]


def generate_launch_description():
    """Expose desktop settings without launching robot processing nodes."""
    return LaunchDescription([
        DeclareLaunchArgument('mapping_config', default_value=os.path.join(
            get_package_share_directory('online_map_package'), 'config', 'mapping_real.yaml')),
        DeclareLaunchArgument('map_topic', default_value='',
                              description='Empty uses the complete map in mapping_config.'),
        DeclareLaunchArgument('pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('max_samples', default_value='5000'),
        DeclareLaunchArgument('sample_period', default_value='0.25',
                              description='Minimum AMCL time between retained circles, seconds.'),
        DeclareLaunchArgument('refresh_hz', default_value='4.0'),
        OpaqueFunction(function=_launch_viewer),
    ])
