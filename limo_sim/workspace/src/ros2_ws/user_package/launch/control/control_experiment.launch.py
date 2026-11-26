"""Open the read-only control experiment GUI with the selected app settings."""

import ast
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _launch_experiment(context):
    """Use the same physical footprint and observations as the real controller."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def read(name):
        with open(os.path.expanduser(value(name)), encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    mapping, controller = read('mapping_config'), read('controller_params_file')
    clock = value('use_sim_time').lower()
    if clock not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    costmap = controller['local_costmap']['local_costmap']['ros__parameters']
    footprint = ast.literal_eval(costmap['footprint'])
    padding = float(costmap.get('footprint_padding', 0.0))
    if not math.isfinite(padding) or padding < 0:
        raise ValueError('Invalid controller footprint padding')
    polygon = [float(coordinate) + (padding if coordinate > 0 else
               -padding if coordinate < 0 else 0.0)
               for point in footprint for coordinate in point]
    executor = controller.get('path_executor', {}).get('ros__parameters', {})
    follow = controller['controller_server']['ros__parameters']['FollowPath']
    scale = max(abs(float(follow.get('min_vel_x', 0.0))), float(follow['max_vel_x']))
    return [Node(
        package='user_package', executable='control_experiment',
        name='control_experiment', output='screen', parameters=[{
            'use_sim_time': clock == 'true',
            'map_topic': value('map_topic') or mapping['map_servers']['complete_topic'],
            'path_topic': value('path_topic') or executor.get('path_topic', '/limo/planning/path'),
            'goal_topic': value('goal_topic'),
            'scan_topic': value('scan_topic') or costmap.get('obstacle_layer', {}).get(
                'scan', {}).get('topic', '/scan'),
            'debug_topic': value('debug_topic') or follow.get('MPC', {}).get(
                'Debug', {}).get('topic', '/limo/control/mpc_debug'),
            'odom_topic': value('odom_topic') or executor.get(
                'success_odom_topic', '/odometry/filtered'),
            'base_frame': executor.get('robot_base_frame', 'base_link'),
            'footprint': polygon,
            'speed_scale_max': float(value('speed_scale_max') or scale),
            'sample_period': float(value('sample_period')),
            'max_samples': int(value('max_samples')),
        }],
    )]


def generate_launch_description():
    """Start only the desktop observer; the real LIMO application runs separately."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mapping_config', default_value=os.path.join(
            get_package_share_directory('online_map_package'), 'config', 'mapping_real.yaml')),
        DeclareLaunchArgument('controller_params_file', default_value=os.path.join(
            get_package_share_directory('limo_controller'), 'config', 'control_real.yaml')),
        DeclareLaunchArgument('map_topic', default_value=''),
        DeclareLaunchArgument('path_topic', default_value=''),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('debug_topic', default_value=''),
        DeclareLaunchArgument('odom_topic', default_value=''),
        DeclareLaunchArgument(
            'speed_scale_max', default_value='',
            description='Maximum speed color in m/s; empty uses controller limit.'),
        DeclareLaunchArgument('sample_period', default_value='0.1'),
        DeclareLaunchArgument('max_samples', default_value='50000'),
        OpaqueFunction(function=_launch_experiment),
    ])
