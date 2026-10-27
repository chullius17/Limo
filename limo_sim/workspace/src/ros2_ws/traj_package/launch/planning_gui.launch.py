"""Open the planning viewer on the same costmap topic as the goal bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _launch_gui(context):
    topic = LaunchConfiguration('costmap_topic').perform(context)
    path = os.path.expanduser(LaunchConfiguration('planner_params_file').perform(context))
    config = {}
    if not topic or os.path.isfile(path):
        with open(path, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
    bridge = config.get('rviz_goal_bridge', {}).get('ros__parameters', {})
    costmap = config.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
    if not topic:
        topic = bridge.get('costmap_topic', '/global_costmap/costmap')
    params = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'costmap_topic': topic,
        'footprint_topic': (LaunchConfiguration('footprint_topic').perform(context)
                            or topic.rsplit('/', 1)[0] + '/published_footprint'),
        'robot_base_frame': costmap.get(
            'robot_base_frame', bridge.get('robot_base_frame', 'base_link')),
        'footprint_padding': float(costmap.get('footprint_padding', 0.0)),
        'goal_topic': bridge.get('goal_topic', '/goal_pose'),
        'adjusted_goal_topic': bridge.get('adjusted_goal_topic', '/adjusted_goal_pose'),
        'adjusted_start_topic': bridge.get('adjusted_start_topic', '/adjusted_start_pose'),
        'planning_log_topic': bridge.get('planning_log_topic', '/limo/planning/log'),
        'image_topic': LaunchConfiguration('image_topic'),
        'image_publish_rate': ParameterValue(
            LaunchConfiguration('image_publish_rate'), value_type=float),
    }
    if 'footprint' in costmap:
        footprint = costmap['footprint']
        params['footprint'] = footprint if isinstance(footprint, str) else repr(footprint)
    return [Node(
        package='traj_package', executable='planning_gui', name='planning_gui',
        output='screen', parameters=[params])]


def generate_launch_description():
    """Expose clock, planner configuration and an optional topic override."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('planner_params_file', default_value=os.path.join(
            get_package_share_directory('traj_package'), 'config', 'traj_sim.yaml')),
        DeclareLaunchArgument(
            'costmap_topic', default_value='',
            description='Optional override; otherwise use the goal bridge costmap topic.'),
        DeclareLaunchArgument(
            'footprint_topic', default_value='',
            description='Published planner footprint; default follows the costmap namespace.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/limo/planning/preview/image/compressed',
            description='PNG CompressedImage output of the map and all overlays.'),
        DeclareLaunchArgument('image_publish_rate', default_value='5.0'),
        OpaqueFunction(function=_launch_gui),
    ])
