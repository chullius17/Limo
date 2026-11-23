"""Launch a shadow-free circuit GUI with live Gazebo obstacle positions."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('custom_start')
    world = os.path.abspath(os.path.expanduser(value('world')))
    actions = []
    if value('start_gazebo').lower() not in ('true', 'false'):
        raise ValueError('start_gazebo must be true or false')
    if value('start_gazebo').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'limo_circuit.launch.py')),
            launch_arguments={'world': world, 'use_sim_time': 'true'}.items()))
    actions.append(Node(
        package='user_package', executable='circuit_experiment', output='screen',
        arguments=['--world', world, '--model-states-topic', value('model_states_topic'),
                   '--link-states-topic', value('link_states_topic'),
                   '--robot-link', value('robot_link'),
                   '--refresh-hz', value('refresh_hz')],
        parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    """Use an existing Gazebo by default; optional startup uses the same world."""
    share = get_package_share_directory('custom_start')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            share, 'worlds', 'limo_obstacle_world.world')),
        DeclareLaunchArgument('start_gazebo', default_value='false'),
        DeclareLaunchArgument('model_states_topic', default_value='/gazebo/model_states'),
        DeclareLaunchArgument('link_states_topic', default_value='/gazebo/link_states'),
        DeclareLaunchArgument('robot_link', default_value='limo::base_link'),
        DeclareLaunchArgument('refresh_hz', default_value='20.0'),
        OpaqueFunction(function=_launch),
    ])
