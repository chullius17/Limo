"""Launch the live EKF experiment for simulation or the physical LIMO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    """Parse a strict ROS launch boolean."""
    if value.lower() not in ('true', 'false'):
        raise ValueError('Expected true or false, got {!r}'.format(value))
    return value.lower() == 'true'


def _launch_experiment(context):
    """Resolve the experiment profile and optionally reuse the robot bringup."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile = value('profile')
    if profile not in ('real', 'sim'):
        raise ValueError('profile must be real or sim')
    sim_time = _boolean(value('use_sim_time')) if value('use_sim_time') else profile == 'sim'
    actions = []
    if _boolean(value('start_bringup')):
        if sim_time != (profile == 'sim'):
            raise ValueError('With start_bringup, use_sim_time must match profile')
        arguments = ({'use_sim_time': 'true'} if profile == 'sim' else
                     {'use_camera': 'false', 'open_rviz': 'false'})
        if value('bringup_config'):
            arguments['config_file'] = os.path.expanduser(value('bringup_config'))
        filename = 'limo_circuit.launch.py' if profile == 'sim' else 'limo_real.launch.py'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('custom_start'), 'launch', filename)),
            launch_arguments=arguments.items()))
    actions.append(Node(
        package='user_package', executable='ekf_experiment', name='ekf_experiment',
        output='screen', parameters=[{
            'use_sim_time': sim_time,
            'raw_topic': value('raw_topic'),
            'filtered_topic': value('filtered_topic'),
            'history_seconds': float(value('history_seconds')),
            'max_samples': int(value('max_samples')),
            'max_sync_gap': float(value('max_sync_gap')),
            'refresh_hz': float(value('refresh_hz')),
        }]))
    return actions


def generate_launch_description():
    """Use one viewer; profile selects the default clock and optional bringup."""
    options = [
        ('profile', 'real', 'real or sim; selects the default ROS clock.'),
        ('use_sim_time', '', 'Optional clock override for an existing robot or rosbag.'),
        ('start_bringup', 'false', 'Also start robot/Gazebo and EKF; use only once.'),
        ('bringup_config', '', 'Optional custom_start YAML for the selected bringup.'),
        ('raw_topic', '/odom', 'Original nav_msgs/Odometry topic.'),
        ('filtered_topic', '/odometry/filtered', 'EKF nav_msgs/Odometry topic.'),
        ('history_seconds', '120.0', 'Duration retained in plots and exports.'),
        ('max_samples', '20000', 'Maximum retained samples per topic.'),
        ('max_sync_gap', '0.2', 'Maximum interpolation interval in seconds.'),
        ('refresh_hz', '5.0', 'GUI redraw rate; acquisition keeps the source rate.'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in options
    ] + [OpaqueFunction(function=_launch_experiment)])
