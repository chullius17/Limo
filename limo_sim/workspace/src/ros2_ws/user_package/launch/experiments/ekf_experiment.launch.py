"""Launch the live EKF experiment for simulation or the physical LIMO."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    """Parse a strict ROS launch boolean."""
    if value.lower() not in ('true', 'false'):
        raise ValueError('Expected true or false, got {!r}'.format(value))
    return value.lower() == 'true'


def _launch_experiment(context):
    """Resolve the experiment profile and launch the viewer."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile = value('profile')
    if profile not in ('real', 'sim'):
        raise ValueError('profile must be real or sim')
    sim_time = _boolean(value('use_sim_time')) if value('use_sim_time') else profile == 'sim'
    return [Node(
        package='user_package', executable='ekf_experiment', name='ekf_experiment',
        output='screen', parameters=[{
            'use_sim_time': sim_time,
            'raw_topic': value('raw_topic'),
            'filtered_topic': value('filtered_topic'),
            'history_seconds': float(value('history_seconds')),
            'max_samples': int(value('max_samples')),
            'max_sync_gap': float(value('max_sync_gap')),
            'refresh_hz': float(value('refresh_hz')),
        }])]


def generate_launch_description():
    """Use one viewer; profile selects the default clock."""
    options = [
        ('profile', 'real', 'real or sim; selects the default ROS clock.'),
        ('use_sim_time', '', 'Optional clock override for an existing robot or rosbag.'),
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
