"""Open the AMCL experiment recorder alongside an already running localization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    settings = {
        'use_sim_time': ('true', bool, 'Use the simulation clock.'),
        'pose_topic': ('/amcl_pose', str, 'AMCL pose and full covariance.'),
        'diagnostics_topic': ('/amcl/cv_diagnostics', str, 'Actual CV fusion counts.'),
        'laser_diagnostics_topic': ('/amcl/laser_diagnostics', str, 'Actual laser model counts.'),
        'history_seconds': ('120.0', float, 'Visible history; CSVs keep the full session.'),
        'max_samples': ('20000', int, 'Maximum visible samples per stream.'),
        'refresh_hz': ('5.0', float, 'Graph refresh frequency.'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value, description=description)
          for key, (value, _, description) in settings.items()],
        Node(
            package='online_map_package', executable='amcl_experiment',
            name='amcl_experiment', output='screen', parameters=[{
                key: ParameterValue(LaunchConfiguration(key), value_type=kind)
                for key, (_, kind, _) in settings.items()
            }]),
    ])
