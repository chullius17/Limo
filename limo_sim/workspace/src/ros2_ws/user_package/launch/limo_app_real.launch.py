"""Launch the complete LIMO application with the physical-robot map profile."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch mapping, planning, and control for the physical LIMO."""
    share = get_package_share_directory('user_package')
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_params_file', default_value=os.path.join(
                get_package_share_directory('traj_package'), 'config', 'traj_real.yaml'),
            description='Trajectory profile for the physical LIMO.'),
        DeclareLaunchArgument(
            'controller_params_file', default_value=os.path.join(
                get_package_share_directory('limo_controller'), 'config', 'control_real.yaml'),
            description='Controller and velocity-mux profile for the physical LIMO.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'limo_app.launch.py')),
            launch_arguments={
                'profile': 'real',
                'use_sim_time': 'false',
                'planner_params_file': LaunchConfiguration('planner_params_file'),
                'controller_params_file': LaunchConfiguration('controller_params_file'),
            }.items(),
        ),
    ])
