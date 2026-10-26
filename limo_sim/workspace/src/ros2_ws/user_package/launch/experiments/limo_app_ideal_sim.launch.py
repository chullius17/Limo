"""Plan on the ideal circuit map and control using live simulation sensors."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_map(filename='ideal_map.yaml'):
    """Locate the ideal map in either the source or installed workspace."""
    relative = Path(
        'src/ros2_ws/offline_map_package/experiments/sim/ideal_map') / filename
    for start in (Path(__file__).resolve(), Path(
            get_package_share_directory('user_package')).resolve()):
        for root in start.parents:
            candidate = root / relative
            if candidate.is_file():
                return str(candidate)
    return ''


def _include(package, filename, arguments):
    """Keep subsystem launch arguments in separate scopes."""
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items())])


def _launch_experiment(context):
    """Reuse ideal localization and the standard simulation control pipeline."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        selected = value(name).lower()
        if selected not in ('true', 'false'):
            raise ValueError('{} must be true or false'.format(name))
        return selected

    map_yaml = os.path.abspath(os.path.expanduser(value('map_yaml')))
    if not os.path.isfile(map_yaml):
        raise ValueError(
            'Ideal map YAML not found: {}. Set map_yaml:=/path/to/ideal_map.yaml'
            .format(map_yaml))
    mapping_share = get_package_share_directory('online_map_package')
    with open(os.path.join(mapping_share, 'config', 'mapping_sim.yaml'),
              encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    preview = value('start_mpc_preview')
    if preview:
        preview = flag('start_mpc_preview')

    return [
        _include('online_map_package', 'experiments/online_map_ideal_sim.launch.py', {
            'map_yaml': map_yaml,
            'laser_map_yaml': os.path.expanduser(value('laser_map_yaml')),
            'use_sim_time': 'true',
            'start_cv': flag('start_cv'),
            'start_rviz': flag('start_rviz'),
            'start_monitor': 'false',
        }),
        # Ideal localization does not start the semantic grid. Preserve the
        # normal sensor source consumed by control_sim.yaml's StaticLayer.
        Node(
            package='online_map_package', executable='local_ctrl_map',
            name='local_ctrl_map', output='screen', parameters=[{
                **profile['local_ctrl_map'],
                'use_sim_time': True,
                'base_frame': profile['amcl']['base_frame_id'],
                'odometry_frame': profile['amcl']['odom_frame_id'],
            }]),
        _include('traj_package', 'trajectory.launch.py', {
            'robot_model': 'sim',
            'use_sim_time': 'true',
            'map_topic': '/map',
            'planner_params_file': os.path.expanduser(value('planner_params_file')),
        }),
        _include('limo_controller', 'control.launch.py', {
            'robot_model': 'sim',
            'use_sim_time': 'true',
            'autostart': 'true',
            'start_gui': flag('start_control_gui'),
            'controller_params_file': os.path.expanduser(value('controller_params_file')),
            'start_mpc_preview': preview,
            'mpc_preview_params_file': os.path.expanduser(value('mpc_preview_params_file')),
        }),
    ]


def generate_launch_description():
    """Compose ideal planning, wall/CV localization and live sensor control."""
    options = [
        ('map_yaml', _default_map(), 'Ideal circuit map YAML.'),
        ('laser_map_yaml', _default_map('ideal_laser_map.yaml'),
         'Gazebo perimeter wall map for laser localization.'),
        ('start_cv', 'true', 'Start CV; set false if it is already running.'),
        ('start_rviz', 'true', 'Open RViz for initial pose and goal selection.'),
        ('start_control_gui', 'true', 'Open the START/PAUSE control interface.'),
        ('start_mpc_preview', '', 'Optional true/false override for the MPC preview.'),
        ('planner_params_file', os.path.join(
            get_package_share_directory('traj_package'), 'config', 'traj_sim.yaml'),
         'Simulation planner/bridge YAML.'),
        ('controller_params_file', os.path.join(
            get_package_share_directory('limo_controller'), 'config', 'control_sim.yaml'),
         'Simulation controller/mux YAML; local costmap uses laser and CV.'),
        ('mpc_preview_params_file', os.path.join(
            get_package_share_directory('limo_controller'), 'config', 'mpc_preview_sim.yaml'),
         'MPC preview rendering YAML.'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in options
    ] + [OpaqueFunction(function=_launch_experiment)])
