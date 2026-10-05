"""Compose mapping, planning, and control for a LIMO application profile."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, arguments=None):
    """Include one subsystem without duplicating its internal parameters."""
    package_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', launch_file)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def _boolean(value):
    """Parse a strict ROS launch boolean."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    raise ValueError('Expected true or false, got {!r}'.format(value))


def _optional_boolean(context, name, default):
    """Use the default when a boolean launch argument is empty."""
    value = LaunchConfiguration(name).perform(context)
    return default if value == '' else _boolean(value)


def _launch_app(context):
    """Resolve overrides and compose the selected application's subsystems."""
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('sim', 'real'):
        raise ValueError('profile must be sim or real')

    simulation = profile == 'sim'
    use_sim_time = str(_boolean(
        LaunchConfiguration('use_sim_time').perform(context))).lower()
    start_gui = _optional_boolean(
        context, 'start_control_gui', simulation)

    if not simulation and start_gui:
        raise ValueError(
            'The real application is headless; use desktop_app.launch.py '
            'for the control GUI.')

    map_topic = LaunchConfiguration('map_topic').perform(context)
    online_map_arguments = {'use_sim_time': use_sim_time}
    online_map_file = 'online_map_{}.launch.py'.format(profile)
    mapping_config = LaunchConfiguration('mapping_config').perform(context)
    if mapping_config:
        online_map_file = 'online_map.launch.py'
        online_map_arguments['config_file'] = os.path.expanduser(mapping_config)
    if not simulation:
        online_map_arguments.update(
            mode='backend', start_rviz='false')
    start_cv = LaunchConfiguration('start_cv').perform(context)
    if start_cv:
        online_map_arguments['start_cv'] = str(_boolean(start_cv)).lower()
    cv_config = LaunchConfiguration('cv_config').perform(context)
    if cv_config:
        online_map_arguments['cv_config'] = cv_config

    trajectory_arguments = {
        'robot_model': profile,
        'map_topic': map_topic,
        'use_sim_time': use_sim_time,
    }
    controller_arguments = {
        'robot_model': profile,
        'use_sim_time': use_sim_time,
        'autostart': 'true',
        'start_gui': str(start_gui).lower(),
        'start_mpc_preview': str(_optional_boolean(
            context, 'start_mpc_preview', True)).lower(),
    }
    preview_file = LaunchConfiguration('mpc_preview_params_file').perform(context)
    # Forward a concrete default: the parent's empty value would otherwise
    # suppress the controller launch's default in the shared launch context.
    controller_arguments['mpc_preview_params_file'] = (
        os.path.expanduser(preview_file) if preview_file else os.path.join(
            get_package_share_directory('limo_controller'),
            'config', 'mpc_preview_sim.yaml'))
    for name, arguments, package, prefix in (
            ('planner_params_file', trajectory_arguments, 'traj_package', 'traj'),
            ('controller_params_file', controller_arguments, 'limo_controller', 'control')):
        params_file = LaunchConfiguration(name).perform(context)
        # Explicit filenames replace the app's empty launch configuration in
        # each child; an inherited empty value would suppress its YAML default.
        arguments[name] = os.path.expanduser(params_file) if params_file else os.path.join(
            get_package_share_directory(package), 'config', prefix + '_' + profile + '.yaml')

    return [
        _include('online_map_package', online_map_file, online_map_arguments),
        _include('traj_package', 'trajectory.launch.py', trajectory_arguments),
        _include('limo_controller', 'control.launch.py', controller_arguments),
    ]


def generate_launch_description():
    """Create the application; default to the sim profile using the wall clock."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='sim', choices=['sim', 'real'],
            description='Application profile for mapping, planning, and control.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock instead of the wall clock.'),
        DeclareLaunchArgument(
            'mapping_config', default_value='',
            description='Mapping YAML override; empty uses the real/sim profile.'),
        DeclareLaunchArgument(
            'planner_params_file', default_value='',
            description='Planner/bridge YAML override; empty follows the app profile.'),
        DeclareLaunchArgument(
            'controller_params_file', default_value='',
            description='Controller YAML override; empty follows the app profile.'),
        DeclareLaunchArgument(
            'start_mpc_preview', default_value='',
            description='Allow the MPC preview; its enabled flag comes from the controller YAML.'),
        DeclareLaunchArgument(
            'mpc_preview_params_file', default_value='',
            description='Optional MPC preview rendering YAML override.'),
        DeclareLaunchArgument(
            'map_topic', default_value='/map',
            description='Global OccupancyGrid consumed by traj_package.'),
        DeclareLaunchArgument(
            'start_cv', default_value='',
            description=(
                'Override the mapping profile CV setting; empty uses launch.start_cv '
                'from YAML. Set false when CV is already running.')),
        DeclareLaunchArgument(
            'cv_config', default_value='',
            description='Optional CV YAML override; empty uses the real/sim mapping profile.'),
        DeclareLaunchArgument(
            'start_control_gui', default_value='',
            description=(
                'Simulation GUI override; the real GUI runs in desktop_app.launch.py.')),
        OpaqueFunction(function=_launch_app),
    ])
