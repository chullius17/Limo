"""Complete mixed-AMCL example: CV, exported static maps, lifecycle and RViz."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError('Expected true or false, got {!r}'.format(value))
    return value.lower() == 'true'


def _parameters(path):
    with open(path, encoding='utf-8') as stream:
        content = yaml.safe_load(stream)
    if not isinstance(content, dict):
        raise ValueError('Expected a ROS parameter file: ' + str(path))
    result = {}
    for name in ('/**', 'amcl', '/amcl'):
        section = content.get(name, {})
        parameters = section.get('ros__parameters', {}) if isinstance(section, dict) else None
        if not isinstance(parameters, dict):
            raise ValueError('Expected amcl.ros__parameters in ' + str(path))
        result.update(parameters)
    if not result:
        raise ValueError('No amcl parameters found in ' + str(path))
    return result


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    profile = value('profile')
    if profile not in ('sim', 'real'):
        raise ValueError('profile must be sim or real')
    use_sim_time = _boolean(value('use_sim_time')) if value('use_sim_time') else profile == 'sim'
    clock = {'use_sim_time': use_sim_time}
    share = Path(get_package_share_directory('limo_amcl'))
    filename = Path(value('config_file')).expanduser() if value('config_file') else (
        share / 'config' / ('amcl_real.yaml' if profile == 'real' else 'amcl.yaml'))
    parameters = _parameters(filename)
    parameters.update(clock)
    for name, kind in (('cv_enabled', bool), ('laser_weight_factor', float), ('cv_weight_factor', float),
                       ('map_topic', str), ('cv_map_topic', str), ('scan_topic', str), ('cv_cloud_topic', str)):
        if value(name):
            parameters[name] = _boolean(value(name)) if kind is bool else kind(value(name))
    global_frame = parameters.get('global_frame_id', 'map')
    laser_topic = parameters.get('map_topic', '/limo/map_package/online/maps/laser_map')
    cv_topic = parameters.get('cv_map_topic', '/limo/map_package/online/maps/cv_obstacle')
    complete_topic = value('complete_map_topic')
    actions, managed = [], []

    if _boolean(value('start_maps')):
        directory = Path(value('map_directory')).expanduser().resolve()
        basename = value('map_name')
        if not basename or Path(basename).name != basename or basename in ('.', '..'):
            raise ValueError('map_name must be a nonempty filename stem')
        specs = (
            ('limo_amcl_complete_map_server', 'map_yaml', '_complete.yaml', complete_topic),
            ('limo_amcl_laser_map_server', 'laser_map_yaml', '_laser.yaml', laser_topic),
            ('limo_amcl_cv_map_server', 'cv_map_yaml', '_cv_obstacle.yaml', cv_topic),
        )
        selected = []
        for name, argument, suffix, topic in specs:
            path = Path(value(argument)).expanduser().resolve() if value(argument) else directory / (basename + suffix)
            if not path.is_file():
                raise ValueError('Map file does not exist: {}. Save maps with limo_map, or set '
                                 'map_directory/map_name or {}.'.format(path, argument))
            selected.append((name, path, topic))
        topics = [topic for name, path, topic in selected]
        if len(set(topics)) != len(topics):
            raise ValueError('Complete, laser and CV maps must use distinct topics')
        for name, path, topic in selected:
            actions.append(Node(
                package='nav2_map_server', executable='map_server', name=name,
                output='screen', parameters=[{
                    'yaml_filename': str(path), 'frame_id': global_frame, **clock,
                }], remappings=[('map', topic), ('map_metadata', topic + '_metadata')],
            ))
            managed.append(name)

    if _boolean(value('start_cv')):
        cv_share = get_package_share_directory('limo_cloud_utilities')
        # Child config_file/profile arguments cannot overwrite localization settings.
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cv_share, 'launch', profile + '.launch.py')),
            launch_arguments={
                'profile': profile, 'mode': 'backend', 'start_rviz': 'false',
                'config_file': os.path.expanduser(value('cv_config_file')),
                'use_sim_time': str(use_sim_time).lower(), 'enable_telemetry': 'false',
            }.items(),
        )]))

    if _boolean(value('start_amcl')):
        # Generated parameters apply to AMCL and its Foxy TF/message-filter helper.
        typed = {key: ParameterValue(setting, value_type=str) if isinstance(setting, str) else setting
                 for key, setting in parameters.items()}
        actions.append(Node(
            package='limo_amcl', executable='limo_amcl_node', name='amcl',
            output='screen', parameters=[typed],
        ))
        managed.append('amcl')
    if managed and _boolean(value('start_lifecycle_manager')):
        # Maps configure/activate first, then AMCL; no SLAM is started here.
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_limo_amcl', output='screen',
            parameters=[{'autostart': True, 'node_names': managed, **clock}],
        ))

    if _boolean(value('start_rviz')):
        from limo_rviz.overlay import lane_overlay_bridge
        rviz_file = Path(value('rviz_config')).expanduser() if value('rviz_config') else share / 'config' / (
            'amcl_real.rviz' if profile == 'real' else 'amcl.rviz')
        bridge, remappings = lane_overlay_bridge(str(rviz_file), 'limo_amcl_rviz')
        if bridge:
            actions.append(Node(**bridge, parameters=[clock]))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='limo_amcl_rviz', output='screen',
            arguments=['-d', str(rviz_file), '-f', global_frame], parameters=[clock],
            remappings=remappings + [
                ('/map', complete_topic),
                ('/limo/map_package/online/maps/laser_map', laser_topic),
                ('/limo/map_package/online/maps/cv_obstacle', cv_topic),
                ('/scan', parameters.get('scan_topic', '/scan')),
                ('/limo/cv_package/visual_ptcld/points', parameters.get(
                    'cv_cloud_topic', '/limo/cv_package/visual_ptcld/points')),
            ], additional_env={'__GLVND_DISALLOW_PATCHING': os.environ.get('__GLVND_DISALLOW_PATCHING', '1')},
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim', description='sim or real CV/AMCL settings.'),
        DeclareLaunchArgument('use_sim_time', default_value='', description='Empty follows profile.'),
        DeclareLaunchArgument('config_file', default_value='', description='AMCL ROS params override.'),
        DeclareLaunchArgument('map_directory', default_value='ros2_maps/semantic',
                              description='limo_map export directory; relative to the launch working directory.'),
        DeclareLaunchArgument('map_name', default_value='limo_map', description='Export filename stem.'),
        DeclareLaunchArgument('map_yaml', default_value='', description='Override complete map file.'),
        DeclareLaunchArgument('laser_map_yaml', default_value='', description='Override laser map file.'),
        DeclareLaunchArgument('cv_map_yaml', default_value='', description='Override semantic CV map file.'),
        DeclareLaunchArgument('complete_map_topic', default_value='/map'),
        DeclareLaunchArgument('start_maps', default_value='true'),
        DeclareLaunchArgument('start_cv', default_value='true'),
        DeclareLaunchArgument('start_amcl', default_value='true'),
        DeclareLaunchArgument('start_lifecycle_manager', default_value='true',
                              description='Disable if an existing Nav2 manager manages these nodes.'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('cv_config_file', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('map_topic', default_value='', description='Override laser map input topic.'),
        DeclareLaunchArgument('cv_map_topic', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('cv_cloud_topic', default_value=''),
        DeclareLaunchArgument('cv_enabled', default_value='', description='Empty uses the AMCL profile.'),
        DeclareLaunchArgument('laser_weight_factor', default_value=''),
        DeclareLaunchArgument('cv_weight_factor', default_value=''),
        OpaqueFunction(function=_launch),
    ])
