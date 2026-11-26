"""Complete mapping example: CV, semantic cloud, SLAM, mapper and desktop tools."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError('Expected true or false, got {!r}'.format(value))
    return value.lower() == 'true'


def _launch_mapping(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    profile = value('profile')
    if profile not in ('sim', 'real'):
        raise ValueError('profile must be sim or real')
    use_sim_time = _boolean(value('use_sim_time')) if value('use_sim_time') else profile == 'sim'
    clock = str(use_sim_time).lower()
    share = get_package_share_directory('limo_map')
    mapper_file = os.path.expanduser(value('config_file')) or os.path.join(
        share, 'config', 'map_real.yaml' if profile == 'real' else 'map.yaml')
    with open(mapper_file, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError('Expected a ROS parameter mapping in ' + mapper_file)
    mapper = {}
    for section in ('/**', 'limo_map', '/limo_map'):
        parameters = document.get(section, {}).get('ros__parameters', {})
        if not isinstance(parameters, dict):
            raise ValueError('Expected ros__parameters mapping in ' + mapper_file)
        mapper.update(parameters)
    map_frame = mapper.get('map_frame', 'map')
    odom_frame = mapper.get('odom_frame', 'odom')
    prefix = mapper.get('output_prefix', '/limo/map_package/offline').rstrip('/')
    reference_topic = mapper.get('reference_map_topic', '/map')
    actions = []

    if _boolean(value('start_cv')):
        cv_share = get_package_share_directory('limo_cloud_utilities')
        # Scope child arguments such as config_file/profile so they cannot change
        # the mapper or GUI configuration in the parent launch context.
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cv_share, 'launch', profile + '.launch.py')),
            launch_arguments={
                'profile': profile,
                'mode': 'backend',
                'config_file': os.path.expanduser(value('cv_config_file')),
                'use_sim_time': clock,
                'start_rviz': 'false',
                'enable_telemetry': 'false',
            }.items(),
        )]))

    if _boolean(value('start_slam')):
        if mapper.get('pose_source', 'tf') != 'tf':
            raise ValueError('Set start_slam:=false when using an external Cartographer backend')
        slam_file = os.path.expanduser(value('slam_config_file')) or os.path.join(
            share, 'config', 'slam_real.yaml' if profile == 'real' else 'slam.yaml')
        # Explicit node-qualified overrides also win over named YAML on Foxy.
        arguments = ['--ros-args']
        for name, setting in {'use_sim_time': clock, 'map_frame': map_frame,
                              'odom_frame': odom_frame,
                              'resolution': mapper.get('resolution', 0.05)}.items():
            arguments += ['-p', 'slam_toolbox:{}:={}'.format(name, setting)]
        actions.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen', parameters=[slam_file],
            arguments=arguments, remappings=[('map', reference_topic)],
        ))

    actions.append(Node(
        package='limo_map', executable='limo_map_node', name='limo_map',
        output='screen', parameters=[mapper_file],
        arguments=['--ros-args', '-p', 'limo_map:use_sim_time:=' + clock],
    ))

    if _boolean(value('start_gui')):
        gui_file = os.path.expanduser(value('gui_config_file')) or os.path.join(
            share, 'config', 'gui.yaml')
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'map_save_gui.launch.py')),
            launch_arguments={
                'config_file': gui_file,
                'save_service': value('save_service') or prefix + '/map_saver/save_map',
                'status_topic': value('status_topic') or prefix + '/map_saver/status',
                'use_sim_time': clock,
            }.items(),
        )]))

    if _boolean(value('start_rviz')):
        from limo_rviz.overlay import lane_overlay_bridge
        rviz_file = os.path.expanduser(value('rviz_config'))
        if not rviz_file:
            rviz_file = os.path.join(get_package_share_directory('limo_rviz'), 'config',
                                     'offline_map_real.rviz' if profile == 'real' else 'offline_map.rviz')
        bridge, remappings = lane_overlay_bridge(rviz_file, 'limo_map_rviz')
        if bridge:
            actions.append(Node(**bridge, parameters=[{'use_sim_time': use_sim_time}]))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='limo_map_rviz', output='screen',
            arguments=['-d', rviz_file, '-f', map_frame],
            remappings=remappings + [
                ('/limo/map_package/offline/map/combined_grid', prefix + '/map/combined_grid'),
                ('/limo/cv_package/visual_ptcld/points', mapper.get(
                    'cloud_topic', '/limo/cv_package/visual_ptcld/points')),
            ], parameters=[{'use_sim_time': use_sim_time}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim',
                              description='sim or real; selects mapper, CV and viewer profiles.'),
        DeclareLaunchArgument('use_sim_time', default_value='',
                              description='Empty follows the profile: sim=true, real=false.'),
        DeclareLaunchArgument('start_cv', default_value='true',
                              description='Start CV with the limo_cloud_utilities cloud stage.'),
        DeclareLaunchArgument('start_slam', default_value='true',
                              description='Start SLAM Toolbox to provide /map and map->odom.'),
        DeclareLaunchArgument('start_gui', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='Mapper ROS params file; empty selects the profile.'),
        DeclareLaunchArgument('cv_config_file', default_value='',
                              description='CV pipeline YAML; empty selects cv_sim/cv_real.yaml.'),
        DeclareLaunchArgument('slam_config_file', default_value='',
                              description='SLAM ROS params file; empty selects sim/real tuning.'),
        DeclareLaunchArgument('gui_config_file', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz file path; empty selects the mapping viewer.'),
        DeclareLaunchArgument('save_service', default_value='',
                              description='Empty follows the mapper output_prefix.'),
        DeclareLaunchArgument('status_topic', default_value='',
                              description='Empty follows the mapper output_prefix.'),
        OpaqueFunction(function=_launch_mapping),
    ])
