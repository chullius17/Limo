"""Standalone depth correction; camera drivers and the RGB detector run separately."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_depth(context):
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('sim', 'real'):
        raise ValueError('profile must be sim or real')
    clock = LaunchConfiguration('use_sim_time').perform(context).lower()
    if clock and clock not in ('true', 'false'):
        raise ValueError('use_sim_time must be true, false or empty')
    clock = clock or ('true' if profile == 'sim' else 'false')
    share = get_package_share_directory('limo_depth')
    config = os.path.expanduser(LaunchConfiguration('config_file').perform(context))
    config = config or os.path.join(share, 'config', profile + '.yaml')
    parameters = {}
    for path in (os.path.join(share, 'config', 'depth.yaml'), config):
        with open(path, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError('Expected a ROS parameter mapping in ' + path)
        matched = False
        for section in ('/**', 'limo_depth', '/limo_depth'):
            if section not in document:
                continue
            matched = True
            settings = document[section].get('ros__parameters', {})
            if not isinstance(settings, dict):
                raise ValueError('Expected ros__parameters mapping in ' + path)
            parameters.update(settings)
        if not matched:
            raise ValueError('Expected a limo_depth or /** parameter section in ' + path)
    # Merge into a single source: on Foxy, named YAML sections can take precedence
    # over CLI or wildcard overrides even when those overrides appear later.
    parameters['use_sim_time'] = clock == 'true'
    return [Node(
        package='limo_depth', executable='limo_depth_node', name='limo_depth',
        output='screen', parameters=[parameters],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim', description='sim or real'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='ROS params file; empty selects config/<profile>.yaml.'),
        DeclareLaunchArgument('use_sim_time', default_value='',
                              description='Empty follows profile: sim=true, real=false.'),
        OpaqueFunction(function=_launch_depth),
    ])
