"""Start the C++ Qt map-save GUI, optionally targeting a remote mapper."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gui_node(context):
    arguments = ['--ros-args', '-p', 'limo_map_save_gui:use_sim_time:=' +
                 LaunchConfiguration('use_sim_time').perform(context)]
    for name in ('save_service', 'status_topic'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            # Node-qualified CLI overrides also take precedence over a named
            # YAML section on Foxy. Empty arguments defer to config/C++ defaults.
            arguments += ['-p', 'limo_map_save_gui:{}:={}'.format(name, value)]
    return [
        Node(
            package='limo_map', executable='map_save_gui', name='limo_map_save_gui',
            output='screen', arguments=arguments,
            parameters=[LaunchConfiguration('config_file')],
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('limo_map'), 'config', 'gui.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('save_service', default_value=''),
        DeclareLaunchArgument('status_topic', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_gui_node),
    ])
