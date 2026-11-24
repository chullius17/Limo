"""Open a standalone MPC inspector with the selected controller's geometry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
import yaml


def _launch_gui(context):
    controller_file = os.path.expanduser(
        LaunchConfiguration('controller_params_file').perform(context))
    with open(controller_file, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    controller = config.get('controller_server', {}).get('ros__parameters', {})
    guidance = controller.get('FollowPath', {}).get('MppiPath', {}).get('ObstacleGuidance', {})
    tube = config.get('local_costmap', {}).get('local_costmap', {}).get(
        'ros__parameters', {}).get('path_distance_layer', {})
    executor = config.get('path_executor', {}).get('ros__parameters', {})
    params = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool).evaluate(context),
        'goal_circle_radius': float(executor.get('goal_circle_radius', 0.20)),
        'trigger_margin': float(guidance.get('trigger_margin', 0.05)),
        'trigger_front_extension': float(guidance.get('trigger_front_extension', 0.0)),
    }
    if 'tube_radius' in tube:
        params['tube_radius'] = float(tube['tube_radius'])
    preview = RewrittenYaml(
        source_file=LaunchConfiguration('mpc_preview_params_file'), root_key='',
        param_rewrites={key: str(value).lower() for key, value in params.items()},
        convert_types=True)
    return [Node(
        package='limo_controller', executable='mpc_gui', name='mpc_preview',
        output='screen', parameters=[preview, params])]


def generate_launch_description():
    """Select real/sim clock and controller profile without starting control."""
    share = get_package_share_directory('limo_controller')
    model = LaunchConfiguration('robot_model')
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='real', choices=['real', 'sim']),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression([
            "'", model, "' == 'sim'"])),
        DeclareLaunchArgument('controller_params_file', default_value=[
            os.path.join(share, 'config', 'control_'), model, '.yaml']),
        DeclareLaunchArgument('mpc_preview_params_file', default_value=os.path.join(
            share, 'config', 'mpc_preview_sim.yaml')),
        OpaqueFunction(function=_launch_gui),
    ])
