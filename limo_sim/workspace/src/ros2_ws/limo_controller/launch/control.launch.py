# Copyright 2026 Giulio Cataldo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the Nav2 controller using a real or simulation YAML profile."""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def controller_node(context, configured_params, log_level):
    """Use the chosen YAML without injecting geometry over custom parameters."""
    profile = LaunchConfiguration('robot_model').perform(context)
    if profile not in ('real', 'sim'):
        raise ValueError('robot_model must be sim or real')
    return [Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', '/cmd_vel_autonomy')],
    )]


def mpc_preview_node(context, configured_params, preview_params):
    """Use the selected controller YAML to enable MPC telemetry and its preview."""
    with open(configured_params.perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    params = config['controller_server']['ros__parameters']
    enabled = params.get('FollowPath', {}).get('MPC', {}).get('Debug', {}).get('enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('FollowPath.MPC.Debug.enabled must be a YAML boolean')
    return [Node(
        package='limo_controller', executable='mpc_preview', name='mpc_preview',
        output='screen', parameters=[preview_params],
        condition=IfCondition(PythonExpression([
            str(enabled), " and '", LaunchConfiguration('start_mpc_preview'),
            "'.lower() == 'true'"])),
    )]


def control_gui_node(context, configured_params, preview_params):
    """Connect the GUI to the selected controller and renderer telemetry topics."""
    with open(configured_params.perform(context), encoding='utf-8') as stream:
        controller = yaml.safe_load(stream)
    with open(preview_params.perform(context), encoding='utf-8') as stream:
        preview = yaml.safe_load(stream)['mpc_preview']['ros__parameters']
    debug = controller['controller_server']['ros__parameters'].get(
        'FollowPath', {}).get('MPC', {}).get('Debug', {})
    return [Node(
        package='limo_controller', executable='control_gui', name='control_gui',
        output='screen', condition=IfCondition(LaunchConfiguration('start_gui')),
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'mpc_debug_topic': debug.get('topic', '/limo/control/mpc_debug'),
            'mpc_image_topic': preview.get(
                'image_topic', '/limo/control/mpc_preview/image/compressed'),
            'mpc_stale_timeout': preview.get('stale_timeout', 1.0),
        }],
    )]


def generate_launch_description():
    """Create the controller server and its lifecycle manager."""
    package_share = get_package_share_directory('limo_controller')
    robot_model = LaunchConfiguration('robot_model')
    default_params_file = [
        os.path.join(package_share, 'config', 'control_'), robot_model, '.yaml']
    profile_clock = PythonExpression(["'", robot_model, "' == 'sim'"])

    controller_params_file = LaunchConfiguration('controller_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    require_chassis_status = LaunchConfiguration('require_chassis_status')
    preview_params = RewrittenYaml(
        source_file=LaunchConfiguration('mpc_preview_params_file'),
        root_key='', param_rewrites={'use_sim_time': use_sim_time}, convert_types=True)

    configured_params = RewrittenYaml(
        source_file=controller_params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    controller_server = OpaqueFunction(
        function=controller_node,
        args=[configured_params, log_level],
    )

    cmd_vel_mux = Node(
        package='limo_controller',
        executable='cmd_vel_mux',
        name='twist_mux',
        output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_limo_controller',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': ['controller_server'],
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    path_executor = Node(
        package='limo_controller',
        executable='path_executor',
        name='path_executor',
        output='screen',
        parameters=[configured_params, {
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            # A custom YAML cannot accidentally disable the physical guard.
            'require_chassis_status': ParameterValue(
                require_chassis_status, value_type=bool),
        }],
    )

    control_gui = OpaqueFunction(
        function=control_gui_node, args=[configured_params, preview_params],
    )
    mpc_preview = OpaqueFunction(
        function=mpc_preview_node, args=[configured_params, preview_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model', default_value='real', choices=['real', 'sim'],
            description='Select control_real.yaml or control_sim.yaml.',
        ),
        DeclareLaunchArgument(
            'controller_params_file',
            default_value=default_params_file,
            description='Controller and velocity-mux YAML; default follows robot_model.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=profile_clock,
            description='Clock override; default is true for sim, false for real.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Configure and activate controller nodes automatically.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for controller processes.',
        ),
        DeclareLaunchArgument(
            'start_gui',
            default_value=profile_clock,
            description='Local GUI override; real uses desktop_app.launch.py by default.',
        ),
        DeclareLaunchArgument(
            'require_chassis_status',
            default_value=PythonExpression(["'", robot_model, "' == 'real'"]),
            description='Require healthy command-mode feedback before START.',
        ),
        DeclareLaunchArgument(
            'start_mpc_preview', default_value='false',
            description=(
                'Opt in to the image renderer when MPC.Debug.enabled is true; '
                'telemetry is independent.'),
        ),
        DeclareLaunchArgument(
            'mpc_preview_params_file',
            default_value=os.path.join(package_share, 'config', 'mpc_preview_sim.yaml'),
            description='MPC preview rendering and compression settings.',
        ),
        controller_server,
        cmd_vel_mux,
        # Foxy lifecycle_manager performs startup only once.  When the whole
        # application starts at the same time as mapping, planning and RViz,
        # Fast DDS discovery can expose controller_server after that attempt.
        # Give the server time to advertise its lifecycle services first.
        TimerAction(period=3.0, actions=[lifecycle_manager]),
        path_executor,
        control_gui,
        mpc_preview,
    ])
