"""Launch the Nav2 controller using a real or simulation YAML profile."""

import os

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
    start_gui = LaunchConfiguration('start_gui')

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
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    control_gui = Node(
        package='limo_controller',
        executable='control_gui',
        name='control_gui',
        output='screen',
        condition=IfCondition(start_gui),
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
        controller_server,
        cmd_vel_mux,
        # Foxy lifecycle_manager performs startup only once.  When the whole
        # application starts at the same time as mapping, planning and RViz,
        # Fast DDS discovery can expose controller_server after that attempt.
        # Give the server time to advertise its lifecycle services first.
        TimerAction(period=3.0, actions=[lifecycle_manager]),
        path_executor,
        control_gui,
    ])
