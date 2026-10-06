"""Launch Gazebo with the custom LIMO circuit and the Ackermann LIMO robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from custom_start.launch_config import boolean, load_profile, scalar, setting


def _launch_circuit(context):
    profile = load_profile(context, ('launch', 'spawn'))
    settings = profile['launch']

    def value(name):
        return setting(context, settings, name)

    custom_start_share = get_package_share_directory('custom_start')
    limo_car_share = get_package_share_directory('limo_car')

    ekf_config = os.path.join(custom_start_share, 'config', 'ekf.yaml')
    model_path = os.path.join(custom_start_share, 'models')

    world = value('world')
    if not os.path.isabs(world):
        world = os.path.join(custom_start_share, world)
    gui = value('gui')
    use_sim_time = value('use_sim_time')
    camera_x = value('camera_x')
    camera_y = value('camera_y')
    camera_z = value('camera_z')
    camera_roll = value('camera_roll')
    camera_pitch = value('camera_pitch')
    camera_yaw = value('camera_yaw')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_car_share, 'launch', 'ackermann.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Reproduce the physical-camera TF chain used by limo_real.launch.py.
    camera_mount_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_base_link_to_camera',
        arguments=[
            camera_x, camera_y, camera_z,
            # Foxy positional order: yaw, pitch, roll.
            camera_yaw, camera_pitch, camera_roll,
            'base_link', 'camera_link',
        ],
    )

    depth_optical_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_camera_to_depth_optical',
        arguments=[
            '0.0', '0.0', '0.0',
            # Same optical convention used by the physical Astra driver:
            # roll=-pi/2, pitch=0, yaw=-pi/2.
            '-1.57079632679', '0.0', '-1.57079632679',
            'camera_link', 'depth_camera_frame_optical',
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': boolean(use_sim_time, 'use_sim_time')},
        ],
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose', world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        condition=IfCondition(gui),
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'limo',
            '-x', scalar(profile['spawn'], 'x'),
            '-y', scalar(profile['spawn'], 'y'),
            '-z', scalar(profile['spawn'], 'z'),
            '-Y', scalar(profile['spawn'], 'yaw'),
        ],
        output='screen',
    )

    return [
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [model_path, ':', EnvironmentVariable(
                'GAZEBO_MODEL_PATH', default_value='')],
        ),
        robot_state_publisher,
        camera_mount_transform,
        depth_optical_transform,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        ekf_node,
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('custom_start'),
        'config', 'limo_circuit.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='YAML profile for the Gazebo circuit.'),
        DeclareLaunchArgument(
            'world', default_value='',
            description=(
                'Gazebo world path; relative paths use custom_start share.'),
        ),
        DeclareLaunchArgument(
            'gui', default_value='',
            description='Start the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='',
            description='Use the clock published by Gazebo.',
        ),
        DeclareLaunchArgument(
            'camera_x', default_value='',
            description='Camera X offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'camera_y', default_value='',
            description='Camera Y offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'camera_z', default_value='',
            description='Camera Z offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'camera_roll', default_value='',
            description='Camera roll relative to base_link in radians.',
        ),
        DeclareLaunchArgument(
            'camera_pitch', default_value='',
            description='Camera pitch relative to base_link in radians.',
        ),
        DeclareLaunchArgument(
            'camera_yaw', default_value='',
            description='Camera yaw relative to base_link in radians.',
        ),
        OpaqueFunction(function=_launch_circuit),
    ])
