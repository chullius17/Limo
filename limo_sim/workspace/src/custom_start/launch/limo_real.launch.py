"""Start the physical LIMO robot with wheel odometry and EKF fusion."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from custom_start.launch_config import load_profile, scalar, setting


def _launch_real(context):
    profile = load_profile(context, ('launch', 'camera_driver'))
    settings = profile['launch']

    def value(name):
        return setting(context, settings, name)

    custom_start_share = get_package_share_directory('custom_start')
    limo_base_share = get_package_share_directory('limo_base')

    port_name = value('port_name')
    use_lidar = value('use_lidar')
    use_camera = value('use_camera')
    camera_x = value('camera_x')
    camera_y = value('camera_y')
    camera_z = value('camera_z')
    camera_roll = value('camera_roll')
    camera_pitch = value('camera_pitch')
    camera_yaw = value('camera_yaw')
    open_rviz = value('open_rviz')

    limo_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_base_share, 'launch', 'limo_base.launch.py')
        ),
        launch_arguments={
            'port_name': port_name,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic_name': 'odom',
            'imu_topic_name': '/limo/imu',
            # The EKF is the only publisher of odom -> base_link.
            'pub_odom_tf': 'false',
            # This chassis reports mode 2 even after its mechanical Ackermann
            # conversion, so force the matching Twist-to-steering conversion.
            'motion_mode': '1',
            'steering_left_scale': value('steering_left_scale'),
            'steering_right_scale': value('steering_right_scale'),
        }.items(),
    )

    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
            'base_link', 'imu_link',
        ],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_base_share, 'launch', 'open_ydlidar_launch.py')
        ),
        condition=IfCondition(use_lidar),
    )

    camera_mount_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        condition=IfCondition(use_camera),
        arguments=[
            camera_x, camera_y, camera_z,
            # Foxy uses the legacy positional order: yaw, pitch, roll.
            camera_yaw, camera_pitch, camera_roll,
            'base_link', 'camera_link',
        ],
    )

    camera = GroupAction(
        condition=IfCondition(use_camera),
        actions=[
            SetRemap(
                src='/camera/color/image_raw',
                dst='/rgb/image_raw',
            ),
            SetRemap(
                src='/camera/color/camera_info',
                dst='/rgb/camera_info',
            ),
            SetRemap(
                src='/camera/depth/image_raw',
                dst='/depth_camera/depth/image_raw',
            ),
            SetRemap(
                src='/camera/depth/camera_info',
                dst='/depth_camera/depth/camera_info',
            ),
            SetRemap(
                src='/camera/depth/points',
                dst='/depth/points',
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('astra_camera'),
                        'launch',
                        'dabai_u3.launch.xml',
                    ])
                ),
                launch_arguments={
                    name: scalar(profile['camera_driver'], name)
                    for name in profile['camera_driver']
                }.items(),
            ),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        # Keep the node-specific YAML overrides on the root namespace.
        namespace='/',
        output='screen',
        parameters=[
            os.path.join(custom_start_share, 'config', 'ekf.yaml'),
            os.path.join(custom_start_share, 'config', 'ekf_real.yaml'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(open_rviz),
    )

    return [
        limo_base,
        imu_transform,
        lidar,
        camera_mount_transform,
        camera,
        ekf,
        rviz,
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('custom_start'),
        'config', 'limo_real.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='YAML profile for the physical robot.'),
        # 2026-10-01 cmd_vel/IMU trial indicates direct protocol steering.
        # Expose overrides so this chassis calibration is reversible.
        DeclareLaunchArgument(
            'steering_left_scale', default_value='',
            description='Left inner-wheel/protocol angle ratio (>= 1.0).'),
        DeclareLaunchArgument(
            'steering_right_scale', default_value='',
            description='Right inner-wheel/protocol angle ratio (>= 1.0).'),
        DeclareLaunchArgument(
            'port_name', default_value='',
            description='Serial device name used by the physical LIMO.',
        ),
        DeclareLaunchArgument(
            'use_lidar', default_value='',
            description='Start the physical YDLidar driver.',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='',
            description='Start the physical Astra depth camera.',
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
        DeclareLaunchArgument(
            'open_rviz', default_value='',
            description='Start RViz.',
        ),
        OpaqueFunction(function=_launch_real),
    ])
