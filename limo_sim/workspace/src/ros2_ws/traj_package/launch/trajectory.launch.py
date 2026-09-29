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

"""Launch the LIMO Nav2 global planner with SMAC Hybrid-A*."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Create the global planner and goal bridge for the selected model."""
    package_share = get_package_share_directory('traj_package')
    robot_model = LaunchConfiguration('robot_model')
    default_params_file = [
        os.path.join(package_share, 'config', 'traj_'), robot_model, '.yaml']
    profile_clock = PythonExpression(["'", robot_model, "' == 'sim'"])

    planner_params_file = LaunchConfiguration('planner_params_file')
    map_topic = LaunchConfiguration('map_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Clock and map-topic overrides apply to either model profile or custom YAML.
    configured_params = RewrittenYaml(
        source_file=planner_params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            (
                'global_costmap.global_costmap.ros__parameters.'
                'static_layer.map_topic'
            ): map_topic,
            (
                'global_costmap.global_costmap.ros__parameters.'
                'border_follow_layer.source_topic'
            ): map_topic,
        },
        convert_types=True,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_global_planner',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': ['planner_server'],
        }],
    )

    # RViz publishes selected poses on /goal_pose, while planner_server exposes
    # an action. The bridge is included to simplify graphical goal selection.
    rviz_goal_bridge = Node(
        package='traj_package',
        executable='rviz_goal_bridge',
        name='rviz_goal_bridge',
        output='screen',
        parameters=[configured_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model', default_value='real', choices=['real', 'sim'],
            description='Select traj_real.yaml or traj_sim.yaml.',
        ),
        DeclareLaunchArgument(
            'planner_params_file',
            default_value=default_params_file,
            description='Planner/bridge YAML override; default follows robot_model.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value=(
                '/limo/map_package/online/map/combined_grid'
            ),
            description='OccupancyGrid used by the global planner.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=profile_clock,
            description='Clock override; default is true for sim, false for real.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Configure and activate the Nav2 planner automatically.',
        ),
        planner_server,
        lifecycle_manager,
        rviz_goal_bridge,
    ])
