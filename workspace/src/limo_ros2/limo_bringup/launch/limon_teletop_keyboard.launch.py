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

"""Launch keyboard teleoperation on the high-priority velocity channel."""

# Original AgileX imports (disabled):
# import os
# import sys
# from ament_index_python.packages import get_package_share_directory
# LIMO removal: this launch does not read paths or command-line arguments,
# so these imports were unused. The remaining imports construct the ROS launch.
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        # Original AgileX code (disabled):
        # launch.actions.DeclareLaunchArgument(
        #     name='repeat_rate',
        #     default_value='50.0'
        # ),
        # LIMO removal: repeat_rate was declared but its parameter forwarding
        # was already commented out below, so the argument had no effect.
        launch_ros.actions.Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist',
            # Original AgileX code (already disabled upstream):
            # parameters=[
            #     {
            #         'repeat_rate': launch.substitutions.LaunchConfiguration('repeat_rate')
            #     }
            # ]
            # LIMO addition: publish manual commands on a separate channel for
            # priority arbitration with autonomous commands. A separately launched
            # velocity mux must forward /cmd_vel_teleop to the driver's /cmd_vel;
            # this launch only starts the keyboard node, not the mux.
            remappings=[('cmd_vel', '/cmd_vel_teleop')],
            # Show keyboard instructions and node output in the terminal.
            output='screen',
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
