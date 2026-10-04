# Copyright 2026 Hyunho Cho
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

"""View the FR5 in RViz with a joint_state_publisher GUI (no hardware).

    ros2 launch cho_description_fr5 view_fr5.launch.py
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory('cho_description_fr5')
    urdf_path = os.path.join(desc, 'urdf', 'fr5.urdf.xacro')

    # 'mock' keeps the ros2_control block hardware-agnostic so it expands without
    # a simulator attached; robot_state_publisher only needs the kinematics.
    robot_description = {
        'robot_description': xacro.process_file(
            urdf_path, mappings={'hardware': 'mock'}
        ).toxml()
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(desc, 'rviz', 'view_robot.rviz')],
        ),
    ])
