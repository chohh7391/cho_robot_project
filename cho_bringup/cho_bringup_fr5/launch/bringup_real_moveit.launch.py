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

"""Bring up the real FR5 with MoveIt; no motion is commanded at startup.

The caller remains responsible for verifying the configured IP, physical
workspace, and planning scene before submitting any execution request.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata


def generate_launch_description():
    metadata = load_moveit_metadata('fr5', 'cho_moveit_fr5')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_bringup_fr5'), '/launch/bringup_real_robot.launch.py'
        ]),
        launch_arguments={
            'controller_name': metadata['hold_controller'],
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_moveit_fr5'), '/launch/moveit.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'publish_static_scene': 'true',
            'floor_frame': LaunchConfiguration('floor_frame'),
            'floor_size': LaunchConfiguration('floor_size'),
            'floor_position': LaunchConfiguration('floor_position'),
            'scene_ready_timeout': LaunchConfiguration('scene_ready_timeout'),
            'controller_ready_timeout': LaunchConfiguration('controller_ready_timeout'),
            'activate_controller_after_scene': metadata['trajectory_controller'],
            'deactivate_controller_after_scene': metadata['hold_controller'],
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='',
            description='FR5 controller IP; empty uses the bringup config file.'
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        robot,
        moveit,
    ])
