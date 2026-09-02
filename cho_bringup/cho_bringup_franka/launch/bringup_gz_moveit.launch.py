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

"""Official FR3 Gazebo + MoveIt composition entry point."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata


def generate_launch_description():
    metadata = load_moveit_metadata('franka', 'cho_moveit_franka')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_bringup_franka'), '/launch/bringup_gz_robot.launch.py']),
        launch_arguments={
            'robot_type': 'fr3', 'namespace': '', 'load_gripper': 'true',
            'franka_hand': 'franka_hand', 'ee_name': metadata['ee_link'],
            'control_mode': 'position',
            'controller_name': metadata['hold_controller'],
            'load_moveit_controller': 'true', 'load_ft_sensor': 'false',
            'use_sim_time': 'true', 'launch_rviz': 'false',
            'world': LaunchConfiguration('world'),
        }.items())
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_moveit_franka'), '/launch/moveit.launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'floor_frame': LaunchConfiguration('floor_frame'),
            'floor_size': LaunchConfiguration('floor_size'),
            'floor_position': LaunchConfiguration('floor_position'),
            'scene_ready_timeout': LaunchConfiguration('scene_ready_timeout'),
            'controller_ready_timeout': LaunchConfiguration('controller_ready_timeout'),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        robot, moveit,
    ])
