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

"""Official single- or bimanual OpenArm MuJoCo + MoveIt entry point."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata


def generate_launch_description():
    metadata = load_moveit_metadata('openarm', 'cho_moveit_openarm')
    def setup(context):
        enabled = LaunchConfiguration('mujoco_mit_prototype').perform(context).lower() == 'true'
        bimanual = LaunchConfiguration('bimanual').perform(context).lower() == 'true'
        arm = LaunchConfiguration('arm').perform(context)
        controllers_file = LaunchConfiguration('controllers_file').perform(context)
        if enabled and controllers_file:
            raise RuntimeError('controllers_file overrides are forbidden for paired MIT MoveIt')
        if enabled and (not bimanual or arm != 'both'):
            raise RuntimeError(
                'paired MIT MoveIt requires bimanual:=true and arm:=both')
        robot_args = {
            'bimanual': str(bimanual).lower(),
            'control_mode': 'torque' if enabled else 'position',
            'mujoco_position_profile': 'moveit',
            'controller_name': metadata['hold_controller'],
            'use_sim_time': 'true', 'use_rviz': 'false',
            'mujoco_mit_prototype': str(enabled).lower(),
        }
        if enabled:
            robot_args.update({
                'mit_controller_name': 'bimanual_follow_joint_trajectory_mit_controller',
                'mit_arm': 'both',
                # Paired MIT runtime validation is deliberately headless. Keep
                # the legacy wrapper path on the robot launch's GUI default.
                'mujoco_mit_headless': 'true',
            })
        elif controllers_file:
            robot_args['controllers_file'] = controllers_file
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('cho_bringup_openarm'), 'launch',
                'bringup_mujoco_robot.launch.py'])), launch_arguments=robot_args.items())
        moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('cho_moveit_openarm'), 'launch', 'moveit.launch.py'])),
            launch_arguments={
                'use_sim_time': 'true', 'bimanual': str(bimanual).lower(), 'arm': arm,
                'mit_paired': str(enabled).lower(),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'floor_frame': LaunchConfiguration('floor_frame'),
                'floor_size': LaunchConfiguration('floor_size'),
                'floor_position': LaunchConfiguration('floor_position'),
                'scene_ready_timeout': LaunchConfiguration('scene_ready_timeout'),
                'controller_ready_timeout': LaunchConfiguration('controller_ready_timeout'),
            }.items())
        return [robot, moveit]
    return LaunchDescription([
        DeclareLaunchArgument('bimanual', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('arm', default_value='single',
                              description='single, left, right, or both MoveIt profile'),
        DeclareLaunchArgument('mujoco_mit_prototype', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='Legacy-only controller YAML override'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        OpaqueFunction(function=setup),
    ])
