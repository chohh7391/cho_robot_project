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

"""Bring up MuJoCo FR5 with MoveIt from a collision-free bootstrap pose.

The resolved gripper goes to MoveIt as well as to the robot bringup. It has to:
move_group and RViz expand the description themselves, and a model without the
gripper joint rejects gripper_finger_joint out of /joint_states with
"Joint 'gripper_finger_joint' not found in model 'fr5'".
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata

package_share = get_package_share_directory('cho_bringup_fr5')
# Same by-path load the other bringups use: launch_utils lives in lib/.
_launch_utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_fr5', 'utils', 'launch_utils.py')
)
_spec = importlib.util.spec_from_file_location('fr5_launch_utils', _launch_utils_path)
launch_utils = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch_utils)


def setup_includes(context):
    metadata = load_moveit_metadata('fr5', 'cho_moveit_fr5')
    # No connection config file in simulation, so the fallback is 'none'.
    gripper = launch_utils.resolve_gripper(
        LaunchConfiguration('gripper').perform(context),
        LaunchConfiguration('load_gripper').perform(context),
        'none')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_bringup_fr5'), '/launch/bringup_mujoco_robot.launch.py'
        ]),
        launch_arguments={
            'controller_name': metadata['hold_controller'],
            'use_sim_time': 'true',
            'mujoco_initial_keyframe': LaunchConfiguration('mujoco_initial_keyframe'),
            # The resolved NAME goes down, not load_gripper: the robot launch
            # would otherwise re-derive the same answer from the same inputs.
            'gripper': gripper,
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_moveit_fr5'), '/launch/moveit.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'publish_static_scene': 'true',
            'gripper': gripper,
            'floor_frame': LaunchConfiguration('floor_frame'),
            'floor_size': LaunchConfiguration('floor_size'),
            'floor_position': LaunchConfiguration('floor_position'),
            'scene_ready_timeout': LaunchConfiguration('scene_ready_timeout'),
            'controller_ready_timeout': LaunchConfiguration('controller_ready_timeout'),
            'activate_controller_after_scene': metadata['trajectory_controller'],
            'deactivate_controller_after_scene': metadata['hold_controller'],
        }.items(),
    )
    return [robot, moveit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'gripper', default_value='',
            description=(
                'End-effector gripper by name: none | ag95. Simulation defaults to '
                'none. Applied to the controllers and to the MoveIt model alike.'
            ),
        ),
        DeclareLaunchArgument(
            'load_gripper', default_value='config',
            description=(
                'Boolean spelling of the same choice; config (the default) means '
                'none in simulation. Contradicting gripper:= is a launch error.'
            ),
        ),
        DeclareLaunchArgument(
            'mujoco_initial_keyframe',
            default_value='home1',
            description='Planning-scene-safe, non-singular MuJoCo bootstrap keyframe for MoveIt',
        ),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='180.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='60.0'),
        OpaqueFunction(function=setup_includes),
    ])
