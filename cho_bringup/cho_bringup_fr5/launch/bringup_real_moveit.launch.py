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

"""Bring up the real FR5 with MoveIt; the arm commands no motion at startup.

The caller remains responsible for verifying the configured IP, physical
workspace, and planning scene before submitting any execution request.

The one exception to "no motion" is the gripper: with `open_on_activate` set in
fr5.config.yaml the hardware opens the jaws once as it activates, so a run
starts from a known opening. Clear the jaws before launching.

Either `load_gripper:=true` (the cho_bringup_franka spelling) or `gripper:=ag95`
selects the AG-95; `config` / empty defers to fr5.config.yaml.

The resolved gripper is handed to MoveIt as well as to the robot bringup. It has
to be: move_group and RViz expand the description themselves, and a model
without the gripper joint rejects gripper_finger_joint out of /joint_states with
"Joint 'gripper_finger_joint' not found in model 'fr5'".
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata


import importlib.util

package_share = get_package_share_directory('cho_bringup_fr5')
# Same by-path load as bringup_real_robot: launch_utils lives in lib/.
_launch_utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_fr5', 'utils', 'launch_utils.py')
)
_spec = importlib.util.spec_from_file_location('fr5_launch_utils', _launch_utils_path)
launch_utils = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch_utils)


def setup_includes(context):
    metadata = load_moveit_metadata('fr5', 'cho_moveit_fr5')

    # Same precedence the robot bringup uses: explicit arg, then the config
    # file, then none. Resolved here so both halves are given the same value
    # rather than each deriving its own.
    config_path = LaunchConfiguration('config_file').perform(context)
    with open(config_path) as f:
        fr5_cfg = (yaml.safe_load(f) or {}).get('fr5', {})
    gripper = launch_utils.resolve_gripper(
        LaunchConfiguration('gripper').perform(context),
        LaunchConfiguration('load_gripper').perform(context),
        fr5_cfg.get('gripper'))

    # Resolved here rather than forwarded raw: the empty default has to become
    # the registry value before it reaches a float parameter downstream.
    velocity_scaling = LaunchConfiguration('max_velocity_scaling_factor').perform(context) or \
        str(metadata['max_velocity_scaling_factor'])
    acceleration_scaling = \
        LaunchConfiguration('max_acceleration_scaling_factor').perform(context) or \
        str(metadata['max_acceleration_scaling_factor'])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cho_bringup_fr5'), '/launch/bringup_real_robot.launch.py'
        ]),
        launch_arguments={
            'controller_name': metadata['hold_controller'],
            'robot_ip': LaunchConfiguration('robot_ip'),
            'config_file': config_path,
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
            'use_sim_time': 'false',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'publish_static_scene': 'true',
            'gripper': gripper,
            'floor_frame': LaunchConfiguration('floor_frame'),
            'floor_size': LaunchConfiguration('floor_size'),
            'floor_position': LaunchConfiguration('floor_position'),
            'scene_ready_timeout': LaunchConfiguration('scene_ready_timeout'),
            'controller_ready_timeout': LaunchConfiguration('controller_ready_timeout'),
            'max_velocity_scaling_factor': velocity_scaling,
            'max_acceleration_scaling_factor': acceleration_scaling,
            'activate_controller_after_scene': metadata['trajectory_controller'],
            'deactivate_controller_after_scene': metadata['hold_controller'],
        }.items(),
    )
    return [robot, moveit]


def generate_launch_description():
    bringup = get_package_share_directory('cho_bringup_fr5')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='',
            description='FR5 controller IP; empty uses the bringup config file.'
        ),
        DeclareLaunchArgument(
            'gripper', default_value='',
            description=(
                'End-effector gripper by name: none | ag95. Empty uses the bringup '
                'config file. Applied to the controllers and to the MoveIt model alike.'
            ),
        ),
        DeclareLaunchArgument(
            'load_gripper', default_value='config',
            description=(
                'Boolean spelling of the same choice, as cho_bringup_franka uses: '
                'true loads the AG-95, false loads none, config (the default) defers '
                'to the config file. Contradicting gripper:= is a launch error.'
            ),
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(bringup, 'config', 'real', 'fr5.config.yaml'),
            description='FR5 connection settings (robot_ip, gripper, ...).',
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        DeclareLaunchArgument(
            'max_velocity_scaling_factor', default_value='',
            description=(
                'Fraction of the joint_limits.yaml velocity MoveIt execution runs at; '
                'empty uses moveit.execution from cho_robot_config. Lower it for a first '
                'motion on hardware - joint_trajectory_controller has no per-cycle clamp '
                'of its own, unlike task_space_ik_controller.'
            ),
        ),
        DeclareLaunchArgument(
            'max_acceleration_scaling_factor', default_value='',
            description='As above, for acceleration.',
        ),
        OpaqueFunction(function=setup_includes),
    ])
