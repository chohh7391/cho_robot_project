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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from cho_bringup_common import (
    make_spawner_node,
    runtime_param_cleanup,
    write_position_arm_param_file,
)
from cho_robot_config import motion_limit_parameters


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    kinematics_params_file = LaunchConfiguration('kinematics_params_file')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_name = LaunchConfiguration('controller_name')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_tool_communication = LaunchConfiguration('use_tool_communication')
    description_file = LaunchConfiguration('description_file')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    activate_joint_controller = LaunchConfiguration('activate_joint_controller')

    ee_name = LaunchConfiguration('ee_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)
    # The robot's MoveIt joint/Cartesian limits bound the point-to-point goals.
    runtime_param_file = write_position_arm_param_file(
        bringup_type, ee_name, motion_limit_parameters('ur5e'),
        prefix='cho_ur_runtime_params_')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py',
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'runtime_config_package': 'cho_bringup_ur',
                'description_package': 'cho_description_ur',
                'description_file': description_file,
                'controllers_file': controllers_file,
                'initial_joint_controller': initial_joint_controller,
                'activate_joint_controller': activate_joint_controller,
                'kinematics_params_file': kinematics_params_file,
                'launch_rviz': launch_rviz,
                'use_tool_communication': use_tool_communication,
            }.items(),
        ),
        make_spawner_node(
            [controller_name],
            runtime_param_file,
            controller_manager='/controller_manager',
            timeout=controller_manager_timeout,
        ),
        runtime_param_cleanup(runtime_param_file),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='UR robot type passed to ur_robot_driver.',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address of the UR controller.',
        ),
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cho_description_ur'),
                'config',
                LaunchConfiguration('ur_type'),
                'default_kinematics.yaml',
            ]),
            description=('Calibration YAML extracted from the physical robot, or nominal '
                         'default kinematics.'),
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='real/controllers.yaml',
            description='Controller YAML path relative to cho_bringup_ur/config.',
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='Initial Cho controller to activate.',
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ur.urdf.xacro',
            description='UR description file forwarded to ur_robot_driver.',
        ),
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='joint_trajectory_controller',
            description='Initial UR driver joint controller.',
        ),
        DeclareLaunchArgument(
            'activate_joint_controller',
            default_value='false',
            description='Whether ur_robot_driver activates the initial joint controller.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Forwarded to the UR driver launch.',
        ),
        DeclareLaunchArgument(
            'use_tool_communication',
            default_value='false',
            description='Forwarded to the UR driver launch.',
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='real',
            description='Cho controller bringup type.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='Cho controller end-effector frame.',
        ),
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='30',
            description='Controller manager service timeout for spawner.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
