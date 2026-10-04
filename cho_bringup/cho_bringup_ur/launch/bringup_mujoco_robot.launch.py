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

import os

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from cho_bringup_common import (
    chain_spawners,
    make_spawner_node,
    runtime_param_cleanup,
    write_position_arm_param_file,
)
from cho_robot_config import motion_limit_parameters


SWITCHABLE_CONTROLLERS = [
    'joint_space_position_controller',
    'task_space_ik_controller',
]


def setup_control_environment(context):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    ee_name = LaunchConfiguration('ee_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)

    if controller_name not in SWITCHABLE_CONTROLLERS:
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {SWITCHABLE_CONTROLLERS}"
        )

    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    controller_config = LaunchConfiguration('controllers_file').perform(context)
    # The robot's MoveIt joint/Cartesian limits bound the point-to-point goals.
    runtime_param_file = write_position_arm_param_file(
        bringup_type, ee_name, motion_limit_parameters('ur5e'),
        prefix='cho_ur_mujoco_runtime_params_')

    # ur5e.urdf carries a `hardware` xacro arg so the same file can emit either the
    # MuJoCo or the Isaac ros2_control block, so it has to be expanded rather than
    # read verbatim. xacro also resolves the $(find cho_description_ur) inside it.
    robot_description_content = xacro.process_file(
        urdf_path, mappings={'hardware': 'mujoco'}
    ).toxml()

    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            controller_config,
            runtime_param_file,
            {
                'ee_name': ee_name,
                'bringup_type': bringup_type,
            },
        ],
        remappings=[('~/robot_description', '/robot_description')],
    )

    spawner_kwargs = {
        'runtime_param_file': runtime_param_file,
        'controller_manager': '/controller_manager',
        'timeout': controller_manager_timeout,
    }
    active_spawner = make_spawner_node(
        ['joint_state_broadcaster', controller_name], **spawner_kwargs)
    inactive_spawner = make_spawner_node(
        [controller for controller in SWITCHABLE_CONTROLLERS if controller != controller_name],
        active=False, **spawner_kwargs)

    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco,
                on_start=chain_spawners(active_spawner, [inactive_spawner]),
            )
        ),
        runtime_param_cleanup(runtime_param_file),
    ]

    return [node_robot_state_publisher, node_mujoco] + event_handlers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description=('Cho controller to activate: joint_space_position_controller or '
                         'task_space_ik_controller'),
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='End-effector frame name used by task-space controller',
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='mujoco',
            description='Cho controller bringup type',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                get_package_share_directory('cho_description_ur'),
                'urdf',
                'ur5e.urdf',
            ),
            description='URDF file used for MuJoCo robot_description',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(
                get_package_share_directory('cho_bringup_ur'),
                'config',
                'mujoco',
                'controllers.yaml',
            ),
            description='Controller YAML file loaded by mujoco_ros2_control',
        ),
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='30',
            description='Controller manager service timeout for spawners',
        ),
        OpaqueFunction(function=setup_control_environment),
    ])
