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

"""Bring up the FR5 in Gazebo (Ignition/Gazebo Sim).

    ros2 launch cho_bringup_fr5 bringup_gz_robot.launch.py \
         controller_name:=joint_space_position_controller

Gazebo spawns at the canonical non-singular ``home 1`` ready pose. Re-commanding
``home 1`` is optional before switching from ``joint_space_position_controller``
to ``task_space_ik_controller`` and sending a task-space ``reach`` goal.

fr5.urdf.xacro is expanded with hardware:=gazebo, which emits the
gz_ros2_control/GazeboSimSystem ros2_control block plus the Gazebo system
plugin; the plugin hosts the controller_manager inside Gazebo and loads
config/gz/controllers.yaml. The spawners then talk to that controller_manager
once the entity is in the world.
"""

import os

import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from cho_bringup_common import (
    chain_spawners,
    load_package_utils,
    make_spawner_node,
    prepend_to_search_paths,
    runtime_param_cleanup,
)

launch_utils = load_package_utils('cho_bringup_fr5')

# What config/gz/controllers.yaml sets as well; this bringup has no arguments
# for them. The description's default gripper is none, so there is no tool
# envelope either.
BRINGUP_TYPE = 'gz'
EE_NAME = 'wrist3_link'


SWITCHABLE_CONTROLLERS = [
    'joint_trajectory_controller',
    'joint_space_position_controller',
    'task_space_ik_controller',
]
CONTROLLER_MODES = list(SWITCHABLE_CONTROLLERS)


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    controller_name = LaunchConfiguration('controller_name').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    allow_renaming = LaunchConfiguration('allow_renaming')

    if controller_name not in CONTROLLER_MODES:
        if controller_name == 'moveit':
            raise RuntimeError(
                "'moveit' is not a ros2_control controller. Launch "
                "bringup_gz_moveit.launch.py instead.")
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {CONTROLLER_MODES}"
        )
    active_controller = controller_name

    fr5_desc = get_package_share_directory('cho_description_fr5')
    bringup = get_package_share_directory('cho_bringup_fr5')
    urdf_path = os.path.join(fr5_desc, 'urdf', 'fr5.urdf.xacro')
    controllers_file = os.path.join(bringup, 'config', 'gz', 'controllers.yaml')

    # sdformat converts package://cho_description_fr5/... mesh URIs to
    # model://cho_description_fr5/....  Gazebo therefore needs the parent of
    # the package share directory on its resource path in order to resolve
    # both visual and collision meshes.
    resource_path_actions = prepend_to_search_paths(
        ['IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'], os.path.dirname(fr5_desc))

    # The controller_manager lives inside the Gazebo plugin and loads
    # config/gz/controllers.yaml from the description, so the runtime
    # parameters (the MoveIt motion limits, mainly) reach the controllers
    # through the spawners' -p, as in cho_bringup_ur's Gazebo bringup.
    runtime_param_file = launch_utils.create_runtime_param_file(
        BRINGUP_TYPE, EE_NAME, prefix='cho_fr5_gz_runtime_params_')

    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={'hardware': 'gazebo', 'simulation_controllers': controllers_file},
    ).toxml()
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_content,
            '-name', robot_name,
            '-allow_renaming', allow_renaming,
        ],
    )

    gz_launch_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': [' -r -v 4 ', world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gz_launch_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': [' -s -r -v 4 ', world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(fr5_desc, 'rviz', 'view_robot.rviz')],
        # On the Gazebo clock like everything else here, or its TF lookups
        # compare wall time against sim-time stamps and drop every transform.
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    spawner_kwargs = {
        'runtime_param_file': runtime_param_file,
        'controller_manager': '/controller_manager',
        'timeout': LaunchConfiguration('controller_manager_timeout').perform(context),
    }
    active_controller_spawner = make_spawner_node(
        ['joint_state_broadcaster', active_controller], **spawner_kwargs)
    inactive_controller_spawner = make_spawner_node(
        [c for c in SWITCHABLE_CONTROLLERS if c != active_controller], active=False, **spawner_kwargs)

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=chain_spawners(active_controller_spawner, [inactive_controller_spawner]) + [rviz],
        )
    )

    actions = resource_path_actions + [
        robot_state_publisher,
        gz_spawn_entity,
        gz_launch_with_gui,
        gz_launch_without_gui,
        clock_bridge,
        delayed_spawners,
        runtime_param_cleanup(runtime_param_file),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='Initial Cho arm controller to activate.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='empty.sdf'),
        DeclareLaunchArgument('robot_name', default_value='fr5'),
        DeclareLaunchArgument('allow_renaming', default_value='true'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        OpaqueFunction(function=launch_setup),
    ])
