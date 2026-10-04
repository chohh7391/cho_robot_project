# Copyright (c) 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bring up the FR5 in Isaac Sim.

    ros2 launch cho_bringup_fr5 bringup_isaac_robot.launch.py \
         controller_name:=joint_space_position_controller

The FR5 spawns at all-zero joint positions, which is a wrist singularity. Use
the joint-space action client's ``home 1`` command first, then switch to the
task-space controller before sending ``reach`` goals::

    ros2 control switch_controllers \
        --activate task_space_ik_controller \
        --deactivate joint_space_position_controller

Same three-part structure as the UR Isaac bringup (see
cho_bringup_ur/launch/bringup_isaac_robot.launch.py):

  1. cho_simulation_isaac's run_isaac_sim.py under Isaac's python.sh, driven by this
     bringup package's config/isaac/robot_profile.json.
  2. a controller_manager whose hardware plugin is
     topic_based_ros2_control/TopicBasedSystem (the isaac branch of
     fr5.urdf.xacro). It is the mujoco_ros2_control build of ros2_control_node,
     which is upstream's node plus sim-clock pacing.
  3. the spawners, held back until Isaac announces itself, then the command gate.

The FR5 has no gripper and is position-controlled only, so there is a single
hardware component and no control_mode argument.

Build the USD asset once before the first run:
    ~/isaacsim/python.sh <cho_simulation_isaac share>/isaac/convert_urdf_to_usd.py \
        --urdf <cho_description_fr5 share>/urdf/fr5.urdf.xacro \
        --usd-path <cho_description_fr5 share>/usd \
        --ros-package cho_description_fr5:<cho_description_fr5 share> \
        --require-link wrist3_link
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro
from cho_bringup_common import (
    chain_spawners,
    check_isaac_install,
    DEFAULT_ISAAC_SIM_PATH,
    gate_failure_argument,
    isaac_command_gate,
    isaac_controller_startup,
    isaac_sim_command,
    isaac_sim_process,
    load_package_utils,
    make_spawner_node,
    runtime_param_cleanup,
    shutdown_on_gate_failure,
)

launch_utils = load_package_utils('cho_bringup_fr5')

SWITCHABLE_CONTROLLERS = [
    'joint_trajectory_controller',
    'joint_space_position_controller',
    'task_space_ik_controller',
]


def setup_control_environment(context):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    ee_name = LaunchConfiguration('ee_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    cm_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)
    isaac_sim_path = LaunchConfiguration('isaac_sim_path').perform(context)
    robot_usd = LaunchConfiguration('robot_usd').perform(context)
    physics_rate = LaunchConfiguration('physics_rate').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    device = LaunchConfiguration('device').perform(context)

    if controller_name not in SWITCHABLE_CONTROLLERS:
        if controller_name == 'moveit':
            raise RuntimeError(
                "'moveit' is not a ros2_control controller. Launch "
                "bringup_isaac_moveit.launch.py instead.")
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {SWITCHABLE_CONTROLLERS}"
        )

    bringup_path = get_package_share_directory('cho_bringup_fr5')
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    controller_config = LaunchConfiguration('controllers_file').perform(context)
    # Isaac carries no gripper, so no tool envelope.
    runtime_param_file = launch_utils.create_runtime_param_file(
        bringup_type, ee_name, prefix='cho_fr5_isaac_runtime_params_')

    robot_description = {
        'robot_description': xacro.process_file(
            urdf_path, mappings={'hardware': 'isaac'}
        ).toxml()
    }

    isaac_python = check_isaac_install(isaac_sim_path, robot_usd, [
        '--urdf', urdf_path,
        '--usd-path', os.path.dirname(os.path.dirname(robot_usd)),
        '--ros-package', f"cho_description_fr5:{get_package_share_directory('cho_description_fr5')}",
        '--require-link', 'wrist3_link',
    ])
    isaac_sim = isaac_sim_process(isaac_sim_command(
        isaac_python, robot_usd,
        os.path.join(bringup_path, 'config', 'isaac', 'robot_profile.json'),
        'position', physics_rate, device, headless=headless.lower() == 'true'))

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    node_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            controller_config,
            runtime_param_file,
            {'ee_name': ee_name, 'bringup_type': bringup_type},
        ],
        remappings=[('~/robot_description', '/robot_description')],
        on_exit=Shutdown(),
    )

    spawner_kwargs = {
        'runtime_param_file': runtime_param_file,
        'controller_manager': '/controller_manager',
        'timeout': cm_timeout,
    }
    active_spawner = make_spawner_node(['joint_state_broadcaster', controller_name], **spawner_kwargs)
    # Loaded whether or not the active spawner succeeded, as before.
    inactive_spawner = make_spawner_node(
        [c for c in SWITCHABLE_CONTROLLERS if c != controller_name], active=False, **spawner_kwargs)

    # Spawners once Isaac is stepping; the command gate once the requested
    # controller is active (see isaac_controller_startup for both reasons).
    event_handlers = isaac_controller_startup(
        isaac_sim, chain_spawners(active_spawner, [inactive_spawner]), active_spawner,
        isaac_command_gate({'use_sim_time': use_sim_time}),
        shutdown_on_failure=shutdown_on_gate_failure(context))
    event_handlers.append(runtime_param_cleanup(runtime_param_file))

    return [isaac_sim, node_robot_state_publisher, node_ros2_control] + event_handlers


def generate_launch_description():
    fr5_desc = get_package_share_directory('cho_description_fr5')
    bringup = get_package_share_directory('cho_bringup_fr5')
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description=(
                'joint_trajectory_controller, joint_space_position_controller, '
                'or task_space_ik_controller'
            ),
        ),
        DeclareLaunchArgument('ee_name', default_value='wrist3_link'),
        DeclareLaunchArgument('bringup_type', default_value='isaac'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(fr5_desc, 'urdf', 'fr5.urdf.xacro'),
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(bringup, 'config', 'isaac', 'controllers.yaml'),
        ),
        DeclareLaunchArgument('controller_manager_timeout', default_value='60'),
        DeclareLaunchArgument(
            'robot_usd',
            # The URDF importer names the subdirectory and the .usda after the URDF.
            default_value=os.path.join(fr5_desc, 'usd', 'fr5', 'fr5.usda'),
            description='USD asset for Isaac; build it with isaac/convert_urdf_to_usd.py',
        ),
        DeclareLaunchArgument('isaac_sim_path', default_value=DEFAULT_ISAAC_SIM_PATH),
        DeclareLaunchArgument(
            'physics_rate',
            default_value='250',
            description='MUST match controller_manager.update_rate in controllers_file',
        ),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('device', default_value='cpu', choices=['cpu', 'cuda']),
        gate_failure_argument(),
        OpaqueFunction(function=setup_control_environment),
    ])
