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

  1. cho_simulation_isaac's run_isaac_sim.py under Isaac's python.sh, driven by the
     fr5.json robot profile.
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
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

SWITCHABLE_CONTROLLERS = [
    'joint_trajectory_controller',
    'joint_space_position_controller',
    'task_space_ik_controller',
]

DEFAULT_ISAAC_SIM_PATH = os.path.join(os.path.expanduser('~'), 'isaacsim')

# Printed by run_isaac_sim.py once physics is stepping and the ROS 2 bridge is
# publishing. The spawners key off it.
ISAAC_READY_MARKER = '[isaac_sim] running:'


def create_runtime_controller_params(ee_name, bringup_type):
    import tempfile

    import yaml

    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='cho_fr5_isaac_runtime_params_',
        dir=runtime_dir,
    )
    params = {
        '/**': {
            'joint_space_position_controller': {
                'ros__parameters': {
                    'bringup_type': bringup_type,
                    'control_mode': 'position',
                },
            },
            'task_space_ik_controller': {
                'ros__parameters': {
                    'bringup_type': bringup_type,
                    'control_mode': 'position',
                    'ee_name': ee_name,
                },
            },
        },
    }
    with os.fdopen(fd, 'w') as runtime_file:
        yaml.safe_dump(params, runtime_file)
    return runtime_path


def cleanup_runtime_controller_params(runtime_path):
    def cleanup(context, *args, **kwargs):
        if os.path.exists(runtime_path):
            os.unlink(runtime_path)
        return []

    return OpaqueFunction(function=cleanup)


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

    isaac_path = get_package_share_directory('cho_simulation_isaac')
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    controller_config = LaunchConfiguration('controllers_file').perform(context)
    runtime_param_file = create_runtime_controller_params(ee_name, bringup_type)

    robot_description = {
        'robot_description': xacro.process_file(
            urdf_path, mappings={'hardware': 'isaac'}
        ).toxml()
    }

    isaac_python = os.path.join(isaac_sim_path, 'python.sh')
    if not os.path.exists(isaac_python):
        raise RuntimeError(
            f"Isaac Sim interpreter not found at '{isaac_python}'. "
            'Pass isaac_sim_path:=<isaac sim install dir>.'
        )
    if not os.path.exists(robot_usd):
        raise RuntimeError(
            f"Isaac robot USD not found at '{robot_usd}'.\nBuild it once with:\n"
            f"  {isaac_python} {os.path.join(isaac_path, 'isaac', 'convert_urdf_to_usd.py')} "
            f'--urdf {urdf_path} --usd-path {os.path.dirname(os.path.dirname(robot_usd))} '
            f'--ros-package cho_description_fr5:'
            f"{get_package_share_directory('cho_description_fr5')} "
            '--require-link wrist3_link'
        )

    isaac_cmd = [
        isaac_python,
        os.path.join(isaac_path, 'isaac', 'run_isaac_sim.py'),
        '--robot-usd', robot_usd,
        '--robot-profile', os.path.join(isaac_path, 'isaac', 'robots', 'fr5.json'),
        '--control-mode', 'position',
        '--physics-rate', physics_rate,
        '--device', device,
    ]
    if headless.lower() == 'true':
        isaac_cmd.append('--headless')

    isaac_sim = ExecuteProcess(cmd=isaac_cmd, output='screen', on_exit=Shutdown())

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

    active_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', controller_name,
            '-p', runtime_param_file,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
        ],
        output='screen',
    )

    inactive = [c for c in SWITCHABLE_CONTROLLERS if c != controller_name]
    inactive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *inactive,
            '-p', runtime_param_file,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
            '--inactive',
        ],
        output='screen',
    )

    isaac_command_gate = Node(
        package='cho_simulation_isaac',
        executable='isaac_command_gate.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Isaac needs tens of seconds to boot, and until it publishes /clock the
    # controller_manager's realtime loop is parked in wait_until_started(). A
    # spawner started before that dies on the controller switch's own 5 s timeout.
    started = {'spawners': False}

    def start_spawners_when_isaac_is_ready(event):
        if started['spawners']:
            return None
        if ISAAC_READY_MARKER not in event.text.decode(errors='replace'):
            return None
        started['spawners'] = True
        return [active_spawner]

    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessIO(
                target_action=isaac_sim,
                on_stdout=start_spawners_when_isaac_is_ready,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=[inactive_spawner, isaac_command_gate],
            )
        ),
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[cleanup_runtime_controller_params(runtime_param_file)],
            )
        ),
    ]

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
        OpaqueFunction(function=setup_control_environment),
    ])
