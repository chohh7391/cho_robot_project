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

"""Bring up the Franka FR3 in Isaac Sim, same interface as the other environments.

    ros2 launch cho_bringup_franka bringup_isaac_robot.launch.py \
         control_mode:=torque controller_name:=task_space_qp_controller

Unlike Gazebo (controller_manager inside the sim plugin) and MuJoCo (simulator
inside the hardware component), Isaac Sim runs in its own process under its own
Python interpreter. This launch therefore starts three things and orders them:

  1. isaac/run_isaac_franka.py under Isaac's python.sh -- physics, the OmniGraph
     ROS 2 bridge, /clock and /isaac_joint_states.
  2. a controller_manager whose hardware plugin is
     topic_based_ros2_control/TopicBasedSystem (declared in the URDF's
     IsaacArmSystem / IsaacHandSystem blocks). It is the mujoco_ros2_control
     build of ros2_control_node: that binary contains no MuJoCo code at all, it
     is upstream's node plus `wait_until_started()` on the clock and sim-time
     pacing, which is exactly what a sim-clocked controller_manager needs.
  3. the controller spawners, then scripts/isaac_command_gate.py once the
     requested controller is active -- see that script for why the gate exists.

Before the first run, build the USD asset once:
    ~/isaacsim/python.sh <share>/cho_bringup_franka/isaac/convert_urdf_to_usd.py --help
"""

import importlib.util
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

package_share = get_package_share_directory('cho_bringup_franka')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_franka', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')
spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

ALWAYS_ACTIVE_CONTROLLERS = launch_utils.ALWAYS_ACTIVE_CONTROLLERS
create_controller_spawners = launch_utils.create_controller_spawners
create_runtime_param_file = launch_utils.create_runtime_param_file
create_runtime_param_cleanup = launch_utils.create_runtime_param_cleanup
get_initial_active_controller = launch_utils.get_initial_active_controller
get_switchable_controllers = launch_utils.get_switchable_controllers

DEFAULT_ISAAC_SIM_PATH = os.path.join(os.path.expanduser('~'), 'isaacsim')

# Printed by cho_bringup_isaac's run_isaac_sim.py once physics is stepping and the ROS 2
# bridge is publishing. The spawners key off it; see setup_control_environment.
ISAAC_READY_MARKER = '[isaac_sim] running:'


def generate_launch_description():

    description_path = get_package_share_directory('cho_description_franka')
    bringup_path = get_package_share_directory('cho_bringup_franka')
    isaac_path = get_package_share_directory('cho_bringup_isaac')

    declared_arguments = [
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            description='Choose control mode: position, velocity, torque',
            choices=['position', 'velocity', 'torque'],
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='task_space_impedance_controller',
            description='Which controller to activate initially'
        ),
        DeclareLaunchArgument(
            'vla',
            default_value='false',
            description='If true, forces vla_controller to be the active controller'
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Enable Franka gripper controllers and mock gripper server'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='isaac',
            description=(
                'Global bringup type injected to all controllers. Anything other than '
                '"real"/"gazebo" makes the controllers feed forward the full non-linear '
                'effects, which is what Isaac needs: PhysX applies the commanded joint '
                'torques raw and compensates nothing.'
            ),
        ),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=os.path.join(
                description_path, 'urdf', 'fr3_with_ft_sensor', 'fr3_franka_hand.urdf'),
            description='Xacro/URDF used to build the Isaac robot_description'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(bringup_path, 'config', 'isaac', 'controllers.yaml'),
            description='Controller YAML loaded by the controller_manager'
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='fr3_hand_tcp',
            description='Name of End-Effector',
            choices=['fr3_link7', 'fr3_hand', 'fr3_hand_tcp']
        ),
        DeclareLaunchArgument(
            'robot_usd',
            # Layout is the URDF importer's own convention: it derives both the
            # subdirectory and the .usda filename from the URDF basename.
            default_value=os.path.join(
                description_path, 'usd', 'fr3_with_ft_sensor',
                'fr3_franka_hand', 'fr3_franka_hand.usda'),
            description=(
                'USD asset for Isaac. Build it once with '
                'isaac/convert_urdf_to_usd.py; it is not tracked in git.'
            ),
        ),
        DeclareLaunchArgument(
            'isaac_sim_path',
            default_value=DEFAULT_ISAAC_SIM_PATH,
            description='Isaac Sim installation directory (must contain python.sh)'
        ),
        DeclareLaunchArgument(
            'physics_rate',
            default_value='250',
            description=(
                'Isaac physics steps per second. MUST match controller_manager.update_rate '
                'in controllers_file: the controller_manager is paced by the /clock Isaac '
                'publishes, so a mismatch makes cycles fire in bursts with a zero measured '
                'period.'
            ),
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Isaac Sim without a viewport'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Isaac physics device',
            choices=['cpu', 'cuda'],
        ),
        DeclareLaunchArgument(
            'publish_ft',
            default_value='false',
            description='Publish an emulated Bota FT wrench and serve /bota_ft_sensor/tare'
        ),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                LaunchConfiguration('xacro_file'),
                ' control_mode:=',
                LaunchConfiguration('control_mode'),
                ' hardware:=isaac',
            ]),
            value_type=str
        )
    }

    payload_config_file = os.path.join(bringup_path, 'config', 'payload.yaml')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    mock_gripper = Node(
        package='cho_bringup_franka',
        executable='mock_franka_gripper.py',
        parameters=[
            use_sim_time,
            {
                # Matches simulation_gripper_controller.interface_name in
                # config/isaac/controllers.yaml and the IsaacHandSystem block,
                # which command the fingers by position in every control_mode.
                'command_mode': 'position',
            }
        ],
        condition=IfCondition(LaunchConfiguration('load_gripper')),
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[use_sim_time, robot_description]
    )

    def setup_control_environment(context, *args, **kwargs):
        mode = LaunchConfiguration('control_mode').perform(context)
        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        load_gripper = LaunchConfiguration('load_gripper').perform(context)
        use_vla = LaunchConfiguration('vla').perform(context)
        b_type = LaunchConfiguration('bringup_type').perform(context)
        ee_name = LaunchConfiguration('ee_name').perform(context)
        isaac_sim_path = LaunchConfiguration('isaac_sim_path').perform(context)
        robot_usd = LaunchConfiguration('robot_usd').perform(context)
        physics_rate = LaunchConfiguration('physics_rate').perform(context)
        headless = LaunchConfiguration('headless').perform(context)
        device = LaunchConfiguration('device').perform(context)
        publish_ft = LaunchConfiguration('publish_ft').perform(context)

        load_gripper_bool = load_gripper.lower() == 'true'
        always_active_controllers = [
            controller for controller in ALWAYS_ACTIVE_CONTROLLERS
            if load_gripper_bool or controller not in (
                'simulation_gripper_controller',
                'gripper_controller',
            )
        ]

        initial_active_controller = get_initial_active_controller(ctrl_name, use_vla)
        switchable_controllers = get_switchable_controllers(
            control_mode=mode,
            use_vla=use_vla,
            requested_controller=ctrl_name,
        )
        all_runtime_param_controllers = (
            always_active_controllers + switchable_controllers
        )
        runtime_param_file = create_runtime_param_file(
            payload_config_path=payload_config_file,
            controller_names=all_runtime_param_controllers,
            bringup_type=b_type,
            control_mode=mode,
            ee_name=ee_name,
        )
        controller_spawners = create_controller_spawners(
            always_active_controllers=always_active_controllers,
            switchable_controllers=switchable_controllers,
            initial_active_controller=initial_active_controller,
            # runtime params are loaded directly on the controller_manager below,
            # so no spawner -p file handoff is needed here.
            use_sim_time=use_sim_time,
            timeout=60,
        )

        # create_controller_spawners() returns the active spawner as its only
        # top-level Node (the inactive one is nested in an event handler). The
        # gate must not open until that spawner has exited, i.e. until the
        # requested controller is actually active.
        spawner_nodes = [action for action in controller_spawners if isinstance(action, Node)]
        if len(spawner_nodes) != 1:
            raise RuntimeError(
                'expected exactly one top-level spawner Node from '
                f'create_controller_spawners(), got {len(spawner_nodes)}'
            )
        active_spawner = spawner_nodes[0]

        isaac_python = os.path.join(isaac_sim_path, 'python.sh')
        if not os.path.exists(isaac_python):
            raise RuntimeError(
                f"Isaac Sim interpreter not found at '{isaac_python}'. "
                "Pass isaac_sim_path:=<isaac sim install dir>."
            )
        if not os.path.exists(robot_usd):
            raise RuntimeError(
                f"Isaac robot USD not found at '{robot_usd}'.\nBuild it once with:\n"
                f"  {isaac_python} {os.path.join(isaac_path, 'isaac', 'convert_urdf_to_usd.py')} "
                f"--urdf {LaunchConfiguration('xacro_file').perform(context)} "
                f"--usd-path {os.path.dirname(robot_usd)} "
                f"--ros-package cho_description_franka:{description_path}"
            )

        isaac_cmd = [
            isaac_python,
            os.path.join(isaac_path, 'isaac', 'run_isaac_sim.py'),
            '--robot-usd', robot_usd,
            '--robot-profile', os.path.join(isaac_path, 'isaac', 'robots', 'fr3.json'),
            '--control-mode', mode,
            '--physics-rate', physics_rate,
            '--device', device,
        ]
        if headless.lower() == 'true':
            isaac_cmd.append('--headless')
        if publish_ft.lower() == 'true':
            isaac_cmd.append('--publish-ft')

        isaac_sim = ExecuteProcess(
            cmd=isaac_cmd,
            output='screen',
            # The environment is inherited, which is what makes ROS_DISTRO (so Isaac
            # binds the system Humble libraries rather than its bundled ones),
            # ROS_DOMAIN_ID and FASTRTPS_DEFAULT_PROFILES_FILE from dds/dds_mode.sh
            # reach the simulator.
            on_exit=Shutdown(),
        )

        node_ros2_control = Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                use_sim_time,
                robot_description,
                LaunchConfiguration('controllers_file'),
                runtime_param_file,
            ],
            remappings=[('~/robot_description', '/robot_description')],
            on_exit=Shutdown(),
        )

        isaac_command_gate = Node(
            package='cho_bringup_isaac',
            executable='isaac_command_gate.py',
            output='screen',
            parameters=[use_sim_time],
        )

        # Turns Isaac's raw geometry_msgs/Wrench into the Bota driver's stamped
        # topic and serves /bota_ft_sensor/tare, which the forge task trees call.
        isaac_ft_sensor = Node(
            package='cho_bringup_isaac',
            executable='isaac_ft_sensor.py',
            output='screen',
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('publish_ft')),
        )

        # The spawners must not run until Isaac is actually stepping.
        #
        # The controller_manager's realtime loop blocks in wait_until_started()
        # until the first /clock arrives, and Isaac needs tens of seconds to boot.
        # A spawner started before that gets the controller_manager's services
        # (they are up immediately) and then asks for a controller switch, which
        # only completes from inside the realtime loop - so it dies on the switch's
        # own 5 s timeout long before the simulator is ready:
        #     [controller_manager] Switch controller timed out after 5.000000 seconds!
        #     [spawner] Failed to activate controller : joint_state_broadcaster
        # --controller-manager-timeout does not help; it covers waiting for the
        # services, not the switch. So key off the simulator announcing itself.
        started = {'spawners': False}

        def start_spawners_when_isaac_is_ready(event):
            if started['spawners']:
                return None
            if ISAAC_READY_MARKER not in event.text.decode(errors='replace'):
                return None
            started['spawners'] = True
            return controller_spawners

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
                    on_exit=[isaac_command_gate],
                )
            ),
            RegisterEventHandler(
                event_handler=OnShutdown(
                    on_shutdown=[create_runtime_param_cleanup(runtime_param_file)],
                )
            ),
        ]

        return [isaac_sim, node_ros2_control, isaac_ft_sensor] + event_handlers

    return LaunchDescription(
        declared_arguments + [
            mock_gripper,
            node_robot_state_publisher,
            OpaqueFunction(function=setup_control_environment)
        ]
    )
