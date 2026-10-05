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

"""
Bring up the Franka FR3 in Isaac Sim, same interface as the other environments.

    ros2 launch cho_bringup_franka bringup_isaac_robot.launch.py \
         control_mode:=torque controller_name:=task_space_qp_controller

Unlike Gazebo (controller_manager inside the sim plugin) and MuJoCo (simulator
inside the hardware component), Isaac Sim runs in its own process under its own
Python interpreter. This launch therefore starts three things and orders them:

  1. cho_simulation_isaac's run_isaac_sim.py under Isaac's python.sh -- physics, the OmniGraph
     ROS 2 bridge, /clock and /isaac_joint_states.
  2. a controller_manager whose hardware plugin is
     topic_based_ros2_control/TopicBasedSystem (declared in the URDF's
     IsaacArmSystem / IsaacHandSystem blocks). It is the mujoco_ros2_control
     build of ros2_control_node: that binary contains no MuJoCo code at all, it
     is upstream's node plus `wait_until_started()` on the clock and sim-time
     pacing, which is exactly what a sim-clocked controller_manager needs.
  3. the controller spawners, then cho_simulation_isaac's isaac_command_gate.py once the
     requested controller is active -- see that script for why the gate exists.

Before the first run, build the USD asset once:
    ~/isaacsim/python.sh <cho_simulation_isaac share>/isaac/convert_urdf_to_usd.py --help
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cho_bringup_common import (
    check_isaac_install,
    create_controller_spawners,
    DEFAULT_ISAAC_SIM_PATH,
    gate_failure_argument,
    isaac_command_gate,
    isaac_controller_startup,
    isaac_sim_command,
    isaac_sim_process,
    load_package_utils,
    runtime_param_cleanup,
    shutdown_on_gate_failure,
    top_level_spawner,
)

launch_utils = load_package_utils('cho_bringup_franka')


def generate_launch_description():

    description_path = get_package_share_directory('cho_description_franka')
    bringup_path = get_package_share_directory('cho_bringup_franka')

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
            default_value='',
            description="Controllers' end-effector frame; empty means fr3_hand_tcp with "
                        'the hand and fr3_link8 without it',
            choices=['', 'fr3_link7', 'fr3_link8', 'fr3_hand', 'fr3_hand_tcp']
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
        gate_failure_argument(),
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
        # The MuJoCo/Isaac descriptions carry the hand even without a gripper.
        ee_name = launch_utils.resolve_ee_name(
            LaunchConfiguration('ee_name').perform(context), True)
        isaac_sim_path = LaunchConfiguration('isaac_sim_path').perform(context)
        robot_usd = LaunchConfiguration('robot_usd').perform(context)
        physics_rate = LaunchConfiguration('physics_rate').perform(context)
        headless = LaunchConfiguration('headless').perform(context)
        device = LaunchConfiguration('device').perform(context)
        publish_ft = LaunchConfiguration('publish_ft').perform(context)

        launch_utils.check_controller_matches_mode(ctrl_name, mode, use_vla)
        always_active_controllers = launch_utils.always_active_controllers(load_gripper)

        initial_active_controller = launch_utils.get_initial_active_controller(ctrl_name, use_vla)
        switchable_controllers = launch_utils.get_switchable_controllers(
            control_mode=mode,
            use_vla=use_vla,
            requested_controller=ctrl_name,
        )
        all_runtime_param_controllers = (
            always_active_controllers + switchable_controllers
        )
        # Everything that can refuse the launch runs before the runtime
        # parameter file is written: a refusal raises out of here, and the
        # cleanup handler that would delete the file is never registered.
        shutdown_on_failure = shutdown_on_gate_failure(context)
        controller_spawners = create_controller_spawners(
            always_active=always_active_controllers,
            switchable_controllers=switchable_controllers,
            initial_active_controllers=initial_active_controller,
            # runtime params are loaded directly on the controller_manager below,
            # so no spawner -p file handoff is needed here.
            use_sim_time=use_sim_time,
            timeout=60,
        )
        # The gate must not open until this spawner has exited, i.e. until the
        # requested controller is actually active.
        active_spawner = top_level_spawner(controller_spawners)

        isaac_python = check_isaac_install(isaac_sim_path, robot_usd, [
            '--urdf', LaunchConfiguration('xacro_file').perform(context),
            '--usd-path', os.path.dirname(robot_usd),
            '--ros-package', f'cho_description_franka:{description_path}',
        ])
        isaac_cmd = isaac_sim_command(
            isaac_python, robot_usd,
            os.path.join(bringup_path, 'config', 'isaac', 'robot_profile.json'),
            mode, physics_rate, device, headless=headless.lower() == 'true')
        if publish_ft.lower() == 'true':
            isaac_cmd.append('--publish-ft')
        isaac_sim = isaac_sim_process(isaac_cmd)

        runtime_param_file = launch_utils.create_runtime_param_file(
            payload_config_path=payload_config_file,
            controller_names=all_runtime_param_controllers,
            bringup_type=b_type,
            control_mode=mode,
            ee_name=ee_name,
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

        # Turns Isaac's raw geometry_msgs/Wrench into the Bota driver's stamped
        # topic and serves /bota_ft_sensor/tare, which the forge task trees call.
        isaac_ft_sensor = Node(
            package='cho_simulation_isaac',
            executable='isaac_ft_sensor.py',
            output='screen',
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('publish_ft')),
        )

        # Spawners once Isaac is stepping; the command gate once the requested
        # controller is active (see isaac_controller_startup for both reasons).
        event_handlers = isaac_controller_startup(
            isaac_sim, controller_spawners, active_spawner, isaac_command_gate(use_sim_time),
            shutdown_on_failure=shutdown_on_failure)
        event_handlers.append(runtime_param_cleanup(runtime_param_file))

        return [isaac_sim, node_ros2_control, isaac_ft_sensor] + event_handlers

    return LaunchDescription(
        declared_arguments + [
            mock_gripper,
            node_robot_state_publisher,
            OpaqueFunction(function=setup_control_environment)
        ]
    )
