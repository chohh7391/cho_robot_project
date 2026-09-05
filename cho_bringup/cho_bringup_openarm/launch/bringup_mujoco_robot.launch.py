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

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO, OnShutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

package_share = get_package_share_directory('cho_bringup_openarm')
# launch_utils is installed under lib/, not as an importable python package, so
# it is loaded by path the same way cho_bringup_franka does it.
launch_utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_openarm', 'utils', 'launch_utils.py')
)
spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

# Printed by the resource manager once the hardware component is fully up. The
# component name comes from our own ros2_control block, so it is stable.
HARDWARE_READY_MARKER = "Successful 'activate' of hardware 'OpenArmHardwareInterface'"


def generate_launch_description():
    description_path = get_package_share_directory('cho_description_openarm')
    bringup_path = get_package_share_directory('cho_bringup_openarm')

    declared_arguments = [
        DeclareLaunchArgument(
            'control_mode', default_value='torque', choices=['position', 'velocity', 'torque'],
            description='Command interface the arm controllers drive'),
        DeclareLaunchArgument(
            'mujoco_position_profile', default_value='standard',
            choices=['standard', 'moveit'],
            description='Position actuator tuning profile; moveit reduces consecutive-goal sag'),
        DeclareLaunchArgument(
            'mujoco_mit_prototype', default_value='false', choices=['true', 'false'],
            description='Opt in to the isolated MIT MuJoCo hardware wrapper. '
                        'The default standard backend is unchanged.'),
        DeclareLaunchArgument(
            'mujoco_mit_headless', default_value='false', choices=['true', 'false'],
            description='Run the opt-in MIT MuJoCo wrapper without its GUI'),
        DeclareLaunchArgument(
            'return_to_zero', default_value='true', choices=['true', 'false'],
            description='Ramp the selected MIT action controller to the bounded nominal-zero '
                        'joint posture before accepting actions. Set false to opt out.'),
        DeclareLaunchArgument(
            'mit_controller_name', default_value='joint_position_mit_controller',
            choices=['joint_position_mit_controller',
                     'joint_impedance_mit_controller',
                     'task_space_impedance_mit_controller',
                     'single_arm_follow_joint_trajectory_mit_controller',
                     'bimanual_follow_joint_trajectory_mit_controller'],
            description='MIT producer for the opt-in prototype. The single-arm FJT '
                        'producer operates one selected arm of a bimanual model.'),
        DeclareLaunchArgument(
            'mit_arm', default_value='both_independent',
            choices=['left', 'right', 'both_independent', 'both'],
            description='Direct MIT bimanual ownership: left, right, or two independent '
                        '7-axis instances (both_independent). `both` is reserved exclusively '
                        'for the paired 14-axis MoveIt FJT.'),
        DeclareLaunchArgument(
            'controller_name', default_value='joint_space_impedance_controller',
            description='Controller class to activate initially. On a bimanual '
                        'build this names the class; one instance per arm is spawned.'),
        DeclareLaunchArgument(
            'bimanual', default_value='false', choices=['true', 'false'],
            description='Bring up the two-arm torso instead of a single arm'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use MuJoCo simulation time'),
        DeclareLaunchArgument(
            'bringup_type', default_value='mujoco',
            description='Injected into every controller; selects environment-specific behaviour'),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=os.path.join(
                description_path, 'robots', 'openarm_v10', 'openarm_v10.urdf.xacro'),
            description='Description entry point'),
        DeclareLaunchArgument(
            'controllers_file', default_value='',
            description='Controller YAML loaded by mujoco_ros2_control. Empty '
                        'selects controllers.yaml or controllers_bimanual.yaml '
                        'to match the bimanual argument.'),
        DeclareLaunchArgument(
            'ee_name', default_value='',
            description='End-effector frame for the single-arm build. Empty uses '
                        'openarm_hand_tcp; ignored when bimanual, where each arm '
                        'sets its own ee_name in the controllers file.'),
        DeclareLaunchArgument(
            'hand', default_value='true', choices=['true', 'false'],
            description='Include the parallel-link hand and spawn a gripper controller for '
                        'it. The controller is robot-independent and speaks metres of '
                        'opening width, so the same Gripper action and ~/width_command '
                        'topic drive it here and on hardware.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', description='Start RViz'),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', LaunchConfiguration('xacro_file'),
                ' hardware:=mujoco',
                ' control_mode:=', LaunchConfiguration('control_mode'),
                ' mujoco_position_profile:=', LaunchConfiguration('mujoco_position_profile'),
                ' mujoco_mit_prototype:=', LaunchConfiguration('mujoco_mit_prototype'),
                ' mujoco_mit_headless:=', LaunchConfiguration('mujoco_mit_headless'),
                ' bimanual:=', LaunchConfiguration('bimanual'),
                ' hand:=', LaunchConfiguration('hand'),
            ]),
            value_type=str,
        )
    }
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[use_sim_time, robot_description],
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    def setup_control_environment(context, *args, **kwargs):
        del args, kwargs
        mode = LaunchConfiguration('control_mode').perform(context)
        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        bringup_type = LaunchConfiguration('bringup_type').perform(context)
        bimanual = launch_utils.as_bool(LaunchConfiguration('bimanual').perform(context))
        mit_prototype = launch_utils.as_bool(
            LaunchConfiguration('mujoco_mit_prototype').perform(context))
        canonical_xacro = os.path.join(
            description_path, 'robots', 'openarm_v10', 'openarm_v10.urdf.xacro')
        launch_utils.enforce_mujoco_mit_description(
            mit_prototype, LaunchConfiguration('xacro_file').perform(context), canonical_xacro)
        mit_controller_base = LaunchConfiguration('mit_controller_name').perform(context)
        mit_arm = LaunchConfiguration('mit_arm').perform(context)
        return_to_zero = launch_utils.as_bool(
            LaunchConfiguration('return_to_zero').perform(context))
        requested_controllers_file = LaunchConfiguration('controllers_file').perform(context)
        mit_selection = launch_utils.resolve_mujoco_mit_selection(
            mit_prototype, mode, bimanual, mit_controller_base, mit_arm,
            requested_controllers_file)
        # The default MuJoCo backend does not expose the direct MIT action
        # contract, so the default-on initialization applies only after the
        # caller has opted into that backend.  This keeps the legacy default
        # launch usable while direct MIT controllers initialize by default.
        if return_to_zero and mit_prototype:
            if mit_controller_base not in launch_utils.RETURN_TO_ZERO_MIT_CONTROLLERS:
                raise RuntimeError(
                    "return_to_zero is supported only with joint_impedance_mit_controller "
                    "or task_space_impedance_mit_controller.")
        if ctrl_name == 'moveit':
            raise RuntimeError(
                "'moveit' is not a ros2_control controller. Launch "
                "bringup_mujoco_moveit.launch.py instead.")
        # Bimanual gives each arm its own ee_name in the controllers file, so the
        # runtime override has to stay out of the way there.
        ee_name = (LaunchConfiguration('ee_name').perform(context)
                   or ('' if bimanual else 'openarm_hand_tcp'))

        hand = launch_utils.as_bool(LaunchConfiguration('hand').perform(context))
        always_active = launch_utils.always_active_controllers(bimanual)
        # Its own spawner, after the arm: a hand that fails to load must not be
        # able to stop the arm controller from ever being spawned.
        optional = launch_utils.gripper_controllers(bimanual) if hand else []
        mit_controller_names = (mit_selection['controller_names'] if mit_selection else [])
        switchable_controllers = (mit_controller_names if mit_prototype else
            launch_utils.get_switchable_controllers(
                control_mode=mode, requested_controller=ctrl_name, bimanual=bimanual))

        mit_profile_overrides = None
        if mit_prototype:
            mit_profile_overrides = {}
            selection_overrides = mit_selection['controller_overrides']
            for name in mit_controller_names:
                mit_profile_overrides[name] = {
                    'safety_profile_file': os.path.join(
                        description_path, 'config', 'mit_safety_profiles_v1.yaml'),
                    **selection_overrides.get(name, {}),
                }
                if return_to_zero and mit_prototype:
                    # The direct producer owns the entire trajectory after its
                    # hardware/session handshake, so no second controller can
                    # race to command these interfaces.
                    mit_profile_overrides[name].update({
                        'return_to_zero': True,
                        'return_to_zero_duration': 5.0,
                    })
                    if mit_controller_base == 'task_space_impedance_mit_controller':
                        # A distinct joint-space initialization phase.  The
                        # task controller only enables Cartesian actions after
                        # this phase has converged, then resumes its ordinary
                        # task gains (no second posture ramp).
                        mit_profile_overrides[name].update({
                            'startup_kp': [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0],
                            'startup_kd': [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
                            'startup_duration': 5.0,
                        })
                    else:
                        mit_profile_overrides[name].update({
                            'return_to_zero_kp': [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0],
                            'return_to_zero_kd': [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
                        })

        runtime_param_file = launch_utils.create_runtime_param_file(
            controller_names=always_active + optional + switchable_controllers,
            bringup_type=bringup_type,
            control_mode=mode,
            ee_name=ee_name,
            controller_overrides=mit_profile_overrides,
        )
        controllers_file = requested_controllers_file or os.path.join(
            bringup_path, 'config', 'mujoco',
            ((mit_selection['controllers_file'])
             if mit_prototype else
             ('controllers_bimanual.yaml' if bimanual else 'controllers.yaml')))

        controller_spawners = launch_utils.create_controller_spawners(
            always_active=always_active,
            optional_controllers=optional,
            switchable_controllers=switchable_controllers,
            initial_active_controllers=(mit_controller_names if mit_prototype else
                                        launch_utils.per_arm(ctrl_name, bimanual)),
            # The runtime params are loaded directly on the node below, so no
            # spawner '-p' handoff is needed.
            use_sim_time=use_sim_time,
            timeout=60,
        )

        node_mujoco_ros2_control = Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                use_sim_time,
                robot_description,
                controllers_file,
                runtime_param_file,
            ],
            remappings=[('~/robot_description', '/robot_description')],
            on_exit=Shutdown(),
        )

        # The spawners must not run until the hardware is actually up.
        #
        # mujoco_ros2_control brings the controller_manager services up
        # immediately, but then spends seconds compiling the MJCF and its meshes
        # before the resource manager has a single interface. A spawner started
        # on process-start reaches configure_controller during that window and
        # dies outright:
        #     [spawner] Failed to configure controller
        # --controller-manager-timeout does not help; it covers waiting for the
        # services, not for the hardware behind them. So key off the resource
        # manager announcing this hardware component active. Same approach as the
        # Franka Isaac bringup, which hits the same race for a different reason.
        started = {'spawners': False}

        def start_spawners_when_hardware_is_active(event):
            if started['spawners']:
                return None
            if HARDWARE_READY_MARKER not in event.text.decode(errors='replace'):
                return None
            started['spawners'] = True
            return controller_spawners

        actions = [
            node_mujoco_ros2_control,
            RegisterEventHandler(
                event_handler=OnProcessIO(
                    target_action=node_mujoco_ros2_control,
                    # ROS 2 C++ logging goes to stderr; watch both so this does
                    # not silently stop working if that ever changes.
                    on_stdout=start_spawners_when_hardware_is_active,
                    on_stderr=start_spawners_when_hardware_is_active,
                )
            ),
            RegisterEventHandler(
                event_handler=OnShutdown(
                    on_shutdown=[launch_utils.create_runtime_param_cleanup(runtime_param_file)],
                )
            ),
        ]
        return actions

    return LaunchDescription(
        declared_arguments + [
            node_robot_state_publisher,
            node_rviz,
            OpaqueFunction(function=setup_control_environment),
        ]
    )
