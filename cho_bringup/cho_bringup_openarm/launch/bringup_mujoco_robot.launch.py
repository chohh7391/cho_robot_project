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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
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
            'use_rviz', default_value='false', description='Start RViz'),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', LaunchConfiguration('xacro_file'),
                ' hardware:=mujoco',
                ' control_mode:=', LaunchConfiguration('control_mode'),
                ' bimanual:=', LaunchConfiguration('bimanual'),
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
        # Bimanual gives each arm its own ee_name in the controllers file, so the
        # runtime override has to stay out of the way there.
        ee_name = (LaunchConfiguration('ee_name').perform(context)
                   or ('' if bimanual else 'openarm_hand_tcp'))

        always_active = launch_utils.always_active_controllers(bimanual)
        switchable_controllers = launch_utils.get_switchable_controllers(
            control_mode=mode, requested_controller=ctrl_name, bimanual=bimanual)

        runtime_param_file = launch_utils.create_runtime_param_file(
            controller_names=always_active + switchable_controllers,
            bringup_type=bringup_type,
            control_mode=mode,
            ee_name=ee_name,
        )
        controllers_file = LaunchConfiguration('controllers_file').perform(context) or os.path.join(
            bringup_path, 'config', 'mujoco',
            'controllers_bimanual.yaml' if bimanual else 'controllers.yaml')

        controller_spawners = launch_utils.create_controller_spawners(
            always_active=always_active,
            switchable_controllers=switchable_controllers,
            initial_active_controllers=launch_utils.per_arm(ctrl_name, bimanual),
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

        return [
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

    return LaunchDescription(
        declared_arguments + [
            node_robot_state_publisher,
            node_rviz,
            OpaqueFunction(function=setup_control_environment),
        ]
    )
