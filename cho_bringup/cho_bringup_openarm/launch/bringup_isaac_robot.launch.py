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

"""Bring up OpenArm v1.0 in Isaac Sim, same interface as the other environments.

    ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py \
         control_mode:=torque controller_name:=joint_space_impedance_controller
    ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py bimanual:=true

Unlike MuJoCo, where the simulator lives inside the hardware component, Isaac Sim
runs in its own process under its own Python interpreter. This launch therefore
starts three things and orders them:

  1. cho_simulation_isaac's run_isaac_sim.py under Isaac's python.sh, driven by this
     bringup package's robot_profile.json (or robot_profile_bimanual.json) - physics, the
     OmniGraph ROS 2 bridge, /clock and /isaac_joint_states.
  2. a controller_manager whose hardware plugin is
     topic_based_ros2_control/TopicBasedSystem (the IsaacArmSystem /
     IsaacHandSystem blocks in the description). It is the mujoco_ros2_control
     build of ros2_control_node: that binary contains no MuJoCo code, it is
     upstream's node plus wait_until_started() on the clock and sim-time pacing,
     which is exactly what a sim-clocked controller_manager needs.
  3. the controller spawners, then cho_simulation_isaac's isaac_command_gate.py once
     the requested controller is active - see that script for why the gate exists.

Build the USD asset once before the first run:
    ~/isaacsim/python.sh <cho_simulation_isaac share>/isaac/convert_urdf_to_usd.py --help
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
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

package_share = get_package_share_directory('cho_bringup_openarm')
launch_utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_openarm', 'utils', 'launch_utils.py')
)
spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

# Printed by run_isaac_sim.py once physics is stepping and the ROS 2 bridge is
# publishing. The spawners key off it; see setup_control_environment.
ISAAC_READY_MARKER = '[isaac_sim] running:'


def generate_launch_description():
    description_path = get_package_share_directory('cho_description_openarm')
    bringup_path = get_package_share_directory('cho_bringup_openarm')
    isaac_path = get_package_share_directory('cho_simulation_isaac')

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
            description='Isaac publishes /clock; everything downstream must follow it'),
        DeclareLaunchArgument(
            'bringup_type', default_value='isaac',
            description='Injected into every controller'),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=os.path.join(
                description_path, 'robots', 'openarm_v10', 'openarm_v10.urdf.xacro'),
            description='Description entry point'),
        DeclareLaunchArgument(
            'controllers_file', default_value='',
            description='Controller YAML loaded by the controller_manager. Empty '
                        'selects controllers.yaml or controllers_bimanual.yaml to '
                        'match the bimanual argument.'),
        DeclareLaunchArgument(
            'ee_name', default_value='',
            description='End-effector frame for the single-arm build. Empty uses '
                        'openarm_hand_tcp; ignored when bimanual, where each arm '
                        'sets its own ee_name in the controllers file.'),
        DeclareLaunchArgument(
            'robot_usd',
            default_value='',
            description='USD asset for Isaac. Empty picks the single-arm or '
                        'bimanual asset to match the bimanual argument. Build it '
                        'once with cho_simulation_isaac/isaac/convert_urdf_to_usd.py; '
                        'it is not tracked in git. The layout is the URDF '
                        "importer's own convention: it derives the subdirectory "
                        'and the .usda filename from the URDF basename.'),
        DeclareLaunchArgument(
            'isaac_sim_path', default_value=os.path.expanduser('~/isaacsim'),
            description='Isaac Sim install directory (the one holding python.sh)'),
        DeclareLaunchArgument(
            'physics_rate', default_value='250.0',
            description='Isaac physics rate. MUST equal controller_manager update_rate '
                        'in the controllers file: the manager is paced by /clock.'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run Isaac without a viewport'),
        DeclareLaunchArgument(
            'device', default_value='cpu', choices=['cpu', 'cuda'],
            description='Isaac physics device'),
        DeclareLaunchArgument(
            'physics_engine', default_value='physx', choices=['physx', 'newton'],
            description='Isaac physics backend. Both are fully supported: torque, '
                        'position and velocity, single arm and bimanual, on the '
                        'same gains and with the same measured error. newton '
                        'boots the isaacsim.exp.full.newton Kit experience and '
                        'selects the asset\'s "physics" Physics variant (NOT the '
                        '"mujoco" one its own auto-switch picks - that variant '
                        'carries no UsdPhysics.DriveAPI, so Newton installs no '
                        'position or velocity actuator at all and silently '
                        'ignores those targets). newton applies no Coulomb joint '
                        'friction and no effort limit, so it tracks a little '
                        'tighter than physx in torque mode and the controller\'s '
                        'own clip_torque is what bounds the command.'),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', LaunchConfiguration('xacro_file'),
                ' hardware:=isaac',
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

    def setup_control_environment(context, *args, **kwargs):
        del args, kwargs
        mode = LaunchConfiguration('control_mode').perform(context)
        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        bringup_type = LaunchConfiguration('bringup_type').perform(context)
        ee_name = LaunchConfiguration('ee_name').perform(context)
        isaac_sim_path = LaunchConfiguration('isaac_sim_path').perform(context)
        bimanual = launch_utils.as_bool(LaunchConfiguration('bimanual').perform(context))
        variant = 'openarm_v10_bimanual' if bimanual else 'openarm_v10'
        robot_usd = LaunchConfiguration('robot_usd').perform(context) or os.path.join(
            description_path, 'usd', variant, variant, f'{variant}.usda')
        physics_rate = LaunchConfiguration('physics_rate').perform(context)
        headless = LaunchConfiguration('headless').perform(context)
        device = LaunchConfiguration('device').perform(context)
        physics_engine = LaunchConfiguration('physics_engine').perform(context)
        xacro_file = LaunchConfiguration('xacro_file').perform(context)

        # Bimanual gives each arm its own ee_name in the controllers file, so the
        # runtime override has to stay out of the way there.
        ee_name = ee_name or ('' if bimanual else 'openarm_hand_tcp')
        controllers_file = LaunchConfiguration('controllers_file').perform(context) or os.path.join(
            bringup_path, 'config', 'isaac',
            'controllers_bimanual.yaml' if bimanual else 'controllers.yaml')
        profile = os.path.join(
            bringup_path, 'config', 'isaac',
            'robot_profile_bimanual.json' if bimanual else 'robot_profile.json')

        always_active = launch_utils.always_active_controllers(bimanual)
        switchable_controllers = launch_utils.get_switchable_controllers(
            control_mode=mode, requested_controller=ctrl_name, bimanual=bimanual)
        runtime_param_file = launch_utils.create_runtime_param_file(
            controller_names=always_active + switchable_controllers,
            bringup_type=bringup_type,
            control_mode=mode,
            ee_name=ee_name,
        )
        controller_spawners = launch_utils.create_controller_spawners(
            always_active=always_active,
            switchable_controllers=switchable_controllers,
            initial_active_controllers=launch_utils.per_arm(ctrl_name, bimanual),
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
                f'create_controller_spawners(), got {len(spawner_nodes)}')
        active_spawner = spawner_nodes[0]

        isaac_python = os.path.join(isaac_sim_path, 'python.sh')
        if not os.path.exists(isaac_python):
            raise RuntimeError(
                f"Isaac Sim interpreter not found at '{isaac_python}'. "
                'Pass isaac_sim_path:=<isaac sim install dir>.')
        if not os.path.exists(robot_usd):
            raise RuntimeError(
                f"Isaac robot USD not found at '{robot_usd}'.\nBuild it once with:\n"
                f"  {isaac_python} {os.path.join(isaac_path, 'isaac', 'convert_urdf_to_usd.py')}"
                f' --urdf {xacro_file}'
                f' --xacro-arg hardware:=isaac --xacro-arg bimanual:={str(bimanual).lower()}'
                ' --strip-links "^world$"'
                ' --strip-links ""'
                f' --usd-path {os.path.dirname(robot_usd)}'
                f' --ros-package cho_description_openarm:{description_path}')

        isaac_cmd = [
            isaac_python,
            os.path.join(isaac_path, 'isaac', 'run_isaac_sim.py'),
            '--robot-usd', robot_usd,
            '--robot-profile', profile,
            '--control-mode', mode,
            '--physics-rate', physics_rate,
            '--device', device,
            '--physics-engine', physics_engine,
        ]
        if headless.lower() == 'true':
            isaac_cmd.append('--headless')

        isaac_sim = ExecuteProcess(
            cmd=isaac_cmd,
            output='screen',
            # The runner resolves the Newton experience file out of the install
            # directory, which only the launch knows.
            additional_env={'ISAAC_SIM_PATH': isaac_sim_path},
            # The environment is inherited, which is what makes ROS_DISTRO (so
            # Isaac binds the system Humble libraries rather than its bundled
            # ones), ROS_DOMAIN_ID and FASTRTPS_DEFAULT_PROFILES_FILE reach the
            # simulator.
            on_exit=Shutdown(),
        )

        node_ros2_control = Node(
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

        isaac_command_gate = Node(
            package='cho_simulation_isaac',
            executable='isaac_command_gate.py',
            output='screen',
            parameters=[use_sim_time],
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

        return [
            isaac_sim,
            node_ros2_control,
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
                    on_shutdown=[launch_utils.create_runtime_param_cleanup(runtime_param_file)],
                )
            ),
        ]

    return LaunchDescription(
        declared_arguments + [
            node_robot_state_publisher,
            OpaqueFunction(function=setup_control_environment),
        ]
    )
