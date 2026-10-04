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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cho_bringup_common import (
    check_isaac_install,
    create_controller_spawners,
    DEFAULT_ISAAC_SIM_PATH,
    isaac_command_gate,
    isaac_controller_startup,
    isaac_sim_command,
    isaac_sim_process,
    load_package_utils,
    runtime_param_cleanup,
    top_level_spawner,
)

launch_utils = load_package_utils('cho_bringup_openarm')


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
            'isaac_sim_path', default_value=DEFAULT_ISAAC_SIM_PATH,
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
        controller_spawners = create_controller_spawners(
            always_active=always_active,
            switchable_controllers=switchable_controllers,
            initial_active_controllers=launch_utils.per_arm(ctrl_name, bimanual),
            use_sim_time=use_sim_time,
            timeout=60,
        )
        # The gate must not open until this spawner has exited, i.e. until the
        # requested controller is actually active.
        active_spawner = top_level_spawner(controller_spawners)

        # The description bolts the arm (and the bimanual torso) to a `world`
        # link, which confuses the importer's articulation-root resolution -
        # fix_base already anchors it - so the build command strips it (see
        # cho_description_openarm/usd/README.md). One --strip-links: it takes a
        # single regex, and a second one replaces the first rather than adding to it.
        isaac_python = check_isaac_install(isaac_sim_path, robot_usd, [
            '--urdf', xacro_file,
            '--xacro-arg', 'hardware:=isaac',
            '--xacro-arg', f'bimanual:={str(bimanual).lower()}',
            '--strip-links', '^world$',
            '--usd-path', os.path.dirname(robot_usd),
            '--ros-package', f'cho_description_openarm:{description_path}',
        ])
        isaac_sim = isaac_sim_process(
            isaac_sim_command(
                isaac_python, robot_usd, profile, mode, physics_rate, device,
                headless=headless.lower() == 'true',
                extra_args=['--physics-engine', physics_engine]),
            # The runner resolves the Newton experience file out of the install
            # directory, which only the launch knows.
            additional_env={'ISAAC_SIM_PATH': isaac_sim_path},
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

        # Spawners once Isaac is stepping; the command gate once the requested
        # controller is active (see isaac_controller_startup for both reasons).
        return [isaac_sim, node_ros2_control] + isaac_controller_startup(
            isaac_sim, controller_spawners, active_spawner, isaac_command_gate(use_sim_time)
        ) + [runtime_param_cleanup(runtime_param_file)]

    return LaunchDescription(
        declared_arguments + [
            node_robot_state_publisher,
            OpaqueFunction(function=setup_control_environment),
        ]
    )
