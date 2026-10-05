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

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cho_bringup_common import (
    as_bool,
    create_controller_spawners,
    load_package_utils,
    prepend_to_search_paths,
    runtime_param_cleanup,
)

launch_utils = load_package_utils('cho_bringup_franka')


def generate_launch_description():
    pkg_description = get_package_share_directory('cho_description_franka')
    pkg_bringup = get_package_share_directory('cho_bringup_franka')

    # The Franka Gazebo ros2_control plugin is installed outside Gazebo's
    # default system-plugin search path in some overlay layouts.  Resolve the
    # active installation instead of assuming a workspace location, and keep
    # any caller-provided search paths after it.
    franka_plugin_path = os.path.join(
        get_package_prefix('franka_ign_ros2_control'), 'lib'
    )
    plugin_path_actions = prepend_to_search_paths(
        ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH'], franka_plugin_path)

    # 1. Launch Arguments
    declared_arguments = [
        DeclareLaunchArgument('load_gripper', default_value='true'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand'),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Must stay empty: the controller_manager inside the Gazebo plugin '
                        'is not namespaced, so a non-empty value is refused.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('vla', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='franka_robot'),
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            # NOTE: 'velocity' is accepted (vla_controller supports it and the URDF already
            # declares the command_interface -- verified working via MuJoCo and, at the code
            # level, on real hardware) but does NOT currently work in Gazebo: velocity
            # commands are computed correctly (confirmed via debug logging) but are not
            # applied by the physics engine -- the arm free-drifts under gravity for ~2s
            # after activation and then stops responding to any further velocity command.
            # Root cause traced to extern/franka_ros2/franka_gazebo/franka_ign_ros2_control
            # (vendored, not to be edited here) rather than to this project's own code.
            # Do not use control_mode:=velocity with Gazebo until that plugin is fixed/
            # replaced; use MuJoCo or real hardware instead.
            description=('Choose control mode: position, torque '
                         '(velocity is NOT functional in Gazebo -- see NOTE above)'),
            choices=['position', 'velocity', 'torque']
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='task_space_impedance_controller',
            description='Actual ros2_control controller to activate initially'
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='gazebo',
            description='Global bringup type injected to all controllers'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Name of the Gazebo world file to load (e.g., custom_world.sdf)'
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='',
            description="Controllers' end-effector frame; empty means fr3_hand_tcp with "
                        'the hand and fr3_link8 without it',
            choices=['', 'fr3_link7', 'fr3_link8', 'fr3_hand', 'fr3_hand_tcp']
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('load_moveit_controller', default_value='false'),
        DeclareLaunchArgument('load_ft_sensor', default_value='true'),
    ]

    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # 2. Gazebo / Rviz / Bridge Nodes
    # Gazebo resolves the description's model:// mesh URIs against the parent
    # of the package share directory.
    resource_path_actions = prepend_to_search_paths(
        ['GZ_SIM_RESOURCE_PATH'], os.path.dirname(pkg_description))
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # build gz_args from a LaunchConfiguration list instead of a hardcoded string
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[use_sim_time],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'visualize_franka.rviz')],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # 3. OpaqueFunction: mode-dependent dynamic setup
    def launch_setup(context: LaunchContext, *args, **kwargs):
        robot_type_str = LaunchConfiguration('robot_type').perform(context)
        load_gripper_str = LaunchConfiguration('load_gripper').perform(context)
        franka_hand_str = LaunchConfiguration('franka_hand').perform(context)
        namespace_str = LaunchConfiguration('namespace').perform(context)
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        # The controller_manager lives inside the Gazebo plugin, which this
        # description does not namespace, so the spawners, the clock bridge and
        # RViz all talk to the root namespace. A namespace used to reach only
        # robot_state_publisher and the create node, which moved /tf under it
        # and broke TF for everything else - any value was wrong. Refused
        # rather than ignored, so nobody believes the robot is namespaced.
        if namespace_str:
            raise RuntimeError(
                f"namespace:={namespace_str} is not supported in Gazebo: the "
                "controller_manager inside the Gazebo plugin is not namespaced, so a "
                "namespaced robot_state_publisher only moves /tf away from it. Leave "
                "namespace empty.")
        use_vla = LaunchConfiguration('vla').perform(context)
        requested_mode = LaunchConfiguration('control_mode').perform(context)

        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        b_type = LaunchConfiguration('bringup_type').perform(context)
        ee_name = launch_utils.resolve_ee_name(
            LaunchConfiguration('ee_name').perform(context), as_bool(load_gripper_str))
        if ctrl_name == 'moveit':
            raise RuntimeError(
                "'moveit' is not a ros2_control controller. Launch "
                "bringup_gz_moveit.launch.py instead.")
        mode = requested_mode
        requested_controller = ctrl_name
        launch_utils.check_controller_matches_mode(requested_controller, mode, use_vla)
        load_moveit_controller = as_bool(
            LaunchConfiguration('load_moveit_controller').perform(context))
        load_ft_sensor = as_bool(
            LaunchConfiguration('load_ft_sensor').perform(context))

        # Controller list + runtime param file: in Gazebo, controller_manager runs
        # inside the Gazebo plugin, so this file has to be injected via the URDF's
        # <parameters> tag -- built before xacro processing and passed in as a mapping.
        # Loaded as a node parameter the same way as real/mujoco, so no spawner -p
        # handoff is needed.
        initial_active_controller = launch_utils.get_initial_active_controller(
            requested_controller, use_vla)
        switchable_controllers = launch_utils.get_switchable_controllers(
            control_mode=mode,
            use_vla=use_vla,
            requested_controller=requested_controller,
            extra_torque_controllers=(
                ['moveit_joint_trajectory_controller'] if load_moveit_controller else []
            ),
        )
        # Position-mode MoveIt execution also needs the standard trajectory
        # controller loaded inactive until the planning-scene gate switches it.
        if (load_moveit_controller and
                'moveit_joint_trajectory_controller' not in switchable_controllers):
            switchable_controllers.append('moveit_joint_trajectory_controller')
        always_active_controllers = launch_utils.always_active_controllers(load_gripper_str)
        all_runtime_param_controllers = [
            controller for controller in always_active_controllers + switchable_controllers
            if controller != 'moveit_joint_trajectory_controller'
        ]
        payload_config_path = os.path.join(pkg_bringup, 'config', 'payload.yaml')
        runtime_param_file = launch_utils.create_runtime_param_file(
            payload_config_path=payload_config_path,
            controller_names=all_runtime_param_controllers,
            bringup_type=b_type,
            control_mode=mode,
            ee_name=ee_name,
        )

        # --- Xacro processing ---
        gazebo_effort_str = 'false' if mode == 'position' else 'true'

        xacro_path = os.path.join(
            pkg_description,
            'robots', robot_type_str, f'{robot_type_str}.urdf.xacro'
        )

        doc = xacro.process_file(
            xacro_path,
            mappings={
                'robot_type': robot_type_str,
                'hand': load_gripper_str,
                'ros2_control': 'true',
                'gazebo': 'true',
                'ee_id': franka_hand_str,
                'gazebo_effort': gazebo_effort_str,
                # A composition wrapper can omit this optional adapter when its
                # planning model is the arm plus hand only.
                'special_connection': 'ft_sensor' if load_ft_sensor else '',
                'xyz_ee': '0 0 0',
                'runtime_param_file': runtime_param_file,
            }
        )
        robot_description_content = doc.toxml()

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': ParameterValue(robot_description_content, value_type=str)},
                use_sim_time,
            ]
        )

        # --- Gazebo Spawner ---
        # The description itself, not '-topic robot_description': anything else
        # latching a URDF on that topic (a move_group's planning model has no
        # ros2_control block) could be what Gazebo spawns, without the plugin.
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', robot_description_content, '-name', robot_name_str],
            output='screen',
        )

        controller_spawners = create_controller_spawners(
            always_active=always_active_controllers,
            switchable_controllers=switchable_controllers,
            initial_active_controllers=initial_active_controller,
            # runtime params are injected into the Gazebo controller_manager via
            # the URDF <parameters> tag above, so no spawner -p handoff is needed.
            use_sim_time=use_sim_time,
            timeout=60,
        )

        mock_gripper = Node(
            package='cho_bringup_franka',
            executable='mock_franka_gripper.py',
            parameters=[
                use_sim_time,
                {
                    'command_mode': 'effort',
                    'max_effort': 12.0,
                    'effort_kp': 10.0,
                    'effort_kd': 3.0,
                    'position_deadband': 0.001,
                    'velocity_deadband': 0.002,
                },
            ],
            output='screen',
            condition=IfCondition(load_gripper_str),
        )

        delayed_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=controller_spawners + [mock_gripper],
            )
        )
        cleanup_runtime_param = runtime_param_cleanup(runtime_param_file)

        actions = [
            robot_state_publisher,
            spawn_robot,
            delayed_controller_spawner,
            cleanup_runtime_param,
        ]
        actions.append(rviz)
        return actions

    return LaunchDescription(
        declared_arguments + plugin_path_actions + resource_path_actions + [
            gazebo_sim,
            clock_bridge,
            OpaqueFunction(function=launch_setup)
        ]
    )
