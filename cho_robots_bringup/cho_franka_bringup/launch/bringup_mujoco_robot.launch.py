import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

package_share = get_package_share_directory('cho_franka_bringup')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_franka_bringup', 'utils')
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


def generate_launch_description():

    description_path = os.path.join(get_package_share_directory('cho_franka_description'))
    bringup_path = os.path.join(get_package_share_directory('cho_franka_bringup'))

    declared_arguments = [
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            description='Choose control mode: position, torque'
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
            'bringup_type',
            default_value='mujoco',
            description='Global bringup type injected to all controllers'
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='fr3_hand_tcp',
            description='Name of End-Effector',
            choices=['fr3_link7', 'fr3_hand', 'fr3_hand_tcp']
        )
    ]

    # xacro_file = os.path.join(description_path, 'urdf', 'fr3', 'fr3_franka_hand.urdf')
    xacro_file = os.path.join(description_path, 'urdf', 'fr3_with_ft_sensor', 'fr3_franka_hand.urdf')
    
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' control_mode:=', LaunchConfiguration('control_mode')]),
            value_type=str
        )
    }

    controller_config_file = os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml')
    payload_config_file = os.path.join(bringup_path, 'config', 'payload.yaml')

    mock_gripper = Node(
        package='cho_franka_bringup',
        executable='mock_franka_gripper.py',
        parameters=[
            {
                'use_sim_time': True,
                'command_mode': 'position',
            }
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    def setup_control_environment(context, *args, **kwargs):
        mode = LaunchConfiguration('control_mode').perform(context)
        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        use_vla = LaunchConfiguration('vla').perform(context)
        b_type = LaunchConfiguration('bringup_type').perform(context)
        ee_name = LaunchConfiguration('ee_name').perform(context)

        initial_active_controller = get_initial_active_controller(ctrl_name, use_vla)
        switchable_controllers = get_switchable_controllers(
            control_mode=mode,
            use_vla=use_vla,
            requested_controller=ctrl_name,
        )
        all_runtime_param_controllers = (
            ALWAYS_ACTIVE_CONTROLLERS + switchable_controllers
        )
        runtime_param_file = create_runtime_param_file(
            payload_config_path=payload_config_file,
            controller_names=all_runtime_param_controllers,
            bringup_type=b_type,
            control_mode=mode,
            ee_name=ee_name,
        )
        controller_spawners = create_controller_spawners(
            always_active_controllers=ALWAYS_ACTIVE_CONTROLLERS,
            switchable_controllers=switchable_controllers,
            initial_active_controller=initial_active_controller,
            runtime_param_file=runtime_param_file,
            use_sim_time={'use_sim_time': True},
        )

        node_mujoco_ros2_control = Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                robot_description,
                controller_config_file,
                runtime_param_file,
            ],
            remappings=[('~/robot_description', '/robot_description')],
        )

        event_handlers = [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node_mujoco_ros2_control,
                    on_start=controller_spawners,
                )
            ),
            RegisterEventHandler(
                event_handler=OnShutdown(
                    on_shutdown=[create_runtime_param_cleanup(runtime_param_file)],
                )
            )
        ]

        return [node_mujoco_ros2_control] + event_handlers

    return LaunchDescription(
        declared_arguments + [
            mock_gripper,
            node_robot_state_publisher,
            OpaqueFunction(function=setup_control_environment)
        ]
    )
