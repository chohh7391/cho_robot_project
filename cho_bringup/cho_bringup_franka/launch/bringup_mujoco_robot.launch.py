import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

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


def generate_launch_description():

    description_path = os.path.join(get_package_share_directory('cho_description_franka'))
    bringup_path = os.path.join(get_package_share_directory('cho_bringup_franka'))

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
            default_value='mujoco',
            description='Global bringup type injected to all controllers'
        ),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=os.path.join(description_path, 'urdf', 'fr3_with_ft_sensor', 'fr3_franka_hand.urdf'),
            description='Xacro/URDF file used to build the MuJoCo robot_description'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml'),
            description='Controller YAML file loaded by mujoco_ros2_control'
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='fr3_hand_tcp',
            description='Name of End-Effector',
            choices=['fr3_link7', 'fr3_hand', 'fr3_hand_tcp']
        )
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                LaunchConfiguration('xacro_file'),
                ' control_mode:=',
                LaunchConfiguration('control_mode'),
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
            # runtime params are loaded directly on node_mujoco_ros2_control
            # below, so no spawner -p file handoff is needed here.
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
                LaunchConfiguration('controllers_file'),
                runtime_param_file,
            ],
            remappings=[('~/robot_description', '/robot_description')],
            on_exit=Shutdown(),
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
