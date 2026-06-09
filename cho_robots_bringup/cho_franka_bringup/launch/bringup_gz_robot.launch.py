import importlib.util
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    pkg_description = get_package_share_directory('cho_franka_description')
    pkg_bringup = get_package_share_directory('cho_franka_bringup')
    
    # 1. Launch Arguments
    declared_arguments = [
        DeclareLaunchArgument('load_gripper', default_value='true'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand'),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('vla', default_value='false'),
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            description='Choose control mode: position, torque',
            choices=['position', 'torque']
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='task_space_impedance_controller',
            description='Which controller to activate initially'
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
            default_value='fr3_hand_tcp',
            description='Name of End-Effector',
            choices=['fr3_link7', 'fr3_hand', 'fr3_hand_tcp']
        )
    ]

    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # 2. Gazebo / Rviz / Bridge Nodes
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(pkg_description)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # 하드코딩된 문자열 대신 LaunchConfiguration 리스트로 연결
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
        parameters=[use_sim_time]
    )

    # 3. OpaqueFunction: 모드에 따른 동적 설정
    def launch_setup(context: LaunchContext, *args, **kwargs):
        robot_type_str = LaunchConfiguration('robot_type').perform(context)
        load_gripper_str = LaunchConfiguration('load_gripper').perform(context)
        franka_hand_str = LaunchConfiguration('franka_hand').perform(context)
        use_vla = LaunchConfiguration('vla').perform(context)
        mode = LaunchConfiguration('control_mode').perform(context)
        
        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        b_type = LaunchConfiguration('bringup_type').perform(context)
        ee_name = LaunchConfiguration('ee_name').perform(context)

        # --- Xacro 파싱 ---
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
                'special_connection': 'ft_sensor',
                'xyz_ee': '0 0 0',
            }
        )
        
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(doc.toxml(), value_type=str), 'use_sim_time': True}]
        )

        # --- Gazebo Spawner ---
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'franka_robot'],
            output='screen',
        )

        initial_active_controller = get_initial_active_controller(ctrl_name, use_vla)
        switchable_controllers = get_switchable_controllers(
            control_mode=mode,
            use_vla=use_vla,
            requested_controller=ctrl_name,
            extra_torque_controllers=['joint_trajectory_controller'],
        )
        all_runtime_param_controllers = (
            ALWAYS_ACTIVE_CONTROLLERS + switchable_controllers
        )
        payload_config_path = os.path.join(pkg_bringup, 'config', 'payload.yaml')
        runtime_param_file = create_runtime_param_file(
            payload_config_path=payload_config_path,
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
            use_sim_time=use_sim_time,
        )

        mock_gripper = Node(
            package='cho_franka_bringup',
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
            output='screen'
        )

        delayed_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=controller_spawners + [mock_gripper],
            )
        )
        cleanup_runtime_param = RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[create_runtime_param_cleanup(runtime_param_file)],
            )
        )

        return [
            robot_state_publisher,
            spawn_robot,
            delayed_controller_spawner,
            cleanup_runtime_param,
        ]

    return LaunchDescription(
        declared_arguments + [
            gazebo_sim,
            clock_bridge,
            rviz,
            OpaqueFunction(function=launch_setup)
        ]
    )
