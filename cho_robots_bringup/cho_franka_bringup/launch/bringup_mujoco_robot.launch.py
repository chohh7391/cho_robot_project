import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    description_path = os.path.join(get_package_share_directory('cho_franka_description'))
    bringup_path = os.path.join(get_package_share_directory('cho_franka_bringup'))

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='torque',
        description='Choose control mode: position, torque, or special',
        choices=['position', 'torque', 'special']
    )

    xacro_file = os.path.join(description_path, 'urdf', 'fr3', 'fr3_franka_hand.urdf')
    # xacro_file = os.path.join(description_path, 'urdf', 'fr3_with_ft_sensor', 'fr3_franka_hand.urdf')
    
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' control_mode:=', LaunchConfiguration('control_mode')]),
            value_type=str
        )
    }

    controller_config_file = os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': os.path.join(description_path, 'xml', 'fr3', 'scene.xml')},
            # {'mujoco_model_path': os.path.join(description_path, 'xml', 'fr3_with_ft_sensor', 'scene.xml')},
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    mock_gripper = Node(
        package='cho_franka_bringup',
        executable='mock_franka_gripper.py',
        parameters=[{'use_sim_time': True}]
    )

    # [최적화 2] ExecuteProcess 대신 ROS 2 표준 spawner 사용
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_simulation_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simulation_gripper_controller'],
        output='screen'
    )
    
    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    def setup_controllers_based_on_mode(context, *args, **kwargs):
        mode = LaunchConfiguration('control_mode').perform(context)

        position_controller_lists = [('ik_controller', True)]
        torque_controller_lists = [
            ('gravity_compensation_controller', False),
            ('joint_space_impedance_controller', False),
            ('task_space_impedance_controller', False),
            ('operational_space_controller', False),
            ('joint_space_qp_controller', False),
            ('task_space_qp_controller', True),
        ]
        special_controller_lists = [('vla_controller', True)]

        target_list = []
        if mode == 'position':
            target_list = position_controller_lists
        elif mode == 'torque':
            target_list = torque_controller_lists
        elif mode == 'special':
            target_list = special_controller_lists

        # [최적화 3] spawner를 사용한 동적 컨트롤러 로드
        load_target_controllers = [
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name] if active else [name, '--inactive'],
                output='screen'
            ) for name, active in target_list
        ]

        all_controllers_to_load = load_target_controllers + [load_simulation_gripper_controller, load_gripper_controller]

        return [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=all_controllers_to_load,
                )
            )
        ]

    return LaunchDescription([
        control_mode_arg,
        node_mujoco_ros2_control,
        mock_gripper,
        node_robot_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        OpaqueFunction(function=setup_controllers_based_on_mode)
    ])