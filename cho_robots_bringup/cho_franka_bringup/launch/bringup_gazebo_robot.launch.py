import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

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
        # MuJoCo 코드와 동일하게 position, torque, special로 구분
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            description='Choose control mode: position, torque, or special',
            choices=['position', 'torque', 'special']
        ),
    ]

    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # 2. Gazebo / Rviz / Bridge Nodes
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(pkg_description)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
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
        mode = LaunchConfiguration('control_mode').perform(context)

        # --- Xacro 파싱 (모드에 따른 gazebo_effort 자동 설정) ---
        # position 모드일 때만 effort 제어를 끄고(false), 나머지는 켭니다(true)
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

        # --- 컨트롤러 그룹핑 (MuJoCo 코드와 동일한 구조) ---
        position_controller_lists = [('ik_controller', True)]
        
        torque_controller_lists = [
            ('gravity_compensation_controller', False),
            ('joint_space_impedance_controller', False),
            ('task_space_impedance_controller', False),
            ('operational_space_controller', True), # 필요에 따라 기본 Active 설정 변경 가능
            ('joint_space_qp_controller', False),
            ('task_space_qp_controller', False),
        ]
        
        special_controller_lists = [('vla_controller', True)]

        target_list = []
        if mode == 'position':
            target_list = position_controller_lists
        elif mode == 'torque':
            target_list = torque_controller_lists
        elif mode == 'special':
            target_list = special_controller_lists

        controllers_config = PathJoinSubstitution([pkg_bringup, 'config', 'gazebo', 'controllers.yaml'])

        # 선택된 모드의 컨트롤러들 스폰
        load_target_controllers = [
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name] if active else [name, '--inactive'],
                parameters=[controllers_config, use_sim_time],
                output='screen'
            ) for name, active in target_list
        ]

        # 항상 켜져야 하는 공통 컨트롤러들
        common_controllers = [
            Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], parameters=[controllers_config, use_sim_time], output='screen'),
            Node(package='controller_manager', executable='spawner', arguments=['simulation_gripper_controller'], parameters=[controllers_config, use_sim_time], output='screen'),
            Node(package='controller_manager', executable='spawner', arguments=['gripper_controller'], parameters=[controllers_config, use_sim_time], output='screen'),
        ]

        mock_gripper = Node(
            package='cho_franka_bringup',
            executable='mock_franka_gripper.py',
            parameters=[use_sim_time],
            output='screen'
        )

        # 로봇 스폰이 끝난 후 공통 컨트롤러 + 타겟 컨트롤러 + Mock 그리퍼 실행
        delayed_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=common_controllers + load_target_controllers + [mock_gripper],
            )
        )

        return [robot_state_publisher, spawn_robot, delayed_controller_spawner]

    return LaunchDescription(
        declared_arguments + [
            gazebo_sim,
            clock_bridge,
            rviz,
            OpaqueFunction(function=launch_setup)
        ]
    )