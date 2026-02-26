import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def render_xacro(context: LaunchContext, robot_type, load_gripper, franka_hand):
    # Substitution 객체를 문자열로 변환
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    xacro_path = os.path.join(
        get_package_share_directory('cho_franka_description'),
        'robots', robot_type_str, f'{robot_type_str}.urdf.xacro'
    )

    # xacro 프로세싱
    doc = xacro.process_file(
        xacro_path,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true', # if you want position control mode, change to false
            # 'special_connection': 'ft_sensor',
            # 'xyz_ee': "0 0 0.038",
        }
    )
    
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(doc.toxml(), value_type=str), 'use_sim_time': True}]
    )]

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
    ]

    # Substitutions
    ns = LaunchConfiguration('namespace')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # 2. Nodes & Actions
    # Gazebo 실행
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(pkg_description)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot Spawner (Gazebo에 개체 생성)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'franka_robot'],
        output='screen',
    )

    # Clock Bridge (Gazebo -> ROS 2 시간 동기화)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[use_sim_time],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'visualize_franka.rviz')],
        parameters=[use_sim_time]
    )

    # 3. Controllers Setup
    controllers_config = PathJoinSubstitution([pkg_bringup, 'config', 'gazebo', 'controllers.yaml'])

    controllers_to_spawn = [
        # defualt controllers
        ('joint_state_broadcaster', True),
        ('simulation_gripper_controller', True),
        ('gripper_controller', True),
        # choose one controller
        # ('ik_controller', True), # position controller
        # ('gravity_compensation_controller', False), # effort controller
        ('joint_space_impedance_controller', False), # effort controller
        # ('task_space_impedance_controller', False), # effort controller
        ('operational_space_controller', True), # effort controller
        # ('joint_space_qp_controller', False), # effort controller
        # ('task_space_qp_controller', False), # effort controller
        # ('vla_controller', False), # position/effort controller
    ]

    # 리스트 컴프리헨션으로 Spawner 노드들을 한 번에 생성
    spawner_nodes = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name] + (['--inactive'] if not active else []),
            parameters=[controllers_config, use_sim_time],
        ) for name, active in controllers_to_spawn
    ]

    # Mock Gripper
    mock_gripper = Node(
        package='cho_franka_bringup',
        executable='mock_franka_gripper.py',
        parameters=[use_sim_time],
        output='screen'
    )

    # 4. Event Handler (로봇 스폰이 완료된 후 컨트롤러와 Mock 노드 실행)
    delayed_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=spawner_nodes + [mock_gripper],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            gazebo_sim,
            clock_bridge,
            rviz,
            # OpaqueFunction은 리스트에 직접 추가
            OpaqueFunction(function=render_xacro, args=[
                LaunchConfiguration('robot_type'), 
                LaunchConfiguration('load_gripper'), 
                LaunchConfiguration('franka_hand')
            ]),
            spawn_robot,
            delayed_controller_spawner
        ]
    )