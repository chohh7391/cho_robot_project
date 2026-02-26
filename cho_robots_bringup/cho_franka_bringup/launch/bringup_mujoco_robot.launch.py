import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 설정 및 인자 선언
    pkg_description = FindPackageShare('cho_franka_description')
    pkg_bringup = FindPackageShare('cho_franka_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo/MuJoCo) clock if true'
    )

    # 2. 로봇 설명 (URDF/Xacro)
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_description, "urdf", "fr3", "fr3_franka_hand.urdf"]),
            # PathJoinSubstitution([pkg_description, "urdf", "fr3_with_ft_sensor", "fr3_franka_hand.urdf"]),
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # 3. 노드 정의
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, # ros2_control_node에도 description이 필요한 경우가 많음
            PathJoinSubstitution([pkg_bringup, 'config', 'mujoco', 'controllers.yaml']),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

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

    spawner_nodes = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'] + (['--inactive'] if not active else []),
            parameters=[{"use_sim_time": use_sim_time}]
        ) for name, active in controllers_to_spawn
    ]

    # 5. 이벤트 핸들러 (그룹화하여 관리)
    delayed_actions = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=spawner_nodes + [
                Node(
                    package='cho_franka_bringup',
                    executable='mock_franka_gripper.py',
                    parameters=[{"use_sim_time": use_sim_time}]
                )
            ]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        control_node,
        delayed_actions,
        # RViz 등은 병렬로 띄워도 무방함
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['--display-config', PathJoinSubstitution([pkg_description, 'rviz', 'visualize_franka.rviz']), '-f', 'base'],
            parameters=[{"use_sim_time": use_sim_time}]
        ),
    ])