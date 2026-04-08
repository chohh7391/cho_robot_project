import importlib.util
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# constant for the controller name parameter
CONTROLLER_EXAMPLE = 'controller'

package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')

spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

load_yaml = launch_utils.load_yaml


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)
    nodes = []

    for _, config in configs.items():
        namespace = config['namespace']
        
        # 1. Franka 하드웨어 및 로봇 스테이트 퍼블리셔 (franka_bringup 기반)
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])
                ),
                launch_arguments={
                    'robot_type': str(config['robot_type']),
                    'arm_prefix': str(config['arm_prefix']),
                    'namespace': str(namespace),
                    'robot_ip': str(config['robot_ip']),
                    'load_gripper': str(config['load_gripper']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'fake_sensor_commands': str(config['fake_sensor_commands']),
                    'joint_state_rate': str(config['joint_state_rate']),
                    'controllers_yaml': PathJoinSubstitution([
                        FindPackageShare('cho_franka_bringup'), 'config', 'real', 'controllers.yaml'
                    ]),
                }.items(),
            )
        )

        # 공통 Spawner 파라미터 설정 (반복 줄이기 위함)
        common_params = [PathJoinSubstitution([
            FindPackageShare('cho_franka_bringup'), 'config', 'real', 'controllers.yaml'
        ])]

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

        # 리스트 컴프리헨션으로 Spawner 노드들을 생성하여 nodes 리스트에 추가
        nodes.extend([
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[name] + (['--inactive'] if not active else []) + ['--controller-manager-timeout', '30'],
                parameters=common_params,
                output='screen',
            ) for name, active in controllers_to_spawn
        ])

    # RViz 실행 로직
    if any(str(config.get('use_rviz', 'false')).lower() == 'true' for config in configs.values()):
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', PathJoinSubstitution([
                FindPackageShare('cho_franka_description'), 'rviz', 'visualize_franka.rviz'
            ]), '-f', 'base'],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('cho_franka_bringup'), 'config', 'real', 'franka.config.yaml']
                ),
                description='Path to the robot configuration file to load',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )