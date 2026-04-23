import importlib.util
import os
import sys
import tempfile
import yaml

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
    
    # ---------------------------------------------------------
    # [추가된 로직] 터미널 인자 읽어오기
    # ---------------------------------------------------------
    mode = LaunchConfiguration('control_mode').perform(context)
    ctrl_name = LaunchConfiguration('controller_name').perform(context)
    is_vla = LaunchConfiguration('vla').perform(context)
    b_type = LaunchConfiguration('bringup_type').perform(context)

    active_ctrl = 'vla_controller' if is_vla == 'true' else ctrl_name

    position_controllers = ['ik_controller']
    torque_controllers = [
        'gravity_compensation_controller',
        'joint_space_impedance_controller',
        'task_space_impedance_controller',
        'operational_space_controller',
        'joint_space_qp_controller',
        'task_space_qp_controller'
    ]

    controllers_to_load = position_controllers if mode == 'position' else torque_controllers
    
    if active_ctrl and active_ctrl not in controllers_to_load:
        controllers_to_load.append(active_ctrl)

    internal_mode = 'effort' if mode == 'torque' else mode

    # ---------------------------------------------------------
    # [추가된 로직] Payload 파일 + 동적 파라미터 단일 YAML 병합
    # ---------------------------------------------------------
    pkg_bringup = get_package_share_directory('cho_franka_bringup')
    payload_config_path = os.path.join(pkg_bringup, 'config', 'payload.yaml')
    
    dynamic_params_dict = {}
    if os.path.exists(payload_config_path):
        with open(payload_config_path, 'r') as f:
            dynamic_params_dict = yaml.safe_load(f) or {}

    if '/**' not in dynamic_params_dict:
        dynamic_params_dict['/**'] = {}

    for ctrl in controllers_to_load + ['ee_state_broadcaster']:
        if ctrl not in dynamic_params_dict['/**']:
            dynamic_params_dict['/**'][ctrl] = {'ros__parameters': {}}
        elif 'ros__parameters' not in dynamic_params_dict['/**'][ctrl]:
            dynamic_params_dict['/**'][ctrl]['ros__parameters'] = {}
            
        dynamic_params_dict['/**'][ctrl]['ros__parameters']['bringup_type'] = b_type
        dynamic_params_dict['/**'][ctrl]['ros__parameters']['control_mode'] = internal_mode

    temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(dynamic_params_dict, temp_yaml_file)
    temp_yaml_file.close()
    # ---------------------------------------------------------

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

        # 공통 Spawner 파라미터 설정
        common_params = [PathJoinSubstitution([
            FindPackageShare('cho_franka_bringup'), 'config', 'real', 'controllers.yaml'
        ])]

        # ---------------------------------------------------------
        # [수정된 로직] 선택된 컨트롤러들 Spawner 실행 (-p 옵션으로 임시 YAML 전달)
        # ---------------------------------------------------------
        for name in controllers_to_load:
            spawner_args = [name, '-p', temp_yaml_file.name]
            if name != active_ctrl:
                spawner_args.append('--inactive')
            
            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=spawner_args + ['--controller-manager-timeout', '30'],
                    parameters=common_params,
                    output='screen',
                )
            )

        # 공통 컨트롤러들 (항상 켜져야 하는 것들)
        for common_ctrl in ['gripper_controller', 'ee_state_broadcaster']:
            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[common_ctrl, '-p', temp_yaml_file.name, '--controller-manager-timeout', '30'],
                    parameters=common_params,
                    output='screen',
                )
            )

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
            # [추가된 로직] Launch Argument 선언
            DeclareLaunchArgument(
                'control_mode',
                default_value='torque',
                description='Choose control mode: position, torque',
                choices=['position', 'torque']
            ),
            DeclareLaunchArgument(
                'controller_name',
                default_value='operational_space_controller',
                description='Which controller to activate initially'
            ),
            DeclareLaunchArgument(
                'vla',
                default_value='false',
                description='If true, forces vla_controller to be the active controller'
            ),
            DeclareLaunchArgument(
                'bringup_type',
                default_value='real',
                description='Global bringup type injected to all controllers'
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )