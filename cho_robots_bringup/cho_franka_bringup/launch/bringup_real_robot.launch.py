import importlib.util
import os
import tempfile
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ---------------------------------------------------------------------------
# load_yaml 유틸 (franka_bringup의 launch_utils.py 재활용)
# ---------------------------------------------------------------------------
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
    # ------------------------------------------------------------------
    # 1. Launch Arguments 읽기
    # ------------------------------------------------------------------
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)

    mode      = LaunchConfiguration('control_mode').perform(context)
    ctrl_name = LaunchConfiguration('controller_name').perform(context)
    is_vla    = LaunchConfiguration('vla').perform(context)
    b_type    = LaunchConfiguration('bringup_type').perform(context)

    active_ctrl = 'vla_controller' if is_vla == 'true' else ctrl_name

    # ------------------------------------------------------------------
    # 2. 컨트롤러 목록 결정
    # ------------------------------------------------------------------
    position_controllers = ['ik_controller']
    torque_controllers = [
        'gravity_compensation_controller',
        'joint_space_impedance_controller',
        'task_space_impedance_controller',
        'operational_space_controller',
        'joint_space_qp_controller',
        'task_space_qp_controller',
    ]
    controllers_to_load = position_controllers if mode == 'position' else torque_controllers
    if active_ctrl and active_ctrl not in controllers_to_load:
        controllers_to_load.append(active_ctrl)

    internal_mode = 'effort' if mode == 'torque' else mode

    # ------------------------------------------------------------------
    # 3. Payload + 동적 파라미터 병합 → 임시 YAML
    # ------------------------------------------------------------------
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
        dynamic_params_dict['/**'][ctrl]['ros__parameters']['bringup_type']  = b_type
        dynamic_params_dict['/**'][ctrl]['ros__parameters']['control_mode']  = internal_mode

    temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(dynamic_params_dict, temp_yaml_file)
    temp_yaml_file.close()

    # ------------------------------------------------------------------
    # 4. 로봇별 노드 생성 (franka.launch.py 의존성 제거 → 직접 구성)
    # ------------------------------------------------------------------
    nodes = []

    for _, config in configs.items():
        namespace             = str(config['namespace'])
        robot_type_str        = str(config['robot_type'])
        arm_prefix_str        = str(config.get('arm_prefix', ''))
        robot_ip_str          = str(config['robot_ip'])
        load_gripper_str      = str(config['load_gripper'])
        use_fake_hw_str       = str(config['use_fake_hardware'])
        fake_sensor_cmds_str  = str(config['fake_sensor_commands'])
        joint_state_rate_int  = int(config.get('joint_state_rate', 30))
        load_gripper_bool     = load_gripper_str.lower() == 'true'

        # ---- (A) URDF 생성 (xacro) ----
        urdf_path = PathJoinSubstitution([
            FindPackageShare('cho_franka_description'),
            'robots', robot_type_str, f'{robot_type_str}.urdf.xacro',
        ]).perform(context)

        robot_description = xacro.process_file(
            urdf_path,
            mappings={
                'ros2_control':         'true',
                'robot_type':           robot_type_str,
                'arm_prefix':           arm_prefix_str,
                'robot_ip':             robot_ip_str,
                'hand':                 load_gripper_str,
                'use_fake_hardware':    use_fake_hw_str,
                'fake_sensor_commands': fake_sensor_cmds_str,
                'special_connection':   'ft_sensor',
                'xyz_ee':               '0 0 0.0175',
            },
        ).toprettyxml(indent='  ')

        # ---- (B) controllers.yaml 경로 ----
        controllers_yaml = PathJoinSubstitution([
            FindPackageShare('cho_franka_bringup'), 'config', 'real', 'controllers.yaml'
        ]).perform(context)

        joint_state_publisher_sources = [
            'franka/joint_states',
            'franka_gripper/joint_states',
        ]

        # ---- (C) Franka 코어 노드 (구 franka.launch.py 내용 직접 이식) ----
        nodes += [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace=namespace,
                parameters=[
                    controllers_yaml,
                    {'robot_description': robot_description},
                    {'load_gripper': load_gripper_bool},
                ],
                remappings=[('joint_states', joint_state_publisher_sources[0])],
                output='screen',
                on_exit=Shutdown(),
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=namespace,
                parameters=[{
                    'source_list': joint_state_publisher_sources,
                    'rate': joint_state_rate_int,
                    'use_robot_description': False,
                }],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=['franka_robot_state_broadcaster', '--controller-manager-timeout', '30'],
                parameters=[{'robot_type': robot_type_str}],
                condition=UnlessCondition(use_fake_hw_str),
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'
                    ])
                ]),
                launch_arguments={
                    'namespace':         namespace,
                    'robot_ip':          robot_ip_str,
                    'use_fake_hardware': use_fake_hw_str,
                }.items(),
                condition=IfCondition(load_gripper_str),
            ),
        ]

        # ---- (D) 선택된 컨트롤러 Spawner (-p 옵션으로 임시 YAML 주입) ----
        for name in controllers_to_load:
            spawner_args = [name, '-p', temp_yaml_file.name]
            if name != active_ctrl:
                spawner_args.append('--inactive')

            nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=spawner_args + ['--controller-manager-timeout', '30'],
                output='screen',
            ))

        # ---- (E) 항상 켜져야 하는 공통 컨트롤러들 ----
        for common_ctrl in ['gripper_controller', 'ee_state_broadcaster']:
            nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[common_ctrl, '-p', temp_yaml_file.name,
                           '--controller-manager-timeout', '30'],
                output='screen',
            ))

    # ------------------------------------------------------------------
    # 5. RViz (use_rviz=true 인 config 가 하나라도 있으면)
    # ------------------------------------------------------------------
    if any(str(c.get('use_rviz', 'false')).lower() == 'true' for c in configs.values()):
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('cho_franka_bringup'), 'config', 'real', 'franka.config.yaml']
            ),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            description='Choose control mode: position, torque',
            choices=['position', 'torque'],
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='operational_space_controller',
            description='Which controller to activate initially',
        ),
        DeclareLaunchArgument(
            'vla',
            default_value='false',
            description='If true, forces vla_controller to be the active controller',
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='real',
            description='Global bringup type injected to all controllers',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])