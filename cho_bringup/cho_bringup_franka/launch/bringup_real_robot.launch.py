import importlib.util
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ---------------------------------------------------------------------------
# cho_bringup_franka launch utility loading
# ---------------------------------------------------------------------------
package_share = get_package_share_directory('cho_bringup_franka')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_franka', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')
spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)
load_yaml = launch_utils.load_yaml
as_bool = launch_utils.as_bool
create_controller_spawners = launch_utils.create_controller_spawners
create_runtime_param_file = launch_utils.create_runtime_param_file
create_runtime_param_cleanup = launch_utils.create_runtime_param_cleanup
get_initial_active_controller = launch_utils.get_initial_active_controller
get_switchable_controllers = launch_utils.get_switchable_controllers

REAL_ALWAYS_ACTIVE_CONTROLLERS = [
    'joint_state_broadcaster',
    'ee_state_broadcaster',
    'gripper_controller',
]


def generate_robot_nodes(context):
    # ------------------------------------------------------------------
    # 1. Launch Arguments 읽기
    # ------------------------------------------------------------------
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)

    mode = LaunchConfiguration('control_mode').perform(context)
    ctrl_name = LaunchConfiguration('controller_name').perform(context)
    use_vla = LaunchConfiguration('vla').perform(context)
    b_type = LaunchConfiguration('bringup_type').perform(context)
    ee_name = LaunchConfiguration('ee_name').perform(context)

    # ------------------------------------------------------------------
    # 2. Controller preparation policy
    # ------------------------------------------------------------------
    initial_active_controller = get_initial_active_controller(ctrl_name, use_vla)
    switchable_controllers = get_switchable_controllers(
        control_mode=mode,
        use_vla=use_vla,
        requested_controller=ctrl_name,
        extra_torque_controllers=['joint_trajectory_controller'],
    )

    pkg_bringup = get_package_share_directory('cho_bringup_franka')
    payload_config_path = os.path.join(pkg_bringup, 'config', 'payload.yaml')
    runtime_param_file = create_runtime_param_file(
        payload_config_path=payload_config_path,
        controller_names=REAL_ALWAYS_ACTIVE_CONTROLLERS + switchable_controllers,
        bringup_type=b_type,
        control_mode=mode,
        ee_name=ee_name,
    )

    # ------------------------------------------------------------------
    # 3. 로봇별 노드 생성 (franka.launch.py 의존성 제거 → 직접 구성)
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
            FindPackageShare('cho_description_franka'),
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
                'xyz_ee':               '0 0 0',
            },
        ).toprettyxml(indent='  ')

        # ---- (B) controllers.yaml 경로 ----
        controllers_yaml = PathJoinSubstitution([
            FindPackageShare('cho_bringup_franka'), 'config', 'real', 'controllers.yaml'
        ]).perform(context)

        joint_state_publisher_sources = [
            'franka/joint_states',
            'franka_gripper/joint_states',
        ]

        # ---- (C) Franka 코어 노드 ----
        ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                runtime_param_file,
                {'robot_description': robot_description},
                {'load_gripper': load_gripper_bool},
            ],
            remappings=[('joint_states', joint_state_publisher_sources[0])],
            output='screen',
            on_exit=Shutdown(),
        )

        nodes += [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            ros2_control_node,
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

        # Spawner들은 ros2_control_node 가 떠야 controller_manager 서비스에 접속할 수
        # 있다. active 묶음이 끝난 뒤 inactive 묶음이 뜨도록 launch_utils 에서
        # 체인으로 구성해 controller_manager 요청이 겹치지 않게 한다.
        always_active_controllers = list(REAL_ALWAYS_ACTIVE_CONTROLLERS)
        if not as_bool(use_fake_hw_str):
            always_active_controllers.insert(0, 'franka_robot_state_broadcaster')

        controller_spawners = create_controller_spawners(
            always_active_controllers=always_active_controllers,
            switchable_controllers=switchable_controllers,
            initial_active_controller=initial_active_controller,
            # runtime params are loaded directly on ros2_control_node above, so
            # no spawner -p file handoff is needed here.
            namespace=namespace,
            timeout=60,
        )

        nodes.append(
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=ros2_control_node,
                    on_start=controller_spawners,
                )
            )
        )

    # ------------------------------------------------------------------
    # 4. RViz
    # ------------------------------------------------------------------
    if any(str(c.get('use_rviz', 'false')).lower() == 'true' for c in configs.values()):
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', PathJoinSubstitution([
                FindPackageShare('cho_description_franka'), 'rviz', 'visualize_franka.rviz'
            ]), '-f', 'base'],
            output='screen',
        ))

    nodes.append(
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[create_runtime_param_cleanup(runtime_param_file)],
            )
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('cho_bringup_franka'), 'config', 'real', 'franka.config.yaml']
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
        DeclareLaunchArgument(
            'ee_name',
            default_value='fr3_hand_tcp',
            description='Name of End-Effector',
            choices=['fr3_link7', 'fr3_hand', 'fr3_hand_tcp']
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
