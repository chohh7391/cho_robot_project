import os
import tempfile
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
        is_vla = LaunchConfiguration('vla').perform(context)
        mode = LaunchConfiguration('control_mode').perform(context)
        
        ctrl_name = LaunchConfiguration('controller_name').perform(context)
        b_type = LaunchConfiguration('bringup_type').perform(context)

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

        # ---------------------------------------------------------
        # [핵심 로직] Payload 파일 + 동적 파라미터를 하나의 YAML로 완벽하게 병합
        # ---------------------------------------------------------
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

        # 1. payload.yaml 파일 읽어오기
        payload_config_path = os.path.join(pkg_bringup, 'config', 'payload.yaml')
        with open(payload_config_path, 'r') as f:
            dynamic_params_dict = yaml.safe_load(f) or {}

        # ✅ offset.yaml 병합 추가
        offset_config_path = os.path.join(pkg_bringup, 'config', 'offset.yaml')
        with open(offset_config_path, 'r') as f:
            offset_params_dict = yaml.safe_load(f) or {}

        def deep_merge(base: dict, override: dict) -> dict:
            for key, val in override.items():
                if key in base and isinstance(base[key], dict) and isinstance(val, dict):
                    deep_merge(base[key], val)
                else:
                    base[key] = val
            return base

        dynamic_params_dict = deep_merge(dynamic_params_dict, offset_params_dict)

        # 2. 동적 파라미터(bringup_type, control_mode) 덮어쓰기
        for ctrl in controllers_to_load:
            if ctrl not in dynamic_params_dict['/**']:
                dynamic_params_dict['/**'][ctrl] = {'ros__parameters': {}}
            elif 'ros__parameters' not in dynamic_params_dict['/**'][ctrl]:
                dynamic_params_dict['/**'][ctrl]['ros__parameters'] = {}
                
            dynamic_params_dict['/**'][ctrl]['ros__parameters']['bringup_type'] = b_type
            dynamic_params_dict['/**'][ctrl]['ros__parameters']['control_mode'] = internal_mode

        # 3. 완성된 단일 임시 YAML 파일 생성
        temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(dynamic_params_dict, temp_yaml_file)
        temp_yaml_file.close()

        # ---------------------------------------------------------
        # Spawner 노드 생성 ( '-p' 옵션을 통해 Gazebo 내부로 파라미터 강제 푸시 )
        # ---------------------------------------------------------
        load_target_controllers = []
        for name in controllers_to_load:
            # -p 옵션은 spawner가 시작되기 전 controller_manager로 파라미터를 전송합니다!
            spawner_args = [name, '-p', temp_yaml_file.name]
            if name != active_ctrl:
                spawner_args.append('--inactive')
                
            load_target_controllers.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=spawner_args,
                    parameters=[use_sim_time], # 파라미터 리스트에서는 YAML 파일 제거
                    output='screen'
                )
            )

        # 항상 켜져야 하는 공통 컨트롤러들
        common_controllers = [
            Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '-p', temp_yaml_file.name], parameters=[use_sim_time], output='screen'),
            Node(package='controller_manager', executable='spawner', arguments=['ee_state_broadcaster', '-p', temp_yaml_file.name], parameters=[use_sim_time], output='screen'),
            Node(package='controller_manager', executable='spawner', arguments=['simulation_gripper_controller', '-p', temp_yaml_file.name], parameters=[use_sim_time], output='screen'),
            Node(package='controller_manager', executable='spawner', arguments=['gripper_controller', '-p', temp_yaml_file.name], parameters=[use_sim_time], output='screen'),
        ]

        mock_gripper = Node(
            package='cho_franka_bringup',
            executable='mock_franka_gripper.py',
            parameters=[use_sim_time],
            output='screen'
        )

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