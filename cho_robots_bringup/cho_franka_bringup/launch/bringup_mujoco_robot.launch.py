import os
import tempfile
import yaml

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

    declared_arguments = [
        DeclareLaunchArgument(
            'control_mode',
            default_value='torque',
            description='Choose control mode: position, torque'
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='task_space_impedance_controller',
            description='Which controller to activate initially'
        ),
        DeclareLaunchArgument(
            'vla',
            default_value='false',
            description='If true, forces vla_controller to be the active controller'
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='mujoco',
            description='Global bringup type injected to all controllers'
        )
    ]

    xacro_file = os.path.join(description_path, 'urdf', 'fr3', 'fr3_franka_hand.urdf')
    
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' control_mode:=', LaunchConfiguration('control_mode')]),
            value_type=str
        )
    }

    controller_config_file = os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml')
    payload_config_file = os.path.join(bringup_path, 'config', 'payload.yaml')

    mock_gripper = Node(
        package='cho_franka_bringup',
        executable='mock_franka_gripper.py',
        parameters=[{'use_sim_time': True}]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_ee_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ee_state_broadcaster'],
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

    def setup_control_environment(context, *args, **kwargs):
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

        # ---------------------------------------------------------
        # [핵심 수정] 동적으로 /** 와일드카드가 포함된 YAML 파일을 생성합니다.
        # ---------------------------------------------------------
        dynamic_params_dict = {'/**': {}}
        internal_mode = 'effort' if mode == 'torque' else mode

        for ctrl in controllers_to_load:
            dynamic_params_dict['/**'][ctrl] = {
                'ros__parameters': {
                    'bringup_type': b_type,
                    'control_mode': internal_mode
                }
            }

        # 임시 YAML 파일 생성 (Launch 파서가 읽은 후 OS가 나중에 알아서 삭제합니다)
        temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(dynamic_params_dict, temp_yaml_file)
        temp_yaml_file.close()
        # ---------------------------------------------------------

        # Mujoco Node 생성 시 동적 생성한 temp_yaml_file의 경로를 전달
        node_mujoco_ros2_control = Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            output='screen',
            parameters=[
                robot_description,
                controller_config_file,
                payload_config_file,
                temp_yaml_file.name, # 동적 파라미터 YAML 파일 주입!
                {'mujoco_model_path': os.path.join(description_path, 'xml', 'fr3', 'scene.xml')}
            ]
        )

        spawner_nodes = []
        for name in controllers_to_load:
            args = [name] if name == active_ctrl else [name, '--inactive']
            spawner_nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=args,
                    output='screen'
                )
            )

        all_spawners = spawner_nodes + [load_simulation_gripper_controller, load_gripper_controller]

        event_handlers = [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node_mujoco_ros2_control,
                    on_start=[load_joint_state_controller, load_ee_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=all_spawners,
                )
            )
        ]

        return [node_mujoco_ros2_control] + event_handlers

    return LaunchDescription(
        declared_arguments + [
            mock_gripper,
            node_robot_state_publisher,
            OpaqueFunction(function=setup_control_environment)
        ]
    )