import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup_control_environment(context):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    ee_name = LaunchConfiguration('ee_name').perform(context)

    ur_desc_path = get_package_share_directory('cho_ur_description')
    bringup_path = get_package_share_directory('cho_ur_bringup')

    urdf_path = os.path.join(ur_desc_path, 'urdf', 'ur5e.urdf')
    controller_config = os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml')

    # Resolve $(find cho_ur_description) substitution baked into the pre-built URDF
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read().replace(
            '$(find cho_ur_description)', ur_desc_path
        )

    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            robot_description,
            controller_config,
            {
                'ee_name': ee_name,
                'bringup_type': 'mujoco',
            },
        ],
        remappings=[('~/robot_description', '/robot_description')],
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawner_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco,
                on_start=[spawner_jsb, spawner_ctrl],
            )
        ),
    ]

    return [node_robot_state_publisher, node_mujoco] + event_handlers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_controller',
            description='Cho controller to activate: joint_space_controller or task_space_ik_controller',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='End-effector frame name used by task-space controller',
        ),
        OpaqueFunction(function=setup_control_environment),
    ])