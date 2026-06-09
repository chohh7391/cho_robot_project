import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


SWITCHABLE_CONTROLLERS = [
    'joint_space_position_controller',
    'task_space_ik_controller',
]


def setup_control_environment(context):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    ee_name = LaunchConfiguration('ee_name').perform(context)

    if controller_name not in SWITCHABLE_CONTROLLERS:
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {SWITCHABLE_CONTROLLERS}"
        )

    ur_desc_path = get_package_share_directory('cho_description_ur')
    bringup_path = get_package_share_directory('cho_bringup_ur')

    urdf_path = os.path.join(ur_desc_path, 'urdf', 'ur5e.urdf')
    controller_config = os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml')

    # Resolve $(find cho_description_ur) substitution baked into the pre-built URDF
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read().replace(
            '$(find cho_description_ur)', ur_desc_path
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

    active_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            controller_name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
        ],
        output='screen',
    )

    inactive_controllers = [
        controller for controller in SWITCHABLE_CONTROLLERS
        if controller != controller_name
    ]

    inactive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *inactive_controllers,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--inactive',
        ],
        output='screen',
    )

    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco,
                on_start=[active_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=[inactive_spawner],
            )
        ),
    ]

    return [node_robot_state_publisher, node_mujoco] + event_handlers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='Cho controller to activate: joint_space_position_controller or task_space_ik_controller',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='End-effector frame name used by task-space controller',
        ),
        OpaqueFunction(function=setup_control_environment),
    ])
