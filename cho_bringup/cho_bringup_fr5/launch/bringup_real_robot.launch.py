"""Bring up the real FR5 over the vendor libfairino hardware interface.

    ros2 launch cho_bringup_fr5 bringup_real_robot.launch.py \
         controller_name:=joint_space_position_controller

Set the controller IP in config/real/fr5.config.yaml first (FR5 default
192.168.58.2). Unlike the UR bringup there is no vendor control.launch to
include, so this composes the stack directly: fr5.urdf.xacro is expanded with
hardware:=fairino (+robot_ip), robot_state_publisher and the standard
controller_manager/ros2_control_node come up, and the spawners start once the
node is running.

Safety: start with joint_state_broadcaster only to confirm state read-back,
then bring up joint_trajectory_controller / the cho controller at low speed.
The cho controllers own the velocity / delta-q limits (the vendor write() does
not clamp), and on_deactivate calls the vendor StopMotion().
"""

import os
import tempfile

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SWITCHABLE_CONTROLLERS = [
    'joint_space_position_controller',
    'task_space_ik_controller',
]


def create_runtime_controller_params(ee_name, bringup_type):
    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='cho_fr5_real_runtime_params_',
        dir=runtime_dir,
    )
    params = {
        '/**': {
            'joint_space_position_controller': {
                'ros__parameters': {
                    'bringup_type': bringup_type,
                    'control_mode': 'position',
                },
            },
            'task_space_ik_controller': {
                'ros__parameters': {
                    'bringup_type': bringup_type,
                    'control_mode': 'position',
                    'ee_name': ee_name,
                },
            },
        },
    }
    with os.fdopen(fd, 'w') as runtime_file:
        yaml.safe_dump(params, runtime_file)
    return runtime_path


def cleanup_runtime_controller_params(runtime_path):
    def cleanup(context, *args, **kwargs):
        if os.path.exists(runtime_path):
            os.unlink(runtime_path)
        return []

    return OpaqueFunction(function=cleanup)


def setup_control_environment(context):
    fr5_desc = get_package_share_directory('cho_description_fr5')
    bringup = get_package_share_directory('cho_bringup_fr5')

    controller_name = LaunchConfiguration('controller_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    cm_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)

    # Connection settings from fr5.config.yaml, overridable via launch args.
    config_path = LaunchConfiguration('config_file').perform(context)
    with open(config_path) as f:
        fr5_cfg = (yaml.safe_load(f) or {}).get('fr5', {})

    robot_ip = LaunchConfiguration('robot_ip').perform(context) or str(fr5_cfg.get('robot_ip', '192.168.58.2'))
    ee_name = LaunchConfiguration('ee_name').perform(context) or str(fr5_cfg.get('ee_name', 'wrist3_link'))

    if controller_name not in SWITCHABLE_CONTROLLERS:
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {SWITCHABLE_CONTROLLERS}"
        )

    urdf_path = os.path.join(fr5_desc, 'urdf', 'fr5.urdf.xacro')
    controllers_file = os.path.join(bringup, 'config', 'real', 'controllers.yaml')
    runtime_param_file = create_runtime_controller_params(ee_name, bringup_type)

    robot_description = {
        'robot_description': xacro.process_file(
            urdf_path,
            mappings={'hardware': 'fairino', 'robot_ip': robot_ip},
        ).toxml()
    }

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            controllers_file,
            runtime_param_file,
            robot_description,
            {'ee_name': ee_name, 'bringup_type': bringup_type},
        ],
        on_exit=Shutdown(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    active_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            controller_name,
            '-p', runtime_param_file,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
        ],
        output='screen',
    )

    inactive_controllers = [
        'joint_trajectory_controller',
        *[c for c in SWITCHABLE_CONTROLLERS if c != controller_name],
    ]
    inactive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *inactive_controllers,
            '-p', runtime_param_file,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
            '--inactive',
        ],
        output='screen',
    )

    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros2_control_node,
                on_start=[active_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=[inactive_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[cleanup_runtime_controller_params(runtime_param_file)],
            )
        ),
    ]

    return [robot_state_publisher, ros2_control_node] + event_handlers


def generate_launch_description():
    bringup = get_package_share_directory('cho_bringup_fr5')
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='joint_space_position_controller or task_space_ik_controller',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='',
            description='FR5 controller IP; falls back to config_file if empty.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='',
            description='End-effector frame; falls back to config_file if empty.',
        ),
        DeclareLaunchArgument('bringup_type', default_value='real'),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(bringup, 'config', 'real', 'fr5.config.yaml'),
            description='FR5 connection settings (robot_ip, ee_name, ...).',
        ),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        OpaqueFunction(function=setup_control_environment),
    ])
