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

The arm commands no motion at startup: the hardware latches its measured pose
into the command on activation. The gripper is the exception - with
`open_on_activate` set in fr5.config.yaml the hardware opens the jaws once as it
activates, so a run starts from a known opening. Clear the jaws before
launching, or set it false to activate in place.
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


import importlib.util

package_share = get_package_share_directory('cho_bringup_fr5')
# launch_utils is installed under lib/, not as an importable python package, so
# it is loaded by path the same way cho_bringup_franka and _openarm do it.
_launch_utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'cho_bringup_fr5', 'utils', 'launch_utils.py')
)
_spec = importlib.util.spec_from_file_location('fr5_launch_utils', _launch_utils_path)
launch_utils = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch_utils)


SWITCHABLE_CONTROLLERS = [
    'joint_trajectory_controller',
    'joint_space_position_controller',
    'task_space_ik_controller',
]

# Keys accepted under `fr5.gripper_config` in the config file, with the values
# used when the file omits them. They are forwarded to the hardware component as
# xacro params so it can call SetGripperConfig/ActGripper itself; the defaults
# describe the DAHUAN gripper registered as PGI-140 on end-effector port 1.
GRIPPER_CONFIG_DEFAULTS = {
    'company': 4,
    'device': 0,
    'softversion': 0,
    'bus': 1,
    'index': 1,
    'speed_percent': 30,
    'force_percent': 30,
    'percent_at_closed': 0,
    'percent_at_open': 100,
    'open_on_activate': False,
    'apply_config': False,
    'required': False,
}


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
    gripper = launch_utils.resolve_gripper(
        LaunchConfiguration('gripper').perform(context),
        LaunchConfiguration('load_gripper').perform(context),
        fr5_cfg.get('gripper'))

    # RS485 gripper registration, forwarded into the description so the hardware
    # component can register and activate the gripper itself. Same YAML -> launch
    # -> xacro param route robot_ip takes. See fr5.config.yaml for the encoding.
    gripper_cfg = fr5_cfg.get('gripper_config') or {}

    def as_xacro(value):
        # xacro mappings are strings; str(True) would give "True", which reads
        # as neither of the spellings a xacro:if test expects.
        return str(value).lower() if isinstance(value, bool) else str(value)

    gripper_mappings = {
        f'gripper_{key}': as_xacro(gripper_cfg.get(key, default))
        for key, default in GRIPPER_CONFIG_DEFAULTS.items()
    }
    unknown = set(gripper_cfg) - set(GRIPPER_CONFIG_DEFAULTS)
    if unknown:
        raise RuntimeError(
            f"Unknown gripper_config keys in {config_path}: {sorted(unknown)}. "
            f"Valid keys: {sorted(GRIPPER_CONFIG_DEFAULTS)}")

    if controller_name not in SWITCHABLE_CONTROLLERS:
        if controller_name == 'moveit':
            raise RuntimeError(
                "'moveit' is not a ros2_control controller. Launch "
                "bringup_real_moveit.launch.py instead.")
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
            mappings={'hardware': 'fairino', 'robot_ip': robot_ip,
                      'gripper': gripper, **gripper_mappings},
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

    # The gripper claims only its own joint, so it never contends with an arm
    # controller and comes up active alongside whichever one was selected, the
    # way cho_bringup_franka spawns its own. The hardware's on_activate seeds
    # the command from the measured stroke, so this commands no motion.
    active_controllers = ['joint_state_broadcaster', controller_name]
    if gripper != 'none':
        active_controllers.append('gripper_controller')

    active_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *active_controllers,
            '-p', runtime_param_file,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
        ],
        output='screen',
    )

    inactive_controllers = [
        c for c in SWITCHABLE_CONTROLLERS if c != controller_name
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
            description=(
                'joint_trajectory_controller, joint_space_position_controller, '
                'or task_space_ik_controller'
            ),
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
        DeclareLaunchArgument(
            'gripper',
            default_value='',
            description=(
                'End-effector gripper by name: none | ag95. Falls back to config_file. '
                'ag95 makes the hardware register and activate the gripper from '
                'the gripper_config block in config_file.'
            ),
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='config',
            description=(
                'Boolean spelling of the same choice, as cho_bringup_franka uses: '
                'true loads the AG-95, false loads none, config (the default) defers '
                'to config_file. Contradicting gripper:= is a launch error.'
            ),
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
