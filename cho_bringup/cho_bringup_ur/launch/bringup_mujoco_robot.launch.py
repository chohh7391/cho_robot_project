import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


SWITCHABLE_CONTROLLERS = [
    'joint_space_position_controller',
    'task_space_ik_controller',
]


def create_runtime_controller_params(ee_name, bringup_type):
    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='cho_ur_mujoco_runtime_params_',
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
    controller_name = LaunchConfiguration('controller_name').perform(context)
    ee_name = LaunchConfiguration('ee_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)

    if controller_name not in SWITCHABLE_CONTROLLERS:
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {SWITCHABLE_CONTROLLERS}"
        )

    ur_desc_path = get_package_share_directory('cho_description_ur')
    bringup_path = get_package_share_directory('cho_bringup_ur')

    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    controller_config = LaunchConfiguration('controllers_file').perform(context)
    runtime_param_file = create_runtime_controller_params(ee_name, bringup_type)

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
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            controller_config,
            runtime_param_file,
            {
                'ee_name': ee_name,
                'bringup_type': bringup_type,
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
            '-p',
            runtime_param_file,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            controller_manager_timeout,
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
            '-p',
            runtime_param_file,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            controller_manager_timeout,
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
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[cleanup_runtime_controller_params(runtime_param_file)],
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
        DeclareLaunchArgument(
            'bringup_type',
            default_value='mujoco',
            description='Cho controller bringup type',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                get_package_share_directory('cho_description_ur'),
                'urdf',
                'ur5e.urdf',
            ),
            description='URDF file used for MuJoCo robot_description',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(
                get_package_share_directory('cho_bringup_ur'),
                'config',
                'mujoco',
                'controllers.yaml',
            ),
            description='Controller YAML file loaded by mujoco_ros2_control',
        ),
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='30',
            description='Controller manager service timeout for spawners',
        ),
        OpaqueFunction(function=setup_control_environment),
    ])
