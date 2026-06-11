import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SWITCHABLE_CONTROLLERS = [
    'joint_space_position_controller',
    'task_space_ik_controller',
]


def create_runtime_controller_params(ee_name, bringup_type):
    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='cho_ur_gz_runtime_params_',
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


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    ur_type = LaunchConfiguration('ur_type')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_name = LaunchConfiguration('controller_name').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world_file = LaunchConfiguration('world_file')
    load_gripper = LaunchConfiguration('load_gripper')
    tf_prefix = LaunchConfiguration('tf_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    allow_renaming = LaunchConfiguration('allow_renaming')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    ee_name = LaunchConfiguration('ee_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')
    runtime_param_file = create_runtime_controller_params(ee_name, bringup_type)

    if controller_name not in SWITCHABLE_CONTROLLERS:
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {SWITCHABLE_CONTROLLERS}"
        )

    controller_config = PathJoinSubstitution([
        FindPackageShare('cho_bringup_ur'),
        'config',
        controllers_file,
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('cho_description_ur'),
            'urdf',
            description_file,
        ]),
        ' ',
        'safety_limits:=true',
        ' ',
        'safety_pos_margin:=',
        safety_pos_margin,
        ' ',
        'safety_k_position:=',
        safety_k_position,
        ' ',
        'name:=',
        robot_name,
        ' ',
        'ur_type:=',
        ur_type,
        ' ',
        'tf_prefix:=',
        tf_prefix,
        ' ',
        'sim_ignition:=true',
        ' ',
        'simulation_controllers:=',
        controller_config,
        ' ',
        'load_gripper:=',
        load_gripper,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content},
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_description_content,
            '-name',
            robot_name,
            '-allow_renaming',
            allow_renaming,
        ],
    )

    gz_launch_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': [' -r -v 4 ', world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gz_launch_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': [' -s -r -v 4 ', world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('cho_description_ur'),
                'rviz',
                'view_robot.rviz',
            ]),
        ],
        condition=IfCondition(launch_rviz),
    )

    active_controller_spawner = Node(
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
        'joint_trajectory_controller',
        *[
            controller for controller in SWITCHABLE_CONTROLLERS
            if controller != controller_name
        ],
    ]

    inactive_controller_spawner = Node(
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

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            controller_manager_timeout,
        ],
        output='screen',
        condition=IfCondition(load_gripper),
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                active_controller_spawner,
                gripper_controller_spawner,
                rviz,
            ],
        )
    )

    delayed_inactive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=active_controller_spawner,
            on_exit=[inactive_controller_spawner],
        )
    )

    return [
        robot_state_publisher,
        gz_spawn_entity,
        gz_launch_with_gui,
        gz_launch_without_gui,
        clock_bridge,
        delayed_spawners,
        delayed_inactive_spawner,
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[cleanup_runtime_controller_params(runtime_param_file)],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='UR robot type.',
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ur.urdf.xacro',
            description='Cho UR xacro filename under cho_description_ur/urdf.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='gz/controllers.yaml',
            description='Controller YAML path relative to cho_bringup_ur/config.',
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='Initial Cho arm controller to activate.',
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Attach the Robotiq 2F-85 gripper and spawn gripper_controller.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Start Gazebo with GUI.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.sdf',
            description='Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='ur',
            description='Robot name passed to xacro and Gazebo.',
        ),
        DeclareLaunchArgument(
            'allow_renaming',
            default_value='true',
            description='Allow Gazebo to rename the spawned entity.',
        ),
        DeclareLaunchArgument(
            'safety_pos_margin',
            default_value='0.15',
            description='UR safety position margin forwarded to xacro.',
        ),
        DeclareLaunchArgument(
            'safety_k_position',
            default_value='20',
            description='UR safety k_position forwarded to xacro.',
        ),
        DeclareLaunchArgument(
            'bringup_type',
            default_value='gz',
            description='Cho controller bringup type.',
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='Optional tf/joint prefix.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='Cho controller end-effector frame.',
        ),
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='30',
            description='Controller manager service timeout for spawners.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
