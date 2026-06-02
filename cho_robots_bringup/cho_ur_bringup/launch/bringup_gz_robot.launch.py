from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs

    ur_type = LaunchConfiguration('ur_type')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_name = LaunchConfiguration('controller_name')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world_file = LaunchConfiguration('world_file')
    load_gripper = LaunchConfiguration('load_gripper')
    tf_prefix = LaunchConfiguration('tf_prefix')

    controller_config = PathJoinSubstitution([
        FindPackageShare('cho_ur_bringup'),
        'config',
        controllers_file,
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('cho_ur_description'),
            'urdf',
            description_file,
        ]),
        ' ',
        'safety_limits:=true',
        ' ',
        'safety_pos_margin:=0.15',
        ' ',
        'safety_k_position:=20',
        ' ',
        'name:=ur',
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
            {'use_sim_time': True},
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
            'ur',
            '-allow_renaming',
            'true',
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
                FindPackageShare('cho_ur_description'),
                'rviz',
                'view_robot.rviz',
            ]),
        ],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager',
            '--inactive',
        ],
        output='screen',
    )

    cho_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
        ],
        output='screen',
    )

    robotiq_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robotiq_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
        ],
        output='screen',
        condition=IfCondition(load_gripper),
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner,
                cho_controller_spawner,
                robotiq_controller_spawner,
                rviz,
            ],
        )
    )

    return [
        robot_state_publisher,
        gz_spawn_entity,
        gz_launch_with_gui,
        gz_launch_without_gui,
        clock_bridge,
        delayed_spawners,
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
            description='Cho UR xacro filename under cho_ur_description/urdf.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='gz/controllers.yaml',
            description='Controller YAML path relative to cho_ur_bringup/config.',
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_controller',
            description='Initial Cho arm controller to activate.',
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Attach the Robotiq 2F-85 gripper and spawn robotiq_controller.',
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
            'tf_prefix',
            default_value='',
            description='Optional tf/joint prefix.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='Cho controller end-effector frame. Kept as a wrapper-level contract.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
