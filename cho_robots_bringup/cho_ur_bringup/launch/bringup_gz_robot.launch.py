from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_name = LaunchConfiguration('controller_name')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world_file = LaunchConfiguration('world_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='UR robot type passed to ur_simulation_gz.',
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
            description='Initial Cho controller to activate.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Forwarded to the UR GZ simulation launch.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Forwarded to the UR GZ simulation launch.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.sdf',
            description='Optional world file forwarded to the UR GZ simulation launch.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='Cho controller end-effector frame. Kept as a wrapper-level contract.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_simulation_gz'),
                    'launch',
                    'ur_sim_control.launch.py',
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'runtime_config_package': 'cho_ur_bringup',
                'description_package': 'cho_ur_description',
                'description_file': description_file,
                'controllers_file': controllers_file,
                'initial_joint_controller': 'joint_trajectory_controller',
                'start_joint_controller': 'false',
                'launch_rviz': launch_rviz,
                'gazebo_gui': gazebo_gui,
                'world_file': world_file,
            }.items(),
        ),
        Node(
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
        ),
    ])
