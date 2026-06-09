from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    kinematics_params_file = LaunchConfiguration('kinematics_params_file')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_name = LaunchConfiguration('controller_name')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_tool_communication = LaunchConfiguration('use_tool_communication')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='UR robot type passed to ur_robot_driver.',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address of the UR controller.',
        ),
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cho_description_ur'),
                'config',
                ur_type,
                'default_kinematics.yaml',
            ]),
            description='Calibration YAML extracted from the physical robot, or nominal default kinematics.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='real/controllers.yaml',
            description='Controller YAML path relative to cho_bringup_ur/config.',
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='Initial Cho controller to activate.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Forwarded to the UR driver launch.',
        ),
        DeclareLaunchArgument(
            'use_tool_communication',
            default_value='false',
            description='Forwarded to the UR driver launch.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='Cho controller end-effector frame. Kept as a wrapper-level contract.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py',
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'runtime_config_package': 'cho_bringup_ur',
                'description_package': 'cho_description_ur',
                'description_file': 'ur.urdf.xacro',
                'controllers_file': controllers_file,
                'initial_joint_controller': 'joint_trajectory_controller',
                'activate_joint_controller': 'false',
                'kinematics_params_file': kinematics_params_file,
                'launch_rviz': launch_rviz,
                'use_tool_communication': use_tool_communication,
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
