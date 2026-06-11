import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_runtime_controller_params(ee_name, bringup_type):
    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='cho_ur_runtime_params_',
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
    robot_ip = LaunchConfiguration('robot_ip')
    kinematics_params_file = LaunchConfiguration('kinematics_params_file')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_name = LaunchConfiguration('controller_name')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_tool_communication = LaunchConfiguration('use_tool_communication')
    description_file = LaunchConfiguration('description_file')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    activate_joint_controller = LaunchConfiguration('activate_joint_controller')

    ee_name = LaunchConfiguration('ee_name').perform(context)
    bringup_type = LaunchConfiguration('bringup_type').perform(context)
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)
    runtime_param_file = create_runtime_controller_params(ee_name, bringup_type)

    return [
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
                'description_file': description_file,
                'controllers_file': controllers_file,
                'initial_joint_controller': initial_joint_controller,
                'activate_joint_controller': activate_joint_controller,
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
                '-p',
                runtime_param_file,
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                controller_manager_timeout,
            ],
            output='screen',
        ),
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[cleanup_runtime_controller_params(runtime_param_file)],
            )
        ),
    ]


def generate_launch_description():
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
                LaunchConfiguration('ur_type'),
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
            'description_file',
            default_value='ur.urdf.xacro',
            description='UR description file forwarded to ur_robot_driver.',
        ),
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='joint_trajectory_controller',
            description='Initial UR driver joint controller.',
        ),
        DeclareLaunchArgument(
            'activate_joint_controller',
            default_value='false',
            description='Whether ur_robot_driver activates the initial joint controller.',
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
            'bringup_type',
            default_value='real',
            description='Cho controller bringup type.',
        ),
        DeclareLaunchArgument(
            'ee_name',
            default_value='tool0',
            description='Cho controller end-effector frame.',
        ),
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='30',
            description='Controller manager service timeout for spawner.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
