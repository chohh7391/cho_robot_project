"""Bring up the FR5 in Gazebo (Ignition/Gazebo Sim).

    ros2 launch cho_bringup_fr5 bringup_gz_robot.launch.py \
         controller_name:=joint_space_position_controller

Gazebo spawns at the canonical non-singular ``home 1`` ready pose. Re-commanding
``home 1`` is optional before switching from ``joint_space_position_controller``
to ``task_space_ik_controller`` and sending a task-space ``reach`` goal.

fr5.urdf.xacro is expanded with hardware:=gazebo, which emits the
gz_ros2_control/GazeboSimSystem ros2_control block plus the Gazebo system
plugin; the plugin hosts the controller_manager inside Gazebo and loads
config/gz/controllers.yaml. The spawners then talk to that controller_manager
once the entity is in the world.
"""

import os

import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


SWITCHABLE_CONTROLLERS = [
    'joint_trajectory_controller',
    'joint_space_position_controller',
    'task_space_ik_controller',
]
CONTROLLER_MODES = list(SWITCHABLE_CONTROLLERS)


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    controller_name = LaunchConfiguration('controller_name').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    allow_renaming = LaunchConfiguration('allow_renaming')

    if controller_name not in CONTROLLER_MODES:
        if controller_name == 'moveit':
            raise RuntimeError(
                "'moveit' is not a ros2_control controller. Launch "
                "bringup_gz_moveit.launch.py instead.")
        raise RuntimeError(
            f"Unknown controller_name '{controller_name}'. "
            f"Valid options: {CONTROLLER_MODES}"
        )
    active_controller = controller_name

    fr5_desc = get_package_share_directory('cho_description_fr5')
    bringup = get_package_share_directory('cho_bringup_fr5')
    urdf_path = os.path.join(fr5_desc, 'urdf', 'fr5.urdf.xacro')
    controllers_file = os.path.join(bringup, 'config', 'gz', 'controllers.yaml')

    # sdformat converts package://cho_description_fr5/... mesh URIs to
    # model://cho_description_fr5/....  Gazebo therefore needs the parent of
    # the package share directory on its resource path in order to resolve
    # both visual and collision meshes.
    resource_root = os.path.dirname(fr5_desc)
    ignition_resource_path = os.pathsep.join(filter(None, [
        resource_root,
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    ]))
    gz_resource_path = os.pathsep.join(filter(None, [
        resource_root,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={'hardware': 'gazebo', 'simulation_controllers': controllers_file},
    ).toxml()
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_content,
            '-name', robot_name,
            '-allow_renaming', allow_renaming,
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
        arguments=['-d', os.path.join(fr5_desc, 'rviz', 'view_robot.rviz')],
        condition=IfCondition(launch_rviz),
    )

    cm_timeout = LaunchConfiguration('controller_manager_timeout')

    active_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            active_controller,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
        ],
        output='screen',
    )

    inactive_controllers = [
        c for c in SWITCHABLE_CONTROLLERS if c != active_controller
    ]
    inactive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *inactive_controllers,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', cm_timeout,
            '--inactive',
        ],
        output='screen',
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[active_controller_spawner, rviz],
        )
    )
    delayed_inactive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=active_controller_spawner,
            on_exit=[inactive_controller_spawner],
        )
    )

    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ignition_resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        robot_state_publisher,
        gz_spawn_entity,
        gz_launch_with_gui,
        gz_launch_without_gui,
        clock_bridge,
        delayed_spawners,
        delayed_inactive_spawner,
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_space_position_controller',
            description='Initial Cho arm controller to activate.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='empty.sdf'),
        DeclareLaunchArgument('robot_name', default_value='fr5'),
        DeclareLaunchArgument('allow_renaming', default_value='true'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        OpaqueFunction(function=launch_setup),
    ])
