from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata


def generate_launch_description():
    metadata = load_moveit_metadata('ur5e', 'cho_moveit_ur')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    share = FindPackageShare(metadata['config_package'])

    def validate_timeouts(context):
        attempts = int(context.perform_substitution(LaunchConfiguration('scene_max_attempts')))
        ready = float(context.perform_substitution(LaunchConfiguration('scene_ready_timeout')))
        controller = float(context.perform_substitution(
            LaunchConfiguration('controller_ready_timeout')))
        switch = float(context.perform_substitution(LaunchConfiguration('switch_response_timeout')))
        minimum = attempts * 4.0 + max(0, attempts - 1) * 0.5
        minimum += controller + 2.5 + 2.0 * switch + 12.5 + 5.0
        if attempts < 1 or controller <= 0.0 or switch <= 10.0 or ready < minimum:
            raise RuntimeError(
                f'Unsafe MoveIt gate timeouts (ready={ready}, required>={minimum})')
        return []
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'move_group.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    # The publisher/waiter implement the common floor-first controller-switch gate.
    static_scene = Node(
        package='cho_moveit_common', executable='publish_static_scene.py',
        name='ur_static_planning_scene', output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'ready_service': metadata['ready_service'],
            'frame_id': LaunchConfiguration('floor_frame'),
            'size_csv': LaunchConfiguration('floor_size'),
            'position_csv': LaunchConfiguration('floor_position'),
            'max_attempts': ParameterValue(LaunchConfiguration('scene_max_attempts'), value_type=int),
            'controller_ready_timeout': ParameterValue(
                LaunchConfiguration('controller_ready_timeout'), value_type=float),
            'switch_response_timeout': ParameterValue(
                LaunchConfiguration('switch_response_timeout'), value_type=float),
            'activate_controller': metadata['trajectory_controller'],
            'deactivate_controller': metadata['hold_controller'],
        }])
    wait = Node(
        package='cho_moveit_common', executable='wait_for_static_scene.py',
        name='wait_for_ur_static_planning_scene', output='screen', parameters=[{
            'ready_service': metadata['ready_service'],
            'timeout': ParameterValue(LaunchConfiguration('scene_ready_timeout'), value_type=float)}])
    bridge = Node(
        package='cho_moveit_common', executable='moveit_action_bridge.py',
        name='ur_moveit_action_bridge', output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'robot_type': metadata['robot_type'],
            'ready_service': metadata['ready_service'],
            'planning_group': metadata['planning_group'],
            'ee_link': metadata['ee_link'],
            'world_frame': metadata['base_frame'],
            'joint_names': metadata['joint_names'],
            'trajectory_controller': metadata['trajectory_controller']}])
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'moveit_rviz.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    def after_gate(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'UR static planning scene gate failed ({event.returncode})'))]
        if context.perform_substitution(launch_rviz).lower() in ('true', '1', 'yes'):
            return [rviz]
        return []

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_max_attempts', default_value='10'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        DeclareLaunchArgument('switch_response_timeout', default_value='15.0'),
        OpaqueFunction(function=validate_timeouts),
        move_group, bridge, static_scene, wait,
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=after_gate)),
    ])
