from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from cho_robot_config import load_moveit_metadata


def _rviz_include(package_share, use_sim_time, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'moveit_rviz.launch.py'])
        ),
        condition=condition,
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )


def generate_launch_description():
    metadata = load_moveit_metadata('fr5', 'cho_moveit_fr5')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_static_scene = LaunchConfiguration('publish_static_scene')
    package_share = FindPackageShare(metadata['config_package'])

    def validate_timeouts(context):
        attempts = int(context.perform_substitution(LaunchConfiguration('scene_max_attempts')))
        ready_timeout = float(context.perform_substitution(LaunchConfiguration('scene_ready_timeout')))
        controller_timeout = float(
            context.perform_substitution(LaunchConfiguration('controller_ready_timeout'))
        )
        switch_timeout = float(
            context.perform_substitution(LaunchConfiguration('switch_response_timeout'))
        )
        if attempts < 1 or controller_timeout <= 0.0 or switch_timeout <= 10.0:
            raise RuntimeError(
                'scene_max_attempts must be positive, controller_ready_timeout must be '
                'positive, and switch_response_timeout must exceed 10 seconds'
            )
        # Each apply attempt can spend 2 s discovering the service and another
        # 2 s waiting for its response. Controller-state polling can overrun
        # each nominal deadline by one 2 s query plus the 0.5 s poll delay.
        apply_budget = attempts * (2.0 + 2.0) + max(0, attempts - 1) * 0.5
        controller_poll_overrun = 2.0 + 0.5
        switch_budget = 2.0 * switch_timeout
        post_switch_budget = 10.0 + controller_poll_overrun
        launch_margin = 5.0
        minimum_ready_timeout = apply_budget + controller_timeout
        minimum_ready_timeout += controller_poll_overrun + switch_budget
        minimum_ready_timeout += post_switch_budget + launch_margin
        if ready_timeout < minimum_ready_timeout:
            raise RuntimeError(
                f'scene_ready_timeout={ready_timeout:.1f}s is shorter than the '
                f'configured gate worst-case budget {minimum_ready_timeout:.1f}s'
            )
        return []

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'move_group.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    static_scene = Node(
        package='cho_moveit_common',
        executable='publish_static_scene.py',
        name='fr5_static_planning_scene',
        output='screen',
        condition=IfCondition(publish_static_scene),
        parameters=[{
            'use_sim_time': use_sim_time,
            'ready_service': metadata['ready_service'],
            'frame_id': LaunchConfiguration('floor_frame'),
            'size_csv': LaunchConfiguration('floor_size'),
            'position_csv': LaunchConfiguration('floor_position'),
            'max_attempts': ParameterValue(
                LaunchConfiguration('scene_max_attempts'), value_type=int),
            'controller_ready_timeout': ParameterValue(
                LaunchConfiguration('controller_ready_timeout'), value_type=float),
            'switch_response_timeout': ParameterValue(
                LaunchConfiguration('switch_response_timeout'), value_type=float),
            'activate_controller': LaunchConfiguration('activate_controller_after_scene'),
            'deactivate_controller': LaunchConfiguration('deactivate_controller_after_scene'),
        }],
    )
    wait_for_scene = Node(
        package='cho_moveit_common',
        executable='wait_for_static_scene.py',
        name='wait_for_fr5_static_planning_scene',
        output='screen',
        condition=IfCondition(publish_static_scene),
        parameters=[{
            'ready_service': metadata['ready_service'],
            'timeout': ParameterValue(
                LaunchConfiguration('scene_ready_timeout'), value_type=float)}],
    )
    action_bridge = Node(
        package='cho_moveit_common',
        executable='moveit_action_bridge.py',
        name='fr5_moveit_action_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_type': metadata['robot_type'],
            'ready_service': metadata['ready_service'],
            'planning_group': metadata['planning_group'],
            'ee_link': metadata['ee_link'],
            'world_frame': metadata['base_frame'],
            'joint_names': metadata['joint_names'],
            'trajectory_controller': metadata['trajectory_controller'],
        }],
    )

    gated_rviz = _rviz_include(package_share, use_sim_time)

    def after_gate(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'FR5 static planning scene gate failed with exit code {event.returncode}'
            ))]
        if context.perform_substitution(launch_rviz).lower() in ('true', '1', 'yes'):
            return [gated_rviz]
        return []

    gate_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scene,
            on_exit=after_gate,
        )
    )
    direct_rviz = _rviz_include(
        package_share,
        use_sim_time,
        condition=IfCondition(PythonExpression([
            "'", launch_rviz, "'.lower() in ('true', '1', 'yes') and '",
            publish_static_scene, "'.lower() not in ('true', '1', 'yes')",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('publish_static_scene', default_value='true'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_max_attempts', default_value='10'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        DeclareLaunchArgument('switch_response_timeout', default_value='15.0'),
        DeclareLaunchArgument(
            'activate_controller_after_scene',
            default_value=metadata['trajectory_controller']),
        DeclareLaunchArgument(
            'deactivate_controller_after_scene',
            default_value=metadata['hold_controller']),
        OpaqueFunction(function=validate_timeouts),
        move_group,
        action_bridge,
        static_scene,
        wait_for_scene,
        gate_handler,
        direct_rviz,
    ])
