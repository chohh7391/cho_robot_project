from cho_robot_config import load_moveit_metadata
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def build_gate_controller_parameters(metadata, mit_paired):
    """Build controller parameters without serializing an empty ROS array.

    Humble launch_ros evaluates an untyped empty Python list as the tuple ``()``
    and rejects it before the node starts. Omitting the optional deactivate
    parameter preserves the node's declared empty STRING_ARRAY default.
    """
    if mit_paired:
        return {
            'activate_controllers': ['bimanual_follow_joint_trajectory_mit_controller'],
        }
    return {
        'activate_controllers': metadata['trajectory_controllers'],
        'deactivate_controllers': metadata['hold_controllers'],
    }


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    def setup(context):
        bimanual = LaunchConfiguration('bimanual').perform(context).lower() == 'true'
        profile = LaunchConfiguration('arm').perform(context) if bimanual else 'single'
        if bimanual and profile not in ('left', 'right', 'both'):
            raise RuntimeError('bimanual OpenArm arm must be left, right, or both')
        metadata = load_moveit_metadata('openarm', 'cho_moveit_openarm', profile)
        share = FindPackageShare(metadata['config_package'])
        move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'move_group.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time, 'bimanual': str(bimanual).lower(),
                              'mit_paired': LaunchConfiguration('mit_paired')}.items())
        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'moveit_rviz.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'bimanual': str(bimanual).lower()}.items())
        mit_paired = LaunchConfiguration('mit_paired').perform(context).lower() == 'true'
        if mit_paired and (not bimanual or profile != 'both'):
            raise RuntimeError('mit_paired requires bimanual:=true and arm:=both')
        gate_controller_parameters = build_gate_controller_parameters(metadata, mit_paired)
        trajectory_controllers = gate_controller_parameters['activate_controllers']
        trajectory_controller = (trajectory_controllers[0] if mit_paired else
                                 metadata['trajectory_controller'])
        static_scene = Node(package='cho_moveit_common', executable='publish_static_scene.py',
            name='openarm_static_planning_scene', output='screen', parameters=[{
                'use_sim_time': use_sim_time, 'ready_service': metadata['ready_service'],
                'frame_id': LaunchConfiguration('floor_frame'), 'size_csv': LaunchConfiguration('floor_size'),
                'position_csv': LaunchConfiguration('floor_position'), 'max_attempts': 10,
                'controller_ready_timeout': ParameterValue(
                    LaunchConfiguration('controller_ready_timeout'), value_type=float),
                'switch_response_timeout': 15.0,
                **gate_controller_parameters}])
        wait = Node(package='cho_moveit_common', executable='wait_for_static_scene.py',
            name='wait_for_openarm_static_planning_scene', output='screen', parameters=[{
                'ready_service': metadata['ready_service'],
                'timeout': ParameterValue(
                    LaunchConfiguration('scene_ready_timeout'), value_type=float)}])
        bridge = Node(package='cho_moveit_common', executable='moveit_action_bridge.py',
            name=f'openarm_{profile}_moveit_action_bridge', output='screen', parameters=[{
                'use_sim_time': use_sim_time, 'robot_type': 'openarm', 'profile': profile,
                'ready_service': metadata['ready_service'], 'planning_group': metadata['planning_group'],
                'ee_link': metadata['ee_link'], 'world_frame': metadata['base_frame'],
                'joint_names': metadata['joint_names'],
                'trajectory_controller': trajectory_controller,
                'trajectory_controllers': trajectory_controllers,
                'supports_task': metadata['supports_task'],
                'max_velocity_scaling_factor': metadata['max_velocity_scaling_factor'],
                'max_acceleration_scaling_factor': metadata['max_acceleration_scaling_factor']}])
        def after_gate(event, gate_context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='OpenArm planning-scene gate failed'))]
            enabled = LaunchConfiguration('launch_rviz').perform(gate_context).lower()
            return [rviz] if enabled in ('true', '1', 'yes', 'on') else []

        return [move_group, bridge, static_scene, wait,
                RegisterEventHandler(OnProcessExit(target_action=wait,
                    on_exit=after_gate))]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('bimanual', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mit_paired', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('arm', default_value='single'),
        DeclareLaunchArgument('floor_frame', default_value='world'),
        DeclareLaunchArgument('floor_size', default_value='4.0,4.0,0.10'),
        DeclareLaunchArgument('floor_position', default_value='0.0,0.0,-0.05'),
        DeclareLaunchArgument('scene_ready_timeout', default_value='210.0'),
        DeclareLaunchArgument('controller_ready_timeout', default_value='90.0'),
        OpaqueFunction(function=setup)])
