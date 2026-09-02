from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from cho_robot_config import load_moveit_metadata


def build_config(bimanual=False, mit_paired=False):
    profile = 'left' if bimanual else 'single'
    metadata = load_moveit_metadata('openarm', 'cho_moveit_openarm', profile)
    return (
        MoveItConfigsBuilder('openarm', package_name=metadata['config_package'])
        .robot_description(file_path='config/openarm.urdf.xacro', mappings={
            'ros2_control': 'false', 'hardware': 'mujoco',
            'control_mode': 'position', 'bimanual': str(bimanual).lower()})
        .robot_description_semantic(file_path=(
            'config/openarm_bimanual.srdf' if bimanual else 'config/openarm.srdf'))
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path=(
            'config/joint_limits_bimanual.yaml' if bimanual else 'config/joint_limits.yaml'))
        .trajectory_execution(file_path=(
            ('config/moveit_controllers_bimanual_mit.yaml' if mit_paired else
             'config/moveit_controllers_bimanual.yaml')
            if bimanual else 'config/moveit_controllers.yaml'))
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .planning_scene_monitor(
            publish_planning_scene=True, publish_geometry_updates=True,
            publish_state_updates=True, publish_transforms_updates=True)
        .to_moveit_configs())


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    def start(context):
        bimanual = LaunchConfiguration('bimanual').perform(context).lower() == 'true'
        mit_paired = LaunchConfiguration('mit_paired').perform(context).lower() == 'true'
        if mit_paired and not bimanual:
            raise RuntimeError('mit_paired requires bimanual:=true')
        config = build_config(bimanual, mit_paired)
        return [Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                     parameters=[config.to_dict(), {
                         'use_sim_time': use_sim_time,
                         'publish_robot_description': True,
                         'publish_robot_description_semantic': True,
                         'allow_trajectory_execution': True,
                         'moveit_manage_controllers': False,
                     }])]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bimanual', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mit_paired', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
