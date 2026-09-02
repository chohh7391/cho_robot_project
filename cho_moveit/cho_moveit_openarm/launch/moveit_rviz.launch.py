from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from cho_robot_config import load_moveit_metadata


def build_config(bimanual=False):
    # Launch files are installed as standalone scripts and are not importable as
    # sibling Python modules. Keep this small builder local, as the UR and Franka
    # MoveIt packages do, so installed launch resolution is deterministic.
    metadata = load_moveit_metadata(
        'openarm', 'cho_moveit_openarm', 'left' if bimanual else 'single')
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
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs())


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    def start(context):
        bimanual = LaunchConfiguration('bimanual').perform(context).lower() == 'true'
        config = build_config(bimanual)
        return [Node(package='rviz2', executable='rviz2', name='moveit_rviz', output='screen',
                     parameters=[config.robot_description, config.robot_description_semantic,
                                 config.robot_description_kinematics,
                                 config.planning_pipelines, config.joint_limits,
                                 {'use_sim_time': use_sim_time}])]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bimanual', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start)])
