from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from cho_robot_config import load_moveit_metadata


def moveit_config():
    metadata = load_moveit_metadata('franka', 'cho_moveit_franka')
    return (
        MoveItConfigsBuilder('fr3', package_name=metadata['config_package'])
        .robot_description(file_path='config/fr3.urdf.xacro', mappings={
            'hand': 'true', 'ros2_control': 'false', 'gazebo': 'true',
            'special_connection': '', 'xyz_ee': '0 0 0'})
        .robot_description_semantic(file_path='config/fr3.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs())


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = moveit_config()
    rviz = Node(
        package='rviz2', executable='rviz2', name='moveit_rviz', output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('cho_description_franka'), 'rviz', 'visualize_franka.rviz'])],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, {'use_sim_time': use_sim_time}])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'), rviz])
