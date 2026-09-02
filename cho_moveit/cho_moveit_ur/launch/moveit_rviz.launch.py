from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from cho_robot_config import load_moveit_metadata


def generate_launch_description():
    metadata = load_moveit_metadata('ur5e', 'cho_moveit_ur')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = (
        MoveItConfigsBuilder('ur', package_name=metadata['config_package'])
        .robot_description(file_path='config/ur5e.urdf.xacro', mappings={
            'ur_type': 'ur5e', 'load_gripper': 'false'})
        .robot_description_semantic(file_path='config/ur5e.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs())
    rviz = Node(
        package='rviz2', executable='rviz2', name='moveit_rviz', output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'), 'rviz', 'view_robot.rviz'])],
        parameters=[
            config.robot_description, config.robot_description_semantic,
            config.robot_description_kinematics, config.planning_pipelines,
            config.joint_limits, {'use_sim_time': use_sim_time}])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'), rviz])
