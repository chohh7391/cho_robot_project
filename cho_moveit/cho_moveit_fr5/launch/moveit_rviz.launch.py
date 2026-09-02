from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from cho_robot_config import load_moveit_metadata


def generate_launch_description():
    metadata = load_moveit_metadata('fr5', 'cho_moveit_fr5')
    use_sim_time = LaunchConfiguration('use_sim_time')
    moveit_config = (
        MoveItConfigsBuilder('fr5', package_name=metadata['config_package'])
        .robot_description(mappings={'hardware': 'mock'})
        .robot_description_semantic(file_path='config/fr5.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(metadata['config_package']), 'rviz', 'moveit.rviz']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rviz,
    ])
