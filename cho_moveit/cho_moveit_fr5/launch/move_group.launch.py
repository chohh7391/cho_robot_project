from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description': True,
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'moveit_manage_controllers': False,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        move_group,
    ])
