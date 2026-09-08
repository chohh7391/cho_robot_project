from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from cho_robot_config import load_moveit_metadata


def move_group_node(context):
    metadata = load_moveit_metadata('fr5', 'cho_moveit_fr5')
    # MoveItConfigsBuilder expands the xacro eagerly and only takes plain
    # strings, so the gripper has to be resolved here rather than handed in as
    # a LaunchConfiguration.
    gripper = LaunchConfiguration('gripper').perform(context)
    # Registering the cuMotion pipeline only loads the planner plugin; OMPL stays
    # the default, so a request without an explicit pipeline_id is unaffected.
    # The plugin needs the cumotion planner node running to answer at all - see
    # cumotion_planner.launch.py.
    cumotion = LaunchConfiguration('cumotion').perform(context).lower() in (
        'true', '1', 'yes')
    pipelines = ['ompl', 'isaac_ros_cumotion'] if cumotion else ['ompl']
    moveit_config = (
        MoveItConfigsBuilder('fr5', package_name=metadata['config_package'])
        .robot_description(mappings={'hardware': 'mock', 'gripper': gripper})
        # The SRDF is a xacro too: the gripper's disable_collisions pair only
        # applies when the description actually has the gripper.
        .robot_description_semantic(file_path='config/fr5.srdf',
                                    mappings={'gripper': gripper})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=pipelines)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    return [Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_robot_description': True,
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'moveit_manage_controllers': False,
            },
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'cumotion',
            default_value='false',
            description=(
                'Also register the isaac_ros_cumotion planning pipeline. Default '
                'false: OMPL stays the only pipeline and nothing changes. When '
                'true, move_group loads the cuMotion planner plugin, which needs '
                'the planner node from cumotion_planner.launch.py to be running.'
            ),
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='none',
            description=(
                'none | ag95. Must match the gripper the bringup expanded the '
                'description with: otherwise move_group models a different robot '
                'than the controllers drive and rejects gripper_finger_joint out '
                'of /joint_states.'
            ),
        ),
        OpaqueFunction(function=move_group_node),
    ])
