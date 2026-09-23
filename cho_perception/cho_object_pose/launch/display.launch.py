"""An rviz that stays up across task runs and remembers what the cameras found.

    ros2 launch cho_object_pose display.launch.py

Start it once, beside the bringup and the camera stack, and leave it. Every task
that includes ``object_pose.launch.py`` starts its own pose node and takes it
down again when the tree finishes; this launch owns neither. What it adds is
``object_marker_memory``, which holds each object on screen after its
detections stop -- full strength for ``hold_sec``, then fading over ``fade_sec``
to ``floor_alpha`` of its own alpha -- and draws a line of sight from every
camera that is seeing it right now. rviz shows that node's output only, never
the pose node's raw markers, so nothing is drawn twice.

``cameras_config`` defaults to this package's cameras.yaml, the same file the
task launch is given as ``object_pose_cameras_config``. It only supplies the
optical frame each sight line starts from; pass the same file both places.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('cho_object_pose')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'cameras_config', default_value=os.path.join(share, 'config', 'cameras.yaml'),
            description='Camera table, for each sight line\'s optical frame. Empty draws '
                        'no sight lines.'),
        DeclareLaunchArgument('marker_topic', default_value='/perception/object_markers',
                              description='What the pose node draws on.'),
        DeclareLaunchArgument('memory_topic', default_value='/perception/object_markers/memory',
                              description='What rviz is pointed at.'),
        DeclareLaunchArgument('visibility_topic', default_value='/perception/object_visibility'),
        DeclareLaunchArgument('hold_sec', default_value='3.0',
                              description='Seconds an object stays at full strength after '
                                          'its last detection.'),
        DeclareLaunchArgument('fade_sec', default_value='5.0',
                              description='Seconds it then takes to dim to floor_alpha.'),
        DeclareLaunchArgument('floor_alpha', default_value='0.25',
                              description='Fraction of its own alpha a long-unseen object '
                                          'keeps. 0 fades it out completely.'),
        DeclareLaunchArgument('forget_sec', default_value='0.0',
                              description='Stop drawing an object this long after it was last '
                                          'seen. 0 keeps it until this launch is stopped.'),
        DeclareLaunchArgument('sight_lines', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(share, 'rviz', 'object_memory.rviz')),

        Node(
            package='cho_object_pose',
            executable='object_marker_memory',
            name='object_marker_memory',
            output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'cameras_config': LaunchConfiguration('cameras_config'),
                'input_topic': LaunchConfiguration('marker_topic'),
                'output_topic': LaunchConfiguration('memory_topic'),
                'visibility_topic': LaunchConfiguration('visibility_topic'),
                'hold_sec': ParameterValue(LaunchConfiguration('hold_sec'), value_type=float),
                'fade_sec': ParameterValue(LaunchConfiguration('fade_sec'), value_type=float),
                'floor_alpha_fraction': ParameterValue(
                    LaunchConfiguration('floor_alpha'), value_type=float),
                'forget_sec': ParameterValue(LaunchConfiguration('forget_sec'), value_type=float),
                'draw_sight_lines': ParameterValue(
                    LaunchConfiguration('sight_lines'), value_type=bool),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='object_memory_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': sim_time,
            }],
            output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
