"""Run the object-pose node against an already-running detector.

Kept separate from realsense_apriltag's launch so that the perception half can
be restarted, retuned or run against a bag without touching the camera.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare the tuning arguments and start the node."""
    share = get_package_share_directory('cho_object_pose')
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='franka'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='Empty takes model.arm_base_link from cho_robot_config. That is '
                        'NOT model.base_frame, which is world for Franka and absent from '
                        'the published TF tree.'),
        DeclareLaunchArgument(
            'objects_config',
            default_value=os.path.join(share, 'config', 'objects.yaml')),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description="Must match the detector's frame_prefix. Only needed once a "
                        'second camera exists, and then it is mandatory on both sides.'),
        DeclareLaunchArgument('min_samples', default_value='5'),
        DeclareLaunchArgument('window_sec', default_value='0.5'),
        DeclareLaunchArgument('max_position_spread_m', default_value='0.01'),
        DeclareLaunchArgument('report_period_sec', default_value='2.0'),
        Node(
            package='cho_object_pose',
            executable='object_pose_node',
            name='object_pose_node',
            # A LaunchConfiguration is a string. Without an explicit value_type
            # the numeric arguments arrive as strings and the node's typed
            # parameter declarations reject them at start-up.
            parameters=[{
                'robot_type': LaunchConfiguration('robot_type'),
                'base_frame': LaunchConfiguration('base_frame'),
                'objects_config': LaunchConfiguration('objects_config'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'min_samples': ParameterValue(
                    LaunchConfiguration('min_samples'), value_type=int),
                'window_sec': ParameterValue(
                    LaunchConfiguration('window_sec'), value_type=float),
                'max_position_spread_m': ParameterValue(
                    LaunchConfiguration('max_position_spread_m'), value_type=float),
                'report_period_sec': ParameterValue(
                    LaunchConfiguration('report_period_sec'), value_type=float),
            }],
            output='screen',
        ),
    ])
