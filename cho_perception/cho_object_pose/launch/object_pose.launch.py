"""Run the object-pose node against already-running detectors.

Kept separate from the detector launch beside it so that the perception half can
be restarted, retuned or run against a bag without touching the camera.

``cameras_config`` is what turns one camera into several. Give it the same file
``detectors.launch.py`` was given and the node subscribes to every detector and
fuses their views of each tag; leave it empty and the historical single-camera
``detections_topic`` / ``frame_prefix`` pair is used instead.
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
    """Declare the tuning arguments and start the node."""
    share = get_package_share_directory('cho_object_pose')
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='franka'),
        # IT HAS TO MATCH THE CAMERA DRIVERS AND THE BRINGUP. Every age on the
        # visibility topic is this node's clock minus an image stamp, so a node
        # on wall time against drivers on /clock reports every camera stale
        # while it is publishing poses. The node says so out loud when it
        # happens, but the fix is here.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='Empty takes model.arm_base_link from cho_robot_config. That is '
                        'NOT model.base_frame, which is world for Franka and absent from '
                        'the published TF tree.'),
        DeclareLaunchArgument(
            'objects_config',
            default_value=os.path.join(share, 'config', 'objects.yaml')),
        DeclareLaunchArgument(
            'cameras_config', default_value='',
            description='The cameras to fuse (see config/cameras.yaml). MUST be the same '
                        'file detectors.launch.py was given. Empty uses the single-camera '
                        'detections_topic / frame_prefix pair below.'),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description="Must match the detector's frame_prefix. Only needed once a "
                        'second camera exists, and then it is mandatory on both sides.'),
        DeclareLaunchArgument('min_samples', default_value='5'),
        DeclareLaunchArgument(
            'min_cameras', default_value='1',
            description='How many different cameras must have contributed to the window. '
                        '1 publishes on whichever camera can see the tag; 2 makes '
                        'independent agreement a requirement, since the spread gate then '
                        'has to be met ACROSS cameras and not just over time.'),
        DeclareLaunchArgument('window_sec', default_value='0.5'),
        DeclareLaunchArgument('max_position_spread_m', default_value='0.01'),
        DeclareLaunchArgument('report_period_sec', default_value='2.0'),
        DeclareLaunchArgument(
            'publish_visibility', default_value='true',
            description='Publish what every camera can see of every object, as '
                        'cho_interfaces/ObjectVisibilityArray. This is what a task tree '
                        'triggers an occlusion recovery from; the periodic log says the '
                        'same thing to a human.'),
        DeclareLaunchArgument('visibility_topic',
                              default_value='/perception/object_visibility'),
        DeclareLaunchArgument(
            'visibility_period_sec', default_value='0.2',
            description='How often to publish it. A rate, not a lifetime -- how old a '
                        "camera's word may be before it stops counting is window_sec."),
        DeclareLaunchArgument(
            'publish_markers', default_value='true',
            description='Draw each object at its detected pose, sized from the object '
                        "table's `shape`, for rviz beside the robot model."),
        DeclareLaunchArgument('marker_topic', default_value='/perception/object_markers'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start rviz2 with the robot model, TF and the object markers. '
                        'The robot model comes from the bringup, not from here.'),
        Node(
            package='cho_object_pose',
            executable='object_pose_node',
            name='object_pose_node',
            # A LaunchConfiguration is a string. Without an explicit value_type
            # the numeric arguments arrive as strings and the node's typed
            # parameter declarations reject them at start-up.
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'robot_type': LaunchConfiguration('robot_type'),
                'base_frame': LaunchConfiguration('base_frame'),
                'objects_config': LaunchConfiguration('objects_config'),
                'cameras_config': LaunchConfiguration('cameras_config'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'min_samples': ParameterValue(
                    LaunchConfiguration('min_samples'), value_type=int),
                'min_cameras': ParameterValue(
                    LaunchConfiguration('min_cameras'), value_type=int),
                'window_sec': ParameterValue(
                    LaunchConfiguration('window_sec'), value_type=float),
                'max_position_spread_m': ParameterValue(
                    LaunchConfiguration('max_position_spread_m'), value_type=float),
                'report_period_sec': ParameterValue(
                    LaunchConfiguration('report_period_sec'), value_type=float),
                'publish_visibility': ParameterValue(
                    LaunchConfiguration('publish_visibility'), value_type=bool),
                'visibility_topic': LaunchConfiguration('visibility_topic'),
                'visibility_period_sec': ParameterValue(
                    LaunchConfiguration('visibility_period_sec'), value_type=float),
                'publish_markers': ParameterValue(
                    LaunchConfiguration('publish_markers'), value_type=bool),
                'marker_topic': LaunchConfiguration('marker_topic'),
            }],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'objects.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
