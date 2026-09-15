"""Run the AprilTag detector against an already-running camera.

Nothing here starts a camera. Point ``image_topic`` at whatever is publishing --
``cho_realsense``, an OAK, a bag -- and the detector emits ``/detections`` plus
one TF frame per tag, named by the same convention ``cho_object_pose``'s node
looks up.

The image must be RECTIFIED. A stream the device rectifies itself (the D435's
infra1) can be fed straight in; a raw one needs ``rectify:=true``, which
inserts an ``image_proc`` node. Handing a distorted image to the detector does
not fail, it returns a biased pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from cho_object_pose.geometry import tag_frame_name

PACKAGE = 'cho_object_pose'


def _tag_frames_override(apriltag_config, frame_prefix=''):
    """Derive ``tag.frames`` from ``tag.ids`` in *apriltag_config*.

    A parameter file cannot compute one of its own keys from another, and a
    hand-written frames list is exactly the kind of duplicate that goes stale
    the first time an id is added. The name comes from geometry.tag_frame_name,
    which is also what the node looks up, so the two cannot drift.
    """
    with open(apriltag_config, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    parameters = (document.get('/**') or {}).get('ros__parameters') or {}
    ids = ((parameters.get('tag') or {}).get('ids')) or []
    if not ids:
        raise RuntimeError(
            f'{apriltag_config} declares no tag.ids, so no tag would get a '
            'named TF frame and the pose node would find nothing to look up.')
    return {'tag.frames': [tag_frame_name(tag_id, frame_prefix) for tag_id in ids]}


def _launch_setup(context, *args, **kwargs):
    """Wire the detector, and a rectifier ahead of it when asked."""
    image_topic = LaunchConfiguration('image_topic').perform(context)
    if not image_topic:
        raise RuntimeError(
            f'[{PACKAGE}] image_topic is required -- this launch starts no camera. '
            'cho_realsense prints the topic it publishes when it starts.')

    info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    if not info_topic:
        # image_transport's convention: camera_info is the image's sibling.
        info_topic = f"{image_topic.rsplit('/', 1)[0]}/camera_info"

    apriltag_config = LaunchConfiguration('apriltag_config').perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    nodes = []

    if LaunchConfiguration('rectify').perform(context).lower() == 'true':
        rectified = f'{image_topic}_rect'
        nodes.append(Node(
            package='image_proc', executable='rectify_node', name='rectify',
            remappings=[('image', image_topic),
                        ('camera_info', info_topic),
                        ('image_rect', rectified)],
            output='screen'))
        image_topic = rectified

    nodes.append(Node(
        package='apriltag_ros', executable='apriltag_node', name='apriltag_node',
        parameters=[apriltag_config,
                    _tag_frames_override(apriltag_config, frame_prefix)],
        remappings=[('image_rect', image_topic), ('camera_info', info_topic)],
        output='screen'))

    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        # apriltag_draw subscribes lazily -- it only pulls frames while
        # something is subscribed to /image_tags, which rviz is. Its detection
        # input is called `tags`, NOT `detections`: remapping the latter
        # silently does nothing and looks exactly like "no tag detected".
        nodes.append(Node(
            package='apriltag_draw', executable='apriltag_draw_node', name='apriltag_draw',
            remappings=[('image', image_topic), ('tags', '/detections')],
            output='screen'))
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory(PACKAGE), 'rviz', 'apriltag.rviz')],
            output='screen'))
    return nodes


def generate_launch_description():
    """Declare the arguments and defer the wiring to _launch_setup."""
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='',
            description='Rectified image to detect on. Required; no default, because a '
                        'wrong guess here looks like a camera that sees no tags.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='',
            description="Empty derives it as the image topic's sibling camera_info."),
        DeclareLaunchArgument(
            'rectify', default_value='false',
            description='Insert image_proc ahead of the detector. Needed for a raw '
                        'stream, not for one the device rectifies (D435 infra1).'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description='Prepended to every tag frame, e.g. cam0_tag_9. Empty is fine for '
                        'one camera; a second one MUST set it, here and on the pose node, '
                        'or both publish tag_<id> and one TF child gains two parents.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start apriltag_draw and rviz2 with the detection overlay.'),
        DeclareLaunchArgument(
            'apriltag_config',
            default_value=os.path.join(share, 'config', 'apriltag_36h11.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])
