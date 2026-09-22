"""One AprilTag detector per camera on the bench.

Nothing here starts a camera either -- ``cho_realsense`` and ``cho_oak`` do
that. This is the piece between them and the pose node: it reads
``config/cameras.yaml`` and includes ``apriltag.launch.py`` once per entry, so
every camera gets its own detector node, its own detections topic and its own
tag frame prefix.

The prefix is the reason this exists rather than three hand-written launch
invocations. Detectors left at the default all publish ``tag_0``, which gives
one TF child three parents -- not an error, just transforms that intermittently
resolve through whichever camera published last. Deriving both sides from one
file is what makes that unrepresentable.

    ros2 launch cho_object_pose detectors.launch.py
    ros2 topic hz /side_1/detections /side_2/detections /wrist/detections
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml

from cho_object_pose.cameras import parse_cameras

PACKAGE = 'cho_object_pose'


def _launch_setup(context, *args, **kwargs):
    """Include the detector launch once per configured camera."""
    share = get_package_share_directory(PACKAGE)
    cameras_config = LaunchConfiguration('cameras_config').perform(context)
    with open(cameras_config, encoding='utf-8') as stream:
        cameras = parse_cameras(yaml.safe_load(stream))

    apriltag_config = LaunchConfiguration('apriltag_config').perform(context)
    objects_config = LaunchConfiguration('objects_config').perform(context)
    detector_launch = os.path.join(share, 'launch', 'apriltag.launch.py')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detector_launch),
            launch_arguments={
                'image_topic': camera.image_topic,
                'camera_info_topic': camera.camera_info_topic,
                # The OAK's mono streams come out unrectified; the D435's
                # infra1 is rectified on the device. The table says which.
                'rectify': 'true' if camera.rectify else 'false',
                'frame_prefix': camera.frame_prefix,
                'detections_topic': camera.detections_topic,
                'node_name': f'apriltag_{camera.name}',
                'apriltag_config': apriltag_config,
                # Every camera looks for the same tags at the same printed
                # size: that is a property of the job, not of the camera.
                'objects_config': objects_config,
            }.items(),
        )
        for camera in cameras
    ]


def generate_launch_description():
    """Declare the arguments and defer the wiring to _launch_setup."""
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config',
            default_value=os.path.join(share, 'config', 'cameras.yaml'),
            description='The cameras to run a detector for. The pose node must be given '
                        'the SAME file, or it looks up tag frames no detector publishes.'),
        DeclareLaunchArgument(
            'apriltag_config',
            default_value=os.path.join(share, 'config', 'apriltag_36h11.yaml'),
            description='Shared by every detector: the family, decimate and decode gates '
                        'are properties of the lens and the decoder, not of the camera.'),
        DeclareLaunchArgument(
            'objects_config', default_value='',
            description="The task's object table, forwarded to every detector as the "
                        'source of tag.ids / tag.sizes / tag.frames. Give the SAME file '
                        'the pose node gets, or they look for different tags.'),
        OpaqueFunction(function=_launch_setup),
    ])
