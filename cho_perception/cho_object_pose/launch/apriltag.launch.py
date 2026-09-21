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
from cho_object_pose.objects import parse_objects

PACKAGE = 'cho_object_pose'


def _detector_parameters(apriltag_config):
    """Read the detector config's own ``ros__parameters`` mapping."""
    with open(apriltag_config, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    return (document.get('/**') or {}).get('ros__parameters') or {}


def _tag_overrides(apriltag_config, objects_config='', frame_prefix=''):
    """Derive ``tag.ids`` / ``tag.sizes`` / ``tag.frames`` for the detector.

    A parameter file cannot compute one of its own keys from another, and a
    hand-written frames list is exactly the kind of duplicate that goes stale
    the first time an id is added. The names come from geometry.tag_frame_name,
    which is also what the pose node looks up, so the two cannot drift.

    WHICH tags exist and HOW BIG they are printed follow the job, not the
    optics, so when an object table is given it is the source for all three.
    That is what keeps a task from having to edit this package to add an
    object, and what stops every job's tag ids piling up in a detector config
    that has no reason to know about any of them. The detector config keeps
    what really is a property of the lens and the decoder -- family, decimate,
    max_hamming, qos_profile -- and its own ids/sizes as the standalone default.

    An object that declares no ``tag_size`` falls back to the detector's
    ``size``, and the caller logs which size each tag actually got: silently
    detecting a 39 mm tag as a 40 mm one is a 2.5% range error that nothing
    downstream can see.
    """
    parameters = _detector_parameters(apriltag_config)
    tag = parameters.get('tag') or {}
    default_size = float(parameters.get('size', 0.0)) or None

    if objects_config:
        specs = parse_objects(yaml.safe_load(
            open(objects_config, encoding='utf-8')))
        ids = [spec.tag_id for spec in specs]
        sizes = [spec.tag_size if spec.tag_size is not None else default_size
                 for spec in specs]
        if any(size is None for size in sizes):
            raise RuntimeError(
                f'{objects_config} has an object with no tag_size and '
                f'{apriltag_config} declares no `size` default either, so the '
                'detector would have no edge length to estimate range from.')
        source = objects_config
    else:
        ids = list(tag.get('ids') or [])
        sizes = list(tag.get('sizes') or [])
        if sizes and len(sizes) != len(ids):
            raise RuntimeError(
                f'{apriltag_config} declares {len(ids)} tag.ids and '
                f'{len(sizes)} tag.sizes; they are parallel lists.')
        if not sizes:
            sizes = [default_size] * len(ids)
        source = apriltag_config

    if not ids:
        raise RuntimeError(
            f'{source} declares no tags, so none would get a named TF frame '
            'and the pose node would find nothing to look up.')

    overrides = {
        'tag.ids': [int(tag_id) for tag_id in ids],
        'tag.sizes': [float(size) for size in sizes],
        'tag.frames': [tag_frame_name(tag_id, frame_prefix) for tag_id in ids],
    }
    overrides['_summary'] = ', '.join(
        f'{frame} = {size * 1e3:.1f} mm'
        for frame, size in zip(overrides['tag.frames'], overrides['tag.sizes']))
    return overrides


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
    detections_topic = LaunchConfiguration('detections_topic').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)
    objects_config = LaunchConfiguration('objects_config').perform(context)
    overrides = _tag_overrides(apriltag_config, objects_config, frame_prefix)
    # Said out loud: a tag detected at the wrong edge length returns a
    # confident, wrong range, and nothing downstream can tell.
    print(f'[{PACKAGE}] {node_name} looking for {overrides.pop("_summary")}'
          + (f' (from {objects_config})' if objects_config else ''))
    nodes = []

    if LaunchConfiguration('rectify').perform(context).lower() == 'true':
        rectified = f'{image_topic}_rect'
        nodes.append(Node(
            package='image_proc', executable='rectify_node', name=f'{node_name}_rectify',
            remappings=[('image', image_topic),
                        ('camera_info', info_topic),
                        ('image_rect', rectified)],
            output='screen'))
        image_topic = rectified

    nodes.append(Node(
        package='apriltag_ros', executable='apriltag_node', name=node_name,
        parameters=[apriltag_config, overrides],
        remappings=[('image_rect', image_topic), ('camera_info', info_topic),
                    ('detections', detections_topic)],
        output='screen'))

    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        # apriltag_draw subscribes lazily -- it only pulls frames while
        # something is subscribed to /image_tags, which rviz is. Its detection
        # input is called `tags`, NOT `detections`: remapping the latter
        # silently does nothing and looks exactly like "no tag detected".
        nodes.append(Node(
            package='apriltag_draw', executable='apriltag_draw_node',
            name=f'{node_name}_draw',
            remappings=[('image', image_topic), ('tags', detections_topic)],
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
            'objects_config', default_value='',
            description="The task's object table (cho_task_manager/config/perception/). "
                        'Given, it is the source for tag.ids, tag.sizes and tag.frames -- '
                        'which tags exist and how big they are printed follows the job, '
                        'not the lens. Empty uses the detector config\'s own ids/sizes.'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description='Prepended to every tag frame, e.g. cam0_tag_9. Empty is fine for '
                        'one camera; a second one MUST set it, here and on the pose node, '
                        'or both publish tag_<id> and one TF child gains two parents.'),
        DeclareLaunchArgument(
            'detections_topic', default_value='/detections',
            description='Where this detector publishes. One camera can keep the default; '
                        'several MUST each get their own, or three detectors publish into '
                        'one topic and the pose node cannot tell whose view it is reading.'),
        DeclareLaunchArgument(
            'node_name', default_value='apriltag_node',
            description='Node name, so several detector instances can coexist. Two nodes '
                        'with one name is not an error, it is a parameter set that '
                        'whichever started last wins.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start apriltag_draw and rviz2 with the detection overlay.'),
        DeclareLaunchArgument(
            'apriltag_config',
            default_value=os.path.join(share, 'config', 'apriltag_36h11.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])
