"""Bring up a RealSense D435 and the AprilTag detector against it.

The camera driver is the stock ``realsense2_camera`` package, included through
its own ``rs_launch.py``; the only thing this package contributes to it is a
parameter file, handed over with that launch's ``config_file`` argument. Same
arrangement as ``bota_ft_sensor``: configuration over a vendored driver, no
reimplementation of it.

What comes out is one TF frame per detected tag, named ``tag_<id>``, parented
to the camera's optical frame, plus ``/detections`` carrying the per-detection
decode quality.

What does NOT come out is any transform between the camera and the robot. The
driver publishes the camera's own internals (``camera_link`` down to the
optical frames); the single transform INTO ``camera_link`` is the one thing
that depends on how the camera is mounted -- a bracket on the wrist puts it in
the URDF, a tripod puts it in a static publisher -- and keeping it out of here
is what lets the same launch serve both.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PACKAGE = 'realsense_apriltag'

# USB ids the D435 enumerates under: its own, and the generic identity it
# falls back to on a USB 2 link.
REALSENSE_IDS = ('0b07', '0ad6')

# Tag id -> TF frame name. cho_object_pose.geometry.tag_frame_name() is the
# same function on the consumer side; both exist so that neither package has
# to hand-write a frame string the other one has to match.
TAG_FRAME_PREFIX = 'tag_'


def _usb_link_speed():
    """Mbit/s the RealSense is currently enumerated at, or None if not found.

    Used only to warn. The profile is never chosen automatically: intrinsics
    change with resolution, so a launch that quietly picked a different one
    would silently invalidate a calibration and every pixel-based threshold
    tuned against it.
    """
    for device in glob.glob('/sys/bus/usb/devices/[0-9]*-[0-9]*'):
        try:
            with open(os.path.join(device, 'idVendor'), encoding='utf-8') as handle:
                if handle.read().strip() != '8086':
                    continue
            with open(os.path.join(device, 'idProduct'), encoding='utf-8') as handle:
                if handle.read().strip() not in REALSENSE_IDS:
                    continue
            with open(os.path.join(device, 'speed'), encoding='utf-8') as handle:
                return float(handle.read().strip())
        except (OSError, ValueError):
            continue
    return None


def _tag_frames_override(apriltag_config, frame_prefix=''):
    """Derive ``tag.frames`` from ``tag.ids`` in *apriltag_config*.

    A parameter file cannot compute one of its own keys from another, and a
    hand-written frames list is exactly the kind of duplicate that goes stale
    the first time an id is added. So the list is built here instead, and the
    config file deliberately leaves ``tag.frames`` unset.
    """
    with open(apriltag_config, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    parameters = (document.get('/**') or {}).get('ros__parameters') or {}
    ids = ((parameters.get('tag') or {}).get('ids')) or []
    if not ids:
        raise RuntimeError(
            f'{apriltag_config} declares no tag.ids, so no tag would get a '
            'named TF frame and cho_object_pose would find nothing to look up.')
    return {'tag.frames': [f'{frame_prefix}{TAG_FRAME_PREFIX}{int(tag_id)}'
                           for tag_id in ids]}


def _launch_setup(context, *args, **kwargs):
    stream = LaunchConfiguration('stream').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_namespace = LaunchConfiguration('camera_namespace').perform(context)
    camera_config = LaunchConfiguration('camera_config').perform(context)
    apriltag_config = LaunchConfiguration('apriltag_config').perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    launch_camera = LaunchConfiguration('launch_camera').perform(context).lower() == 'true'
    with_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    if stream not in ('infra1', 'color'):
        raise RuntimeError(f"stream must be 'infra1' or 'color', got '{stream}'")

    # Warn, never adjust. See _usb_link_speed.
    speed = _usb_link_speed()
    if speed is not None and speed < 5000.0 and profile.startswith('848'):
        print(f'\033[33m[{PACKAGE}] the camera is enumerated at {speed:.0f}M (USB 2), where '
              f'the D435 offers only 424x240 and 480x270 on infra, so {profile} will be '
              'refused and the driver will fall back -- grep the log for '
              '"Setting ROS param back to". Pass profile:=480x270x30 to choose the '
              'fallback yourself, and remember intrinsics differ between the two.\033[0m')

    base = f'/{camera_namespace}/{camera_name}'.replace('//', '/')
    nodes = []

    if launch_camera:
        # scoped + forwarding=False, not a bare include: rs_launch.py warns
        # about every launch configuration in its context that is not one of
        # its own parameters, and an unscoped include inherits ours. Without
        # this it prints five screens of yellow text about `stream`,
        # `frame_prefix` and friends, which is how a real warning gets missed.
        nodes.append(GroupAction(scoped=True, forwarding=False, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch', 'rs_launch.py')),
                # The node is given [launch arguments, config file] in that
                # order and the later one wins, so the stream switches have to
                # be arguments -- a copy of them in the config file could not
                # be overridden from the command line.
                launch_arguments={
                    'camera_name': camera_name,
                    'camera_namespace': camera_namespace,
                    'config_file': camera_config,
                    'enable_color': str(stream == 'color').lower(),
                    'enable_infra1': str(stream == 'infra1').lower(),
                    'depth_module.infra_profile': profile,
                    'rgb_camera.color_profile': profile,
                }.items(),
            ),
        ]))

    if stream == 'infra1':
        # The D435 rectifies its IR streams on the device, which is why the
        # topic is called image_rect_raw. No image_proc hop is needed.
        image_topic = f'{base}/infra1/image_rect_raw'
        info_topic = f'{base}/infra1/camera_info'
    else:
        # Colour comes out distorted; apriltag_ros assumes a rectified image
        # and will happily return a biased pose if handed a raw one.
        image_topic = f'{base}/color/image_rect'
        info_topic = f'{base}/color/camera_info'
        nodes.append(Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color',
            namespace=camera_namespace,
            remappings=[('image', f'{base}/color/image_raw'),
                        ('camera_info', info_topic),
                        ('image_rect', image_topic)],
            output='screen',
        ))

    nodes.append(Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[apriltag_config,
                    _tag_frames_override(apriltag_config, frame_prefix)],
        remappings=[('image_rect', image_topic),
                    ('camera_info', info_topic)],
        output='screen',
    ))

    if with_rviz:
        # apriltag_draw subscribes lazily -- it only pulls frames while
        # something is subscribed to /image_tags, which rviz is.
        # Its detection input is called `tags`, NOT `detections`: remapping
        # the latter silently does nothing and the overlay never publishes,
        # which looks exactly like "no tag detected".
        nodes.append(Node(
            package='apriltag_draw',
            executable='apriltag_draw_node',
            name='apriltag_draw',
            remappings=[('image', image_topic),
                        ('tags', '/detections')],
            output='screen',
        ))
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory(PACKAGE), 'rviz', 'apriltag.rviz')],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    """Declare the arguments and defer the wiring to _launch_setup."""
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'stream', default_value='infra1',
            description="'infra1' (global shutter, already rectified, needs the IR "
                        "projector off) or 'color' (rolling shutter, rectified here)."),
        DeclareLaunchArgument(
            'profile', default_value='848x480x30',
            description="'WIDTHxHEIGHTxFPS'. The default needs a USB 3 link; on USB 2 the "
                        'D435 only offers 424x240 and 480x270 on infra, so pass '
                        'profile:=480x270x30 there. Intrinsics differ between the two, so a '
                        'calibration belongs to one profile, not to the camera.'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description='Prepended to every tag frame, e.g. cam0_tag_9. Empty is fine '
                        'for one camera; a second one MUST set it, or both publish '
                        'tag_<id> and the TF tree gains a child with two parents.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start apriltag_draw and rviz2 with a config that shows the '
                        'detection overlay on /image_tags and the tag_<id> TF frames.'),
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='false to run the detector alone, against a bag or a camera '
                        'someone else already started.'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(share, 'config', 'd435.yaml'),
            description="Handed to rs_launch.py's own config_file argument."),
        DeclareLaunchArgument(
            'apriltag_config',
            default_value=os.path.join(share, 'config', 'apriltag_36h11.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])
