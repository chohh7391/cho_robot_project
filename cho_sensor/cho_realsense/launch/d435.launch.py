"""Bring up a RealSense D435 through the stock realsense2_camera launch.

The driver is used exactly as packaged; the only thing this package contributes
is a parameter file, handed over with rs_launch.py's own ``config_file``
argument. Same arrangement as ``bota_ft_sensor``: configuration over a vendored
driver, no reimplementation of it.

No transform between the camera and any robot link is published here. The
driver publishes the camera's own internals (``camera_link`` down to the optical
frames); the single transform INTO ``camera_link`` depends on how the camera is
mounted, and keeping it out of here is what lets the same launch serve a wrist
bracket and a tripod.
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

PACKAGE = 'cho_realsense'

# USB ids the D435 enumerates under: its own, and the generic identity it
# falls back to on a USB 2 link.
REALSENSE_IDS = ('0b07', '0ad6')


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


def _launch_setup(context, *args, **kwargs):
    """Warn about the link, then include rs_launch.py with our parameters."""
    stream = LaunchConfiguration('stream').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_namespace = LaunchConfiguration('camera_namespace').perform(context)

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

    # rs_launch writes its launch arguments into a YAML parameter file, where an
    # all-digit serial parses as an integer and the node refuses it: "parameter
    # {serial_no} is of type {string}, setting it to {integer} is not allowed".
    # A leading underscore keeps YAML on the string branch and the driver strips
    # it -- upstream's documented workaround, confirmed on hardware here.
    serial_no = LaunchConfiguration('serial_no').perform(context)
    if serial_no.isdigit():
        serial_no = f'_{serial_no}'

    base = f'/{camera_namespace}/{camera_name}'.replace('//', '/')
    suffix = 'infra1/image_rect_raw' if stream == 'infra1' else 'color/image_raw'
    print(f'[{PACKAGE}] publishing {base}/{suffix}')

    # scoped + forwarding=False, not a bare include: rs_launch.py warns about
    # every launch configuration in its context that is not one of its own
    # parameters, and an unscoped include inherits ours -- five screens of
    # yellow text, which is how a real warning gets missed.
    return [GroupAction(scoped=True, forwarding=False, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')),
            # The node is given [launch arguments, config file] in that order
            # and the later one wins, so the stream switches have to be
            # arguments -- a copy of them in the config file could not be
            # overridden from the command line.
            launch_arguments={
                'camera_name': camera_name,
                'camera_namespace': camera_namespace,
                'serial_no': serial_no,
                'config_file': LaunchConfiguration('camera_config').perform(context),
                'enable_color': str(stream == 'color').lower(),
                'enable_infra1': str(stream == 'infra1').lower(),
                'depth_module.infra_profile': profile,
                'rgb_camera.color_profile': profile,
            }.items(),
        ),
    ])]


def generate_launch_description():
    """Declare the arguments and defer the wiring to _launch_setup."""
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'stream', default_value='infra1',
            description="'infra1' (global shutter, already rectified on the device) or "
                        "'color' (rolling shutter, needs rectifying downstream)."),
        DeclareLaunchArgument(
            'profile', default_value='848x480x30',
            description="'WIDTHxHEIGHTxFPS'. The default needs a USB 3 link; on USB 2 the "
                        'D435 only offers 424x240 and 480x270 on infra. Intrinsics differ '
                        'between profiles, so a calibration belongs to one, not to a camera.'),
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='Which unit to open, as rs-enumerate-devices prints it. Empty '
                        'takes whichever the driver finds first, undefined with two of '
                        'the same camera. See cho_camera_calibration/config/cameras.yaml.'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(share, 'config', 'd435.yaml'),
            description="Handed to rs_launch.py's own config_file argument."),
        OpaqueFunction(function=_launch_setup),
    ])
