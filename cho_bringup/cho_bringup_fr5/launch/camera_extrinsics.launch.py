"""Publish where each camera sits relative to the FR5.

One ``static_transform_publisher`` per entry in
``config/real/camera_extrinsics.yaml``, which is where the measurements and
what checked them are written down.

Separate from ``bringup_real_robot.launch.py`` on purpose. The cameras are a
bench, not a robot: a replay or a controller test wants the arm without them,
and a perception session wants them back after the tripod moved without
restarting the arm. Including them in the robot bringup would make "the
extrinsics are stale" a reason to power-cycle the FR5.

It is, however, ordered after the robot: the wrist entry hangs off
``wrist3_link``, so without the robot's TF a wrist detection has no path to the
base frame. The side camera's does not care.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PACKAGE = 'cho_bringup_fr5'


def load_transforms(path):
    """Return the validated transform list from an extrinsics file.

    Importable so a test can check the shipped file without a launch context:
    a mistyped quaternion here rotates every object pose downstream and nothing
    at runtime would report it.
    """
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError('camera extrinsics must be a mapping')
    entries = document.get('transforms')
    if not isinstance(entries, list) or not entries:
        raise ValueError("camera extrinsics must contain a non-empty 'transforms' list")

    transforms = []
    for index, entry in enumerate(entries):
        label = f'transforms[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{label} must be a mapping')
        for key in ('name', 'parent_frame', 'child_frame'):
            value = entry.get(key)
            if not isinstance(value, str) or not value:
                raise ValueError(f'{label}.{key} must be a non-empty string')
        if entry['parent_frame'] == entry['child_frame']:
            raise ValueError(f'{label} parents {entry["child_frame"]} to itself')

        for key, length in (('xyz', 3), ('quaternion', 4)):
            value = entry.get(key)
            if (not isinstance(value, list) or len(value) != length
                    or not all(isinstance(item, (int, float))
                               and not isinstance(item, bool) for item in value)):
                raise ValueError(f'{label}.{key} must be {length} numbers')

        # A quaternion that is not unit length is a transcription error, and it
        # scales as well as rotates -- which reads downstream as a camera with
        # the wrong intrinsics rather than as a bad transform.
        norm = sum(component * component for component in entry['quaternion']) ** 0.5
        if abs(norm - 1.0) > 1e-4:
            raise ValueError(
                f'{label}.quaternion has length {norm:.6f}, not 1. Four numbers in '
                'x, y, z, w order -- some CAD tools print w first.')
        transforms.append(entry)

    # Two publishers on one child frame is the silent kind of conflict: no
    # error, just a transform that resolves through whichever arrived last.
    children = [entry['child_frame'] for entry in transforms]
    duplicates = sorted({name for name in children if children.count(name) > 1})
    if duplicates:
        raise ValueError(f'two transforms into the same child frame: {duplicates}')
    return transforms


def _publishers(context, *args, **kwargs):
    path = LaunchConfiguration('extrinsics_config').perform(context)
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"camera_extrinsic_{entry['name']}",
            arguments=[
                '--x', str(entry['xyz'][0]),
                '--y', str(entry['xyz'][1]),
                '--z', str(entry['xyz'][2]),
                '--qx', str(entry['quaternion'][0]),
                '--qy', str(entry['quaternion'][1]),
                '--qz', str(entry['quaternion'][2]),
                '--qw', str(entry['quaternion'][3]),
                '--frame-id', entry['parent_frame'],
                '--child-frame-id', entry['child_frame'],
            ],
        )
        for entry in load_transforms(path)
    ]


def generate_launch_description():
    """Start one static transform publisher per camera in the extrinsics file."""
    default = os.path.join(get_package_share_directory(PACKAGE),
                           'config', 'real', 'camera_extrinsics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_config', default_value=default,
            description='Measured camera poses for this cell. Override it to keep a '
                        'second bench, rather than editing the file in place.'),
        OpaqueFunction(function=_publishers),
    ])
