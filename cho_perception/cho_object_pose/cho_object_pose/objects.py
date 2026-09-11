"""Parsing and validation of the tag-to-object table.

Kept apart from the node for the same reason as ``geometry``: a malformed
objects file should fail a unit test, not a robot.
"""

from collections import namedtuple
import math

from cho_object_pose.geometry import quat_normalize
import numpy as np

DEFAULT_TOPIC_PREFIX = '/perception/object_pose'

ObjectSpec = namedtuple(
    'ObjectSpec',
    'name tag_id topic offset_position offset_orientation top_down_yaw yaw_axis')


def _vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{label} must be a list of {length} numbers')
    if not all(not isinstance(item, bool) and isinstance(item, (int, float))
               and math.isfinite(item) for item in value):
        raise ValueError(f'{label} must contain only finite numbers')
    return np.array([float(item) for item in value])


def parse_objects(document, topic_prefix=DEFAULT_TOPIC_PREFIX):
    """Turn a loaded objects YAML document into validated ObjectSpecs.

    Two objects on one tag id is rejected rather than resolved by ordering:
    the pipeline would publish two different poses from the same detection and
    only the timing would decide which one a task latched.
    """
    if not isinstance(document, dict):
        raise ValueError('objects config must be a mapping')
    entries = document.get('objects')
    if not isinstance(entries, list) or not entries:
        raise ValueError("objects config must contain a non-empty 'objects' list")

    specs = []
    for index, entry in enumerate(entries):
        label = f'objects[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{label} must be a mapping')

        name = entry.get('name')
        if not isinstance(name, str) or not name:
            raise ValueError(f'{label}.name is required')

        tag_id = entry.get('tag_id')
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
            raise ValueError(f'{label}.tag_id must be a non-negative integer')

        offset = entry.get('grasp_offset') or {}
        if not isinstance(offset, dict):
            raise ValueError(f'{label}.grasp_offset must be a mapping')
        offset_position = _vector(offset.get('position', [0.0, 0.0, 0.0]), 3,
                                  f'{label}.grasp_offset.position')
        offset_orientation = quat_normalize(
            _vector(offset.get('orientation', [0.0, 0.0, 0.0, 1.0]), 4,
                    f'{label}.grasp_offset.orientation'))

        yaw_axis = _vector(entry.get('yaw_axis', [1.0, 0.0, 0.0]), 3, f'{label}.yaw_axis')
        if float(np.linalg.norm(yaw_axis)) < 1e-9:
            raise ValueError(f'{label}.yaw_axis must not be the zero vector')

        top_down_yaw = entry.get('top_down_yaw', True)
        if not isinstance(top_down_yaw, bool):
            raise ValueError(f'{label}.top_down_yaw must be true or false')

        topic = entry.get('topic') or f'{topic_prefix}/{name}'
        if not isinstance(topic, str) or not topic:
            raise ValueError(f'{label}.topic must be a non-empty string')

        specs.append(ObjectSpec(name, tag_id, topic, offset_position, offset_orientation,
                                top_down_yaw, yaw_axis / np.linalg.norm(yaw_axis)))

    for field, what in (('name', 'names'), ('tag_id', 'tag ids'), ('topic', 'topics')):
        seen = [getattr(spec, field) for spec in specs]
        duplicates = sorted({item for item in seen if seen.count(item) > 1})
        if duplicates:
            raise ValueError(f'duplicate object {what}: {duplicates}')
    return specs
