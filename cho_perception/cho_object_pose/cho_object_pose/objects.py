"""Parsing and validation of the tag-to-object table.

Kept apart from the node for the same reason as ``geometry``: a malformed
objects file should fail a unit test, not a robot.
"""

from collections import namedtuple
import math

from cho_object_pose.geometry import quat_normalize
import numpy as np

DEFAULT_TOPIC_PREFIX = '/perception/object_pose'

# What rviz may be asked to draw for an object. Three primitives and one escape
# hatch: `mesh` names a file, for a bench that has a model of the real glassware
# and would rather look at it than at a cylinder.
#
# A mesh is held to the SAME contract as the primitives -- `size` is the body's
# bounding box in metres and `origin` is where its centre goes -- so the file
# has to be normalised to a unit box centred on itself. cho_task_manager's
# meshes/usd_to_stl.py does that on export, and the reason is that otherwise
# every mesh would need its own scale factor derived from whatever units its
# author happened to use, which is a number nobody can check by looking.
MARKER_SHAPES = ('cylinder', 'sphere', 'box', 'mesh')

DEFAULT_SHAPE_COLOR = (0.2, 0.7, 1.0, 0.45)

ObjectSpec = namedtuple(
    'ObjectSpec',
    'name tag_id tag_size topic offset_position offset_orientation top_down_yaw '
    'yaw_axis shape')

#: The object's body, for display only. ``origin`` is where the body's centre
#: sits relative to the PUBLISHED pose, which for a standoff is below it -- so
#: the marker shows the vessel on the bench rather than floating at the pose the
#: arm drives to.
ShapeSpec = namedtuple('ShapeSpec', 'type size origin color resource')


def _vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{label} must be a list of {length} numbers')
    if not all(not isinstance(item, bool) and isinstance(item, (int, float))
               and math.isfinite(item) for item in value):
        raise ValueError(f'{label} must contain only finite numbers')
    return np.array([float(item) for item in value])


def _parse_shape(entry, label):
    """Return the object's display body, or None when it declares no shape.

    None is not a failure: a marker is for looking at, and an object nobody
    needs to see in rviz should not have to invent dimensions for one.
    """
    shape = entry.get('shape')
    if shape is None:
        return None
    if not isinstance(shape, dict):
        raise ValueError(f'{label}.shape must be a mapping')

    kind = shape.get('type')
    if kind not in MARKER_SHAPES:
        raise ValueError(f'{label}.shape.type must be one of {list(MARKER_SHAPES)}')

    size = _vector(shape.get('size'), 3, f'{label}.shape.size')
    if not all(value > 0.0 for value in size):
        raise ValueError(f'{label}.shape.size must be three positive extents in metres')

    origin = _vector(shape.get('origin', [0.0, 0.0, 0.0]), 3, f'{label}.shape.origin')
    color = _vector(shape.get('color', list(DEFAULT_SHAPE_COLOR)), 4, f'{label}.shape.color')
    if not all(0.0 <= value <= 1.0 for value in color):
        raise ValueError(f'{label}.shape.color must be four values in [0, 1] (r, g, b, a)')

    resource = shape.get('resource')
    if kind == 'mesh':
        if not isinstance(resource, str) or not resource:
            raise ValueError(f'{label}.shape.resource is required for a mesh, as a '
                             'package:// or file:// URI rviz can load')
    elif resource is not None:
        raise ValueError(f'{label}.shape.resource only applies to a mesh, not to {kind}')

    return ShapeSpec(kind, size, origin, color, resource)


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

        # The printed tag's BLACK SQUARE edge, in metres. Optional: without it
        # the detector's own `size` default applies, which is right for a
        # standalone run and wrong the moment a job prints tags at a different
        # size. A 1% error here is a 1% range error, so it belongs with the
        # object rather than with the optics.
        tag_size = entry.get('tag_size')
        if tag_size is not None:
            if isinstance(tag_size, bool) or not isinstance(tag_size, (int, float)):
                raise ValueError(f'{label}.tag_size must be a number of metres')
            tag_size = float(tag_size)
            if not math.isfinite(tag_size) or tag_size <= 0.0:
                raise ValueError(f'{label}.tag_size must be a positive number of metres')
            if tag_size > 1.0:
                raise ValueError(
                    f'{label}.tag_size is {tag_size}, which is over a metre -- it is '
                    'METRES across the black square, so a 39 mm tag is 0.039')

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

        specs.append(ObjectSpec(name, tag_id, tag_size, topic,
                                offset_position, offset_orientation, top_down_yaw,
                                yaw_axis / np.linalg.norm(yaw_axis),
                                _parse_shape(entry, label)))

    for field, what in (('name', 'names'), ('tag_id', 'tag ids'), ('topic', 'topics')):
        seen = [getattr(spec, field) for spec in specs]
        duplicates = sorted({item for item in seen if seen.count(item) > 1})
        if duplicates:
            raise ValueError(f'duplicate object {what}: {duplicates}')
    return specs
