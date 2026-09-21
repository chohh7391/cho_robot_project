"""Parsing and validation of the camera table.

How many cameras are looking at the bench, and where each one's detections come
out, is launch topology rather than geometry. It is a *file* rather than a
launch argument because two things have to agree on it: the launch that starts
one detector per camera, and this node, which subscribes to every one of them
and looks up a differently-prefixed tag frame for each. Two lists would drift;
one file cannot.

The frame prefix is the part that has to be right. Two detectors left at the
default both publish ``tag_9``, which gives one TF child two parents -- not an
error, just transforms that intermittently resolve through the wrong camera.
That is why duplicates here are rejected rather than tolerated.

``priority`` is the one field that changes what the node DOES with a camera
rather than where it finds it. Cameras left at the default are peers and their
views are fused; a camera given a higher one replaces the lower ones for any
object it can see, which is how a close-up recovery sweep overrides a standing
observer instead of being averaged into it.
"""

from collections import namedtuple
import math

DEFAULT_DETECTIONS_TOPIC = '/detections'

DEFAULT_VISUAL_COLOR = (0.6, 0.6, 0.6, 0.9)

CameraSpec = namedtuple(
    'CameraSpec',
    'name frame_prefix detections_topic image_topic camera_info_topic rectify '
    'priority visual')

#: Where to draw the camera body, for rviz. DIAGNOSTIC, not decoration: the
#: extrinsic that puts a tag in the robot's frame is the one number in this
#: pipeline nothing else checks, and a camera drawn half a metre from where the
#: real one stands says so at a glance -- which a list of tag poses does not.
#:
#: ``frame`` is an existing TF frame, so nothing here publishes a transform;
#: ``position``/``orientation`` are the vendor's own mesh origin within it.
CameraVisual = namedtuple('CameraVisual', 'frame mesh position orientation scale color')


def _floats(entry, key, label, length, default):
    value = entry.get(key, default)
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{label}.{key} must be a list of {length} numbers')
    if not all(not isinstance(item, bool) and isinstance(item, (int, float))
               and math.isfinite(item) for item in value):
        raise ValueError(f'{label}.{key} must contain only finite numbers')
    return tuple(float(item) for item in value)


def _priority(entry, label):
    """How much this camera's word is worth against another's.

    Cameras at the same priority are FUSED, which is what every bench did
    before this field existed and what leaving it out still does. A camera at a
    HIGHER priority REPLACES the lower ones for any object it can currently
    see, instead of being averaged with them -- see visibility.select_by_priority
    for why an override rather than a weighting.

    Integer, and it may be negative: the numbers only ever get compared, so what
    matters is the order and not the size of the gaps.
    """
    value = entry.get('priority', 0)
    # bool is an int in Python, and `priority: true` is a typo rather than a 1.
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{label}.priority must be an integer')
    return value


def _parse_visual(entry, label):
    """Return the camera's display body, or None when it declares none."""
    visual = entry.get('visual')
    if visual is None:
        return None
    if not isinstance(visual, dict):
        raise ValueError(f'{label}.visual must be a mapping')

    frame = _string(visual, 'frame', f'{label}.visual', required=True)
    mesh = _string(visual, 'mesh', f'{label}.visual', required=True)
    position = _floats(visual, 'xyz', f'{label}.visual', 3, [0.0, 0.0, 0.0])
    # Fixed-axis roll-pitch-yaw, so a mounting pose can be copied out of the
    # vendor's xacro exactly as it is written there.
    roll, pitch, yaw = _floats(visual, 'rpy', f'{label}.visual', 3, [0.0, 0.0, 0.0])
    scale = _floats(visual, 'scale', f'{label}.visual', 3, [1.0, 1.0, 1.0])
    if not all(value > 0.0 for value in scale):
        raise ValueError(f'{label}.visual.scale must be three positive numbers')
    color = _floats(visual, 'color', f'{label}.visual', 4, list(DEFAULT_VISUAL_COLOR))
    if not all(0.0 <= value <= 1.0 for value in color):
        raise ValueError(f'{label}.visual.color must be four values in [0, 1] (r, g, b, a)')

    return CameraVisual(frame, mesh, position, (roll, pitch, yaw), scale, color)


def _string(entry, key, label, required=False, default=''):
    value = entry.get(key, default)
    if value is None:
        value = default
    if not isinstance(value, str):
        raise ValueError(f'{label}.{key} must be a string')
    if required and not value:
        raise ValueError(f'{label}.{key} is required')
    return value


def parse_cameras(document):
    """Turn a loaded cameras YAML document into validated CameraSpecs.

    ``image_topic`` is required even though this node never subscribes to an
    image: the detector launch reads the same file, and a camera with no image
    topic is a detector that would be started against nothing.
    """
    if not isinstance(document, dict):
        raise ValueError('cameras config must be a mapping')
    entries = document.get('cameras')
    if not isinstance(entries, list) or not entries:
        raise ValueError("cameras config must contain a non-empty 'cameras' list")

    specs = []
    for index, entry in enumerate(entries):
        label = f'cameras[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{label} must be a mapping')

        name = _string(entry, 'name', label, required=True)
        image_topic = _string(entry, 'image_topic', label, required=True)
        # image_transport's convention: camera_info is the image's sibling.
        camera_info_topic = _string(
            entry, 'camera_info_topic', label,
            default=f"{image_topic.rsplit('/', 1)[0]}/camera_info")
        detections_topic = _string(
            entry, 'detections_topic', label, default=DEFAULT_DETECTIONS_TOPIC)
        if not detections_topic:
            raise ValueError(f'{label}.detections_topic must not be empty')
        frame_prefix = _string(entry, 'frame_prefix', label)

        rectify = entry.get('rectify', False)
        if not isinstance(rectify, bool):
            raise ValueError(f'{label}.rectify must be true or false')

        specs.append(CameraSpec(name, frame_prefix, detections_topic,
                                image_topic, camera_info_topic, rectify,
                                _priority(entry, label),
                                _parse_visual(entry, label)))

    for field, what in (('name', 'names'),
                        ('frame_prefix', 'tag frame prefixes'),
                        ('detections_topic', 'detections topics')):
        seen = [getattr(spec, field) for spec in specs]
        duplicates = sorted({item for item in seen if seen.count(item) > 1})
        if duplicates:
            raise ValueError(
                f'duplicate camera {what}: {duplicates}. Every camera needs its own, '
                'or two detectors publish the same tag frame and one TF child gains '
                'two parents.')
    return specs


def single_camera(frame_prefix='', detections_topic=DEFAULT_DETECTIONS_TOPIC):
    """Return the one-camera table, for a node started without a cameras file.

    Keeps the historical ``frame_prefix`` / ``detections_topic`` parameters
    meaning exactly what they always meant, so nothing that worked with one
    camera has to learn about this file.
    """
    return [CameraSpec('camera', frame_prefix, detections_topic, '', '', False, 0, None)]
