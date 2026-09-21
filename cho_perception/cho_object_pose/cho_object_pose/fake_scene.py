"""A bench that exists only in numbers, for testing the pipeline without cameras.

``mock_publisher`` stands in for this whole package: it puts a fixed pose on the
output topic so a task tree can be wired up. This file stands in for one layer
LOWER -- it replaces the DETECTOR, so ``node.py`` runs for real. Everything
downstream of the optics is then the production path: the aggregation window,
the quality gate, the priority override, the TF lookup at the image stamp, and
the visibility topic a task branches on.

That is what makes it worth having. The pieces this package can test with
ordinary unit tests are the pure ones; what needed a camera was the part where
a moving arm carries a camera and TF has to compose the chain at the right
instant. Against a simulator it does not: point this at a running MuJoCo
bringup and the arm, the transforms and the gates are all real.

Nothing here imports ROS, for the reason ``geometry`` does not: the visibility
model is a handful of decisions about cones and line segments, and those
deserve tests that run in a second.

WHAT IS MODELLED, AND WHAT IS INVENTED
--------------------------------------
Modelled, from the real geometry the caller looks up in TF:

* **field of view** -- the tag has to lie inside a cone about the camera's own
  optical axis. Give a camera no ``fov_deg`` and it is not gated, which is the
  honest setting for a wide lens pointed at the whole bench.
* **occlusion** -- a camera is blocked when a named link sits within
  ``blocker_radius_m`` of its line of sight to the tag. THE ARM OCCLUDING THE
  BENCH IS THE FAILURE THE RECOVERY EXISTS FOR, and here it arises from the
  arm's actual configuration rather than from a flag.
* **apparent size** -- ``tag_size_m * focal_px / range``, which is the number
  ``min_edge_px`` gates on.

Invented, and the one thing not to read as a measurement:

* **decision_margin**. Taken proportional to the apparent edge. That is the
  right SHAPE -- a decode does get better as the tag gets bigger -- but the
  constant is a knob, and what it decides is how far into a recovery raster a
  sweep has to go before it is satisfied. Set ``margin_per_px`` to whatever
  makes the case you want to exercise, and do not quote the result as what a
  real camera returns.
"""

from collections import namedtuple
import math

import numpy as np

#: One simulated camera. ``frame`` is an existing TF frame -- the caller asks
#: TF where it is, so this file never needs to know how it got there, exactly
#: as the real node never names a camera frame. ``half_fov_rad`` is None for a
#: camera that is not field-of-view gated.
SceneCamera = namedtuple(
    'SceneCamera', 'name frame frame_prefix detections_topic half_fov_rad max_range_m')

#: One tag lying on the bench, in the base frame. ``name`` is for the log only.
SceneTag = namedtuple('SceneTag', 'id name position')

#: The whole bench.
Scene = namedtuple(
    'Scene',
    'base_frame tag_size_m focal_px margin_per_px blockers blocker_radius_m '
    'cameras tags')

#: What one camera makes of one tag. ``reason`` is why it saw nothing, and is
#: None exactly when ``visible``.
Sighting = namedtuple('Sighting', 'visible reason range_m edge_px decision_margin')

#: The optical axis, in the camera frame's own axes. +x is the RealSense
#: ``camera_link`` convention, which is what the wrist mount is described in.
OPTICAL_AXIS = np.array([1.0, 0.0, 0.0])

#: Fractions of the line of sight that count as "in between". The ends are
#: excluded so that the camera itself, and the tag, never block their own view.
_BLOCK_RANGE = (0.02, 0.98)


def apparent_edge_px(tag_size_m, focal_px, range_m):
    """Return the tag's edge in pixels at *range_m*, as ``min_edge_px`` reads it."""
    if range_m <= 0.0:
        return float('inf')
    return float(tag_size_m) * float(focal_px) / float(range_m)


def off_axis_rad(axis, to_tag):
    """Angle between the camera's optical *axis* and the direction to the tag."""
    norm = float(np.linalg.norm(to_tag))
    if norm < 1e-9:
        return 0.0
    return math.acos(float(np.clip(np.dot(axis, np.asarray(to_tag) / norm), -1.0, 1.0)))


def blocking_point(eye, tag, points, radius):
    """Find the first *points* entry that stands in the way; None when none does.

    A point blocks when it lies within *radius* of the segment eye -> tag AND
    genuinely between them: a link at the very start of the segment is the
    camera's own mount, and one at the very end is the thing the tag is
    attached to.
    """
    eye = np.asarray(eye, dtype=float)
    direction = np.asarray(tag, dtype=float) - eye
    length_squared = float(np.dot(direction, direction))
    if length_squared < 1e-12:
        return None
    low, high = _BLOCK_RANGE
    for index, point in enumerate(points):
        point = np.asarray(point, dtype=float)
        along = float(np.dot(point - eye, direction) / length_squared)
        if not low < along < high:
            continue
        if float(np.linalg.norm(eye + along * direction - point)) < radius:
            return index
    return None


def sight(scene, camera, eye, rotation, tag_position, blockers=()):
    """Work out what *camera* makes of a tag, given where the camera actually is.

    *eye* and *rotation* place the camera in the base frame; *blockers* are the
    base-frame positions of whatever might get in the way. All of that comes
    from TF, which is why none of it is looked up here.
    """
    eye = np.asarray(eye, dtype=float)
    to_tag = np.asarray(tag_position, dtype=float) - eye
    range_m = float(np.linalg.norm(to_tag))
    if range_m > camera.max_range_m:
        return Sighting(False, f'out of range at {range_m:.2f} m', range_m, 0.0, 0.0)

    if camera.half_fov_rad is not None:
        off = off_axis_rad(np.asarray(rotation, dtype=float) @ OPTICAL_AXIS, to_tag)
        if off > camera.half_fov_rad:
            return Sighting(False, f'{math.degrees(off):.0f} deg off axis',
                            range_m, 0.0, 0.0)

    index = blocking_point(eye, tag_position, blockers, scene.blocker_radius_m)
    if index is not None:
        return Sighting(False, f'blocked by {scene.blockers[index]}', range_m, 0.0, 0.0)

    edge = apparent_edge_px(scene.tag_size_m, scene.focal_px, range_m)
    return Sighting(True, None, range_m, edge, edge * scene.margin_per_px)


# ------------------------------------------------------------------- parsing

def _number(entry, key, label, default=None, positive=True):
    value = entry.get(key, default)
    if value is None:
        raise ValueError(f'{label}.{key} is required')
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label}.{key} must be a number')
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0.0):
        raise ValueError(f'{label}.{key} must be a finite positive number')
    return value


def _string(entry, key, label, default=None):
    value = entry.get(key, default)
    if not isinstance(value, str) or not value:
        raise ValueError(f'{label}.{key} must be a non-empty string')
    return value


def parse_scene(document):
    """Turn a loaded fake-scene YAML into a validated :class:`Scene`."""
    if not isinstance(document, dict):
        raise ValueError('fake scene config must be a mapping')

    base_frame = _string(document, 'base_frame', 'scene')
    blockers = document.get('blockers', [])
    if not isinstance(blockers, list) or not all(
            isinstance(name, str) and name for name in blockers):
        raise ValueError('scene.blockers must be a list of frame names')

    entries = document.get('cameras')
    if not isinstance(entries, list) or not entries:
        raise ValueError("fake scene config must contain a non-empty 'cameras' list")
    cameras = []
    for index, entry in enumerate(entries):
        label = f'cameras[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{label} must be a mapping')
        fov = entry.get('fov_deg')
        if fov is not None:
            fov = math.radians(_number(entry, 'fov_deg', label)) / 2.0
        cameras.append(SceneCamera(
            name=_string(entry, 'name', label),
            frame=_string(entry, 'frame', label),
            frame_prefix=entry.get('frame_prefix', ''),
            detections_topic=_string(entry, 'detections_topic', label),
            half_fov_rad=fov,
            max_range_m=_number(entry, 'max_range_m', label, default=5.0)))
    if not all(isinstance(camera.frame_prefix, str) for camera in cameras):
        raise ValueError('cameras[].frame_prefix must be a string')

    # The same duplicate check the real camera table makes, and for the same
    # reason: two cameras publishing one tag frame gives a TF child two parents.
    prefixes = [camera.frame_prefix for camera in cameras]
    duplicates = sorted({item for item in prefixes if prefixes.count(item) > 1})
    if duplicates:
        raise ValueError(f'duplicate camera tag frame prefixes: {duplicates}')

    entries = document.get('tags')
    if not isinstance(entries, list) or not entries:
        raise ValueError("fake scene config must contain a non-empty 'tags' list")
    tags = []
    for index, entry in enumerate(entries):
        label = f'tags[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{label} must be a mapping')
        tag_id = entry.get('id')
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise ValueError(f'{label}.id must be an integer tag id')
        xyz = entry.get('xyz')
        if (not isinstance(xyz, list) or len(xyz) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in xyz)):
            raise ValueError(f'{label}.xyz must be three finite numbers')
        tags.append(SceneTag(tag_id, entry.get('name', f'tag_{tag_id}'),
                             np.array([float(v) for v in xyz])))
    ids = [tag.id for tag in tags]
    duplicates = sorted({item for item in ids if ids.count(item) > 1})
    if duplicates:
        raise ValueError(f'the same tag id appears twice: {duplicates}')

    return Scene(
        base_frame=base_frame,
        tag_size_m=_number(document, 'tag_size_m', 'scene'),
        focal_px=_number(document, 'focal_px', 'scene'),
        margin_per_px=_number(document, 'margin_per_px', 'scene'),
        blockers=tuple(blockers),
        blocker_radius_m=_number(document, 'blocker_radius_m', 'scene', default=0.1),
        cameras=tuple(cameras),
        tags=tuple(tags))


def parse_blind(entries, scene):
    """``{'camera:tag_id'}`` -> the set of (camera, tag id) pairs to hide.

    Forcing a camera to miss an object is how a scenario is set up -- an
    occlusion by something the arm model does not contain, which is most of
    what stands on a bench. Names are checked against the scene, because a
    typo here silently produces the run you were not trying to test.
    """
    names = {camera.name for camera in scene.cameras}
    ids = {tag.id for tag in scene.tags}
    blind = set()
    for entry in entries:
        camera, _, tag = str(entry).partition(':')
        if not tag or not tag.lstrip('-').isdigit():
            raise ValueError(f"blind entry {entry!r} must be '<camera>:<tag id>'")
        if camera not in names:
            raise ValueError(f"blind entry {entry!r} names no camera in the scene "
                             f'({sorted(names)})')
        if int(tag) not in ids:
            raise ValueError(f"blind entry {entry!r} names no tag in the scene "
                             f'({sorted(ids)})')
        blind.add((camera, int(tag)))
    return blind
