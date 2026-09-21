"""When a sweep would help, and where to sweep to. No ROS in this file.

``cho_object_pose`` now says what every camera can see of every object. This
turns that into the two decisions a recovery leaf has to make, and it is kept
ROS-free for the same reason ``cho_object_pose/geometry.py`` is: the part worth
testing needs no camera, no arm and no clock, and a rule about when to move a
robot deserves a test that can be run in a second.

The rules, and why each is a rule rather than a heuristic:

``not_in_frame``
    The tag is not among the detections a camera is publishing. THIS IS WHAT
    OCCLUSION LOOKS LIKE, and it is also what a tag simply outside the field of
    view looks like -- the perception side cannot tell them apart and does not
    try. Going to look is the right response to both.
``rejected``
    The tag decoded but failed the quality gate: too far, too oblique, too
    blurred. A closer, squarer view is exactly what fixes that, so it is also
    worth sweeping for.
``no_tf`` on the recovery camera
    The tag was seen and could not be placed in the robot's frame. Moving the
    camera moves the same broken chain somewhere else. REFUSE -- and say which
    camera, because for a wrist camera this usually means the robot's TF is not
    up, which no amount of driving will fix.
nothing fresh from anyone
    Every camera ``unknown`` or ``stale``: the detectors are not running, or
    the pose node is talking to topics nobody publishes. REFUSE. Driving an arm
    because a node failed to start is the wrong answer to the wrong question.

What is deliberately NOT here is where to sweep. Which poses put a wrist camera
over a beaker is a fact about one bench and one arm; it comes from the task's
own config (:func:`parse_sweeps`), which is why ``cho_object_pose`` can go on
knowing nothing about a robot.
"""

from collections import namedtuple
import math

#: The visibility vocabulary, as ``cho_interfaces/CameraVisibility`` declares
#: it. Strings here, ``STATE_<UPPER>`` uint8 constants on the wire; the
#: behaviour maps one to the other by name so the two cannot disagree about a
#: number, and ``test_occlusion`` asserts this list is exactly the message's.
STATES = ('unknown', 'ok', 'not_in_frame', 'rejected', 'no_tf', 'suppressed', 'stale')

#: States that mean the camera has said nothing usable recently. A bench where
#: every camera is in one of these is a bench with no perception running.
QUIET_STATES = ('unknown', 'stale')

#: States a closer look can plausibly fix.
RECOVERABLE_STATES = ('not_in_frame', 'rejected')

#: No detection to score. Zero is a real and terrible margin; this is the
#: absence of one, and a threshold has to be able to tell them apart.
NO_SCORE = -1.0

#: One camera's word about one object, as the topic carries it.
CameraView = namedtuple(
    'CameraView', 'camera state detail age_sec priority decision_margin edge_px')
CameraView.__new__.__defaults__ = (NO_SCORE, NO_SCORE)

#: One object's visibility: whether its pose is going out, which camera is
#: overriding the others if any, the node's own status line, and every camera.
ObjectView = namedtuple('ObjectView', 'name publishing override_camera status cameras')

#: What to do about an object that is not being published. ``reason`` is meant
#: to be logged verbatim -- it is the only explanation an operator gets for an
#: arm that either moved or refused to.
Assessment = namedtuple('Assessment', 'action reason')

#: The pose is already available; a sweep would be motion for nothing.
SATISFIED = 'satisfied'
#: Go and look.
SWEEP = 'sweep'
#: A sweep cannot fix this. Fail the leaf instead of driving the arm.
REFUSE = 'refuse'


def camera_view(view, camera):
    """*camera*'s entry in *view*, or None when the object has no such camera."""
    for entry in view.cameras:
        if entry.camera == camera:
            return entry
    return None


def describe_cameras(view):
    """Every camera's word about *view*, with its decode score, for one log line."""
    return ', '.join(
        f'{entry.camera}: {entry.state}'
        + (f' ({entry.detail})' if entry.detail else '')
        + (f' [margin {entry.decision_margin:.0f}, edge {entry.edge_px:.0f}px]'
           if entry.decision_margin > NO_SCORE else '')
        for entry in view.cameras) or 'no cameras'


def assess(view, recovery_camera):
    """Whether sweeping *recovery_camera* over *view*'s object is worth doing.

    Ordered so that the reasons a sweep CANNOT help are found before the
    reasons it might: an arm that drives because a detector was not launched is
    worse than one that refuses and says so.
    """
    if view.publishing:
        return Assessment(
            SATISFIED,
            f"'{view.name}' is already being published ({view.status}); nothing to recover")

    mine = camera_view(view, recovery_camera)
    if mine is None:
        return Assessment(
            REFUSE,
            f"no camera called '{recovery_camera}' is watching '{view.name}'. The sweep "
            'config names a camera the pose node was not given; check it against '
            'cho_object_pose/config/cameras.yaml.')

    if mine.state == 'no_tf':
        return Assessment(
            REFUSE,
            f"'{recovery_camera}' can see '{view.name}' but cannot place it in the "
            f'robot frame ({mine.detail}). Moving the camera moves the same broken '
            'chain; for a wrist camera this is usually the robot bringup not running, '
            'or a missing entry in the extrinsics.')

    if all(entry.state in QUIET_STATES for entry in view.cameras):
        return Assessment(
            REFUSE,
            f"no camera has said anything about '{view.name}' recently "
            f'({describe_cameras(view)}). The detectors are not running, or they are '
            'publishing on topics the pose node was not given -- not something a '
            'sweep can fix.')

    return Assessment(
        SWEEP,
        f"'{view.name}' is not being published ({view.status}); "
        f'{describe_cameras(view)}')


def recovered(view, recovery_camera, min_decision_margin=0.0):
    """True once *recovery_camera* is putting *view*'s object on the wire, well.

    Three conditions, and each rules out a different way of being wrong:

    ``view.publishing``
        alone would also be satisfied by the standing camera getting its view
        back. A fine outcome for the task, but not the sweep having worked, and
        a leaf that claimed credit for it would hide a set of sweep waypoints
        that never sees anything.
    the camera's state being ``ok``
        alone would be satisfied while the pose node is still short of
        ``min_samples`` -- a view, but no pose yet.
    ``min_decision_margin``
        is the one a caller has to choose. The pipeline's own gate is set for
        "good enough to publish a pose from", and the far view that prompted
        the recovery already passes it or the object would not be in the table
        at all. A sweep that stops at the first viewpoint meeting that gate has
        replaced a marginal measurement with another marginal measurement. Hold
        out for a genuinely better one, and the raster of viewpoints keeps
        going until it gets it.
    """
    if not view.publishing:
        return False
    mine = camera_view(view, recovery_camera)
    if mine is None or mine.state != 'ok':
        return False
    if min_decision_margin <= 0.0:
        return True
    # A camera that reports no score cannot clear a threshold. It means the
    # publisher predates the score field, and silently passing would turn a
    # quality requirement into no requirement.
    return mine.decision_margin >= min_decision_margin


# ---------------------------------------------------------- the sweep table

#: One pose in a sweep. ``joints`` is a full joint configuration, not a delta;
#: ``duration`` is how long to take getting there, which is per-waypoint
#: because the first waypoint of a sweep is a long swing from wherever the arm
#: was and the rest are small raster steps. A cell has a commissioning rate ceiling
#: (cho_moveit_fr5/config/joint_limits.yaml) and one duration for both would
#: either break it or make every descent crawl.
SweepWaypoint = namedtuple('SweepWaypoint', 'name joints duration')

#: How to go and look at one object.
SweepSpec = namedtuple(
    'SweepSpec',
    'object recovery_camera waypoints waypoint_duration dwell_sec timeout_sec '
    'min_decision_margin')

#: Long enough for the arm to stop ringing and for the pose node to fill a
#: fresh aggregation window at it. Its default window is 0.5 s and it wants
#: ``min_samples`` inside one, so anything under that judges a viewpoint on
#: samples taken while the arm was still arriving.
DEFAULT_DWELL_SEC = 1.5

#: Per-waypoint move time. Slow: the arm is carrying a camera it is about to
#: believe, and a fast move blurs the frames at the end of it.
DEFAULT_WAYPOINT_DURATION = 4.0

#: Whole-sweep ceiling, including the moves. Generous enough for four waypoints
#: at the defaults with the detections lagging.
DEFAULT_TIMEOUT_SEC = 90.0

#: Decode confidence a recovered view has to reach before the sweep stops
#: descending. The pipeline publishes from 35 (cho_object_pose's
#: ``min_decision_margin``), and the far view that prompted the recovery is
#: already at or above it, so accepting 35 here would trade one marginal
#: measurement for another. 55 is comfortably above what a 39 mm tag returns at
#: the far end of a sweep and comfortably below the 66-68 measured at close
#: range on this bench.
DEFAULT_MIN_DECISION_MARGIN = 55.0

_DEFAULT_KEYS = ('recovery_camera', 'waypoint_duration', 'dwell_sec', 'timeout_sec',
                 'min_decision_margin')


def _positive(value, label, allow_zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a number')
    value = float(value)
    if not math.isfinite(value) or (value < 0.0 if allow_zero else value <= 0.0):
        raise ValueError(
            f'{label} must be {"at least 0" if allow_zero else "greater than 0"}; '
            f'got {value}')
    return value


def _waypoints(entry, label, joint_names, default_duration):
    raw = entry.get('waypoints')
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{label}.waypoints must be a non-empty list")
    waypoints = []
    for index, item in enumerate(raw):
        where = f'{label}.waypoints[{index}]'
        if not isinstance(item, dict):
            raise ValueError(f'{where} must be a mapping')
        name = item.get('name')
        if not isinstance(name, str) or not name:
            raise ValueError(f'{where}.name must be a non-empty string')
        joints = item.get('joints')
        if not isinstance(joints, list) or not joints:
            raise ValueError(f'{where}.joints must be a non-empty list')
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in joints):
            raise ValueError(f'{where}.joints must contain only finite numbers')
        # The count, not the values: joint LIMITS belong to the robot
        # description and the controller enforces them. A wrong count is the
        # error this can see, and it is the one that would otherwise reach an
        # action server as a goal the arm interprets by position in a list.
        if joint_names is not None and len(joints) != len(joint_names):
            raise ValueError(
                f'{where}.joints has {len(joints)} values but the robot has '
                f'{len(joint_names)} joints ({list(joint_names)})')
        waypoints.append(SweepWaypoint(
            name, tuple(float(value) for value in joints),
            _positive(item.get('duration', default_duration), f'{where}.duration')))
    return tuple(waypoints)


def parse_sweeps(document, joint_names=None):
    """Turn a loaded sweep YAML into ``{object name: SweepSpec}``.

    ``joint_names`` is the robot's own joint list, from the registry. Passing it
    turns a miscounted waypoint into a build-time error rather than a goal the
    action server fills in by position.
    """
    if not isinstance(document, dict):
        raise ValueError('sweep config must be a mapping')

    defaults = document.get('defaults', {})
    if not isinstance(defaults, dict):
        raise ValueError('sweep config defaults must be a mapping')
    unknown = sorted(set(defaults) - set(_DEFAULT_KEYS))
    if unknown:
        raise ValueError(
            f'sweep config defaults has no such setting(s): {unknown}. '
            f'Valid: {list(_DEFAULT_KEYS)}')

    entries = document.get('sweeps')
    if not isinstance(entries, list) or not entries:
        raise ValueError("sweep config must contain a non-empty 'sweeps' list")

    sweeps = {}
    for index, entry in enumerate(entries):
        label = f'sweeps[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{label} must be a mapping')
        name = entry.get('object')
        if not isinstance(name, str) or not name:
            raise ValueError(f'{label}.object must be a non-empty string')
        if name in sweeps:
            raise ValueError(
                f"{label}: '{name}' already has a sweep. Two ways to look at one "
                'object is two things to keep in step; give it one.')

        camera = entry.get('recovery_camera', defaults.get('recovery_camera'))
        if not isinstance(camera, str) or not camera:
            raise ValueError(
                f'{label}.recovery_camera must name the camera that does the looking '
                '(or set defaults.recovery_camera). It has to be a camera name from '
                "cho_object_pose's cameras.yaml, and it should be the one with the "
                'higher priority, or its close-up view will be medianed into the far '
                'one instead of replacing it.')

        sweeps[name] = SweepSpec(
            object=name,
            recovery_camera=camera,
            waypoints=_waypoints(
                entry, label, joint_names,
                _positive(entry.get('waypoint_duration',
                                    defaults.get('waypoint_duration',
                                                 DEFAULT_WAYPOINT_DURATION)),
                          f'{label}.waypoint_duration')),
            waypoint_duration=_positive(
                entry.get('waypoint_duration',
                          defaults.get('waypoint_duration', DEFAULT_WAYPOINT_DURATION)),
                f'{label}.waypoint_duration'),
            dwell_sec=_positive(
                entry.get('dwell_sec', defaults.get('dwell_sec', DEFAULT_DWELL_SEC)),
                f'{label}.dwell_sec', allow_zero=True),
            timeout_sec=_positive(
                entry.get('timeout_sec', defaults.get('timeout_sec', DEFAULT_TIMEOUT_SEC)),
                f'{label}.timeout_sec'),
            # 0 accepts whatever the pipeline was willing to publish, which is
            # the old behaviour and is right for a bench whose only problem is
            # line of sight rather than range.
            min_decision_margin=_positive(
                entry.get('min_decision_margin',
                          defaults.get('min_decision_margin',
                                       DEFAULT_MIN_DECISION_MARGIN)),
                f'{label}.min_decision_margin', allow_zero=True),
        )
    return sweeps


def load_sweeps(path, joint_names=None):
    """``parse_sweeps`` over a YAML file. Kept apart so the parser stays pure."""
    import yaml
    with open(path, encoding='utf-8') as stream:
        return parse_sweeps(yaml.safe_load(stream), joint_names=joint_names)
