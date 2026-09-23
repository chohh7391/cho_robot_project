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
not published for long enough
    OCCLUSION IS A DURATION, not an instant. A pose that stops arriving for one
    tick is a dropped frame, a motion blur, a sample window that briefly fell
    under ``min_samples``; a pose that stops arriving for seconds is something
    standing in the way. ``min_unseen_sec`` is where a bench draws that line,
    and below it the answer is :data:`WAIT` -- hold still and look again --
    rather than sending an arm across the cell because of one bad frame. The
    default is 0, which keeps the old instant trigger for benches that want it.
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
#: The pose has only just stopped arriving. Hold still and look again.
WAIT = 'wait'


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


def _contributing(view):
    """The cameras actually holding up the pose being published.

    A suppressed camera's sample was thrown away and a stale one's is out of
    the window, so neither is contributing and neither should be able to
    satisfy a requirement on what is.
    """
    return [entry for entry in view.cameras if entry.state == 'ok']


def best_decode(view):
    """The best decode confidence among the contributing cameras, or NO_SCORE."""
    scores = [entry.decision_margin for entry in _contributing(view)
              if entry.decision_margin > NO_SCORE]
    return max(scores) if scores else NO_SCORE


def best_tag_edge_px(view):
    """The biggest the tag appears to any contributing camera, or NO_SCORE.

    THE HONEST PROXY FOR HOW WELL A POSE IS LOCALISED, and a different question
    from ``best_decode``. decision_margin is the DECODER's confidence that it
    read the right bits, and it stays high for a small, oblique, far-away tag
    that decodes perfectly and localises badly -- measured in simulation, two
    standing cameras returned margin 237 on a tag spanning 33 px while sitting
    4 mm from the truth, and the wrist returned a similar margin on the same
    tag at 120 px and 0.1 mm.

    Apparent size is what actually maps to metric error: a corner located to a
    fixed fraction of a pixel is worth range/focal metres, so twice the pixels
    is half the error. A task that wants a pose it can act on asks for pixels.
    """
    edges = [entry.edge_px for entry in _contributing(view)
             if entry.edge_px > NO_SCORE]
    return max(edges) if edges else NO_SCORE


def assess(view, recovery_camera, min_decision_margin=0.0, min_tag_edge_px=0.0,
           planning_target=False, unseen_sec=None, min_unseen_sec=0.0):
    """Whether sweeping *recovery_camera* over *view*'s object is worth doing.

    TWO THINGS TRIGGER A SWEEP, not one. The obvious trigger is an object that
    is not being published at all -- something is in the way. The other is an
    object that IS being published, from a view too poor to act on: a standing
    camera watching a bench from a metre away sees every tag obliquely and
    returns a pose that is present and not accurate. Treating "a pose exists"
    as good enough would leave that case unrecoverable, because the recovery
    would never fire.

    ``planning_target`` IS THE HONEST FORM OF THAT SECOND TRIGGER, and it is a
    fact about the task rather than a number about a picture. An object the arm
    is about to touch has to be measured well; an object it only has to miss
    does not, because the planner already inflates obstacles to swallow the
    difference. So a target is recovered whenever the recovery camera has not
    measured it ITSELF, however good the standing cameras' view looks.

    That rule is not a preference. Measured on the simulated cell, the two
    standing cameras returned the SAME decode margin (237.3 and 237.5) and the
    SAME apparent tag size (32.8 and 33.4 px) while their poses sat 15.9 mm and
    10.2 mm from truth -- a 57% difference in accuracy that NOTHING either
    detector reports distinguishes. The error is a RANGE error (measured: 100%
    of it along the camera's own line of sight), so what predicts it is where
    the camera is, which a threshold on a detector's output cannot see. Hence a
    target's criterion is provenance and not a score.

    It also makes the two ends of the leaf symmetric: for a target, this asks
    for exactly the condition :func:`recovered` accepts, so the sweep cannot
    decline to start for a reason it would not have stopped for.

    ``min_decision_margin`` and ``min_tag_edge_px`` remain as the score-shaped
    triggers, for benches with no notion of a planning target. At 0 they are
    off and only occlusion recovers, which is the oldest behaviour.

    ``min_unseen_sec`` PUTS A CLOCK ON THE OCCLUSION TRIGGER, and only on that
    one. A pose that stopped arriving this instant and a pose that has been
    gone for five seconds look identical in a single snapshot, and they are not
    the same event: the first is a dropped frame or a window that briefly fell
    under ``min_samples``, the second is something in the way. Below the
    threshold this returns :data:`WAIT`, which asks the caller to look again
    rather than to drive. *unseen_sec* is the caller's measurement of how long
    the object has been unpublished -- this file has no clock, deliberately --
    and None means it did not measure, which is treated as "long enough" so
    that a caller who has not been taught to time it keeps the old behaviour.

    The clock is NOT applied to the quality triggers. A view that is too
    oblique is exactly as oblique a second later, so waiting for it only
    delays the sweep that was always going to be needed.

    Ordered so that the reasons a sweep CANNOT help are found before the
    reasons it might: an arm that drives because a detector was not launched is
    worse than one that refuses and says so.
    """
    score = best_decode(view)
    edge = best_tag_edge_px(view)
    mine = camera_view(view, recovery_camera)
    # For a planning target, 'someone is publishing it' is not the question;
    # 'the close camera measured it' is. A missing camera fails this and falls
    # through to the REFUSE below, which names the misconfiguration.
    measured_close = not planning_target or (mine is not None and mine.state == 'ok')
    good_enough = (view.publishing and measured_close
                   and score >= min_decision_margin
                   and edge >= min_tag_edge_px)
    if good_enough:
        detail = ', '.join(
            part for part in (
                f'measured by {recovery_camera}' if planning_target else '',
                f'decode margin {score:.0f} >= {min_decision_margin:.0f}'
                if min_decision_margin > 0.0 else '',
                f'tag {edge:.0f} px >= {min_tag_edge_px:.0f}'
                if min_tag_edge_px > 0.0 else '') if part)
        return Assessment(
            SATISFIED,
            f"'{view.name}' is already being published ({view.status})"
            + (f' at {detail}' if detail else '') + '; nothing to recover')

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

    if view.publishing:
        # The second trigger. Worth its own sentence: an operator watching a
        # pose stream out while the arm goes looking anyway needs to be told
        # that the pose is the reason, not the absence of one.
        short = []
        if planning_target and mine.state != 'ok':
            short.append(
                f"'{recovery_camera}' has not measured it itself, and it is a "
                'planning target')
        if score < min_decision_margin:
            short.append(f'decode margin {score:.0f} < {min_decision_margin:.0f}')
        if edge < min_tag_edge_px:
            short.append(f'tag only {edge:.0f} px across, wanted {min_tag_edge_px:.0f}')
        return Assessment(
            SWEEP,
            f"'{view.name}' is being published, but the best any camera has of it "
            f"is {' and '.join(short)}. Going to look closer -- "
            f'{describe_cameras(view)}')
    # NOT PUBLISHING, and now the only question is for how long. Everything a
    # sweep cannot fix has already been ruled out above, so a short outage here
    # is a dropped frame rather than a reason to refuse -- the caller is asked
    # to look again, not told to give up.
    if min_unseen_sec > 0.0 and unseen_sec is not None and unseen_sec < min_unseen_sec:
        return Assessment(
            WAIT,
            f"'{view.name}' stopped being published {unseen_sec:.1f}s ago, and this "
            f'bench calls it occlusion at {min_unseen_sec:.1f}s. Waiting -- a pose '
            f'that comes back on its own was a dropped frame, not something in the '
            f'way ({view.status})')
    return Assessment(
        SWEEP,
        f"'{view.name}' is not being published ({view.status})"
        + (f' and has not been for {unseen_sec:.1f}s' if unseen_sec is not None else '')
        + f'; {describe_cameras(view)}')


def recovered(view, recovery_camera, min_decision_margin=0.0, min_tag_edge_px=0.0):
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
    # A camera that reports no score cannot clear a threshold. It means the
    # publisher predates the score field, and silently passing would turn a
    # quality requirement into no requirement.
    if min_decision_margin > 0.0 and mine.decision_margin < min_decision_margin:
        return False
    return min_tag_edge_px <= 0.0 or mine.edge_px >= min_tag_edge_px


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
    'min_decision_margin min_tag_edge_px planning_target min_unseen_sec')
#: The judgement fields default to "ask for nothing", as CameraView's scores
#: do. parse_sweeps always fills all of them, so this is not a way to build a
#: half-specified sweep; it is so that adding a criterion does not break every
#: caller that builds one by hand -- which is what happened the last two times.
SweepSpec.__new__.__defaults__ = (0.0, 0.0, False, 0.0)

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

#: How big the tag has to appear before a pose is considered good enough to act
#: on. 0 turns the check off. It tracks the metric error better than the decode
#: margin does -- see best_tag_edge_px -- but only among views of SIMILAR
#: obliquity: measured on the cell, two standing cameras 57% apart in accuracy
#: reported 32.8 and 33.4 px, because the nearer one was the more oblique and
#: the foreshortening cancelled the range. Use it to separate a wrist close-up
#: from a standing view (33 px against 83), not to rank two standing views.
DEFAULT_MIN_TAG_EDGE_PX = 0.0

#: Whether this object is one the planner will act ON rather than merely avoid.
#: Off by default: a bench with no notion of a stage target keeps the older
#: behaviour, where only occlusion and the score gates recover.
DEFAULT_PLANNING_TARGET = False

#: HOW LONG A POSE HAS TO BE MISSING before it counts as occlusion rather than
#: as a dropped frame. 0 fires the instant the pose stops arriving, which is the
#: older behaviour and is wrong on a real bench: the pose node publishes from a
#: window of ``min_samples`` over ``window_sec``, so one blurred frame or one
#: slow TF lookup empties it for a tick. Anything above the node's own window
#: separates the two; the FR5 bench uses 2 s, which is several windows and still
#: well inside a human's idea of "it is not coming back".
DEFAULT_MIN_UNSEEN_SEC = 0.0

_DEFAULT_KEYS = ('recovery_camera', 'waypoint_duration', 'dwell_sec', 'timeout_sec',
                 'min_decision_margin', 'min_tag_edge_px', 'planning_target',
                 'min_unseen_sec')


def _flag(value, label):
    """*value* as a bool, refusing the truthy strings YAML makes easy to write.

    ``planning_target: 'no'`` is a non-empty string and therefore true to
    Python, which would silently send the arm looking at every object on the
    bench. YAML already parses ``true``/``false``/``yes``/``no`` to bools, so
    anything arriving here as a string was written wrong.
    """
    if not isinstance(value, bool):
        raise ValueError(f'{label} must be true or false')
    return value


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
            min_tag_edge_px=_positive(
                entry.get('min_tag_edge_px',
                          defaults.get('min_tag_edge_px', DEFAULT_MIN_TAG_EDGE_PX)),
                f'{label}.min_tag_edge_px', allow_zero=True),
            planning_target=_flag(
                entry.get('planning_target',
                          defaults.get('planning_target', DEFAULT_PLANNING_TARGET)),
                f'{label}.planning_target'),
            min_unseen_sec=_positive(
                entry.get('min_unseen_sec',
                          defaults.get('min_unseen_sec', DEFAULT_MIN_UNSEEN_SEC)),
                f'{label}.min_unseen_sec', allow_zero=True),
        )
    return sweeps


#: What one pass over several objects drives: a single waypoint list, the dwell
#: at each, and the ceiling on the whole pass.
SharedRaster = namedtuple('SharedRaster', 'waypoints dwell_sec timeout_sec')


def shared_raster(sweeps):
    """The one raster a single pass drives for all of *sweeps*.

    A single pass judges every object from the same viewpoints, so it can only
    be built over sweeps that name THE SAME WAYPOINTS -- names, joint
    configurations and durations, in the same order. The FR5 table is solved
    from one area raster and meets that by construction. A table with a raster
    per object does not, and is refused rather than merged: deciding whose
    waypoints win is deciding which object gets looked for badly.

    The dwell is the longest any of them asks for, because a viewpoint is judged
    once for all of them. The ceiling is the most generous, not the sum: the
    raster is driven at most once however many objects are on it.
    """
    sweeps = list(sweeps)
    if not sweeps:
        raise ValueError('a single pass needs at least one sweep')
    first = sweeps[0]
    if not first.waypoints:
        raise ValueError(f"the sweep for '{first.object}' has no waypoints")
    for other in sweeps[1:]:
        if other.waypoints == first.waypoints:
            continue
        where = next((index for index, (mine, theirs)
                      in enumerate(zip(first.waypoints, other.waypoints))
                      if mine != theirs),
                     min(len(first.waypoints), len(other.waypoints)))
        raise ValueError(
            f"'{first.object}' and '{other.object}' are swept over different waypoints "
            f'(first difference at waypoint {where + 1}; {len(first.waypoints)} against '
            f'{len(other.waypoints)}), so one pass cannot look for both. Sweep them '
            'one object at a time instead.')
    return SharedRaster(
        waypoints=first.waypoints,
        dwell_sec=max(sweep.dwell_sec for sweep in sweeps),
        timeout_sec=max(sweep.timeout_sec for sweep in sweeps))


def load_sweeps(path, joint_names=None):
    """``parse_sweeps`` over a YAML file. Kept apart so the parser stays pure."""
    import yaml
    with open(path, encoding='utf-8') as stream:
        return parse_sweeps(yaml.safe_load(stream), joint_names=joint_names)
