"""Why each camera contributed what it did, and whose contribution counts.

Two things live here because they are the same question asked twice. The first
is the vocabulary the node uses to say what a camera did with an object's tag
-- saw it, could not decode it, could not place it in the robot's frame. The
second is :func:`select_by_priority`, which throws some of those contributions
away in favour of a closer camera's. The reason they belong together is that
suppression PRODUCES a status: a camera that was outranked is reported as
``suppressed``, which is the one visibility state nothing in the optics causes.

Nothing here imports ROS, exactly as in ``geometry``. The states are plain
strings, and ``cho_interfaces/CameraVisibility`` carries the same names as
``STATE_<UPPER>`` constants -- the node maps one to the other by name, so the
two cannot drift into disagreeing about a number.

WHY A TOPIC AT ALL. All of this was already known: the node's periodic report
has logged it per camera for as long as there have been two cameras. But a log
line is not something a behaviour tree can read, and "the beaker is not in the
OAK's frame" is precisely the trigger for driving the wrist camera over to look
-- the one fact the motion side had no way to observe.
"""

from collections import namedtuple

#: Every visibility state, in the order the message declares them. The wire
#: format spells each as ``STATE_<UPPER>``; a state added here and not there
#: (or the reverse) fails ``test_visibility``.
STATES = ('unknown', 'ok', 'not_in_frame', 'rejected', 'no_tf', 'suppressed', 'stale')

#: No detection to score.
NO_SCORE = -1.0

#: One camera's verdict on one object. ``detail`` is free text for the human
#: half -- the decode reason, the TF error, the camera that outranked this one.
#:
#: ``decision_margin`` and ``edge_px`` are HOW WELL the tag decoded, carried
#: alongside the state rather than folded into it. The state says a detection
#: passed the pipeline's gate, which is set for "good enough to publish"; a
#: recovery sweep exists precisely to replace a marginal view with a better
#: one, and "good enough to publish" is satisfied by the marginal one. Keeping
#: the number lets the consumer hold out for a better view without this package
#: gaining a second gate, or an opinion about what any particular task needs.
Reason = namedtuple('Reason', 'state detail decision_margin edge_px')
Reason.__new__.__defaults__ = ('', NO_SCORE, NO_SCORE)

#: Nothing heard from this camera about this object yet.
UNKNOWN = Reason('unknown')
#: The camera published detections and this tag was not among them. Occlusion
#: looks like this -- and so does a tag simply outside the field of view, which
#: is why what to do about it is the task's decision and not this package's.
NOT_IN_FRAME = Reason('not_in_frame')


def ok(decision_margin=NO_SCORE, edge_px=NO_SCORE):
    """Record a sample that went into the aggregation window, and how good it was."""
    return Reason('ok', '', float(decision_margin), float(edge_px))


def rejected(detail, decision_margin=NO_SCORE, edge_px=NO_SCORE):
    """Record a tag that decoded but failed the quality gate.

    The score comes too. A sweep that keeps being rejected wants to know
    whether it is close to passing -- 'margin 33 against 35' and 'margin 4'
    are a lower waypoint and a wrong lens respectively.
    """
    return Reason('rejected', str(detail), float(decision_margin), float(edge_px))


def no_tf(detail, decision_margin=NO_SCORE, edge_px=NO_SCORE):
    """Record a tag that passed the gate but could not be placed in the base frame."""
    return Reason('no_tf', str(detail), float(decision_margin), float(edge_px))


def describe(reason):
    """Render the reason as the periodic report prints it.

    Deliberately the same words the node logged before any of this was
    published, so a familiar log line does not change under anyone.
    """
    if reason.state == 'unknown':
        return 'no detection yet'
    if reason.state == 'ok':
        return 'ok'
    if reason.state == 'not_in_frame':
        return 'tag not in frame'
    if reason.state == 'rejected':
        return f'rejected: {reason.detail}'
    if reason.state == 'no_tf':
        return f'no TF {reason.detail}'
    if reason.state == 'suppressed':
        return f'suppressed by {reason.detail}'
    if reason.state == 'stale':
        return f'stale, last said: {reason.detail}'
    raise ValueError(f'unknown visibility state {reason.state!r}')


def describe_score(reason):
    """Render the decode quality, or '' when there was nothing to score."""
    if reason.decision_margin <= NO_SCORE:
        return ''
    return f'margin {reason.decision_margin:.0f}, edge {reason.edge_px:.0f}px'


def current_reason(reason, age_sec, window_sec, suppressed_by=''):
    """*reason* as it stands at report time, given how old it is.

    Staleness is derived from the aggregation window rather than from a
    lifetime of its own, and that is not a shortcut: a sample older than the
    window has already been pruned out of it, so a REASON older than the window
    describes nothing that could still be contributing. A camera whose driver
    died would otherwise keep reporting the last thing it managed to say, which
    reads exactly like a healthy camera.

    Staleness wins over suppression because it is about a different thing: a
    camera that has gone quiet was not outranked, it left.
    """
    if reason.state == 'unknown':
        return reason
    # The score travels with the state through both overlays: a camera that was
    # outranked still measured what it measured, and that is what says whether
    # the override was an improvement or just a preference.
    if age_sec > window_sec:
        return reason._replace(state='stale', detail=describe(reason))
    if suppressed_by:
        return reason._replace(state='suppressed', detail=suppressed_by)
    return reason


#: What survived a priority contest. ``priority`` is the winning tier's value,
#: ``kept`` the cameras in it and ``suppressed`` everything below. ``kept``
#: holds every contributor when no camera outranks another, and ``suppressed``
#: is then empty -- which is the answer for a bench whose cameras are peers.
PrioritySelection = namedtuple('PrioritySelection', 'priority kept suppressed')


def select_by_priority(cameras, priorities):
    """Keep only the highest-priority cameras that contributed, drop the rest.

    THIS IS AN OVERRIDE, NOT A WEIGHTING. A close-up view from a wrist camera
    and a metre-away view from a standing one are not two measurements of the
    same quantity to be averaged -- the near one is simply better, and mixing
    it with the far one produces a median no camera saw and a spread that reads
    as disagreement. So the near one replaces the far one outright, for the
    object it can see and only for that object.

    *cameras* is the set of camera names that actually put a sample in the
    window; a camera that saw nothing does not suppress anything, which is what
    makes the override self-clearing. The moment the wrist loses the tag its
    samples age out of the window and the standing camera is believed again
    with no extra state anywhere -- the recovered pose then expires the same
    way every other sample does.

    *priorities* maps camera name to an integer; a name it does not mention
    counts as 0, which is what "declares no priority" means.
    """
    present = sorted(set(cameras))
    if not present:
        return PrioritySelection(0, (), ())
    ranked = {name: int(priorities.get(name, 0)) for name in present}
    winning = max(ranked.values())
    kept = tuple(name for name in present if ranked[name] == winning)
    suppressed = tuple(name for name in present if ranked[name] < winning)
    return PrioritySelection(winning, kept, suppressed)
