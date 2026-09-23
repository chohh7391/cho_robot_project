"""Take a recording's pour out of the replay, so the pouring controller can do it.

A recorded pour is the one part of a recording that position control cannot
replay faithfully even when the cell matches: the planner that produced it
tipped a simulated vessel by a fixed amount, and how much a real one gives at
that angle depends on how full it is, what is in it, and where the jaws closed
on it. So the replay goes as far as the pour's first waypoint, hands the arm to
the pouring controller -- which measures the vessel, pours by weight and
returns the arm to exactly the configuration it was given it in -- and resumes
from the pour's last waypoint.

That hand-back is only seamless because of what is checked here, before a
tree exists:

* the pour is ONE contiguous run of waypoints labelled ``pouring``;
* it starts and ends at the same configuration. The pouring controller finishes
  where it started, and the replay resumes where the recording's pour ended;
  if those were two places the resume would be a lunge between them. Both
  real-cell recordings meet this exactly (0 rad);
* no gripper event falls inside it. The jaws stay shut through a pour, and a
  recording that says otherwise is not one this can stand in for;

HOW the recording tips the vessel is not assumed either. The pour's deepest
waypoint goes to the controller as a reference configuration, and it tips the
vessel about whatever axis the EE turns about to get there. Recordings differ
exactly there: the first real-cell ones rolled the wrist about the approach
(j6, negative), the riser ones turn about the jaws' closing axis (mostly j4).

Measured on both recordings, the arm is at a full stop at both ends of the pour,
as it is at every operation boundary, so cutting there changes no motion.

Nothing here imports ROS.
"""

from cho_task_manager.utils.trajectory_recording import (
    RecordingRejected,
    Segment,
    _move,
    _operations_in,
)

#: The operation label the recordings give their pour.
POUR_OPERATION = 'pouring'

#: Largest difference [rad], on any joint, between the recorded pour's first and
#: last waypoints. A few milliradians is the resolution the recordings are
#: written at; more than that is a pour that ends somewhere else.
SAME_POSE_TOLERANCE_RAD = 2e-3


class PourSegment(Segment):
    """The recorded pour, standing in the replay for the controller that replaces it."""

    def __init__(self, start, end, source_t0, recorded_duration, reference,
                 operation=POUR_OPERATION):
        super().__init__(
            'pour',
            times=[0.0, recorded_duration],
            positions=[list(start), list(end)],
            operations=(operation,),
            source_t0=source_t0,
        )
        #: How long the recording spent pouring. Informational: the controller
        #: pours by weight and takes as long as that takes.
        self.recorded_duration = recorded_duration
        #: The recording's deepest tilt: the configuration that shows how it
        #: pours, for the controller to tip about the same axis the same way.
        self.reference = list(reference)

    @property
    def start(self):
        """The configuration the pour is handed over in, and returned to."""
        return self.positions[0]

    def __repr__(self):
        return '<Segment pour at t=%.2fs, %.2fs recorded, poured by weight instead>' % (
            self.source_t0, self.recorded_duration)


def pour_span(recording, operation=POUR_OPERATION, tolerance=SAME_POSE_TOLERANCE_RAD):
    """``(first, last)`` waypoint indices of the recording's pour, or refuse."""
    rows = [i for i, label in enumerate(recording.operations) if label == operation]
    if not rows:
        raise RecordingRejected(
            "no waypoint is labelled %r, so there is no pour to hand to the pouring "
            'controller. Replay it without a pour target' % operation)
    first, last = rows[0], rows[-1]
    if rows != list(range(first, last + 1)):
        raise RecordingRejected(
            'the %r waypoints are not one contiguous run (%d of the %d between rows %d '
            'and %d): more than one pour, or one interrupted by something else, is not '
            'something one pour goal can stand in for'
            % (operation, len(rows), last - first + 1, first, last))
    if last == first:
        raise RecordingRejected(
            'the recording pours for a single waypoint (row %d): that is a label, not a '
            'pour' % first)

    t0, t1 = recording.times[first], recording.times[last]
    for when, grasp in recording.gripper_events:
        if t0 < when < t1:
            raise RecordingRejected(
                'the gripper %s at t=%.3f, inside the recorded pour (%.3f..%.3f s). The '
                'jaws stay shut through a pour; this recording is not one a pour goal can '
                'stand in for' % ('closes' if grasp else 'opens', when, t0, t1))

    start, end = recording.positions[first], recording.positions[last]
    worst, joint = max(
        (abs(a - b), name) for a, b, name in zip(start, end, recording.joint_names))
    if worst > tolerance:
        raise RecordingRejected(
            'the recorded pour ends %.4f rad from where it started (%s). The pouring '
            'controller returns the arm to where it was handed over, and the replay '
            'resumes from where the recording\'s pour ended, so those have to be one '
            'place (tolerance %.4f rad)' % (worst, joint, tolerance))
    return first, last


def deepest_waypoint(recording, first, last):
    """The pour's waypoint furthest from where it starts, in joint space.

    A pour tips out and comes back, so that is its deepest tilt -- without
    needing a kinematic model here to say so. The controller, which has one,
    works out the axis from it.
    """
    start = recording.positions[first]
    return max(range(first, last + 1), key=lambda i: sum(
        (q - q0) ** 2 for q, q0 in zip(recording.positions[i], start)))


def splice_pour(recording, segments, operation=POUR_OPERATION,
                tolerance=SAME_POSE_TOLERANCE_RAD):
    """*segments* with the recorded pour replaced by a :class:`PourSegment`.

    The move that contains the pour is cut at its first and last waypoints: the
    part before still runs up to the pour's first waypoint, and the part after
    starts from its last, which is where the controller will have left the arm.
    Every other segment passes through untouched.
    """
    first, last = pour_span(recording, operation, tolerance)
    out = []
    replaced = False
    for segment in segments:
        if replaced or segment.kind != 'move':
            out.append(segment)
            continue
        begin, end = _span_of(recording, segment)
        if not begin <= first <= last <= end:
            out.append(segment)
            continue
        if first > begin:
            before = _move(recording, begin, first)
            # Labelled by what it does: it reaches the pour, and pours nothing.
            before.operations = _operations_in(recording, begin, first - 1)
            out.append(before)
        out.append(PourSegment(
            recording.positions[first], recording.positions[last],
            source_t0=recording.times[first],
            recorded_duration=recording.times[last] - recording.times[first],
            reference=recording.positions[deepest_waypoint(recording, first, last)],
            operation=operation))
        if last < end:
            after = _move(recording, last, end)
            after.operations = _operations_in(recording, last + 1, end)
            out.append(after)
        replaced = True
    if not replaced:
        # pour_span already refused a gripper event inside the pour, so the
        # pour lies within one move; reaching here means the segments were not
        # cut from this recording.
        raise RecordingRejected(
            'the recorded pour (t=%.3f..%.3f) is not inside any one move segment; were '
            'these segments planned from this recording?'
            % (recording.times[first], recording.times[last]))
    return out


def _span_of(recording, segment):
    """``(first, last)`` waypoint indices a move segment was cut from."""
    t0 = segment.source_t0
    t1 = t0 + segment.times[-1]
    begin = min(range(len(recording.times)), key=lambda i: abs(recording.times[i] - t0))
    end = begin
    # A single-waypoint move is padded to two points MIN_SAMPLE_DT apart, and
    # the next recorded waypoint is tens of ms on, so the padding never reaches
    # it and the segment stays one row.
    while end + 1 < len(recording.times) and recording.times[end + 1] <= t1 + 1e-6:
        end += 1
    return begin, end


__all__ = [
    'POUR_OPERATION',
    'PourSegment',
    'SAME_POSE_TOLERANCE_RAD',
    'deepest_waypoint',
    'pour_span',
    'splice_pour',
]
