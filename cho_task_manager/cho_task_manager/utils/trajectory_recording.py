"""Read a recorded joint trajectory, and decide what may be replayed from it.

A trajectory recorded in a simulator and replayed on this arm is **position
control that senses nothing**. Everything in this module exists because of that
one fact, and none of it is book-keeping:

* **The layout is part of the trajectory.** The recording was planned against
  vessels at particular places; the arm will go to those places whether or not
  anything is there. So a replay is gated on the cell matching the layout the
  recording carries, and a mismatch REFUSES rather than warns. A warning that
  scrolls past is how a pour lands on the bench.
* **The gripper is a separate channel.** The CSV has no gripper column -- in the
  recording the gripper was a service, not a topic, and its events were
  recovered from the simulated finger joint, so their times are approximate.
  The trajectory is therefore CUT at each event and the real gripper commanded
  in the gap, which is also what makes the approximate time stop mattering: the
  recording really says "the gripper closed between these two waypoints".
* **Nothing here re-plans or re-interpolates.** The waypoints are replayed as
  recorded. The only sample this module ever drops is one that shares a
  timestamp with its neighbour, which is a recording artefact a trajectory
  controller would reject outright (point times must strictly increase).

The gripper events are the ONLY thing the recording is cut at. Cutting at the
``operation`` labels as well was tried and removed: measured on the transfer
recording, the arm is already at a full stop at every operation boundary (zero
joint speed either side), so the extra cuts produced identical motion in more
steps. The labels are still carried through, because a segment that says
``Move_to_Surface+pouring+Place`` is the one useful thing they give a log line.

A linked workflow (Move -> Transfer -> Stir) changes tools partway through and
CANNOT be replayed by this module as one recording -- this arm carries one
gripper and the repo has no tool-change action. Replay the workflows either
side of a tool change separately.

Nothing here imports ROS: it is the part worth testing without a graph.
"""

import csv
import json
import math
import os

#: Two samples closer together than this [s] are one sample published twice.
#: Well under the ~40 ms a planner publishes at, and well over the
#: sub-millisecond spacing that marks the artefact, so it separates the two
#: without judgement.
MIN_SAMPLE_DT = 0.001

#: Default tolerances for the layout gate. An object further than this from
#: where the recording assumed it refuses the replay. 15 mm is roughly the
#: mouth radius of the flask the pour has to hit, so a layout inside it is one
#: the recorded pour can still land in; beyond it the pour is aimed off the
#: vessel and replaying is pointless.
DEFAULT_POSITION_TOLERANCE_M = 0.015
#: Yaw matters much less for the symmetric vessels and a great deal for a
#: rectangular fixture, so it is checked but loosely.
DEFAULT_YAW_TOLERANCE_DEG = 10.0


class RecordingRejected(ValueError):
    """A recording could not be turned into something replayable."""


class LayoutMismatch(RecordingRejected):
    """The cell is not laid out the way the recording assumes."""


class Segment:
    """One step of a replay: a span of waypoints, or a gripper command.

    ``kind`` is ``'move'`` or ``'gripper'``. A move carries ``times`` (seconds
    from the START OF THAT SEGMENT, so it can be handed to a trajectory builder
    directly) and ``positions``; a gripper carries ``grasp``.
    """

    def __init__(self, kind, times=None, positions=None, grasp=None,
                 operations=(), source_t0=0.0):
        self.kind = kind
        self.times = times
        self.positions = positions
        self.grasp = grasp
        #: The CSV operation labels this segment spans, in order.
        self.operations = tuple(operations)
        #: Where the segment starts on the ORIGINAL recording's clock, so a log
        #: line can be matched back to the CSV and to the meta's operation list.
        #: For a gripper segment this is the EVENT's own time.
        self.source_t0 = source_t0

    @property
    def operation(self):
        """The labels this segment spans, joined - what a log line wants."""
        return '+'.join(self.operations)

    @property
    def duration(self):
        """Seconds of motion, 0 for a gripper command."""
        if self.kind != 'move' or not self.times:
            return 0.0
        return self.times[-1] - self.times[0]

    def __repr__(self):
        """Render the segment the way a log line wants it."""
        if self.kind == 'gripper':
            return '<Segment gripper %s>' % ('close' if self.grasp else 'open')
        return '<Segment move %s %d pts %.2fs>' % (
            self.operation or 'move', len(self.times), self.duration)


class Recording:
    """A recorded trial: its waypoints, its gripper events and the layout it assumes."""

    def __init__(self, times, positions, operations, meta, dropped, source, joint_names):
        self.times = times
        self.positions = positions
        self.operations = operations
        self.meta = meta
        #: (index, time) of every sample dropped as a duplicate timestamp.
        self.dropped = dropped
        self.source = source
        self.joint_names = joint_names

    @property
    def duration(self):
        """Seconds from the first waypoint to the last."""
        return self.times[-1] - self.times[0]

    @property
    def gripper_events(self):
        """``[(t_s, grasp_bool)]`` from the meta, in time order."""
        events = []
        for entry in self.meta.get('gripper_events', []):
            event = entry.get('event')
            if event not in ('open', 'close'):
                raise RecordingRejected(
                    "gripper event %r is neither 'open' nor 'close'" % (event,))
            events.append((float(entry['t_s']), event == 'close'))
        return sorted(events)

    @property
    def layout(self):
        """The object placement the trajectory was planned against."""
        return self.meta.get('layout_the_trajectory_assumes', {})

    @property
    def home(self):
        """The configuration the first waypoint starts from, from the meta.

        Falls back to the first waypoint itself. The two agree to a couple of
        milliradians in practice, and the waypoint is the one the trajectory
        actually begins at, so it is the safer fallback of the two.
        """
        home = self.meta.get('home_arm_rad')
        if home is None:
            return list(self.positions[0])
        if len(home) != len(self.joint_names):
            raise RecordingRejected(
                'meta home_arm_rad has %d values for %d joints'
                % (len(home), len(self.joint_names)))
        return [float(value) for value in home]

    def peak_rate(self):
        """``(rate, index, joint)``: the fastest joint rate the waypoints imply [rad/s].

        The recording was timed by a planner for a simulator. Whether that is
        inside the envelope THIS arm is held to is a separate question, and this
        is the number that answers it.
        """
        peak, at, joint = 0.0, 0, self.joint_names[0]
        for index in range(len(self.times) - 1):
            span = self.times[index + 1] - self.times[index]
            for axis, name in enumerate(self.joint_names):
                rate = abs(self.positions[index + 1][axis] - self.positions[index][axis]) / span
                if rate > peak:
                    peak, at, joint = rate, index, name
        return peak, at, joint


def load_recording(csv_path, meta_path=None, joint_names=None):
    """Read a waypoint CSV and its meta, dropping duplicate-timestamp samples.

    *meta_path* defaults to the CSV's name with ``_waypoints.csv`` replaced by
    ``_meta.json``, which is how the exporter writes the pair. *joint_names*, if
    given, is the arm this will be replayed on: a recording in other joints is
    refused rather than mapped onto it by position.
    """
    if meta_path is None:
        if csv_path.endswith('_waypoints.csv'):
            meta_path = csv_path[:-len('_waypoints.csv')] + '_meta.json'
        else:
            meta_path = os.path.splitext(csv_path)[0] + '_meta.json'
    if not os.path.exists(meta_path):
        raise RecordingRejected(
            'no meta JSON at %s. It carries the gripper events and the layout '
            'the trajectory assumes, neither of which is in the CSV' % meta_path)

    with open(meta_path, encoding='utf-8') as handle:
        meta = json.load(handle)

    recorded_joints = list(meta.get('joint_names') or [])
    if not recorded_joints:
        raise RecordingRejected('%s declares no joint_names' % meta_path)
    if joint_names is not None and recorded_joints != list(joint_names):
        raise RecordingRejected(
            'the recording is in joints %s but this arm is %s'
            % (recorded_joints, list(joint_names)))

    times, positions, operations, dropped = [], [], [], []
    with open(csv_path, encoding='utf-8') as handle:
        for index, row in enumerate(csv.DictReader(handle)):
            missing = [name for name in recorded_joints if name not in row]
            if missing:
                raise RecordingRejected(
                    '%s row %d has no column for %s' % (csv_path, index, missing))
            now = float(row['t_s'])
            if times and now - times[-1] < MIN_SAMPLE_DT:
                # Keep the LATER sample of a pair published at the same instant:
                # the fresher one continues the motion, and the earlier one is
                # the stale tail of what came before it.
                dropped.append((index - 1, times[-1]))
                times.pop()
                positions.pop()
                operations.pop()
            times.append(now)
            positions.append([float(row[name]) for name in recorded_joints])
            operations.append((row.get('operation') or '').strip())

    if len(times) < 2:
        raise RecordingRejected('%s has fewer than two usable waypoints' % csv_path)
    for index in range(len(times) - 1):
        if times[index + 1] <= times[index]:
            raise RecordingRejected(
                'waypoint %d is at t=%.6f, not after %.6f. Times must increase '
                'even after duplicates are dropped'
                % (index + 1, times[index + 1], times[index]))

    declared = meta.get('waypoints')
    if declared is not None and declared != len(times) + len(dropped):
        raise RecordingRejected(
            'the meta declares %d waypoints but the CSV has %d'
            % (declared, len(times) + len(dropped)))

    return Recording(times, positions, operations, meta, dropped, csv_path, recorded_joints)


def load_cell_layout(path):
    """Read the layout the physical or simulated cell actually has.

    Same shape as the meta's ``layout_the_trajectory_assumes``: a mapping of
    object name to ``{'xy': [x, y], 'yaw_deg': deg}``. Declared by whoever set
    the cell up, because nothing on this path perceives it.
    """
    with open(path, encoding='utf-8') as handle:
        loaded = json.load(handle) if path.endswith('.json') else _safe_yaml(handle)
    layout = (loaded or {}).get('layout')
    if not isinstance(layout, dict):
        raise RecordingRejected(
            "%s declares no 'layout' mapping of object -> {xy, yaw_deg}" % path)
    return layout


def _safe_yaml(handle):
    """Parse YAML, imported lazily so the pure-python path has no hard dependency."""
    import yaml
    return yaml.safe_load(handle)


def compare_layout(assumed, actual,
                   position_tolerance_m=DEFAULT_POSITION_TOLERANCE_M,
                   yaw_tolerance_deg=DEFAULT_YAW_TOLERANCE_DEG):
    """Differences between the layout a recording assumes and the cell's own.

    Returns a list of human-readable problems; empty means the cell matches.
    An object the recording assumes and the cell does not declare is a problem:
    the arm will still go there, so "not declared" is not "not in the way".
    """
    problems = []
    for name in sorted(assumed):
        want = assumed[name]
        if name not in actual:
            problems.append(
                '%s: the recording assumes one at (%.4f, %.4f) and the cell '
                'declares none' % (name, want['xy'][0], want['xy'][1]))
            continue
        have = actual[name]
        dx = float(have['xy'][0]) - float(want['xy'][0])
        dy = float(have['xy'][1]) - float(want['xy'][1])
        offset = math.hypot(dx, dy)
        if offset > position_tolerance_m:
            problems.append(
                '%s is %.1f mm from where the recording assumes it '
                '(cell %.4f, %.4f vs recording %.4f, %.4f; tolerance %.1f mm)'
                % (name, offset * 1e3, have['xy'][0], have['xy'][1],
                   want['xy'][0], want['xy'][1], position_tolerance_m * 1e3))
        # A cell entry with no yaw_deg is declaring that this object HAS no
        # meaningful yaw -- which is the truth for the round vessels, drawn as
        # cylinders and modelled as square footprints. Checking it against the
        # recording's number would refuse a cell that matches perfectly.
        if 'yaw_deg' not in have:
            continue
        want_yaw = float(want.get('yaw_deg', 0.0))
        have_yaw = float(have['yaw_deg'])
        yaw_error = abs((have_yaw - want_yaw + 180.0) % 360.0 - 180.0)
        if yaw_error > yaw_tolerance_deg:
            problems.append(
                '%s is %.1f deg from the yaw the recording assumes '
                '(cell %.1f vs recording %.1f; tolerance %.1f deg)'
                % (name, yaw_error, have_yaw, want_yaw, yaw_tolerance_deg))
    return problems


def require_layout(recording, actual,
                   position_tolerance_m=DEFAULT_POSITION_TOLERANCE_M,
                   yaw_tolerance_deg=DEFAULT_YAW_TOLERANCE_DEG):
    """Raise :class:`LayoutMismatch` unless the cell matches the recording.

    This is the gate, and it raises rather than warns on purpose: a blind replay
    against a cell laid out differently does not fail, it puts the arm and
    whatever it is carrying somewhere nobody chose.
    """
    problems = compare_layout(
        recording.layout, actual, position_tolerance_m, yaw_tolerance_deg)
    if problems:
        raise LayoutMismatch(
            'the cell does not match the layout %s assumes:\n  %s'
            % (os.path.basename(recording.source), '\n  '.join(problems)))
    return True


def load_velocity_limits(path):
    """``{joint: max_velocity}`` from a MoveIt ``joint_limits.yaml``.

    That file, not the scaling factor, is where this arm's trajectory path is
    bounded: ``joint_trajectory_controller`` has no per-cycle clamp of its own,
    so nothing downstream of a goal enforces a rate. Its own header says so.
    """
    with open(path, encoding='utf-8') as handle:
        loaded = _safe_yaml(handle) or {}
    limits = {}
    for joint, entry in (loaded.get('joint_limits') or {}).items():
        if entry.get('has_velocity_limits') and entry.get('max_velocity'):
            limits[joint] = float(entry['max_velocity'])
    if not limits:
        raise RecordingRejected('%s declares no joint velocity limits' % path)
    return limits


def required_time_scale(recording, limits, scaling=1.0):
    """How much to stretch the recording so no joint exceeds its ceiling.

    1.0 when it already complies. Returns ``(scale, joint, rate, ceiling)`` so a
    caller can say WHY it slowed down: a silently stretched replay is a timing
    difference between the recording and the arm that stops being visible.

    Stretching is the right answer here and re-planning is not -- the shape of
    the path is the recorded result, and only its clock changes.
    """
    if scaling <= 0.0:
        raise RecordingRejected('velocity scaling must be positive (got %r)' % scaling)
    worst_scale, worst_joint, worst_rate, worst_ceiling = 1.0, None, 0.0, 0.0
    for index in range(len(recording.times) - 1):
        span = recording.times[index + 1] - recording.times[index]
        for axis, joint in enumerate(recording.joint_names):
            ceiling = limits.get(joint)
            if not ceiling:
                continue
            ceiling *= scaling
            rate = abs(recording.positions[index + 1][axis]
                       - recording.positions[index][axis]) / span
            scale = rate / ceiling
            if scale > worst_scale:
                worst_scale, worst_joint = scale, joint
                worst_rate, worst_ceiling = rate, ceiling
    if worst_joint is None:
        return 1.0, None, 0.0, 0.0
    # A hair over the exact factor, so the stretched trajectory lands inside the
    # ceiling rather than on it: point times are quantised to nanoseconds, and a
    # trajectory timed to exactly the limit reads back as just over it.
    return worst_scale * (1.0 + 1e-4), worst_joint, worst_rate, worst_ceiling


def plan_segments(recording):
    """Cut *recording* into moves and gripper commands, in execution order.

    The cuts are the gripper events, and nothing else. Each move's ``times``
    restart at 0, because a trajectory controller times a goal from when it
    accepts it -- a segment that began 22 s into the recording must still ask
    for its first point at 0.
    """
    events = {index: (grasp, when) for index, grasp, when in _event_cuts(recording)}
    last = len(recording.times) - 1
    cuts = sorted(index for index in events if index < last)

    segments = []
    start = 0
    for end in cuts + [last]:
        if end < start:
            continue
        segments.append(_move(recording, start, end))
        if end in events:
            grasp, when = events[end]
            # source_t0 is the EVENT's own time, not the waypoint the cut landed
            # on: the event time is what the recording actually says, and it is
            # what a log line at the rig should show.
            segments.append(Segment(
                'gripper', grasp=grasp,
                operations=_operations_in(recording, start, end), source_t0=when))
        start = end + 1
        if start > last:
            break

    if not any(segment.kind == 'move' for segment in segments):
        raise RecordingRejected('cutting the recording left no motion to replay')
    return segments


def _event_cuts(recording):
    """``[(waypoint_index, grasp, event_time)]``: the last waypoint at or before each event."""
    cuts = []
    for event_time, grasp in recording.gripper_events:
        index = None
        for candidate, now in enumerate(recording.times):
            if now <= event_time:
                index = candidate
            else:
                break
        if index is None:
            raise RecordingRejected(
                'a gripper event at t=%.3f falls before the first waypoint'
                % event_time)
        cuts.append((index, grasp, event_time))
    return cuts


def _operations_in(recording, first, last):
    """The distinct operation labels a span covers, in order, blanks skipped."""
    seen = []
    for index in range(first, last + 1):
        label = recording.operations[index]
        if label and (not seen or seen[-1] != label):
            seen.append(label)
    return tuple(seen)


def _move(recording, first, last):
    """A move segment over waypoints ``[first, last]``, re-timed from zero."""
    operations = _operations_in(recording, first, last)
    if last <= first:
        # One waypoint is not something a controller can interpolate, but it is
        # not nothing either: it is the arm holding still while the gripper
        # works. Give it a span so its point times strictly increase.
        return Segment(
            'move',
            times=[0.0, MIN_SAMPLE_DT],
            positions=[list(recording.positions[first]), list(recording.positions[first])],
            operations=operations,
            source_t0=recording.times[first])

    t0 = recording.times[first]
    return Segment(
        'move',
        times=[recording.times[i] - t0 for i in range(first, last + 1)],
        positions=[list(recording.positions[i]) for i in range(first, last + 1)],
        operations=operations,
        source_t0=t0)
