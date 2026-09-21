"""Check, and keep checking, that the cell is laid out the way a plan assumes.

A recorded trajectory is position control that senses nothing: the arm goes to
where the vessels were when the recording was planned, whether or not anything
is there. ``utils/trajectory_recording`` already gates a replay on that, by
comparing the recording's assumed layout against a YAML file someone keeps in
step with the bench by hand. These two behaviours let a camera answer the same
question instead.

They are the same comparison at two different moments, and the difference in
what each returns is the whole design:

``ObjectLayoutCheckBehavior``
    Once, before the motion. SUCCESS or FAILURE -- it is a gate, so a cell that
    does not match refuses rather than warns.
``ObjectLayoutMonitorBehavior``
    Continuously, beside the motion, as a ``guarded_mission(monitor=...)``
    watchdog. RUNNING while everything it can see is where it should be,
    FAILURE the moment something is not.

**Silence is not a trip.** The monitor holds its opinion when a vessel stops
being visible, unlike ``SafetyMonitorBehavior``, whose staleness IS a trip. The
reason is that the arm occludes the bench on purpose here -- it is reaching
across it -- so a monitor that fired on a gap would abort exactly the runs it
exists to protect. It fires only on a vessel it can SEE, in a place it should
not be.

Yaw is deliberately never measured. ``compare_layout`` skips the yaw check for
an entry that declares none, and these behaviours always declare none: the
published orientation carries a yaw folded into a half turn (a parallel gripper
does not care which way round it grasps), so comparing it against a recording's
yaw would disagree for half of all headings while the vessel sat perfectly
placed. The vessels this watches are round and have no yaw worth checking
anyway; a fixture that does needs a different measurement, not a looser
tolerance here.
"""

import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cho_task_manager.utils.blackboard import TASK_NAMESPACE, read_client, read_if_set
from cho_task_manager.utils.trajectory_recording import (
    DEFAULT_POSITION_TOLERANCE_M,
    compare_layout,
)

# BEST_EFFORT for the same reason SafetyMonitorBehavior uses it: a best-effort
# subscription matches a reliable publisher as well as a best-effort one, while
# a reliable subscription silently never matches a sensor-data publisher.
# cho_object_pose is reliable today; a monitor that receives nothing while
# reporting healthy is the worst outcome available, so it asks for the weaker
# guarantee. Depth 1: only the newest pose can trip anything.
_LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def _measured(name, position):
    """One ``compare_layout`` entry from a detected position.

    ``xy`` only, never ``yaw_deg`` -- see the module docstring.
    """
    return {name: {'xy': [float(position[0]), float(position[1])]}}


def _expected_subset(expected, names):
    """The part of *expected* that *names* covers.

    Passing the whole assumed layout would report every object no camera
    tracks as 'the cell declares none', which is true of the comparison and
    false of the bench. What a camera cannot see stays the declared layout
    file's business, and the caller is told which those are.
    """
    return {name: entry for name, entry in expected.items() if name in names}


class ObjectLayoutCheckBehavior(py_trees.behaviour.Behaviour):
    """SUCCESS when every measured object is where *expected* says, else FAILURE.

    Reads poses another behaviour has already latched -- ``PoseTargetBehavior``
    is the producer -- rather than subscribing itself. That keeps the waiting,
    the timeout and the frame check in the one behaviour that already does them
    well, and leaves this one as the comparison alone.

    ``expected`` is the layout a plan assumes, in the shape
    ``utils/trajectory_recording`` uses: ``{name: {'xy': [x, y]}}``. ``keys``
    maps those names to the blackboard entries holding the detected poses.
    """

    def __init__(
        self,
        name: str,
        expected: dict,
        keys: dict,
        position_tolerance_m: float = DEFAULT_POSITION_TOLERANCE_M,
        namespace: str = TASK_NAMESPACE,
    ):
        super().__init__(name)
        if not keys:
            raise ValueError(
                f'[{name}] no object is measured, so this would pass without '
                'checking anything. Leave it out instead.')
        missing = sorted(set(keys) - set(expected))
        if missing:
            raise ValueError(
                f'[{name}] {missing} are measured but the plan assumes no such '
                'object, so there is nothing to compare them against.')
        self.expected = expected
        self.keys = dict(keys)
        self.position_tolerance_m = position_tolerance_m
        self.namespace = namespace
        self.node = None
        self.board = read_client(name, list(self.keys.values()), namespace)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        return True

    def update(self):
        measured = {}
        for name, key in self.keys.items():
            pose = read_if_set(self.board, key)
            if pose is None:
                self.node.get_logger().error(
                    f'[{self.name}] blackboard {self.namespace}/{key} is not set, '
                    f"so '{name}' was never detected; the behaviour that latches "
                    'it has to run, and succeed, first.')
                return py_trees.common.Status.FAILURE
            measured.update(_measured(name, (pose.position.x, pose.position.y)))

        problems = compare_layout(
            _expected_subset(self.expected, self.keys), measured,
            position_tolerance_m=self.position_tolerance_m)
        unverified = sorted(set(self.expected) - set(self.keys))

        if problems:
            self.node.get_logger().error(
                f'[{self.name}] the cell does not match what the plan assumes:')
            for problem in problems:
                self.node.get_logger().error(f'[{self.name}]   {problem}')
            return py_trees.common.Status.FAILURE

        for name in sorted(measured):
            want = self.expected[name]['xy']
            have = measured[name]['xy']
            offset = ((have[0] - want[0]) ** 2 + (have[1] - want[1]) ** 2) ** 0.5
            self.node.get_logger().info(
                f'[{self.name}] {name} measured at [{have[0]:+.4f}, {have[1]:+.4f}] m, '
                f'{offset * 1e3:.1f} mm from the plan '
                f'(tolerance {self.position_tolerance_m * 1e3:.1f})')
        if unverified:
            # Said out loud rather than passed over: the run is only as checked
            # as the objects a camera actually watches.
            self.node.get_logger().warn(
                f'[{self.name}] NOT measured, still taken on trust from the '
                f'declared layout: {unverified}')
        return py_trees.common.Status.SUCCESS


class ObjectLayoutMonitorBehavior(py_trees.behaviour.Behaviour):
    """RUNNING while every visible object is where *expected* says; FAILURE on drift.

    A watchdog, so it never returns SUCCESS -- ``watched_mission`` puts the
    success condition on the mission alone.

    ``topics`` maps object names to the ``PoseStamped`` topic each is published
    on. Only the objects given are watched, and that set is the caller's
    decision for a reason: a replay that transfers a vessel MOVES it on
    purpose, and a monitor that did not know which one is being carried would
    abort the run it is there to protect. Watch the vessels that are supposed
    to stay put.
    """

    def __init__(
        self,
        name: str,
        expected: dict,
        topics: dict,
        required_frame: str,
        position_tolerance_m: float = DEFAULT_POSITION_TOLERANCE_M,
        report_period_sec: float = 0.0,
    ):
        super().__init__(name)
        if not topics:
            raise ValueError(
                f'[{name}] no object is watched; give at least one, or leave the '
                'monitor out rather than watching nothing')
        missing = sorted(set(topics) - set(expected))
        if missing:
            raise ValueError(
                f'[{name}] {missing} are watched but the plan assumes no such '
                'object, so there is nothing to compare them against.')
        self.expected = expected
        self.topics = dict(topics)
        self.required_frame = required_frame
        self.position_tolerance_m = position_tolerance_m
        self.report_period_sec = report_period_sec
        self.node = None
        self._latest = {}          # name -> (x, y, frame_id)
        self._last_report = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        group = ReentrantCallbackGroup()
        for name, topic in self.topics.items():
            self.node.create_subscription(
                PoseStamped, topic,
                lambda msg, name=name: self._on_pose(name, msg),
                _LATEST, callback_group=group)
        self.node.get_logger().info(
            f'[{self.name}] watching {sorted(self.topics)} stay within '
            f'{self.position_tolerance_m * 1e3:.1f} mm of the plan')
        return True

    def _on_pose(self, name, msg):
        self._latest[name] = (msg.pose.position.x, msg.pose.position.y,
                              msg.header.frame_id)

    def update(self):
        for name, (x, y, frame) in sorted(self._latest.items()):
            if frame != self.required_frame:
                self.node.get_logger().error(
                    f"[{self.name}] {name} arrived in frame '{frame}', expected "
                    f"'{self.required_frame}'. Nothing here transforms frames, so "
                    'the comparison would be meaningless.')
                return py_trees.common.Status.FAILURE
            problems = compare_layout(
                _expected_subset(self.expected, [name]), _measured(name, (x, y)),
                position_tolerance_m=self.position_tolerance_m)
            if problems:
                self.node.get_logger().error(
                    f'[{self.name}] {problems[0]}; stopping the mission')
                return py_trees.common.Status.FAILURE
        self._report()
        # Never SUCCESS: a watchdog has no success condition, and the Parallel's
        # policy selects the mission branch for that.
        return py_trees.common.Status.RUNNING

    def _report(self):
        """Log the measured drift, so a known-good run produces the tolerance."""
        if self.report_period_sec <= 0.0:
            return
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self._last_report is not None and now - self._last_report < self.report_period_sec:
            return
        self._last_report = now
        if not self._latest:
            # Not a trip -- see the module docstring -- but worth saying, because
            # a monitor nobody is feeding looks exactly like a healthy one.
            self.node.get_logger().info(
                f'[{self.name}] nothing visible yet on {sorted(self.topics)}')
            return
        parts = []
        for name, (x, y, _frame) in sorted(self._latest.items()):
            want = self.expected[name]['xy']
            offset = ((x - want[0]) ** 2 + (y - want[1]) ** 2) ** 0.5
            parts.append(f'{name} {offset * 1e3:.1f} mm')
        self.node.get_logger().info(f'[{self.name}] drift: ' + ', '.join(parts))
