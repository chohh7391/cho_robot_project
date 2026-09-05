"""Sample the end-effector pose and report what a probe actually achieved.

The MIT task controller's `peak_wrench` / `peak_tau_ff` are cumulative highs
since activation, so once a larger probe has run earlier in the same session
their growth reads zero for every later probe and says nothing about a gain
change. Measured TCP displacement does not have that problem: it is the
per-probe number a gain comparison actually turns on.
"""

import math

import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

BLACKBOARD_NAMESPACE = '/mit_tuning'
DEFAULT_EE_POSE_TOPIC = '/ee_state/pose'


class EeStateSampleBehavior(py_trees.behaviour.Behaviour):
    """Latch the current TCP position, optionally against an earlier sample.

    With `compare_to` it reports the straight-line displacement since that
    sample. With `commanded` as well it reports the fraction of the commanded
    probe the arm actually covered, which is the figure to compare between
    gain settings.
    """

    def __init__(
        self,
        name: str,
        record_as: str = None,
        compare_to: str = None,
        commanded: float = None,
        topic: str = DEFAULT_EE_POSE_TOPIC,
        timeout_sec: float = 5.0,
    ):
        super().__init__(name)
        self.topic = topic
        self.record_as = record_as
        self.compare_to = compare_to
        self.commanded = commanded
        self.timeout_sec = timeout_sec
        self.node = None
        self.subscription = None
        self._latest = None
        self._deadline = None
        self.board = py_trees.blackboard.Client(name=name, namespace=BLACKBOARD_NAMESPACE)
        for key in (record_as, compare_to):
            if key:
                self.board.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        # The broadcaster publishes best-effort at the controller rate; a
        # reliable subscription would simply never match it.
        self.subscription = self.node.create_subscription(
            PoseStamped, self.topic, self._on_pose,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=ReentrantCallbackGroup(),
        )
        return True

    def _on_pose(self, msg):
        position = msg.pose.position
        self._latest = (position.x, position.y, position.z)

    def initialise(self):
        # Drop any pose cached before this tick so the sample belongs to the
        # motion that just finished, not to the one before it.
        self._latest = None
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.timeout_sec)

    def update(self):
        if self._latest is None:
            if self._deadline is not None and self.node.get_clock().now() > self._deadline:
                self.node.get_logger().error(
                    f'[{self.name}] No {self.topic} message within {self.timeout_sec}s; '
                    'is ee_state_broadcaster active?'
                )
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        sample = self._latest
        lines = [
            f'[{self.name}] TCP = '
            f'[{sample[0]:+.5f}, {sample[1]:+.5f}, {sample[2]:+.5f}] m'
        ]

        before = getattr(self.board, self.compare_to, None) if self.compare_to else None
        if before:
            delta = [a - b for a, b in zip(sample, before)]
            achieved = math.dist(sample, before)
            lines.append(
                f'    displacement  = '
                f'[{delta[0] * 1000:+.2f}, {delta[1] * 1000:+.2f}, {delta[2] * 1000:+.2f}] mm'
            )
            lines.append(f'    magnitude     = {achieved * 1000:.2f} mm')
            if self.commanded:
                lines.append(f'    commanded     = {self.commanded * 1000:.2f} mm')
                lines.append(
                    f'    achieved      = {100.0 * achieved / self.commanded:.1f} % '
                    '(compare this between gain settings)'
                )
        self.node.get_logger().info('\n'.join(lines))

        if self.record_as:
            setattr(self.board, self.record_as, sample)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._deadline = None
