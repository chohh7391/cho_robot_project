"""Latch a pose published on a topic into a blackboard target.

This is the perception seam the task trees did not have. A detector publishes a
``PoseStamped``, this behaviour latches the first one that arrives after it
starts, and a ``TaskSpaceActionBehavior`` with a matching ``target_pose_key``
drives to it -- nothing about the target is known when the tree is built.

Nothing here transforms frames. An absolute TaskSpace goal is interpreted by
the action server in the robot's base frame, so a pose latched in a camera
frame would be driven to as if it were a base-frame pose. ``required_frame``
therefore has no default: the caller has to state which frame it expects, and
a message in any other frame fails the behaviour instead of being obeyed.
"""

import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cho_task_manager.utils.blackboard import TASK_NAMESPACE, write_client


class PoseTargetBehavior(py_trees.behaviour.Behaviour):
    """Record the next pose published on *topic* under blackboard *record_as*."""

    def __init__(
        self,
        name: str,
        record_as: str,
        topic: str,
        required_frame: str,
        timeout_sec: float = 5.0,
        best_effort: bool = False,
        namespace: str = TASK_NAMESPACE,
    ):
        super().__init__(name)
        if not record_as:
            raise ValueError(f'[{name}] record_as is required')
        self.topic = topic
        self.record_as = record_as
        self.required_frame = required_frame
        self.timeout_sec = timeout_sec
        # A detector publishing with default QoS is RELIABLE; a sensor-data
        # publisher is BEST_EFFORT and would never match a reliable
        # subscription, so the choice has to be the caller's.
        self.best_effort = best_effort
        self.namespace = namespace
        self.node = None
        self.subscription = None
        self._latest = None
        self._deadline = None
        self.board = write_client(name, [record_as], namespace)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        reliability = (
            ReliabilityPolicy.BEST_EFFORT if self.best_effort
            else ReliabilityPolicy.RELIABLE
        )
        self.subscription = self.node.create_subscription(
            PoseStamped, self.topic, self._on_pose,
            QoSProfile(depth=1, reliability=reliability),
            callback_group=ReentrantCallbackGroup(),
        )
        return True

    def _on_pose(self, msg):
        self._latest = msg

    def initialise(self):
        # Drop anything cached before this tick: the target must belong to this
        # run, not to a pose that was sitting on the topic beforehand.
        self._latest = None
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.timeout_sec)

    def update(self):
        if self._latest is None:
            if self._deadline is not None and self.node.get_clock().now() > self._deadline:
                self.node.get_logger().error(
                    f'[{self.name}] no {self.topic} message within '
                    f'{self.timeout_sec}s; is the publisher running'
                    f"{' (best_effort=True needed for a sensor-data publisher)' if not self.best_effort else ''}?"
                )
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        msg = self._latest
        frame = msg.header.frame_id
        if self.required_frame is None:
            self.node.get_logger().warn(
                f"[{self.name}] required_frame is None, so the pose is taken as-is "
                f"from frame '{frame}'. An absolute goal is driven in the robot's "
                'base frame; this is only safe if they are the same frame.')
        elif frame != self.required_frame:
            self.node.get_logger().error(
                f"[{self.name}] {self.topic} published a pose in frame '{frame}', "
                f"expected '{self.required_frame}'. Nothing here transforms "
                'frames, so obeying it would drive to the wrong place.')
            return py_trees.common.Status.FAILURE

        position = msg.pose.position
        self.node.get_logger().info(
            f'[{self.name}] target {self.namespace}/{self.record_as} = '
            f'[{position.x:+.5f}, {position.y:+.5f}, {position.z:+.5f}] m '
            f"in '{frame}'")
        setattr(self.board, self.record_as, msg.pose)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._deadline = None
