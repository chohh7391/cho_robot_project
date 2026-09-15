"""Hold the tree open while an external executor drives the arm.

The FR5 TAMP mission runs the other way round from every other task here: the
plan lives in another workspace (sdl_project's ``tamp_server``) and that
process sends its trajectories straight to ``joint_trajectory_controller``.
The tree does not command that motion. Its job is to own the arm's controller
state around it -- switch the trajectory controller in, switch the hold back
out -- and to stay up as the supervisor for as long as it lasts.

This behaviour is the waiting half of that handover. It returns RUNNING while
the external session works, which is what keeps a safety monitor ticking beside
it and keeps the safe-abort branch reachable. Without it the mission would
reach a terminal status the moment the switch succeeded, the node would shut
down, and the arm would be left under a controller with nobody watching.

The session is observed, not negotiated. ``/tamp_current_op`` is a plain String
the executor already publishes: an operator name while a step runs, ``idle``
when the plan is over. Two rules follow from that being a status topic rather
than a protocol:

- The subscription is VOLATILE on purpose. The publisher is TRANSIENT_LOCAL, so
  a late subscriber is handed the last value of the PREVIOUS run -- an ``idle``
  left over from a plan that finished yesterday would end this session before
  it began. Volatile means only values published after this behaviour starts
  are counted.
- ``idle`` ends the session only once some other value has been seen. An
  executor is entitled to announce it is idle before it starts working.

An executor that dies mid-plan publishes nothing further, so the session ends
at ``session_timeout_sec`` rather than at a failure it never reported. That is
deliberate: silence is not completion, and the timeout failing the mission is
what routes the arm to the hold controller through the abort branch.
"""

import py_trees
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

#: Status topic sdl_project's tamp_server already publishes the running
#: operator name on. Not a command channel -- see the module docstring.
DEFAULT_SESSION_TOPIC = '/tamp_current_op'

#: The value that means "no operator is running".
DEFAULT_IDLE_VALUE = 'idle'


class ExternalSessionBehavior(py_trees.behaviour.Behaviour):
    """RUNNING until an external executor reports its session finished.

    SUCCESS when *idle_value* arrives after at least one other value.
    FAILURE when nothing but *idle_value* arrives within *start_timeout_sec*
    (the executor never started), or when the session outlasts
    *session_timeout_sec*.
    """

    def __init__(
        self,
        name: str,
        topic: str = DEFAULT_SESSION_TOPIC,
        idle_value: str = DEFAULT_IDLE_VALUE,
        start_timeout_sec: float = 180.0,
        session_timeout_sec: float = 1800.0,
    ):
        super().__init__(name)
        if session_timeout_sec <= start_timeout_sec:
            raise ValueError(
                f'[{name}] session_timeout_sec ({session_timeout_sec}) must exceed '
                f'start_timeout_sec ({start_timeout_sec}): the start window is the '
                'first part of the session, not a separate one')
        self.topic = topic
        self.idle_value = idle_value
        self.start_timeout_sec = start_timeout_sec
        self.session_timeout_sec = session_timeout_sec
        self.node = None
        self.subscription = None
        self._latest = None
        self._seen_activity = False
        self._start_deadline = None
        self._session_deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscription = self.node.create_subscription(
            String, self.topic, self._on_op,
            # VOLATILE against a TRANSIENT_LOCAL publisher: compatible, and it
            # keeps the previous run's last value out of this one.
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=ReentrantCallbackGroup(),
        )
        return True

    def _on_op(self, msg):
        self._latest = msg.data

    def initialise(self):
        self._latest = None
        self._seen_activity = False
        now = self.node.get_clock().now()
        self._start_deadline = now + Duration(seconds=self.start_timeout_sec)
        self._session_deadline = now + Duration(seconds=self.session_timeout_sec)
        self.node.get_logger().info(
            f'[{self.name}] arm handed over; watching {self.topic} for the '
            f'session to finish (start within {self.start_timeout_sec:.0f}s, '
            f'session under {self.session_timeout_sec:.0f}s)')

    def update(self):
        now = self.node.get_clock().now()

        if self._latest is not None and self._latest != self.idle_value:
            if not self._seen_activity:
                self.node.get_logger().info(
                    f"[{self.name}] external executor started: '{self._latest}'")
            self._seen_activity = True

        if self._seen_activity and self._latest == self.idle_value:
            self.node.get_logger().info(
                f'[{self.name}] external session reported finished')
            return py_trees.common.Status.SUCCESS

        if not self._seen_activity and now > self._start_deadline:
            self.node.get_logger().error(
                f'[{self.name}] no operator published on {self.topic} within '
                f'{self.start_timeout_sec:.0f}s. The trajectory controller is '
                'active and nothing is driving it; is the external executor '
                'running and pointed at this robot?')
            return py_trees.common.Status.FAILURE

        if now > self._session_deadline:
            self.node.get_logger().error(
                f'[{self.name}] external session exceeded '
                f"{self.session_timeout_sec:.0f}s (last operator: "
                f"'{self._latest}'). Failing so the arm is put on its hold "
                'controller rather than left under an unwatched one.')
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._start_deadline = None
        self._session_deadline = None
