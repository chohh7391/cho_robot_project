"""Latch what the scale settles at into the blackboard: a pour's zero.

The HS-AA cannot be tared over RS232, so the only zero a pour has is the empty
receiving vessel's weight, and ``PourActionBehavior`` needs it before the goal
is sent. A tree that is not told the number reads it here, off the same
``cho_interfaces/ScaleReading`` the pouring controller closes its loop on.

Deliberately strict about SETTLED -- the same rule ``fr5_pour_client
--container auto`` applies: the reading has to hold one value, flagged stable,
for ``settle_sec``. Whatever it latches becomes the zero of every gram the pour
reports, and the controller then checks the pan against it -- so a number
taken while something on the pan was still moving would pass that check and
be wrong in every result.
"""

import math

import py_trees
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from cho_interfaces.msg import ScaleReading
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, write_client

DEFAULT_SCALE_TOPIC = '/scale/reading'


class ScaleLatchBehavior(py_trees.behaviour.Behaviour):
    """Record the settled scale reading under blackboard *record_as*."""

    def __init__(
        self,
        name: str,
        record_as: str,
        topic: str = DEFAULT_SCALE_TOPIC,
        settle_sec: float = 2.0,
        timeout_sec: float = 15.0,
        namespace: str = TASK_NAMESPACE,
    ):
        super().__init__(name)
        if not record_as:
            raise ValueError(f'[{name}] record_as is required')
        if not settle_sec > 0.0 or not timeout_sec > settle_sec:
            raise ValueError(
                f'[{name}] settle_sec must be positive and timeout_sec longer than it '
                f'(got {settle_sec}, {timeout_sec})')
        self.record_as = record_as
        self.topic = topic
        self.settle_sec = settle_sec
        self.timeout_sec = timeout_sec
        self.namespace = namespace
        self.node = None
        self.subscription = None
        self._latest = None
        self._last_grams = None
        self._stable_since = None
        self._deadline = None
        self.board = write_client(name, [record_as], namespace)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        # The relay publishes sensor-data QoS (best effort); a reliable
        # subscription would never match it.
        self.subscription = self.node.create_subscription(
            ScaleReading, self.topic, self._on_reading, qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup())
        return True

    def _on_reading(self, msg):
        self._latest = msg

    def initialise(self):
        # Only readings from this tick on: a number sitting on the topic from
        # before may describe a pan that has since been changed.
        self._latest = None
        self._last_grams = None
        self._stable_since = None
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.timeout_sec)

    def update(self):
        now = self.node.get_clock().now()
        msg, self._latest = self._latest, None
        if msg is not None and math.isfinite(msg.grams):
            unchanged = self._last_grams is not None and abs(msg.grams - self._last_grams) < 1e-9
            if unchanged and msg.stable:
                if self._stable_since is None:
                    self._stable_since = now
                elif (now - self._stable_since).nanoseconds * 1e-9 >= self.settle_sec:
                    self.node.get_logger().info(
                        f'[{self.name}] {self.namespace}/{self.record_as} = {msg.grams:.2f} g '
                        f'(held {self.settle_sec:.1f} s, flagged stable)')
                    setattr(self.board, self.record_as, float(msg.grams))
                    return py_trees.common.Status.SUCCESS
            else:
                self._stable_since = None
            self._last_grams = msg.grams

        if now > self._deadline:
            if self._last_grams is None:
                self.node.get_logger().error(
                    f'[{self.name}] no reading on {self.topic} within {self.timeout_sec:.0f} s. '
                    'Is the scale driver up, and the relay (`ros2 run cho_control_tools '
                    'scale_relay`) running?')
            else:
                self.node.get_logger().error(
                    f'[{self.name}] the scale never held one value for {self.settle_sec:.1f} s '
                    f'within {self.timeout_sec:.0f} s (last {self._last_grams:.2f} g): '
                    'something on the pan is still moving')
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._deadline = None
