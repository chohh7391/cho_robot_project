"""Succeed only once the arm is where a step says it will be.

A replay resumes from a configuration another controller was supposed to leave
the arm in. The trajectory controller would take the first waypoint from
wherever the arm actually is, so an arm left somewhere else turns the resume
into a lunge. This checks /joint_states against that configuration first.
"""

import math

import py_trees
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateCheckBehavior(py_trees.behaviour.Behaviour):
    """SUCCESS when every named joint is within *tolerance* [rad] of *target*."""

    def __init__(self, name: str, joint_names: list, target: list, tolerance: float = 0.01,
                 timeout_sec: float = 3.0, topic: str = '/joint_states'):
        super().__init__(name)
        if len(joint_names) != len(target):
            raise ValueError(f'[{name}] {len(joint_names)} joint names for {len(target)} targets')
        if not tolerance > 0.0:
            raise ValueError(f'[{name}] tolerance must be positive, got {tolerance}')
        self.joint_names = list(joint_names)
        self.target = [float(q) for q in target]
        self.tolerance = tolerance
        self.timeout_sec = timeout_sec
        self.topic = topic
        self.node = None
        self._latest = None
        self._deadline = None
        self._worst = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.create_subscription(
            JointState, self.topic, self._on_joints, qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup())
        return True

    def _on_joints(self, msg):
        self._latest = msg

    def initialise(self):
        self._latest = None
        self._worst = None
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.timeout_sec)

    def update(self):
        msg = self._latest
        if msg is not None:
            by_name = dict(zip(msg.name, msg.position))
            if all(name in by_name for name in self.joint_names):
                errors = [(abs(by_name[name] - q), name)
                          for name, q in zip(self.joint_names, self.target)]
                self._worst = max(errors)
                if math.isfinite(self._worst[0]) and self._worst[0] <= self.tolerance:
                    return py_trees.common.Status.SUCCESS
        if self.node.get_clock().now() > self._deadline:
            if self._worst is None:
                why = f'no complete {self.topic} for {self.joint_names}'
            else:
                why = (f'{self._worst[1]} is {self._worst[0]:.4f} rad from where it should be '
                       f'(tolerance {self.tolerance:.4f})')
            self.node.get_logger().error(f'[{self.name}] the arm is not where this step expects: {why}')
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._deadline = None
