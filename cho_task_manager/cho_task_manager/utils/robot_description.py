"""The arm as the running robot description says it is.

One reader for every behaviour that needs a robot's joint limits or
kinematics: the ``/robot_description`` robot_state_publisher latches, parsed by
Pinocchio. A copy of the limits written into a task goes stale the first time
the description changes and nothing says so; the description is what the
controllers were loaded from, so it is the one to check against.

Pinocchio is imported lazily: only the behaviours that need it pay for loading
it, and a tree that uses none never does.
"""

import math

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DEFAULT_ROBOT_DESCRIPTION_TOPIC = '/robot_description'

# robot_state_publisher latches the description: RELIABLE + TRANSIENT_LOCAL, so
# a late subscriber needs TRANSIENT_LOCAL to receive it at all. And RELIABLE,
# not the BEST_EFFORT the sensor monitors use: the latched sample is delivered
# to a late joiner through the reliable protocol's history, which a best-effort
# reader is not guaranteed to get (Fast DDS does not send it one). Every
# description publisher here is robot_state_publisher, which is reliable, so
# nothing is lost by asking for it.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def arm_kinematics(urdf, joint_names, ee_link=None, source=DEFAULT_ROBOT_DESCRIPTION_TOPIC):
    """Pinocchio model, arm joint indices and position limits, from a URDF string.

    *joint_names* are the arm's joints (the registry's ``model.joints``); each
    has to be a single bounded coordinate. *ee_link*, when given, must be a
    frame of the model and its id is returned as ``frame_id``. *source* only
    names where the URDF came from in the error messages.
    """
    import pinocchio as pin

    model = pin.buildModelFromXML(urdf)
    idx_q, idx_v = [], []
    for name in joint_names:
        if not model.existJointName(name):
            raise ValueError(
                f"joint '{name}' is not in {source}; "
                'the registry entry and the running description disagree')
        joint = model.joints[model.getJointId(name)]
        if joint.nq != 1:
            raise ValueError(
                f"joint '{name}' has nq={joint.nq}; a single bounded coordinate per "
                'joint is required, so an unbounded or multi-DOF joint cannot be '
                'checked against a limit')
        idx_q.append(joint.idx_q)
        idx_v.append(joint.idx_v)
    kinematics = {
        'pin': pin,
        'model': model,
        'data': model.createData(),
        'idx_q': idx_q,
        'idx_v': idx_v,
        'lower': [float(model.lowerPositionLimit[i]) for i in idx_q],
        'upper': [float(model.upperPositionLimit[i]) for i in idx_q],
    }
    if ee_link is not None:
        if not model.existFrame(ee_link):
            raise ValueError(f"frame '{ee_link}' is not in {source}")
        kinematics['frame_id'] = model.getFrameId(ee_link)
    return kinematics


def position_limits(urdf, joint_names, source=DEFAULT_ROBOT_DESCRIPTION_TOPIC):
    """``{joint: (lower, upper)}`` for every one of *joint_names*, from a URDF string.

    Raises ValueError for a joint without a finite, non-empty range: a caller
    that asked for a joint's limits is about to check something against them,
    and an unchecked joint is not what it asked for.
    """
    kinematics = arm_kinematics(urdf, joint_names, source=source)
    limits = {}
    for name, lower, upper in zip(joint_names, kinematics['lower'], kinematics['upper']):
        if not (math.isfinite(lower) and math.isfinite(upper)) or upper <= lower:
            raise ValueError(
                f"joint '{name}' has no usable position limits in {source} "
                f'([{lower}, {upper}])')
        limits[name] = (lower, upper)
    return limits


class DescriptionPositionLimits:
    """Position limits of *joint_names*, read from the latched robot description.

    Shared by every leaf of one tree that checks against them: :meth:`setup`
    subscribes once per node however many leaves call it, and the limits are
    parsed once per description received.

    :meth:`resolve` answers ``(limits, None)``, or ``(None, reason)`` while no
    usable description has arrived -- which a caller that is about to send a
    motion must treat as a refusal, not as "no limits".
    """

    def __init__(self, joint_names, topic=DEFAULT_ROBOT_DESCRIPTION_TOPIC):
        self.joint_names = list(joint_names)
        if not self.joint_names:
            raise ValueError('DescriptionPositionLimits needs at least one joint name')
        self.topic = topic
        self.node = None
        self.subscription = None
        self._urdf = None
        self._limits = None
        self._error = None

    def setup(self, node):
        if self.subscription is not None and self.node is node:
            return
        self.node = node
        self.subscription = node.create_subscription(
            String, self.topic, self.on_description, LATCHED_QOS)

    def on_description(self, msg):
        self._urdf = msg.data
        self._limits = None
        self._error = None

    def resolve(self):
        if self._limits is not None:
            return self._limits, None
        if self._urdf is None:
            return None, (f'no {self.topic} received yet, so the joint position '
                          'limits this segment is checked against are unknown')
        if self._error is None:
            try:
                self._limits = position_limits(self._urdf, self.joint_names, self.topic)
            except Exception as exc:            # noqa: BLE001 - reported, not swallowed
                self._error = f'cannot read joint limits from {self.topic}: {exc}'
        return self._limits, self._error
