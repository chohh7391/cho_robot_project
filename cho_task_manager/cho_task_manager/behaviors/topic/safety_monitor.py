# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Watch the arm while a mission runs, and fail the branch when it goes wrong.

Every guard here is *supervisory*, not a safety layer. It ticks with the tree
(100 ms in task_manager_node), so it catches a mission that is heading
somewhere wrong -- pressing into a fixture, walking into a joint stop, folding
through a singularity -- and preempts it. It cannot catch anything that
develops inside a control cycle. That remains the controllers' job:
clip_torque()/clip_position() and their allFinite() guards run at 1 kHz and are
what actually protects the hardware.

Wiring, in ``subtrees/safe_abort.guarded_mission(..., monitor=...)``::

    Parallel(SuccessOnSelected([mission]))
      |- SafetyMonitorBehavior   RUNNING while healthy, FAILURE when tripped
      +- mission

A monitor FAILURE fails the Parallel, which invalidates the mission branch.
That is the point of using Parallel rather than a decorator: py_trees calls
terminate(INVALID) on the running action leaf, and BaseActionBehavior already
cancels its goal there, so the motion stops instead of running to completion
while the tree walks away. The Selector around it then runs the safe abort.

The monitor never returns SUCCESS -- a watchdog has no success condition. The
Parallel's success is decided by the mission alone, which is why the policy
selects it explicitly.

Arming. The monitor waits, RUNNING, only for what it has never had: the first
sample of each input a guard needs, and the robot description. That wait is
bounded by ``arming_timeout_sec``, after which it fails -- a mission that asked
to be watched does not run unwatched. Everything else trips at once, inside the
arming window as well as after it: an input that has been seen and then goes
quiet for ``max_age_sec`` (a sensor dying in the first seconds of a mission is
no less dead), and a description the guards cannot use. The window is on the
node clock and starts with it (utils/clock.py), and a sample that arrives
before /clock is aged from the clock's first reading, not from 0.
"""

import math

import numpy as np
import py_trees
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from cho_task_manager.utils.clock import arm, deadline_after, restamp, seconds_since, stamp
from cho_task_manager.utils.controller_names import arm_model
from cho_task_manager.utils.robot_description import (
    DEFAULT_ROBOT_DESCRIPTION_TOPIC,
    LATCHED_QOS,
    arm_kinematics,
)

DEFAULT_WRENCH_TOPIC = '/bota_ft_sensor/wrench'
DEFAULT_JOINT_STATES_TOPIC = '/joint_states'

# Monitor subscriptions are BEST_EFFORT on purpose, and it is not a
# reliability preference. A BEST_EFFORT subscription matches a RELIABLE
# publisher as well as a best-effort one, while a RELIABLE subscription does
# not match a best-effort publisher at all. The publishers here disagree --
# bota_driver_node and joint_state_broadcaster are reliable, ee_state_broadcaster
# is best effort -- and a monitor that silently receives nothing is worse than
# useless, so it asks for the weaker guarantee and matches everything. Depth 1:
# only the newest sample can trip anything.
_LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# robot_state_publisher latches the description, RELIABLE + TRANSIENT_LOCAL: a
# late subscriber needs TRANSIENT_LOCAL to receive it at all, and RELIABLE to be
# sent the latched sample (utils/robot_description.py). NOT _LATEST's best effort.
_LATCHED = LATCHED_QOS


class SafetyMonitorBehavior(py_trees.behaviour.Behaviour):
    """RUNNING while every enabled guard is satisfied, FAILURE on the first trip.

    Each guard is off unless its threshold is given, because every threshold is
    a commissioning value: it depends on the robot, the payload and the task,
    and a number guessed here would either never fire or abort good missions.
    ``report_period_sec`` logs the measured values so a known-good run produces
    the numbers to set them from.

    Guards:

    ``max_force_n`` / ``max_torque_nm``
        Magnitude of the FT wrench. Magnitudes are rotation invariant, so
        nothing has to know where the sensor is mounted -- unlike a per-axis
        limit, which would need the R_tcp_ft transform from CLAUDE.md.
    ``joint_limit_margin_rad``
        Distance from any monitored joint to the nearer of its URDF limits.
    ``min_manipulability``
        Yoshikawa's index, sqrt(det(J Jᵀ)). NOT det(J): J is 6xn and for the
        7-DOF arms here it is not square, so det(J) does not exist. The two
        agree (up to sign) when n == 6, so this is the same number on a UR or
        an FR5 and the defined one on an FR3 or an OpenArm.
    ``min_singular_value``
        The smallest singular value of J. Same computation, and the easier of
        the two to reason about: it is the worst-case end-effector speed per
        unit joint speed, where the Yoshikawa index is a volume with mixed
        translational and rotational units.

    Both kinematic indices are computed from the LOCAL_WORLD_ALIGNED Jacobian.
    They are invariant to that choice against LOCAL (the two differ by a
    block-diagonal rotation) but NOT against WORLD, whose translation coupling
    changes the singular values.
    """

    def __init__(
        self,
        name: str,
        robot_config: dict,
        max_force_n: float = None,
        max_torque_nm: float = None,
        joint_limit_margin_rad: float = None,
        min_manipulability: float = None,
        min_singular_value: float = None,
        wrench_topic: str = DEFAULT_WRENCH_TOPIC,
        joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC,
        robot_description_topic: str = DEFAULT_ROBOT_DESCRIPTION_TOPIC,
        max_age_sec: float = 0.5,
        arming_timeout_sec: float = 10.0,
        report_period_sec: float = 0.0,
    ):
        super().__init__(name)
        self.max_force_n = max_force_n
        self.max_torque_nm = max_torque_nm
        self.joint_limit_margin_rad = joint_limit_margin_rad
        self.min_manipulability = min_manipulability
        self.min_singular_value = min_singular_value
        if not any((max_force_n, max_torque_nm, joint_limit_margin_rad,
                    min_manipulability, min_singular_value)):
            raise ValueError(
                f'[{name}] no guard is enabled; give at least one threshold, or '
                'leave the monitor out rather than watching nothing')

        model = arm_model(robot_config)
        self.joint_names = list(model['joints'])
        self.ee_link = model['ee_link']

        self.wrench_topic = wrench_topic
        self.joint_states_topic = joint_states_topic
        self.robot_description_topic = robot_description_topic
        self.max_age_sec = max_age_sec
        self.arming_timeout_sec = arming_timeout_sec
        self.report_period_sec = report_period_sec

        self.needs_wrench = max_force_n is not None or max_torque_nm is not None
        self.needs_kinematics = any((
            joint_limit_margin_rad is not None,
            min_manipulability is not None,
            min_singular_value is not None,
        ))
        self.needs_jacobian = (
            min_manipulability is not None or min_singular_value is not None)

        self.node = None
        # received_at is utils.clock.stamp(): a Time, or CLOCK_NOT_STARTED.
        self._wrench = None           # (received_at, [fx, fy, fz], [tx, ty, tz])
        self._joints = None           # (received_at, {name: position})
        self._urdf = None
        self._kinematics = None       # built once the description arrives
        self._model_error = None
        self._arming_deadline = None
        self._last_report = None

    # -- setup ------------------------------------------------------------

    def setup(self, **kwargs):
        self.node = kwargs['node']
        group = ReentrantCallbackGroup()
        if self.needs_wrench:
            self.node.create_subscription(
                WrenchStamped, self.wrench_topic, self._on_wrench, _LATEST,
                callback_group=group)
        if self.needs_kinematics:
            self.node.create_subscription(
                JointState, self.joint_states_topic, self._on_joints, _LATEST,
                callback_group=group)
            self.node.create_subscription(
                String, self.robot_description_topic, self._on_description,
                _LATCHED, callback_group=group)
        self.node.get_logger().info(f'[{self.name}] watching {self._guard_summary()}')
        return True

    def _guard_summary(self):
        parts = []
        if self.max_force_n is not None:
            parts.append(f'|F| <= {self.max_force_n} N')
        if self.max_torque_nm is not None:
            parts.append(f'|T| <= {self.max_torque_nm} Nm')
        if self.joint_limit_margin_rad is not None:
            parts.append(f'joint limit margin >= {self.joint_limit_margin_rad} rad')
        if self.min_manipulability is not None:
            parts.append(f'sqrt(det(J Jt)) >= {self.min_manipulability}')
        if self.min_singular_value is not None:
            parts.append(f'sigma_min(J) >= {self.min_singular_value}')
        return ', '.join(parts)

    # -- subscriptions ----------------------------------------------------

    def _received_at(self):
        return stamp(self.node.get_clock().now())

    def _on_wrench(self, msg):
        force, torque = msg.wrench.force, msg.wrench.torque
        self._wrench = (
            self._received_at(),
            (force.x, force.y, force.z),
            (torque.x, torque.y, torque.z),
        )

    def _on_joints(self, msg):
        self._joints = (self._received_at(), dict(zip(msg.name, msg.position)))

    def _on_description(self, msg):
        self._urdf = msg.data

    # -- kinematics -------------------------------------------------------

    def _build_kinematics(self):
        """Pinocchio model, arm joint indices and limits, from the URDF.

        The parsing is utils/robot_description.arm_kinematics(), shared with
        every other reader of the description so they cannot disagree about
        what a joint's limits are.
        """
        return arm_kinematics(
            self._urdf, self.joint_names, self.ee_link, source=self.robot_description_topic)

    def _jacobian(self, kin, q):
        pin = kin['pin']
        model, data = kin['model'], kin['data']
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        jacobian = pin.computeFrameJacobian(
            model, data, q, kin['frame_id'], pin.LOCAL_WORLD_ALIGNED)
        # Only this profile's joints: on a bimanual model the other arm's
        # columns are zero and would not change the result, but slicing keeps
        # the numbers about the arm the task is actually driving.
        return jacobian[:, kin['idx_v']]

    def _measure_kinematics(self):
        """(measurements, reason) -- reason is set when a guard trips."""
        kin = self._kinematics
        positions = self._joints[1]
        missing = [n for n in self.joint_names if n not in positions]
        if missing:
            return None, (
                f'{self.joint_states_topic} does not carry {missing}; it is not '
                'reporting the joints this profile is meant to watch')

        q = kin['pin'].neutral(kin['model'])
        for index, name in zip(kin['idx_q'], self.joint_names):
            q[index] = positions[name]

        measured = {}
        if self.joint_limit_margin_rad is not None:
            worst, worst_joint = math.inf, None
            for i, name in enumerate(self.joint_names):
                lower, upper = kin['lower'][i], kin['upper'][i]
                if not (math.isfinite(lower) and math.isfinite(upper)) or upper <= lower:
                    continue
                value = positions[name]
                margin = min(value - lower, upper - value)
                if margin < worst:
                    worst, worst_joint = margin, name
            if worst_joint is not None:
                measured['joint_margin'] = (worst, worst_joint)

        if self.needs_jacobian:
            jacobian = self._jacobian(kin, q)
            gram = jacobian @ jacobian.T
            measured['manipulability'] = float(
                math.sqrt(max(float(np.linalg.det(gram)), 0.0)))
            measured['sigma_min'] = float(
                np.linalg.svd(jacobian, compute_uv=False)[-1])

        margin = measured.get('joint_margin')
        if margin is not None and margin[0] < self.joint_limit_margin_rad:
            return measured, (
                f"joint '{margin[1]}' is {margin[0]:.4f} rad from its limit, "
                f'inside the {self.joint_limit_margin_rad} rad margin')
        if (self.min_manipulability is not None
                and measured['manipulability'] < self.min_manipulability):
            return measured, (
                f"manipulability sqrt(det(J Jt)) = {measured['manipulability']:.5f} "
                f'is below {self.min_manipulability}')
        if (self.min_singular_value is not None
                and measured['sigma_min'] < self.min_singular_value):
            return measured, (
                f"sigma_min(J) = {measured['sigma_min']:.5f} is below "
                f'{self.min_singular_value}')
        return measured, None

    def _measure_wrench(self):
        _, force, torque = self._wrench
        force_magnitude = math.sqrt(sum(component * component for component in force))
        torque_magnitude = math.sqrt(sum(component * component for component in torque))
        measured = {'force': force_magnitude, 'torque': torque_magnitude}
        if self.max_force_n is not None and force_magnitude > self.max_force_n:
            return measured, (
                f'|F| = {force_magnitude:.2f} N exceeds {self.max_force_n} N')
        if self.max_torque_nm is not None and torque_magnitude > self.max_torque_nm:
            return measured, (
                f'|T| = {torque_magnitude:.2f} Nm exceeds {self.max_torque_nm} Nm')
        return measured, None

    # -- lifecycle --------------------------------------------------------

    def initialise(self):
        # None while the node clock reads 0; update() takes it then.
        self._arming_deadline = deadline_after(self.node.get_clock(), self.arming_timeout_sec)
        # None so the first armed tick reports: a commissioning run wants the
        # baseline numbers immediately, not one period in.
        self._last_report = None

    def _age(self, attribute, now):
        """Seconds since the sample held in *attribute* arrived, re-stamping one from before /clock."""
        sample = getattr(self, attribute)
        received_at = restamp(sample[0], now)
        if received_at is not sample[0]:
            setattr(self, attribute, (received_at,) + sample[1:])
        return seconds_since(received_at, now)

    def _input_problem(self, now):
        """(waiting, broken): what the monitor has never had, or why it cannot watch.

        At most one is set. *waiting* is worth the arming window -- a first
        sample can still come. *broken* is not: an input that went quiet after
        it was seen, or a description the guards cannot use.
        """
        inputs = []
        if self.needs_wrench:
            inputs.append(('_wrench', self.wrench_topic))
        if self.needs_kinematics:
            inputs.append(('_joints', self.joint_states_topic))
        for attribute, topic in inputs:
            if getattr(self, attribute) is None:
                return f'no {topic} message yet', None
            age = self._age(attribute, now)
            if age > self.max_age_sec:
                return None, f'{topic} is {age:.2f}s stale'
        if self.needs_kinematics and self._kinematics is None:
            if self._urdf is None:
                return f'no {self.robot_description_topic} message yet', None
            if self._model_error is None:
                try:
                    self._kinematics = self._build_kinematics()
                except Exception as exc:            # noqa: BLE001 - reported, not swallowed
                    self._model_error = f'cannot use the robot description: {exc}'
            if self._model_error is not None:
                return None, self._model_error
        return None, None

    def _unarmed_reason(self, now):
        """Why the monitor cannot watch yet, or None once it can."""
        waiting, broken = self._input_problem(now)
        return waiting or broken

    def update(self):
        clock = self.node.get_clock()
        now = clock.now()
        self._arming_deadline = arm(self._arming_deadline, clock, self.arming_timeout_sec)

        waiting, broken = self._input_problem(now)
        if broken is not None:
            # At once, inside the arming window too: a sensor that dies in the
            # first seconds of a mission is as dead as one that dies later, and
            # a monitor frozen at its last good sample watches nothing.
            self.node.get_logger().error(f'[{self.name}] cannot watch the arm: {broken}')
            return py_trees.common.Status.FAILURE
        if waiting is not None:
            if self._arming_deadline is not None and now > self._arming_deadline:
                # Failing, not warning: a mission asked to be watched, and
                # running it unwatched is not a lesser version of that.
                self.node.get_logger().error(
                    f'[{self.name}] cannot watch the arm: {waiting} after '
                    f'{self.arming_timeout_sec:.1f}s')
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        checks = []
        if self.needs_wrench:
            checks.append(self._measure_wrench)
        if self.needs_kinematics:
            checks.append(self._measure_kinematics)

        measured = {}
        for measure in checks:
            values, reason = measure()
            if values:
                measured.update(values)
            if reason is not None:
                self.node.get_logger().error(f'[{self.name}] tripped: {reason}')
                return py_trees.common.Status.FAILURE

        if self.report_period_sec > 0.0 and (
                self._last_report is None
                or seconds_since(self._last_report, now) >= self.report_period_sec):
            self._last_report = now
            self.node.get_logger().info(f'[{self.name}] {self._format(measured)}')
        return py_trees.common.Status.RUNNING

    @staticmethod
    def _format(measured):
        parts = []
        if 'force' in measured:
            parts.append(f"|F| = {measured['force']:.2f} N")
            parts.append(f"|T| = {measured['torque']:.3f} Nm")
        if 'joint_margin' in measured:
            margin, joint = measured['joint_margin']
            parts.append(f'closest limit: {joint} at {margin:.4f} rad')
        if 'manipulability' in measured:
            parts.append(f"sqrt(det(J Jt)) = {measured['manipulability']:.5f}")
            parts.append(f"sigma_min(J) = {measured['sigma_min']:.5f}")
        return ', '.join(parts)

    def terminate(self, new_status):
        self._arming_deadline = None
        self._last_report = None
