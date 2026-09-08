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
"""

import math

import numpy as np
import py_trees
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from cho_task_manager.utils.controller_names import arm_model

DEFAULT_WRENCH_TOPIC = '/bota_ft_sensor/wrench'
DEFAULT_JOINT_STATES_TOPIC = '/joint_states'
DEFAULT_ROBOT_DESCRIPTION_TOPIC = '/robot_description'

# Monitor subscriptions are BEST_EFFORT on purpose, and it is not a
# reliability preference. A BEST_EFFORT subscription matches a RELIABLE
# publisher as well as a best-effort one, while a RELIABLE subscription does
# not match a best-effort publisher at all. The publishers here disagree --
# bota_driver_node and joint_state_broadcaster are reliable, ee_state_broadcaster
# is best effort -- and a monitor that silently receives nothing is worse than
# useless, so it asks for the weaker guarantee and matches everything. Depth 1:
# only the newest sample can trip anything.
_LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# robot_state_publisher latches the description, so a late subscriber needs
# TRANSIENT_LOCAL to receive it at all.
_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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

    def _now(self):
        # Plain float seconds: the guards only ever take differences, and a
        # Time object would drag rclpy arithmetic into every comparison.
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_wrench(self, msg):
        force, torque = msg.wrench.force, msg.wrench.torque
        self._wrench = (
            self._now(),
            (force.x, force.y, force.z),
            (torque.x, torque.y, torque.z),
        )

    def _on_joints(self, msg):
        self._joints = (self._now(), dict(zip(msg.name, msg.position)))

    def _on_description(self, msg):
        self._urdf = msg.data

    # -- kinematics -------------------------------------------------------

    def _build_kinematics(self):
        """Pinocchio model, arm joint indices and limits, from the URDF.

        Imported here rather than at module scope: only the kinematic guards
        need Pinocchio, and a tree that enables none of them should not pay for
        loading it.
        """
        import pinocchio as pin

        model = pin.buildModelFromXML(self._urdf)
        idx_q, idx_v = [], []
        for name in self.joint_names:
            if not model.existJointName(name):
                raise ValueError(
                    f"joint '{name}' is not in {self.robot_description_topic}; "
                    'the registry entry and the running description disagree')
            joint = model.joints[model.getJointId(name)]
            if joint.nq != 1:
                raise ValueError(
                    f"joint '{name}' has nq={joint.nq}; this monitor reads a "
                    'single bounded coordinate per joint, so an unbounded or '
                    'multi-DOF joint cannot be checked against a limit')
            idx_q.append(joint.idx_q)
            idx_v.append(joint.idx_v)
        if not model.existFrame(self.ee_link):
            raise ValueError(
                f"frame '{self.ee_link}' is not in {self.robot_description_topic}")
        return {
            'pin': pin,
            'model': model,
            'data': model.createData(),
            'idx_q': idx_q,
            'idx_v': idx_v,
            'frame_id': model.getFrameId(self.ee_link),
            'lower': [float(model.lowerPositionLimit[i]) for i in idx_q],
            'upper': [float(model.upperPositionLimit[i]) for i in idx_q],
        }

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
        now = self._now()
        self._arming_deadline = now + self.arming_timeout_sec
        # Back-dated so the first armed tick reports: a commissioning run wants
        # the baseline numbers immediately, not one period in.
        self._last_report = now - self.report_period_sec

    def _unarmed_reason(self, now):
        """What the monitor is still missing, or None once it can watch."""
        if self.needs_wrench:
            if self._wrench is None:
                return f'no {self.wrench_topic} message yet'
            if now - self._wrench[0] > self.max_age_sec:
                return (f'{self.wrench_topic} is '
                        f'{now - self._wrench[0]:.2f}s stale')
        if self.needs_kinematics:
            if self._joints is None:
                return f'no {self.joint_states_topic} message yet'
            if now - self._joints[0] > self.max_age_sec:
                return (f'{self.joint_states_topic} is '
                        f'{now - self._joints[0]:.2f}s stale')
            if self._urdf is None:
                return f'no {self.robot_description_topic} message yet'
            if self._kinematics is None:
                if self._model_error is not None:
                    return self._model_error
                try:
                    self._kinematics = self._build_kinematics()
                except Exception as exc:            # noqa: BLE001 - reported, not swallowed
                    self._model_error = f'cannot use the robot description: {exc}'
                    return self._model_error
        return None

    def update(self):
        now = self._now()

        unarmed = self._unarmed_reason(now)
        if unarmed is not None:
            if now > self._arming_deadline:
                # Failing, not warning: a mission asked to be watched, and
                # running it unwatched is not a lesser version of that. Staleness
                # lands here too, so a sensor that dies mid-mission trips the
                # monitor instead of freezing it at its last good sample.
                self.node.get_logger().error(
                    f'[{self.name}] cannot watch the arm: {unarmed}')
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

        if self.report_period_sec > 0.0 and now - self._last_report >= self.report_period_sec:
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
