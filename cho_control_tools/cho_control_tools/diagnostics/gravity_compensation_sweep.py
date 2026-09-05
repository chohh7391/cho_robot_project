"""Measure how well the model gravity term matches the motor torque, posture by posture.

Hand-guide the arm through the postures you care about while this runs. For
every sample it compares the reported motor torque against the Pinocchio
non-linear term for the measured configuration, so the residual it prints is
what gravity compensation is failing to cover.

    ros2 run cho_control_tools gravity_compensation_sweep --seconds 60

Read it with the MIT command contract in mind. Under the task-space controller
in Cartesian mode the motor computes

    tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff

with kp zero, so a stationary joint reports approximately `tau_ff`, and the
controller sets `tau_ff` to the model term. A residual therefore means the
model disagrees with the arm - unmodelled tooling, a wrong link mass, a
mis-signed axis - and not that compensation is switched off. Samples taken
while the arm is actually moving carry the `kd` term and joint friction as
well, so the summary reports the near-stationary ones separately; those are
the trustworthy figures.
"""

import argparse
import math

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARM_JOINTS = [f'openarm_joint{i}' for i in range(1, 8)]
# Above this the kd term and joint friction pollute the comparison.
STATIONARY_RAD_PER_S = 0.05


class GravityCompensationSweep(Node):

    def __init__(self, seconds, ee_frame):
        super().__init__('gravity_compensation_sweep')
        self.declare_parameter('robot_description', '')
        description = self.get_parameter('robot_description').value
        if not description:
            description = self._description_from_robot_state_publisher()
        self.model = pin.buildModelFromXML(description)
        self.data = self.model.createData()
        missing = [n for n in ARM_JOINTS if not self.model.existJointName(n)]
        if missing:
            raise RuntimeError(f'robot_description is missing {missing}')
        self.q_index = [self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_JOINTS]
        self.v_index = [self.model.joints[self.model.getJointId(n)].idx_v for n in ARM_JOINTS]
        self.ee_frame_id = (
            self.model.getFrameId(ee_frame) if self.model.existFrame(ee_frame) else None
        )
        self.samples = []
        self.seconds = seconds
        self.deadline = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.get_logger().info(
            f'Recording for {seconds:.0f}s. Hand-guide the arm through the postures you '
            'care about, pausing a moment at each one.'
        )

    def _description_from_robot_state_publisher(self):
        # robot_state_publisher latches /robot_description, so a transient-local
        # subscription picks it up without a service round trip.
        holder = {}

        def capture(msg):
            holder['xml'] = msg.data

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = self.create_subscription(String, '/robot_description', capture, qos)
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
        while 'xml' not in holder and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(subscription)
        if 'xml' not in holder:
            raise RuntimeError(
                'No latched /robot_description within 5s; is the bringup running?')
        return holder['xml']

    def _on_joint_state(self, msg):
        now = self.get_clock().now()
        if self.deadline is None:
            self.deadline = now + rclpy.duration.Duration(seconds=self.seconds)
        index = {name: i for i, name in enumerate(msg.name)}
        if any(n not in index for n in ARM_JOINTS):
            return
        if len(msg.effort) < len(msg.name) or len(msg.velocity) < len(msg.name):
            return
        # The broadcaster does not guarantee joint order, so resolve by name.
        q7 = np.array([msg.position[index[n]] for n in ARM_JOINTS])
        dq7 = np.array([msg.velocity[index[n]] for n in ARM_JOINTS])
        tau7 = np.array([msg.effort[index[n]] for n in ARM_JOINTS])
        if not (np.isfinite(q7).all() and np.isfinite(tau7).all()):
            return
        q = np.zeros(self.model.nq)
        v = np.zeros(self.model.nv)
        for i, value in zip(self.q_index, q7):
            q[i] = value
        nle = pin.nonLinearEffects(self.model, self.data, q, v)[self.v_index]
        reach = math.nan
        if self.ee_frame_id is not None:
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            reach = float(np.linalg.norm(self.data.oMf[self.ee_frame_id].translation))
        self.samples.append((q7, dq7, tau7, np.array(nle), reach))
        if now >= self.deadline:
            raise KeyboardInterrupt

    def report(self):
        if not self.samples:
            print('No /joint_states samples with effort were received.')
            return
        still = [s for s in self.samples if np.abs(s[1]).max() < STATIONARY_RAD_PER_S]
        print(f'\nsamples: {len(self.samples)} total, {len(still)} near-stationary '
              f'(|dq| < {STATIONARY_RAD_PER_S} rad/s)')
        if not still:
            print('None were stationary enough to compare; pause at each posture next time.')
            return
        model = np.array([s[3] for s in still])
        measured = np.array([s[2] for s in still])
        residual = measured - model
        span = np.abs(model).max(axis=0)
        print('\nper joint, over the near-stationary samples:')
        print('  joint   max|model|   max|residual|   mean residual   worst-case ratio')
        for j in range(7):
            worst = np.abs(residual[:, j]).max()
            ratio = worst / span[j] if span[j] > 1e-6 else math.nan
            print(f'    {j + 1}     {span[j]:8.3f}      {worst:8.3f}       '
                  f'{residual[:, j].mean():+8.3f}        {ratio:8.2f}')
        peak = np.abs(model).max()
        print(f'\nlargest gravity torque visited: {peak:.3f} Nm')
        if peak < 1.0:
            print('That is small: the arm stayed near the gravity-neutral vertical, so this '
                  'sweep does not yet say whether the model holds an extended arm. Guide it '
                  'out sideways and run again.')
        worst_joint = int(np.argmax(np.abs(residual).max(axis=0))) + 1
        print(f'largest residual on joint {worst_joint}: '
              f'{np.abs(residual).max():.3f} Nm')
        print('\nA residual that grows with the gravity torque points at link mass or an '
              'unmodelled payload. One that stays flat regardless of posture is friction '
              'and offset, which gravity compensation is not meant to cover.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--ee-frame', default='openarm_hand_tcp')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = GravityCompensationSweep(args.seconds, args.ee_frame)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.report()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
