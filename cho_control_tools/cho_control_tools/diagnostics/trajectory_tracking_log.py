#!/usr/bin/env python3
"""Record what a trajectory controller was asked for against what it got.

A goal that aborts on state tolerance says only that the error crossed a line.
It does not say what shape the error had, and the shape is the whole diagnosis:

* error proportional to commanded VELOCITY  -> following lag. The tolerance is
  tighter than this hardware and interface can hold, and the honest fix is to
  measure the lag and set the tolerance from it.
* error roughly CONSTANT while moving       -> a bias between the command path
  and the measurement path, which slowing down will not help.
* error growing with ELAPSED TIME regardless of speed -> the arm is executing
  less than it is told, and the deficit accumulates.

`joint_trajectory_controller` already publishes all of it on
`~/controller_state` (reference, feedback, error) at its state_publish_rate, so
nothing has to be instrumented -- it only has to be written down.

The cho FR5 controllers publish the same message on their own
`~/controller_state`, once per 125 Hz update cycle, but fill only `reference`
and `feedback` -- `error` is left empty. Subtracting is therefore not a
convenience here, it is the only way to get an error out of them at all.

    ros2 run cho_control_tools trajectory_tracking_log
    ros2 run cho_control_tools trajectory_tracking_log --output /tmp/run2.csv
    ros2 run cho_control_tools trajectory_tracking_log \
        --topic /task_space_ik_controller/controller_state

Leave it running for the whole session; it prints a per-joint summary on Ctrl-C.
That summary also measures JUDDER, because "was it shaking?" is not a question a
worst-error figure can answer: a joint can sit at 2 mrad of error and get there
by shaking.

Judder is measured on the residual -- measured velocity minus d(reference)/dt --
and never on the raw velocity, which gives the wrong answer rather than a
rougher one. A trapezoidal leg ramps 0 -> 0.09 -> 0 rad/s by design, so a
detector watching the raw signal reports a 0.09 rad/s excursion and calls a
clean move violent. Measured on the residual, that same leg on the FR5 came to
0.0026 rad/s RMS, 2-4% of its peak speed, while 12 Hz of injected shake scores
0.057.
"""

import argparse
import csv
import sys

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node

DEFAULT_TOPIC = '/joint_trajectory_controller/controller_state'


class TrackingLog(Node):
    """Write (t, joint, reference, feedback, error, velocity) rows."""

    def __init__(self, topic, output, swing_threshold=0.01):
        super().__init__('trajectory_tracking_log')
        self.joints = []
        self.rows = 0
        self.worst = {}
        self.peak_speed = {}
        self.reversals = {}
        self.worst_swing = {}
        self.swing_threshold = swing_threshold
        self._previous_reference = {}
        self._residual_sum = {}
        self._residual_count = {}
        self._last_velocity = {}
        self.span = 0.0
        self.derived_error = False
        self._handle = open(output, 'w', newline='')
        self._writer = csv.writer(self._handle)
        self._writer.writerow(
            ['t_s', 'joint', 'reference', 'feedback', 'error', 'velocity'])
        self._t0 = None
        self.create_subscription(
            JointTrajectoryControllerState, topic, self._on_state, 50)
        self.get_logger().info('logging %s -> %s' % (topic, output))

    def _on_state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = stamp
            self.joints = list(msg.joint_names)
        now = stamp - self._t0

        # `desired`/`actual`/`error` on Humble; the newer names are aliases.
        reference = getattr(msg, 'reference', None) or msg.desired
        feedback = getattr(msg, 'feedback', None) or msg.actual
        error = msg.error
        self.span = now

        for index, joint in enumerate(msg.joint_names):
            if index >= len(reference.positions) or index >= len(feedback.positions):
                continue
            if index < len(error.positions):
                value = error.positions[index]
            else:
                # The publisher left `error` empty (every cho FR5 controller
                # does). Subtract rather than skip the row.
                value = reference.positions[index] - feedback.positions[index]
                self.derived_error = True
            # Whichever velocity the publisher actually carries. The FR5
            # controllers send no reference velocity but do send the MEASURED
            # one, which is the useful one for judder anyway.
            if index < len(feedback.velocities):
                velocity = feedback.velocities[index]
            elif index < len(reference.velocities):
                velocity = reference.velocities[index]
            else:
                velocity = float('nan')

            self._writer.writerow([
                '%.4f' % now, joint,
                '%.6f' % reference.positions[index],
                '%.6f' % feedback.positions[index],
                '%.6f' % value,
                '%.6f' % velocity,
            ])
            if abs(value) > abs(self.worst.get(joint, 0.0)):
                self.worst[joint] = value
            if velocity == velocity:                     # not NaN
                if abs(velocity) > self.peak_speed.get(joint, 0.0):
                    self.peak_speed[joint] = abs(velocity)
                self._track_judder(joint, now, reference.positions[index], velocity)
        self.rows += 1
        self._handle.flush()

    def _track_judder(self, joint, now, reference_position, velocity):
        """Measure the velocity the COMMAND does not account for.

        Judder has to be read off the residual, measured velocity minus
        commanded velocity, and this is not a refinement -- on the raw velocity
        the numbers are simply wrong. A trapezoidal leg ramps 0 -> 0.09 -> 0
        rad/s all by itself, so a detector watching the raw signal reports an
        excursion of 0.09 rad/s and calls the move violent, when measured-minus-
        commanded over that same leg is 0.008 rad/s RMS and the joint is in fact
        tracking its profile cleanly.

        No controller here publishes a reference velocity, so it is
        differentiated from the reference POSITION, which every one of them does
        publish.
        """
        previous = self._previous_reference.get(joint)
        self._previous_reference[joint] = (now, reference_position)
        if previous is None or now <= previous[0]:
            return
        commanded = (reference_position - previous[1]) / (now - previous[0])
        residual = velocity - commanded
        self._residual_sum[joint] = self._residual_sum.get(joint, 0.0) + residual ** 2
        self._residual_count[joint] = self._residual_count.get(joint, 0) + 1
        if abs(residual) > self.worst_swing.get(joint, 0.0):
            self.worst_swing[joint] = abs(residual)
        self._count_reversal(joint, residual)

    def _count_reversal(self, joint, velocity):
        """Count turning points whose swing exceeds the threshold.

        NOT sign changes, which was the first thing tried and does not work:
        an oscillation riding on a steady move never crosses zero. 0.1 rad/s
        with a 0.08 rad/s wobble on top stays positive throughout and scores one
        reversal, the same as a smooth move -- while the turning-point count
        separates them 1 from 96.

        The swing threshold is what keeps encoder noise out: only a reversal
        whose excursion exceeds it is counted, so idle chatter is ignored while
        a real oscillation is not.
        """
        state = self._last_velocity.get(joint)
        if state is None:
            self._last_velocity[joint] = {'anchor': velocity, 'extreme': velocity,
                                          'direction': 0}
            return
        if state['direction'] == 0:
            if abs(velocity - state['anchor']) > self.swing_threshold:
                state['direction'] = 1 if velocity > state['anchor'] else -1
                state['extreme'] = velocity
            return
        moving_on = (velocity - state['extreme']) * state['direction'] > 0.0
        if moving_on:
            state['extreme'] = velocity
        elif abs(velocity - state['extreme']) > self.swing_threshold:
            self.reversals[joint] = self.reversals.get(joint, 0) + 1
            swing = abs(state['extreme'] - state['anchor'])
            if swing > self.worst_swing.get(joint, 0.0):
                self.worst_swing[joint] = swing
            state['anchor'] = state['extreme']
            state['extreme'] = velocity
            state['direction'] = -state['direction']

    def report(self):
        """Print the per-joint worst error, which is what the tolerance trips on."""
        if not self.worst:
            print('no controller_state messages were received -- is the '
                  'trajectory controller active, and is the topic name right?',
                  file=sys.stderr)
            return
        print()
        if self.derived_error:
            print('error is reference - feedback: the publisher left `error` empty')
        print('judder = measured velocity - d(reference)/dt; turning points in it')
        print('counted above a %.3f rad/s swing' % self.swing_threshold)
        print('%-8s %16s %12s %12s %12s %11s %11s'
              % ('joint', 'worst error', 'peak |vel|', 'judder RMS', 'judder max',
                 'turnpoints', 'implied Hz'))
        for joint in self.joints:
            value = self.worst.get(joint, 0.0)
            flips = self.reversals.get(joint, 0)
            count = self._residual_count.get(joint, 0)
            rms = (self._residual_sum.get(joint, 0.0) / count) ** 0.5 if count else 0.0
            # Two turning points make one cycle, so this is the frequency a
            # steady oscillation would need to produce that count. Read it only
            # once the count is large and the RMS is a real fraction of the peak.
            hertz = (flips / (2.0 * self.span)) if self.span > 0.0 else 0.0
            print('%-8s %+11.6f rad %12.4f %12.4f %12.4f %11d %11.2f'
                  % (joint, value, self.peak_speed.get(joint, 0.0), rms,
                     self.worst_swing.get(joint, 0.0), flips, hertz))
        print()
        print('%d state messages over %.2f s' % (self.rows, self.span))

    def close(self):
        self._handle.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--output', default='trajectory_tracking.csv')
    parser.add_argument('--swing-threshold', type=float, default=0.01,
                        help='rad/s excursion a velocity reversal must exceed to '
                             'be counted as judder rather than sensor noise')
    args = parser.parse_args(argv)

    rclpy.init()
    node = TrackingLog(args.topic, args.output, args.swing_threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
