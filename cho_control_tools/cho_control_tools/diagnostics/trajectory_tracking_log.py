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

    ros2 run cho_control_tools trajectory_tracking_log
    ros2 run cho_control_tools trajectory_tracking_log --output /tmp/run2.csv

Leave it running for the whole session; it prints a per-joint summary on Ctrl-C.
"""

import argparse
import csv
import sys

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node

DEFAULT_TOPIC = '/joint_trajectory_controller/controller_state'


class TrackingLog(Node):
    """Write (t, joint, reference, feedback, error, reference_velocity) rows."""

    def __init__(self, topic, output):
        super().__init__('trajectory_tracking_log')
        self.joints = []
        self.rows = 0
        self.worst = {}
        self._handle = open(output, 'w', newline='')
        self._writer = csv.writer(self._handle)
        self._writer.writerow(
            ['t_s', 'joint', 'reference', 'feedback', 'error', 'reference_velocity'])
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

        for index, joint in enumerate(msg.joint_names):
            if index >= len(error.positions):
                continue
            value = error.positions[index]
            velocity = (reference.velocities[index]
                        if index < len(reference.velocities) else float('nan'))
            self._writer.writerow([
                '%.4f' % now, joint,
                '%.6f' % reference.positions[index],
                '%.6f' % feedback.positions[index],
                '%.6f' % value,
                '%.6f' % velocity,
            ])
            if abs(value) > abs(self.worst.get(joint, 0.0)):
                self.worst[joint] = value
        self.rows += 1
        self._handle.flush()

    def report(self):
        """Print the per-joint worst error, which is what the tolerance trips on."""
        if not self.worst:
            print('no controller_state messages were received -- is the '
                  'trajectory controller active, and is the topic name right?',
                  file=sys.stderr)
            return
        print()
        print('%-6s %14s' % ('joint', 'worst error'))
        for joint in self.joints:
            value = self.worst.get(joint, 0.0)
            print('%-6s %+14.6f rad  (%+.3f deg)' % (joint, value, value * 57.2958))
        print()
        print('%d state messages written' % self.rows)

    def close(self):
        self._handle.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--output', default='trajectory_tracking.csv')
    args = parser.parse_args(argv)

    rclpy.init()
    node = TrackingLog(args.topic, args.output)
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
