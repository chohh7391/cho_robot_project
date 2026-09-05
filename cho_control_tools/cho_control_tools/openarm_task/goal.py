"""Send one Cartesian goal to an OpenArm MIT task-space controller and report what happened.

    ros2 run cho_control_tools openarm_task_goal --arm right --abs 0.402 -0.1535 0.478 --quat 0.7071 0 0.7071 0
    ros2 run cho_control_tools openarm_task_goal --arm right --rel 0.05 0 0 --duration 5

Absolute goals are in the controller's model-root frame (openarm_body_link0 on
the bimanual torso). Relative goals are TCP-local: the controller applies them
as reference * delta. The report prints the TCP pose before and after, the
action status, the displacement and the controller's task_diagnostics.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from cho_interfaces.action import TaskSpace

from .waypoints import arm_names, rpy_to_quaternion

STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class TaskGoalClient(Node):
    """Thin action client plus the passive reads a commissioning report needs."""

    def __init__(self, arm, node_name='openarm_task_goal'):
        super().__init__(node_name)
        self.names = arm_names(arm)
        self.pose = None
        self.create_subscription(PoseStamped, self.names['pose_topic'], self._on_pose, 10)
        self.action = ActionClient(self, TaskSpace, self.names['action'])
        self.diagnostics_client = self.create_client(Trigger, self.names['diagnostics'])
        self.status_client = self.create_client(Trigger, self.names['protocol_status'])

    def _on_pose(self, message):
        p, o = message.pose.position, message.pose.orientation
        self.pose = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_pose(self, seconds=3.0):
        end = time.time() + seconds
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pose

    def _trigger(self, client):
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result().message if future.result() else None

    def diagnostics(self):
        return self._trigger(self.diagnostics_client)

    def protocol_status(self):
        return self._trigger(self.status_client)

    def healthy(self):
        """True while the MIT protocol reports ACTIVE (status=1)."""
        message = self.protocol_status()
        return message is not None and 'status=1' in message

    def send(self, position, orientation, duration, relative):
        goal = TaskSpace.Goal()
        goal.duration = float(duration)
        goal.relative = bool(relative)
        (goal.target_pose.position.x, goal.target_pose.position.y,
         goal.target_pose.position.z) = [float(v) for v in position]
        (goal.target_pose.orientation.x, goal.target_pose.orientation.y,
         goal.target_pose.orientation.z, goal.target_pose.orientation.w) = [float(v) for v in orientation]
        if not self.action.wait_for_server(timeout_sec=5.0):
            return None, 'action server unavailable'
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None, 'goal rejected (controller not ready, or invalid goal)'
        result_future = handle.get_result_async()
        started = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            # The controller aborts an unreached goal duration + 2 s after start.
            if time.time() - started > duration + 6.0:
                return None, 'result timeout'
        result = result_future.result()
        return result.status, STATUS_NAMES.get(result.status, str(result.status))


def format_pose(pose):
    if pose is None:
        return 'unavailable'
    position, orientation = pose
    return ('pos=(' + ', '.join(f'{v:+.4f}' for v in position) + ') quat=(' +
            ', '.join(f'{v:+.4f}' for v in orientation) + ')')


def displacement_mm(before, after):
    d = [1000.0 * (a - b) for a, b in zip(after[0], before[0])]
    return d, math.sqrt(sum(v * v for v in d))


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--arm', choices=('single', 'left', 'right'), default='right')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--abs', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                      help='absolute TCP position in the model-root frame (m)')
    mode.add_argument('--rel', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                      help='TCP-local displacement (m)')
    parser.add_argument('--quat', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='target orientation; identity (no rotation) when omitted')
    parser.add_argument('--rpy', nargs=3, type=float, metavar=('R', 'P', 'Y'),
                        help='target orientation as roll/pitch/yaw in radians')
    parser.add_argument('--duration', type=float, default=5.0, help='trajectory time (s), at least 0.25')
    parser.add_argument('--settle', type=float, default=2.5,
                        help='seconds to wait after the result before sampling the pose')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.quat is not None and args.rpy is not None:
        print('give --quat or --rpy, not both', file=sys.stderr)
        return 2
    orientation = (0.0, 0.0, 0.0, 1.0)
    if args.quat is not None:
        orientation = tuple(args.quat)
    elif args.rpy is not None:
        orientation = rpy_to_quaternion(*args.rpy)
    position = args.abs if args.abs is not None else args.rel
    relative = args.rel is not None

    rclpy.init()
    node = TaskGoalClient(args.arm)
    try:
        before = node.wait_pose()
        print(f"[{args.arm}] TCP before : {format_pose(before)}")
        print(f"[{args.arm}] diagnostics: {node.diagnostics()}")
        print(f"[{args.arm}] sending {'RELATIVE' if relative else 'ABSOLUTE'} goal position={list(position)} "
              f"orientation={[round(v, 4) for v in orientation]} duration={args.duration}s")
        started = time.time()
        status, text = node.send(position, orientation, args.duration, relative)
        print(f"[{args.arm}] result after {time.time() - started:.1f}s: {text}")
        node.spin_for(args.settle)
        after = node.pose
        print(f"[{args.arm}] TCP after  : {format_pose(after)}")
        if before and after:
            d, magnitude = displacement_mm(before, after)
            print(f"[{args.arm}] displacement: ({d[0]:+.1f}, {d[1]:+.1f}, {d[2]:+.1f}) mm, |d|={magnitude:.1f} mm")
        print(f"[{args.arm}] diagnostics: {node.diagnostics()}")
        print(f"[{args.arm}] protocol   : {node.protocol_status()}")
        return 0 if status == 4 else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
