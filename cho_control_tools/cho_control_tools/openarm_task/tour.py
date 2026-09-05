"""Drive an OpenArm through a list of far Cartesian waypoints, one absolute goal each.

    ros2 run cho_control_tools openarm_task_tour --arm right --dry-run
    ros2 run cho_control_tools openarm_task_tour --arm right --duration 6
    ros2 run cho_control_tools openarm_task_tour --arm right --waypoints my_tour.yaml --first 2 --last 4

Without --waypoints the installed sample (config/openarm_task_tour_right.yaml)
is used. Joint-defined waypoints are converted to poses with Pinocchio forward
kinematics on the live /robot_description, so they are reachable by
construction. Between goals the tool reads the controller's protocol status
and stops the tour if the MIT protocol is no longer ACTIVE. An aborted goal is
not a stop: the controller releases its reference and holds, and the next goal
starts from there. A summary table with the residual error per waypoint is
printed at the end. --dry-run prints the resolved poses and sends nothing.
"""

import argparse
import math
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .goal import TaskGoalClient, displacement_mm, format_pose
from .waypoints import load_waypoints, pose_error, profile_joint_limits, resolve_waypoints


class PinocchioForwardKinematics:
    """FK of the arm's TCP in the model-root frame, from the URDF the bringup published."""

    def __init__(self, urdf_xml, joint_names, ee_frame):
        import numpy as np
        import pinocchio as pin
        self._np, self._pin = np, pin
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        missing = [n for n in joint_names if not self.model.existJointName(n)]
        if missing:
            raise RuntimeError(f'robot_description has no joints {missing}')
        if not self.model.existFrame(ee_frame):
            raise RuntimeError(f'robot_description has no frame {ee_frame}')
        self.q_index = [self.model.joints[self.model.getJointId(n)].idx_q for n in joint_names]
        self.frame_id = self.model.getFrameId(ee_frame)

    def __call__(self, joints):
        q = self._pin.neutral(self.model)
        for index, value in zip(self.q_index, joints):
            q[index] = value
        self._pin.forwardKinematics(self.model, self.data, q)
        self._pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.frame_id]
        quaternion = self._pin.Quaternion(placement.rotation).coeffs()  # x y z w
        return list(placement.translation), list(quaternion)


def latched_robot_description(node, seconds=5.0):
    holder = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        String, '/robot_description', lambda m: holder.setdefault('xml', m.data), qos)
    end = time.time() + seconds
    while 'xml' not in holder and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    if 'xml' not in holder:
        raise RuntimeError('no latched /robot_description within 5 s; is the bringup running?')
    return holder['xml']


def default_waypoint_file(arm):
    share = get_package_share_directory('cho_control_tools')
    return os.path.join(share, 'config', f'openarm_task_tour_{arm}.yaml')


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--arm', choices=('single', 'left', 'right'), default='right')
    parser.add_argument('--waypoints', default=None, help='YAML waypoint file (default: installed sample)')
    parser.add_argument('--duration', type=float, default=6.0, help='trajectory time per waypoint (s)')
    parser.add_argument('--settle', type=float, default=2.5, help='pause after each result before sampling (s)')
    parser.add_argument('--first', type=int, default=0, help='index of the first waypoint to run')
    parser.add_argument('--last', type=int, default=None, help='index of the last waypoint to run')
    parser.add_argument('--dry-run', action='store_true', help='resolve and print the poses, send nothing')
    parser.add_argument('--yes', action='store_true', help='skip the confirmation prompt')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    path = args.waypoints or default_waypoint_file(args.arm)
    # Gate the file against the window this arm is actually driven under. The
    # torso's two arms do not share one, so a posture that is legal on the
    # right can be past a stop on the left.
    try:
        limits = profile_joint_limits(args.arm)
    except Exception as error:                      # noqa: BLE001 - reported, not swallowed
        print(f'could not read the per-arm joint window ({error}); '
              'falling back to the single-arm limits', file=sys.stderr)
        limits = None
    waypoints = load_waypoints(path, limits)
    last = len(waypoints) - 1 if args.last is None else args.last
    if not 0 <= args.first <= last < len(waypoints):
        print(f'--first/--last must select a range inside 0..{len(waypoints) - 1}', file=sys.stderr)
        return 2

    rclpy.init()
    node = TaskGoalClient(args.arm, node_name='openarm_task_tour')
    try:
        fk = None
        if any('joints' in w for w in waypoints):
            fk = PinocchioForwardKinematics(latched_robot_description(node),
                                            node.names['joints'], node.names['ee_frame'])
        resolved = resolve_waypoints(waypoints, fk)[args.first:last + 1]
        print(f'[{args.arm}] {len(resolved)} waypoints from {path}:')
        previous = None
        for index, (name, position, orientation) in enumerate(resolved, start=args.first):
            step = ''
            if previous is not None:
                step = f'  step {100 * math.dist(position, previous):.0f} cm'
            print(f'  [{index}] {name:20s} {format_pose((position, orientation))}{step}')
            previous = position
        if args.dry_run:
            return 0
        if not args.yes:
            answer = input('E-Stop ready and workspace clear? type "go" to start: ').strip().lower()
            if answer != 'go':
                print('not started')
                return 1
        if not node.healthy():
            print(f'[{args.arm}] controller is not ACTIVE: {node.protocol_status()}')
            return 1

        rows = []
        for index, (name, position, orientation) in enumerate(resolved, start=args.first):
            before = node.wait_pose()
            print(f'\n=== [{index}] {name}  target {format_pose((position, orientation))}')
            status, text = node.send(position, orientation, args.duration, relative=False)
            node.spin_for(args.settle)
            after = node.pose
            error_mm, error_deg = (None, None)
            if after is not None:
                translation, angle = pose_error(position, orientation, after[0], after[1])
                error_mm = [1000.0 * v for v in translation]
                error_deg = math.degrees(angle)
                moved = displacement_mm(before, after)[1] if before else float('nan')
                print(f'    result {text}; moved {moved:.0f} mm; residual '
                      f'({error_mm[0]:+.0f}, {error_mm[1]:+.0f}, {error_mm[2]:+.0f}) mm, {error_deg:.1f} deg')
            else:
                print(f'    result {text}; no TCP pose received')
            rows.append((index, name, text, error_mm, error_deg))
            if status is None or not node.healthy():
                print(f'[{args.arm}] stopping: {text}; protocol {node.protocol_status()}')
                break

        print('\nsummary (residual = target - measured after settle):')
        print('  idx  name                  result     dx    dy    dz [mm]   rot [deg]')
        for index, name, text, error_mm, error_deg in rows:
            if error_mm is None:
                print(f'  {index:3d}  {name:20s}  {text:9s}  n/a')
            else:
                print(f'  {index:3d}  {name:20s}  {text:9s}  {error_mm[0]:+5.0f} {error_mm[1]:+5.0f} '
                      f'{error_mm[2]:+5.0f}      {error_deg:5.1f}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
