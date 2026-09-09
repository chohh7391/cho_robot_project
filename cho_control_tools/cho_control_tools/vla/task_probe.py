#!/usr/bin/env python3
"""Task-space-only VLA probe, sized for a first run on real hardware.

TASK ACTION SPACE ONLY, on purpose. VlaMitController's Cartesian path is
TaskSpaceImpedanceMitController's own write_cartesian_torque_target() unchanged
-- the controller derives from it and replaces only where x_des/v_des come from
-- so what runs here is the commissioned law fed by a chunk stream. Its JOINT
action space is new code that has only run in MuJoCo and additionally omits the
joint-limit spring the Cartesian path carries, so this tool never emits a joint
chunk.

Every target is built from the pose the arm is ALREADY at, read from
~/ee_state/pose. The real bringup defaults return_to_zero to false, so the
controller starts wherever the arm was left; an absolute goal written by hand
would be commanded immediately, and while the reference limiter would crawl
there at max_task_lin_vel rather than lunge, it would still be a move nobody
asked for.

Motion is a slow Lissajous inside a small box -- large enough to see, small
enough that the whole path is a short reach from the start pose.

START FROM A NORMAL WORKING POSTURE, not a fully extended one. The Cartesian
error is resolved through a damped J^+, so near full extension a small Cartesian
error becomes a large joint offset. The tool holds the current pose for a few
seconds first and reports how far the arm moved while doing so; a large settle
means the starting posture is wrong, not that tracking is bad.

  ros2 run cho_control_tools vla_task_probe --radius 0.02 --seconds 20
  ros2 run cho_control_tools vla_task_probe --dry-run     # stream nothing, just watch

Stop at any time with Ctrl-C: the stream stops, the controller holds after
stream_timeout_sec and aborts the goal after hold_timeout_sec, without ever
requesting SAFE. To stop the arm deliberately, call the controller's
~/request_safe_stop service.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cho_interfaces.action import VisionLanguageAction
from cho_interfaces.msg import ActionChunk, VlaTelemetry
from geometry_msgs.msg import PoseStamped

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


class TaskProbe(Node):
    def __init__(self, controller, chunk_topic, pose_topic):
        super().__init__('vla_task_probe')
        self.client = ActionClient(
            self, VisionLanguageAction, f'/controller_action_server/{controller}')
        self.pub = self.create_publisher(ActionChunk, chunk_topic, BEST_EFFORT)
        self.create_subscription(
            VlaTelemetry, f'/{controller}/vla_telemetry', self._telem, 10)
        self.create_subscription(PoseStamped, pose_topic, self._pose, 10)
        self.telem = None
        self.pose = None

    def _telem(self, m): self.telem = m
    def _pose(self, m): self.pose = m

    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def tcp(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def quat(self):
        o = self.pose.pose.orientation
        return (o.x, o.y, o.z, o.w)

    def publish_task(self, poses, control_dt):
        m = ActionChunk()
        m.action_space = 'task'
        m.relative_mode = 'absolute'
        m.rotation_type = 'quaternion'
        m.chunk_size = len(poses)
        m.control_dt = control_dt
        arm = []
        for p in poses:
            arm.extend(p)
        m.arm_actions = arm
        self.pub.publish(m)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--controller', default='vla_mit_controller')
    parser.add_argument('--chunk-topic', default='/vla/action/ee_pose')
    parser.add_argument('--pose-topic', default='/ee_state/pose')
    parser.add_argument('--radius', type=float, default=0.02,
                        help='box half-size [m]; keep this small on hardware')
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--rate', type=float, default=15.0)
    parser.add_argument('--chunk', type=int, default=8)
    parser.add_argument('--settle', type=float, default=3.0,
                        help='seconds to hold the current pose before moving')
    parser.add_argument('--dry-run', action='store_true',
                        help='send no goal and no chunks; print telemetry only')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = TaskProbe(args.controller, args.chunk_topic, args.pose_topic)

    print('waiting for the controller...', flush=True)
    if not node.client.wait_for_server(timeout_sec=120.0):
        raise SystemExit('FAIL: action server never appeared. Is the controller active, '
                         'and has its startup ramp settled?')
    node.wait(2.0)
    if node.pose is None:
        raise SystemExit(f'FAIL: nothing on {args.pose_topic}; is the EE broadcaster up?')

    # A second publisher on the pose topic makes every reading here a coin flip
    # between two sources, and the resulting "tracking error" is invented. Seen
    # for real: a umi_control_real node publishing its own pose onto
    # /ee_state/pose alongside the broadcaster produced a 0.38 m spike that
    # looked exactly like a singularity excursion. Refuse rather than report it.
    publishers = node.count_publishers(args.pose_topic)
    if publishers > 1:
        raise SystemExit(
            f'FAIL: {publishers} publishers on {args.pose_topic}. Every pose read '
            f'here would be a coin flip between them and the tracking numbers would '
            f'be meaningless. Stop the other publisher, or point --pose-topic at a '
            f'topic only the EE broadcaster owns.')

    print(f'TCP on arrival = {tuple(round(v, 4) for v in node.tcp())}', flush=True)

    if args.dry_run:
        print('dry run: watching telemetry for 10 s, sending nothing', flush=True)
        for _ in range(10):
            node.wait(1.0)
            t = node.telem
            if t is None:
                print('  (no telemetry yet)', flush=True)
            else:
                print(f'  state={t.stream_state} accepted={t.chunks_accepted} '
                      f'rejected={t.chunks_rejected} horizon={t.remaining_horizon_sec:.2f}s',
                      flush=True)
        rclpy.shutdown()
        return 0

    goal = VisionLanguageAction.Goal()
    goal.model_name = 'task_probe'
    goal.task = 'trace a small box'
    goal.inference_frequency = args.rate
    future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise SystemExit('FAIL: goal rejected (startup ramp not settled, or one is active)')
    print('goal accepted', flush=True)

    period = 1.0 / args.rate
    control_dt = period / 2.0
    r = args.radius

    # Settle first: stream the pose the arm is already at, then re-read it and use
    # THAT as the path origin.
    #
    # Two reasons, and the second is the one that bites. The pose read the instant
    # the server appears is not necessarily a pose the arm is holding -- it may
    # still be finishing a startup ramp -- so anchoring the path there commands a
    # move to a stale pose. And if the arm is near full extension the Cartesian
    # error is resolved through a badly conditioned J, so the first cycles produce
    # large joint offsets; measured in MuJoCo from the nominal-zero posture
    # (arm straight up), that was a 0.38 m excursion before it settled. Holding
    # first makes that transient happen at zero commanded displacement, and makes
    # it visible in the settle report below rather than inside the path.
    print(f'settling for {args.settle:.1f}s before moving...', flush=True)
    before = node.tcp()
    end = time.time() + args.settle
    while time.time() < end and rclpy.ok():
        hold = node.tcp()
        node.publish_task([hold + node.quat()] * args.chunk, control_dt)
        node.wait(period)
    x0, y0, z0 = node.tcp()
    quat = node.quat()
    drift = math.dist(before, (x0, y0, z0))
    print(f'  settled at ({x0:+.4f}, {y0:+.4f}, {z0:+.4f}); '
          f'moved {drift*1000:.1f} mm while settling', flush=True)
    if drift > 0.05:
        print('  WARNING: that is a large settle. The arm was probably not holding '
              'the pose it reported, or it is near full extension where the '
              'Cartesian error resolves through a badly conditioned Jacobian. '
              'Move it to a normal working posture before trusting these numbers.',
              flush=True)
    print('streaming the path', flush=True)

    def offset(t):
        # Lissajous, ramped in over the first two seconds so the path leaves the
        # start pose smoothly rather than stepping to an offset at t = 0.
        ramp = min(1.0, t / 2.0)
        return (ramp * r * math.sin(2.0 * math.pi * 0.10 * t),
                ramp * r * math.sin(2.0 * math.pi * 0.13 * t + 1.1),
                ramp * r * 0.5 * math.sin(2.0 * math.pi * 0.07 * t + 2.2))

    t0 = time.time()
    worst = 0.0
    next_report = 4.0
    try:
        while rclpy.ok():
            t = time.time() - t0
            if t > args.seconds:
                break
            poses = []
            for k in range(args.chunk):
                dx, dy, dz = offset(t + k * control_dt)
                poses.append((x0 + dx, y0 + dy, z0 + dz) + quat)
            node.publish_task(poses, control_dt)
            node.wait(period)

            dx, dy, dz = offset(t)
            err = math.dist((x0 + dx, y0 + dy, z0 + dz), node.tcp())
            worst = max(worst, err)
            if t >= next_report:
                next_report += 4.0
                cx, cy, cz = node.tcp()
                tm = node.telem
                print(f'  t={t:5.1f}s tcp=({cx:+.3f},{cy:+.3f},{cz:+.3f}) '
                      f'err={err*1000:5.1f}mm state={tm.stream_state} '
                      f'accepted={tm.chunks_accepted} rejected={tm.chunks_rejected}',
                      flush=True)
    except KeyboardInterrupt:
        print('\ninterrupted: stopping the stream. The controller holds after '
              'stream_timeout_sec and aborts after hold_timeout_sec.', flush=True)

    print(f'\nworst tracking error {worst*1000:.1f} mm', flush=True)
    print('returning to the start pose', flush=True)
    end = time.time() + 3.0
    while time.time() < end and rclpy.ok():
        node.publish_task([(x0, y0, z0) + quat] * args.chunk, control_dt)
        node.wait(period)
    handle.cancel_goal_async()
    node.wait(2.0)
    tm = node.telem
    print(f'final: state={tm.stream_state} accepted={tm.chunks_accepted} '
          f'rejected={tm.chunks_rejected}', flush=True)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
