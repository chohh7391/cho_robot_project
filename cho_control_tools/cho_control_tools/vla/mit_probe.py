#!/usr/bin/env python3
"""End-to-end probe for a VLA controller against a running simulator.

Streams ActionChunks the way an inference bridge would and asserts what the arm
and the telemetry actually did, which is the half a controller_manager fixture
cannot cover. It found two real defects on its first run against MuJoCo:

  * `resume_on_stream_recovery` had to default ON. With resume off, kHold only
    leaves via hold_timeout -> abort, so one 200 ms gap in a 15 Hz BEST_EFFORT
    stream ended the rollout even though chunks returned immediately.
  * the past-prefix drop used `t <= now`, which discarded waypoint 0 of every
    chunk on the arrival-time path and made `waypoints_dropped_past` report one
    per chunk by construction instead of flagging real inference latency.

Phases: joint-space follow, malformed-chunk rejection, gap recovery, task-space
follow, stream watchdog hold/abort, and reuse after an abort.

Usage (with a sim already running, e.g.
  ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
    mujoco_mit_prototype:=true control_mode:=torque \
    mit_controller_name:=vla_mit_controller):

  ros2 run cho_control_tools vla_mit_probe
  ros2 run cho_control_tools vla_mit_probe --controller vla_controller --joint-index 0

Exit code is 0 only when every check passes.
"""
import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cho_interfaces.action import VisionLanguageAction
from cho_interfaces.msg import ActionChunk, VlaTelemetry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

DEFAULT_CONTROLLER = 'vla_mit_controller'
DEFAULT_CHUNKS = '/vla/action/ee_pose'

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    def __init__(self, controller=DEFAULT_CONTROLLER, chunk_topic=DEFAULT_CHUNKS,
                 joint_index=0):
        super().__init__('vla_mit_probe')
        # Both derived from the controller name, exactly as the controller itself
        # derives its action name from get_node()->get_name().
        action = f'/controller_action_server/{controller}'
        telemetry = f'/{controller}/vla_telemetry'
        self.joint_index = joint_index
        self.client = ActionClient(self, VisionLanguageAction, action)
        self.pub = self.create_publisher(ActionChunk, chunk_topic, BEST_EFFORT)
        self.create_subscription(VlaTelemetry, telemetry, self._telem, 10)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(PoseStamped, '/ee_state/pose', self._pose, 10)
        self.telem = None
        self.joints = None
        self.pose = None

    def _telem(self, m): self.telem = m
    def _joints(self, m): self.joints = m
    def _pose(self, m): self.pose = m

    def q(self, i=None):
        if i is None:
            i = self.joint_index
        return self.joints.position[i] if self.joints else float('nan')

    def tcp(self):
        if not self.pose:
            return (float('nan'),) * 3
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def settle(self, timeout=3.0):
        """Wait until the telemetry counters stop moving, then return a snapshot.

        Telemetry publishes at 10 Hz while chunks arrive at 15 Hz, so a counter
        read straight after a publish loop is a stale sample -- comparing against
        it made the NaN check race against chunks still in flight.
        """
        last = None
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            self.wait(0.25)
            if self.telem is None:
                continue
            now = (self.telem.chunks_accepted, self.telem.chunks_rejected)
            if now == last:
                return self.telem
            last = now
        return self.telem

    def send_goal(self, stream_timeout=0.0):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise SystemExit('FAIL: action server not available')
        goal = VisionLanguageAction.Goal()
        goal.model_name = 'probe'
        goal.task = 'move the arm'
        goal.inference_frequency = 15.0
        goal.stream_timeout = stream_timeout
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        return handle

    def joint_chunk(self, targets, control_dt=0.05):
        m = ActionChunk()
        m.action_space = 'joint'
        m.relative_mode = 'absolute'
        m.chunk_size = len(targets)
        m.control_dt = control_dt
        arm = []
        for value in targets:
            row = [0.0] * 7
            row[self.joint_index] = value
            arm.extend(row)
        m.arm_actions = arm
        return m

    def task_chunk(self, poses, control_dt=0.05):
        m = ActionChunk()
        m.action_space = 'task'
        m.relative_mode = 'absolute'
        m.rotation_type = 'quaternion'
        m.chunk_size = len(poses)
        m.control_dt = control_dt
        arm = []
        for (x, y, z, qx, qy, qz, qw) in poses:
            arm.extend([x, y, z, qx, qy, qz, qw])
        m.arm_actions = arm
        return m


def report(tag, ok, detail):
    print(f'{"PASS" if ok else "FAIL"}  {tag}: {detail}', flush=True)
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--controller', default=DEFAULT_CONTROLLER,
                        help='controller name; the action and telemetry names '
                             'are derived from it')
    parser.add_argument('--chunk-topic', default=DEFAULT_CHUNKS)
    parser.add_argument('--joint-index', type=int, default=0,
                        help='joint the joint-space phase drives')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = Probe(args.controller, args.chunk_topic, args.joint_index)
    # The controller's return-to-zero ramp gates its action server, so waiting
    # for the server is also waiting for the arm to reach its start posture.
    if not node.client.wait_for_server(timeout_sec=60.0):
        raise SystemExit('FAIL: action server never became available')
    node.wait(2.0)
    results = []

    print(f'start: q1={node.q():+.4f}  tcp={tuple(round(v,4) for v in node.tcp())}', flush=True)

    # ---------------- phase 1: joint space ----------------
    handle = node.send_goal()
    results.append(report('joint goal accepted', handle is not None, str(handle is not None)))
    if handle is None:
        raise SystemExit(1)

    q_start = node.q()
    target = 0.35
    # Stream at 15 Hz for 4 s. Each chunk spans 0.4 s (8 x 0.05).
    deadline = time.time() + 4.0
    while time.time() < deadline and rclpy.ok():
        node.pub.publish(node.joint_chunk([target] * 8))
        node.wait(1.0 / 15.0)
    q_end = node.q()
    accepted = node.telem.chunks_accepted if node.telem else 0
    rejected = node.telem.chunks_rejected if node.telem else -1
    state = node.telem.stream_state if node.telem else '?'
    results.append(report('joint chunks accepted', accepted > 30,
                          f'accepted={accepted} rejected={rejected} state={state}'))
    results.append(report('joint 1 followed the reference',
                          q_end - q_start > 0.15,
                          f'q1 {q_start:+.4f} -> {q_end:+.4f} (target {target})'))
    results.append(report('space reported as joint',
                          node.telem.action_space == 'joint', node.telem.action_space))
    results.append(report('horizon is non-empty while streaming',
                          node.telem.remaining_horizon_sec > 0.0,
                          f'{node.telem.remaining_horizon_sec:.3f} s'))

    # ---------------- phase 2: NaN rejection ----------------
    settled = node.settle()
    before_rej = settled.chunks_rejected
    before_acc = settled.chunks_accepted
    bad = node.joint_chunk([target] * 8)
    bad.arm_actions[3] = float('nan')
    for _ in range(5):
        node.pub.publish(bad)
        node.wait(0.1)
    node.settle()
    results.append(report('NaN chunk rejected',
                          node.telem.chunks_rejected > before_rej
                          and node.telem.chunks_accepted == before_acc,
                          f'rejected {before_rej} -> {node.telem.chunks_rejected}, '
                          f'accepted stayed {node.telem.chunks_accepted}'))
    results.append(report('last_reject names the cause',
                          node.telem.last_reject == 'non_finite', node.telem.last_reject))

    # unknown rotation_type + empty payload: the historical UB path
    ub = ActionChunk()
    ub.action_space = 'task'
    ub.rotation_type = 'made_up'
    ub.chunk_size = 1
    ub.control_dt = 0.05
    before_rej = node.settle().chunks_rejected
    for _ in range(3):
        node.pub.publish(ub)
        node.wait(0.1)
    node.settle()
    results.append(report('unknown rotation_type refused',
                          node.telem.chunks_rejected > before_rej,
                          f'rejected -> {node.telem.chunks_rejected}, '
                          f'reason={node.telem.last_reject}'))
    results.append(report('controller survived the malformed chunks',
                          node.q() == node.q(),  # not NaN
                          f'q1={node.q():+.4f}'))

    # ---------------- phase 2b: recovery from the malformed-chunk gap ----------------
    # Only ACCEPTED chunks refresh the watchdog, so the malformed burst above put
    # it into hold. A returning valid stream must bring it back to running;
    # without that, one transient gap would cost the whole goal.
    for _ in range(10):
        node.pub.publish(node.joint_chunk([target] * 8))
        node.wait(1.0 / 15.0)
    results.append(report('stream recovers to running after a gap',
                          node.telem.stream_state == 'running',
                          node.telem.stream_state))

    # ---------------- phase 3: task space ----------------
    node.wait(0.2)
    x0, y0, z0 = node.tcp()
    o = node.pose.pose.orientation
    quat = (o.x, o.y, o.z, o.w)
    deadline = time.time() + 4.0
    while time.time() < deadline and rclpy.ok():
        # Absolute Cartesian goal 4 cm along +x from where we started.
        poses = []
        for k in range(8):
            poses.append((x0 + 0.04, y0, z0) + quat)
        node.pub.publish(node.task_chunk(poses))
        node.wait(1.0 / 15.0)
    x1, y1, z1 = node.tcp()
    results.append(report('space switched to task',
                          node.telem.action_space == 'task', node.telem.action_space))
    results.append(report('TCP followed the Cartesian reference',
                          abs(x1 - x0) > 0.005,
                          f'x {x0:+.4f} -> {x1:+.4f} (target {x0+0.04:+.4f}), '
                          f'|dy|={abs(y1-y0):.4f} |dz|={abs(z1-z0):.4f}'))

    # ---------------- phase 4: stream watchdog ----------------
    print('--- stopping the stream; expect hold then abort ---', flush=True)
    result_future = handle.get_result_async()
    # Confirm we are running before the silence, so 'hold' below is caused by it.
    results.append(report('running before the deliberate silence',
                          node.telem.stream_state == 'running',
                          node.telem.stream_state))
    seen_hold = False
    end = time.time() + 12.0
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.telem and node.telem.stream_state == 'hold':
            seen_hold = True
        if result_future.done():
            break
    results.append(report('watchdog entered hold', seen_hold,
                          f'last state={node.telem.stream_state}'))
    if result_future.done():
        res = result_future.result()
        results.append(report('goal aborted on a dead stream',
                              res.status == 6 and not res.result.is_completed,
                              f'status={res.status} completed={res.result.is_completed}'))
        results.append(report('abort message explains why',
                              'stream' in res.result.message,
                              repr(res.result.message)))
    else:
        results.append(report('goal aborted on a dead stream', False, 'no result within 12 s'))

    # ---------------- phase 5: still usable afterwards ----------------
    node.wait(1.0)
    again = node.send_goal()
    results.append(report('a new goal is accepted after the abort',
                          again is not None, str(again is not None)))
    if again is not None:
        for _ in range(15):
            node.pub.publish(node.joint_chunk([0.1] * 8))
            node.wait(1.0 / 15.0)
        results.append(report('and it drives again',
                              node.telem.stream_state == 'running',
                              node.telem.stream_state))
        again.cancel_goal_async()
        node.wait(1.0)

    print(f'\nfinal: q1={node.q():+.4f} tcp={tuple(round(v,4) for v in node.tcp())} '
          f'accepted={node.telem.chunks_accepted} rejected={node.telem.chunks_rejected} '
          f'dropped_past={node.telem.waypoints_dropped_past}', flush=True)
    passed = sum(1 for r in results if r)
    print(f'\n==== {passed}/{len(results)} checks passed ====', flush=True)
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
