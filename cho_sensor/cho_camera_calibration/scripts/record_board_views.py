#!/usr/bin/env python3
"""Drive a list of arm poses and record what a hand-eye solve needs.

At each pose, once the arm has stopped, it writes down the robot's own forward
kinematics and the CORNER PIXELS of every board tag each camera can see.

Corners, not tag poses. A planar tag's error is almost all along the viewing
ray -- measured at 100% on this bench -- so its depth is the worst thing it
reports, and the camera tilts a hand-eye solve needs make that worse. Fitting a
board to six such poses gave residuals of 16, 31 and 42 mm on the tilted views.
Twenty-four image points against one rigid planar target has no per-tag depth
in it at all, and solve_hand_eye.py reprojects them at about half a pixel.

Nothing is solved here. Moving a real arm is the expensive part, so it happens
once and the data goes to disk; the solve is separate and can be rerun.

    ros2 run cho_camera_calibration record_board_views.py POSES.yaml OUT.json

POSES.yaml is {poses: [{name, joints: [j1..j6]}, ...]}. THE CALLER OWNS ARM
SAFETY: every pose and the joint-space line between consecutive ones has to be
checked against the bench, the tooling and anything standing in the cell before
this is run. What the solve wants of them is in the package README -- large
relative rotations, about varied axes, with the board in frame throughout.
"""
import json
import sys

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import yaml

from cho_interfaces.action import JointSpace

MOVE_SEC = 7.0
SETTLE_SEC = 2.5
SAMPLE_SEC = 2.0


class Recorder(Node):
    def __init__(self):
        super().__init__('record_corners')
        group = ReentrantCallbackGroup()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(
            self, JointSpace,
            '/controller_action_server/joint_space_position_controller',
            callback_group=group)
        self.latest = {'wrist': None, 'oak': None}
        for key, topic in (('wrist', '/wrist/detections'), ('oak', '/oak/detections')):
            self.create_subscription(
                AprilTagDetectionArray, topic,
                lambda m, k=key: self.latest.__setitem__(k, m),
                qos_profile_sensor_data, callback_group=group)

    def move(self, joints, seconds):
        goal = JointSpace.Goal()
        goal.duration = float(seconds)
        goal.target_joints = JointState(position=[float(v) for v in joints])
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, executor=EXEC)
        handle = future.result()
        if not handle.accepted:
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, executor=EXEC)
        return result.result().status == 4

    def wait(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            EXEC.spin_once(timeout_sec=0.05)

    def sample(self, seconds):
        """Take the median corner per tag and the arm pose, over a still window."""
        arm = []
        corners = {'wrist': {}, 'oak': {}}
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            EXEC.spin_once(timeout_sec=0.05)
            try:
                tf = self.buffer.lookup_transform(
                    'base_link', 'wrist3_link', rclpy.time.Time())
                t, r = tf.transform.translation, tf.transform.rotation
                arm.append([t.x, t.y, t.z, r.x, r.y, r.z, r.w])
            except Exception:
                pass
            for key in ('wrist', 'oak'):
                msg = self.latest[key]
                if msg is None:
                    continue
                for det in msg.detections:
                    corners[key].setdefault(str(det.id), []).append(
                        [[c.x, c.y] for c in det.corners])
        out = {}
        for key in ('wrist', 'oak'):
            out[key] = {tag: np.median(np.array(rows), axis=0).tolist()
                        for tag, rows in corners[key].items() if len(rows) > 3}
        return arm, out


rclpy.init()
node = Recorder()
EXEC = MultiThreadedExecutor()
EXEC.add_node(node)
if not node.client.wait_for_server(timeout_sec=15.0):
    raise SystemExit('joint space action server did not appear')

USAGE = ('usage: record_board_views.py POSES.yaml OUT.json  '
         '(see the package README)')

if len(sys.argv) != 3:
    raise SystemExit(USAGE)
poses_path, out_path = sys.argv[1], sys.argv[2]
poses = yaml.safe_load(open(poses_path, encoding='utf-8'))['poses']
records = []
for index, pose in enumerate(poses):
    print(f'[{index + 1}/{len(poses)}] {pose["name"]} -> moving', flush=True)
    if not node.move(pose['joints'], MOVE_SEC):
        print('   move failed, stopping', flush=True)
        break
    node.wait(SETTLE_SEC)
    arm, corners = node.sample(SAMPLE_SEC)
    print(f'   wrist {len(corners["wrist"])} tags, oak {len(corners["oak"])} tags, '
          f'{len(arm)} arm samples', flush=True)
    records.append({
        'name': pose['name'], 'joints': pose['joints'],
        'arm': np.median(np.array(arm), axis=0).tolist() if arm else None,
        'wrist_corners': corners['wrist'], 'oak_corners': corners['oak'],
    })

with open(out_path, 'w', encoding='utf-8') as handle:
    json.dump(records, handle, indent=1)
print(f'\nwrote {out_path} with {len(records)} poses')
node.destroy_node()
rclpy.shutdown()
