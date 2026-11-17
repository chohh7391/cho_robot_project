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

"""Publish a scripted ``controller_data`` stream: teleop_bridge without a headset.

    ros2 run cho_control_tools teleop_bridge_fake_xr
    ros2 run cho_control_tools teleop_bridge_fake_xr --hands left --radius 0.03

The messages have the shape Isaac Capture's teleop_ros2 publishes
(inputs/xr_controller.py). The script, in seconds from the start:

- ``start``: the run-toggle button (right A) is pressed for 0.2 s;
- ``start + 1``: each listed hand squeezes and draws a circle of ``--radius``
  in the XR x-y plane (right and up) every ``--period``, its trigger going
  0 -> 1 -> 0 once per circle, for ``--cycles`` circles; with
  ``--yaw-deg`` it also turns about XR up, back and forth;
- then it releases the squeeze, presses kill (right B) and goes quiet.

With the default mapping XR x and y are -y and z of the arm's goal frame, so
each arm draws its circle in that frame's y-z plane.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import msgpack
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import ByteMultiArray

from cho_control_tools.teleop_bridge.inputs.xr_controller import BUTTONS, SIDES


# Where each hand rests in the XR world frame [m], at chest height, half a metre ahead.
HAND_REST = {'left': (-0.2, 1.0, -0.5), 'right': (0.2, 1.0, -0.5)}
TOGGLE = 'right_primary_click'
KILL = 'right_secondary_click'


def payload(hands, buttons, now_ns):
    """One controller_data map; *hands* maps side -> (position, quaternion, squeeze, trigger)."""
    data = {'timestamp': now_ns}
    for side in SIDES:
        position, quaternion, squeeze, trigger = hands.get(side, (HAND_REST[side], (0.0, 0.0, 0.0, 1.0), 0.0, 0.0))
        data[f'{side}_thumbstick'] = [0.0, 0.0]
        data[f'{side}_trigger_value'] = float(trigger)
        data[f'{side}_squeeze_value'] = float(squeeze)
        for kind in ('grip', 'aim'):
            data[f'{side}_{kind}_position'] = [float(value) for value in position]
            data[f'{side}_{kind}_orientation'] = [float(value) for value in quaternion]
        for button in BUTTONS:
            data[f'{side}_{button}'] = 1.0 if f'{side}_{button}' in buttons else 0.0
        data[f'{side}_is_active'] = True
    return data


def to_message(data) -> ByteMultiArray:
    """As teleop_ros2 sends it: msgpack bytes, one per array element."""
    msg = ByteMultiArray()
    msg.data = [bytes([value]) for value in msgpack.packb(data)]
    return msg


class FakeXr(Node):
    def __init__(self, args):
        super().__init__('teleop_bridge_fake_xr')
        self._args = args
        self._publisher = self.create_publisher(ByteMultiArray, args.topic, 10)
        self._t0 = time.monotonic()
        self._done = False
        self.create_timer(1.0 / args.rate, self._tick)

    @property
    def done(self):
        return self._done

    def _tick(self):
        args = self._args
        t = time.monotonic() - self._t0
        motion_start = args.start + 1.0
        motion_end = motion_start + args.cycles * args.period
        buttons = set()
        if args.start <= t < args.start + 0.2:
            buttons.add(TOGGLE)
        hands = {}
        if motion_start <= t < motion_end:
            phase = 2.0 * math.pi * (t - motion_start) / args.period
            for side in args.hands:
                rest = HAND_REST[side]
                position = (rest[0] + args.radius * (math.cos(phase) - 1.0),
                            rest[1] + args.radius * math.sin(phase),
                            rest[2])
                yaw = math.radians(args.yaw_deg) * math.sin(phase)
                quaternion = Rotation.from_euler('y', yaw).as_quat()
                trigger = 0.5 * (1.0 - math.cos(phase))
                hands[side] = (position, quaternion, 1.0, trigger)
        if motion_end + 0.5 <= t < motion_end + 0.7:
            buttons.add(KILL)
        if t >= motion_end + 1.5:
            self._done = True
            return
        self._publisher.publish(to_message(payload(hands, buttons, time.time_ns())))


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('\n', 1)[1])
    parser.add_argument('--topic', default='/xr_teleop/controller_data')
    parser.add_argument('--rate', type=float, default=90.0, help='messages per second')
    parser.add_argument('--hands', nargs='+', default=list(SIDES), choices=SIDES)
    parser.add_argument('--start', type=float, default=2.0, help='seconds before the run toggle')
    parser.add_argument('--radius', type=float, default=0.05, help='[m]')
    parser.add_argument('--period', type=float, default=4.0, help='seconds per circle')
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--yaw-deg', type=float, default=0.0, help='turn about XR up, amplitude [deg]')
    return parser


def main(args=None):
    parsed, ros_args = build_parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FakeXr(parsed)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
