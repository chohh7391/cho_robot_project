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

"""Drive the arms' action-chunk controllers from a teleoperation device.

    ros2 run cho_control_tools teleop_bridge --robot franka
    ros2 run cho_control_tools teleop_bridge --robot openarm --arms left right
    ros2 run cho_control_tools teleop_bridge --robot art7 --arms left right --config teleop.yaml

Every arm comes from cho_robot_config: ``--arms`` names registry profiles, each
with its own action-chunk controller (a profile that drives several arms, like
OpenArm's ``both``, is refused: name its arms instead). A project repository
points ``CHO_ROBOT_CONFIG_DIR`` at its own registry, as its launch file does.
``--config`` takes a file of settings merged over the input's defaults
(config/teleop_bridge/<input>.yaml).

The run-toggle button starts (one ChunkStream goal per arm, each arm held at the
pose TF gives for its ee_link) and stops (goals canceled); the kill button
stops. While running, each arm moves only while its hand's clutch is engaged.
Operating it: docs/teleop_bridge.md.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ByteMultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from cho_control_tools.teleop_bridge.chunk_streamer import IDLE, ArmStream, resolve_arm
from cho_control_tools.teleop_bridge.clutch import Clutch
from cho_control_tools.teleop_bridge.config import load_config
from cho_control_tools.teleop_bridge.inputs import INPUTS, load_input
from cho_control_tools.teleop_bridge.state import START, STOP, RunState


class _Arm:
    def __init__(self, stream, clutch, hand):
        self.stream = stream
        self.clutch = clutch
        self.hand = hand


class TeleopBridge(Node):
    """The bridge for *targets* (ArmTargets), reading the input module *device*."""

    def __init__(self, targets, config, device):
        super().__init__('teleop_bridge')
        self._config = config
        self._device = device
        self._run = RunState(config.run_toggle, config.kill)
        self._latest = None
        self._latest_time = None
        self._lost_since = None
        self._arms = []
        for target in targets:
            settings = config.arm(target.profile)
            stream = ArmStream(self, target, config.horizon_steps, config.control_dt,
                               config.stream_timeout_sec, config.rate_hz, self._on_arm_end)
            self._arms.append(_Arm(stream, Clutch(settings.clutch), settings.hand))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # Best effort reads a reliable publisher too, and a stale sample is useless anyway.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ByteMultiArray, config.input_topic, self._on_input, qos)
        self._timer = None

    def prepare(self, timeout_sec=5.0):
        """Reach every arm's controllers; False (logged) if one is missing. Call before spinning."""
        for arm in self._arms:
            error = arm.stream.prepare(gripper=self._config.gripper_enabled, timeout_sec=timeout_sec)
            if error:
                self.get_logger().error(error)
                return False
            target = arm.stream.target
            gripper = target.gripper_controller if arm.stream.gripper_range else 'none'
            self.get_logger().info(
                f'{target.profile}: {target.controller} (chunks on {arm.stream.chunk_topic}), '
                f'gripper {gripper}, {target.ee_link} in {target.goal_frame}, {arm.hand} hand')
        self._timer = self.create_timer(self._config.control_dt, self._tick)
        self.get_logger().info(
            f'Waiting for {self._config.input_topic}. {self._config.run_toggle} starts and stops, '
            f'{self._config.kill} stops, squeeze moves an arm.')
        return True

    def _on_input(self, msg):
        try:
            frame = self._device.decode(msg.data, self._config.pose_source, self._config.button_threshold)
        except ValueError as exc:
            self.get_logger().warn(f'dropped an input message: {exc}', throttle_duration_sec=2.0)
            return
        self._latest = frame
        self._latest_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        frame = None
        if self._latest_time is not None:
            if now - self._latest_time <= Duration(seconds=self._config.input_timeout_sec):
                frame = self._latest
        if frame is None:
            self._lost_since = self._lost_since or now
        else:
            self._lost_since = None

        event = self._run.update(frame.buttons if frame is not None else None)
        if (self._run.running and self._lost_since is not None
                and now - self._lost_since > Duration(seconds=self._config.stop_after_input_lost_sec)):
            self.get_logger().error(
                f'no input on {self._config.input_topic} for {self._config.stop_after_input_lost_sec} s: stopping')
            event = self._run.stop()
        if event == START:
            self._start()
        elif event == STOP:
            self._stop('stopped')

        running = self._run.running
        for arm in self._arms:
            # A goal opens once the arm is seeded and its previous goal has ended
            # (a quick stop and start finds the cancel still in flight).
            if running and arm.stream.state == IDLE and arm.clutch.target is not None:
                arm.stream.open()
            if not arm.stream.active:
                continue
            sample = frame.hands.get(arm.hand) if frame is not None else None
            was_engaged = arm.clutch.engaged
            target = arm.clutch.update(sample, enabled=running and frame is not None)
            if arm.clutch.engaged != was_engaged:
                self.get_logger().info(
                    f"{arm.stream.target.profile}: clutch {'engaged' if arm.clutch.engaged else 'released'}")
            if target is not None:
                arm.stream.publish(*target)
            if running and sample is not None and sample.valid:
                arm.stream.publish_gripper(sample.trigger)

    def _start(self):
        for arm in self._arms:
            target = arm.stream.target
            try:
                transform = self._tf_buffer.lookup_transform(target.goal_frame, target.ee_link, Time())
            except TransformException as exc:
                self.get_logger().error(f'{target.profile}: no pose for {target.ee_link} in {target.goal_frame}: {exc}')
                self._run.stop()
                self._stop('not started')
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            arm.clutch.seed((translation.x, translation.y, translation.z),
                            (rotation.x, rotation.y, rotation.z, rotation.w))
        self.get_logger().info('running: squeeze to move an arm')

    def _stop(self, why):
        for arm in self._arms:
            arm.clutch.reset()
            arm.stream.close()
        self.get_logger().info(f'{why}: goals canceled, the arms hold')

    def _on_arm_end(self, stream, reason):
        self.get_logger().error(f'{stream.target.profile}: {reason}')
        if self._run.stop() == STOP:
            self._stop('stopped because an arm ended')


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('\n', 1)[1])
    parser.add_argument('--robot', required=True, help='robot type in cho_robot_config')
    parser.add_argument('--arms', nargs='+', default=['single'],
                        help='registry profiles to drive, one per arm (default: single)')
    parser.add_argument('--input', default=INPUTS[0], choices=INPUTS, help='the teleoperation device')
    parser.add_argument('--config', default=None, help='settings file merged over the input defaults')
    parser.add_argument('--input-topic', default=None, help="the device's topic (default: from the settings)")
    return parser


def _registry():
    try:
        from cho_robot_config import load_robot_config, task_goal_frame
    except ImportError as exc:
        raise ValueError('teleop_bridge reads every arm from cho_robot_config, which is not installed here') from exc
    return load_robot_config, task_goal_frame


def main(args=None):
    parsed, ros_args = build_parser().parse_known_args(args)
    try:
        if len(set(parsed.arms)) != len(parsed.arms):
            raise ValueError(f'--arms names a profile twice: {parsed.arms}')
        device = load_input(parsed.input)
        config = load_config(device, parsed.input, parsed.config)
        if parsed.input_topic:
            config = replace(config, input_topic=parsed.input_topic)
        load_robot_config, task_goal_frame = _registry()
        targets = [resolve_arm(parsed.robot, profile, load_robot_config, task_goal_frame) for profile in parsed.arms]
        for target in targets:
            config.arm(target.profile)
    except (ValueError, OSError) as exc:
        print(f'teleop_bridge: {exc}', file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    node = None
    try:
        node = TeleopBridge(targets, config, device)
        if not node.prepare():
            return 1
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
