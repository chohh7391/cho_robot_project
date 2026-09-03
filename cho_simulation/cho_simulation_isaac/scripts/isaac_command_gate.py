#!/usr/bin/env python3
# Copyright (c) 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tell the Isaac Sim runner that the controllers are up and commands are real.

Background: topic_based_ros2_control zero-initialises its command buffers and
starts publishing them as soon as the hardware component activates, which is
before any controller exists. If Isaac acted on those zeros the arm would fall
(torque mode) or snap to q=0 (position mode). run_isaac_sim.py therefore holds
the home pose with a stiff position drive and keeps its articulation controllers
disabled until this node appears.

bringup_isaac_robot.launch.py starts it from the controller spawner's
OnProcessExit, i.e. once the requested controller is active.

Why sensor_msgs/JointState for a boolean: the Isaac side cannot use rclpy (see
the module docstring of run_isaac_sim.py), so the gate is read through an
OmniGraph ROS2SubscribeJointState node, whose outputs are statically typed. The
runner polls its outputs:jointNames and opens the gate as soon as it is
non-empty; the contents do not matter.

Published repeatedly rather than latched: the OmniGraph subscriber uses its own
default QoS, so re-publishing is the dependable way to cover any start order.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacCommandGate(Node):

    def __init__(self):
        super().__init__('isaac_command_gate')
        self.declare_parameter('gate_topic', '/isaac/commands_enabled')
        self.declare_parameter('publish_rate', 5.0)

        topic = self.get_parameter('gate_topic').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self._publisher = self.create_publisher(JointState, topic, 1)
        self._msg = JointState()
        self._msg.name = ['gate']
        self._msg.position = [1.0]

        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._publish)
        self._publish()
        self.get_logger().info(f"Isaac command gate open, announcing on '{topic}' at {rate:g} Hz")

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
