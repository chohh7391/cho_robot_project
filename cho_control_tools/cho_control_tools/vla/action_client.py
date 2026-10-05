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

"""Drive a VLA controller with a synthetic action-chunk stream (a small circle).

    ros2 run cho_control_tools vla_action_client                       # Franka's vla_controller
    ros2 run cho_control_tools vla_action_client --robot openarm       # the registry's VLA role
    ros2 run cho_control_tools vla_action_client --robot openarm --arm left
    ros2 run cho_control_tools vla_action_client --controller my_vla_controller

The action is the controller's own ``/<controller>/vla`` (cho_interfaces/CONTRACT.md).
Chunks go to the topic the controller reads them from: its ``chunk_topic``
parameter, asked of it at start-up, unless ``--chunk-topic`` names one. A
bimanual OpenArm reads ``/vla/action/<side>``, not the single-arm default.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cho_interfaces.action import VisionLanguageAction
from cho_control_tools.action_names import controller_action_name
from cho_interfaces.msg import ActionChunk
from std_msgs.msg import Header
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# Franka's VLA controller (cho_robot_config franka.yaml controllers.vla), the
# controller this client has always driven.
DEFAULT_CONTROLLER = 'vla_controller'
# What both VLA controllers read when their chunk_topic is left unset.
DEFAULT_CHUNK_TOPIC = '/vla/action/ee_pose'


def _registry_loader():
    """cho_robot_config's load_robot_config, imported only when --robot asks for it.

    Not a declared dependency (see package.xml): like the generic debug client,
    this tool uses the registry when it is installed and works without it when
    the controller is named.
    """
    try:
        from cho_robot_config import load_robot_config
    except ImportError as exc:
        raise ValueError(
            '--robot reads the VLA controller from cho_robot_config, which is not '
            'installed here; name it with --controller instead') from exc
    return load_robot_config


def resolve_controller(controller=None, robot=None, arm='single', load_robot_config=None):
    """The VLA controller to drive.

    *controller* as given; else the registry's ``controllers.vla`` role for
    *robot* and its profile *arm*; else Franka's ``vla_controller``.
    """
    if controller:
        return controller
    if robot is None:
        if arm not in (None, '', 'single'):
            raise ValueError('--arm selects a registry profile and needs --robot')
        return DEFAULT_CONTROLLER
    loader = load_robot_config or _registry_loader()
    name = (loader(robot, arm or 'single').get('controllers') or {}).get('vla')
    if not name:
        raise ValueError(
            f"{robot} ({arm or 'single'}) has no VLA controller in cho_robot_config "
            f'(controllers.vla); name one with --controller')
    return name


def controller_chunk_topic(node, controller, timeout_sec=2.0):
    """The controller's ``chunk_topic`` parameter, or None if it does not say."""
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters, f'/{controller}/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(GetParameters.Request(names=['chunk_topic']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        if response is None or not response.values:
            return None
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING or not value.string_value:
            return None
        return value.string_value
    finally:
        node.destroy_client(client)


class VLAActionTester(Node):
    def __init__(self, controller=DEFAULT_CONTROLLER, chunk_topic=None):
        super().__init__('vla_action_tester')

        # "axis_angle", "euler", "quaternion", "rotation6d"
        self.test_rotation_type = "quaternion"
        self.is_relative = True

        self.controller = controller
        self._action_client = ActionClient(
            self, VisionLanguageAction, controller_action_name(controller, 'vla'))

        self.count = 0
        self.chunk_size = 16
        self.inference_dt = 1/15
        self.dt = self.inference_dt / self.chunk_size
        self.goal_accepted = False
        self.timer = None

        self.get_logger().info(f'Waiting for {controller_action_name(controller, "vla")} ...')
        self._action_client.wait_for_server()
        # Asked once the controller is up, so its parameters can answer.
        if not chunk_topic:
            chunk_topic = controller_chunk_topic(self, controller)
            if chunk_topic is None:
                chunk_topic = DEFAULT_CHUNK_TOPIC
                self.get_logger().warn(
                    f'{controller} did not report its chunk_topic; publishing on '
                    f'{chunk_topic}. Give --chunk-topic if that is not what it reads.')
        self.publisher_ = self.create_publisher(ActionChunk, chunk_topic, 10)

        self.get_logger().info(
            f'VLA Tester: controller={controller}, chunks on {chunk_topic}, '
            f'Mode={self.test_rotation_type}, Relative={self.is_relative}')
        self.send_goal()

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = VisionLanguageAction.Goal()
        goal_msg.model_name = f"tester_{self.test_rotation_type}"
        goal_msg.inference_frequency = 15.0

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.goal_accepted = True
        self.timer = self.create_timer(self.inference_dt, self.publish_action_chunk)

    def publish_action_chunk(self):
        if not self.goal_accepted: return

        msg = ActionChunk()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        msg.action_space = "task"
        msg.rotation_type = self.test_rotation_type
        msg.relative = self.is_relative
        msg.chunk_size = self.chunk_size
        msg.control_dt = self.dt  # explicit waypoint spacing (0.0 would fall back to inference_dt/chunk_size)

        arm_actions = []
        gripper_actions = []

        # circle trajectory parameters
        radius = 0.05
        c_x, c_y, c_z = (0.0, 0.0, 0.0) if self.is_relative else (0.5, 0.0, 0.4)

        # Relative chunks follow the usual VLA convention: each chunk's offsets are
        # relative to the robot state at that chunk's observation (the controller
        # re-anchors per chunk), NOT cumulative from goal start. So emit each
        # waypoint as (trajectory at t_i) - (trajectory at this chunk's start).
        t_chunk_start = self.count * self.chunk_size * self.dt

        for i in range(self.chunk_size):
            t = t_chunk_start + i * self.dt

            if self.is_relative:
                x = radius * (math.cos(t_chunk_start) - math.cos(t))
                y = radius * (math.sin(t) - math.sin(t_chunk_start))
                z = 0.0
            else:
                x = c_x + radius * math.cos(t)
                y = c_y + radius * math.sin(t)
                z = c_z
            arm_actions.extend([x, y, z])

            # --- Rotation Type별 처리 ---
            if self.test_rotation_type == "axis_angle":
                # 바닥을 보는 기본 자세 (Pi, 0, 0)
                arm_actions.extend([3.14159, 0.0, 0.0] if not self.is_relative else [0.0, 0.0, 0.0])

            elif self.test_rotation_type == "euler":
                # (Roll, Pitch, Yaw)
                arm_actions.extend([3.14159, 0.0, 0.0] if not self.is_relative else [0.0, 0.0, 0.0])

            elif self.test_rotation_type == "quaternion":
                # (x, y, z, w)
                rot = R.from_euler('x', 180, degrees=True) if not self.is_relative else R.from_euler('x', 0)
                arm_actions.extend(rot.as_quat().tolist())

            elif self.test_rotation_type == "rotation6d":
                # Scipy를 이용해 회전 행렬을 만든 뒤 v1, v2 추출
                rot_mat = R.from_euler('x', 180, degrees=True).as_matrix() if not self.is_relative else np.eye(3)
                v1 = rot_mat[:, 0] # 첫 번째 컬럼
                v2 = rot_mat[:, 1] # 두 번째 컬럼
                arm_actions.extend(v1.tolist() + v2.tolist())

            gripper_actions.append(-1.0) # Open

        msg.arm_actions = arm_actions
        msg.gripper_actions = gripper_actions

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {self.test_rotation_type} chunk {self.count}')
        self.count += 1

def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('\n', 1)[1])
    target = parser.add_mutually_exclusive_group()
    target.add_argument('--controller', default=None,
                        help=f'VLA controller to drive (default: {DEFAULT_CONTROLLER}, Franka\'s)')
    target.add_argument('--robot', default=None,
                        help="take the controller from cho_robot_config's VLA role for this robot")
    parser.add_argument('--arm', default='single',
                        help='registry profile with --robot, e.g. left/right for OpenArm (default: single)')
    parser.add_argument('--chunk-topic', default=None,
                        help="ActionChunk topic (default: the controller's chunk_topic parameter)")
    return parser


def main(args=None):
    parsed, ros_args = build_parser().parse_known_args(args)
    try:
        controller = resolve_controller(parsed.controller, parsed.robot, parsed.arm)
    except ValueError as exc:
        print(f'vla_action_client: {exc}', file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    tester = None
    try:
        tester = VLAActionTester(controller, parsed.chunk_topic)
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        if tester is not None:
            tester.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
