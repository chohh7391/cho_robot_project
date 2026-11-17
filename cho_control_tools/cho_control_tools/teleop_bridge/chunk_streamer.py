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

"""One arm's side of the bridge: its controllers from the registry, its goal and its chunks.

The arm's ``controllers.action_chunk`` serves ``ChunkStream`` (cho_interfaces/CONTRACT.md).
While running, the bridge holds one goal on it and publishes an absolute
task-space chunk every cycle on the controller's ``chunk_topic``: the clutch's
target, in the profile's absolute goal frame, repeated ``horizon_steps`` times
``control_dt`` apart. Repeating it makes each chunk outlast the next one's
arrival, so the controller never runs dry between chunks; a released clutch
keeps sending the same target, which holds the arm without letting the stream
time out. If the bridge dies, the chunks stop and the controller holds after
the goal's ``stream_timeout`` (cho_action_chunk/DESIGN.md, Stream liveness).

The gripper is the arm's ``controllers.gripper`` through its ``~/width_command``
topic (cho_controller_gripper), which preempts its action at once; its
``width_at_open`` and ``width_at_closed`` parameters scale the trigger.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from action_msgs.msg import GoalStatus
from cho_interfaces.action import ChunkStream
from cho_interfaces.msg import ActionChunk
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import Float64

from cho_control_tools.action_chunk.action_client import DEFAULT_CHUNK_TOPIC, controller_chunk_topic
from cho_control_tools.action_names import controller_action_name


MODEL_NAME = 'teleop_bridge'

IDLE = 'idle'
OPENING = 'opening'
ACTIVE = 'active'
CLOSING = 'closing'

_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


@dataclass(frozen=True)
class ArmTarget:
    """What the registry says about one arm (one profile)."""

    profile: str
    controller: str
    gripper_controller: str | None
    goal_frame: str
    ee_link: str


def resolve_arm(robot, profile, load_robot_config, task_goal_frame) -> ArmTarget:
    """One profile's arm from cho_robot_config (*load_robot_config*, *task_goal_frame*)."""
    config = load_robot_config(robot, profile)
    controllers = config.get('controllers') or {}
    controller = controllers.get('action_chunk')
    arms = config.get('arms') or []
    if not controller and len(arms) > 1:
        raise ValueError(
            f"{robot} profile '{profile}' drives the arms {arms} and has no action-chunk controller: "
            f"an action-chunk controller drives one arm, so name each arm's profile: --arms {' '.join(arms)}")
    if not controller:
        raise ValueError(
            f"{robot} ({profile}) has no action-chunk controller in cho_robot_config (controllers.action_chunk)")
    model = config.get('model') or {}
    # An absolute chunk's pose is in the profile's absolute goal frame; a registry
    # that declares none leaves it to the controller, whose root is the model's base frame.
    goal_frame = task_goal_frame(config, relative=False) or model.get('base_frame')
    ee_link = model.get('ee_link')
    if not goal_frame or not ee_link:
        raise ValueError(f'{robot} ({profile}): the registry names no goal frame or ee_link')
    return ArmTarget(profile, controller, controllers.get('gripper') or None, goal_frame, ee_link)


def build_chunk(position, quaternion, steps, control_dt, seq) -> ActionChunk:
    """An absolute task-space chunk holding one pose (quaternion x, y, z, w) for *steps* waypoints."""
    msg = ActionChunk()
    # Unstamped: the controller times it by arrival (chunk_time_source: arrival).
    msg.seq = seq
    msg.action_space = 'task'
    msg.relative_mode = 'absolute'
    msg.rotation_type = 'quaternion'
    msg.chunk_size = steps
    msg.control_dt = control_dt
    waypoint = [float(value) for value in position] + [float(value) for value in quaternion]
    msg.arm_actions = waypoint * steps
    return msg


def gripper_width(trigger, width_at_open, width_at_closed) -> float:
    """The opening for a trigger value in [0, 1]: 0 open, 1 closed."""
    trigger = min(max(float(trigger), 0.0), 1.0)
    return width_at_open + trigger * (width_at_closed - width_at_open)


def controller_parameters(node, controller, names, timeout_sec=2.0):
    """The controller's parameters among *names* that are set, as Python values."""
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python

    client = node.create_client(GetParameters, f'/{controller}/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return {}
        future = client.call_async(GetParameters.Request(names=list(names)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        if response is None:
            return {}
        return {name: parameter_value_to_python(value)
                for name, value in zip(names, response.values)
                if value.type != ParameterType.PARAMETER_NOT_SET}
    finally:
        node.destroy_client(client)


class ArmStream:
    """One arm's goal and chunk stream. ``on_end(stream, reason)`` hears of a goal the bridge did not end."""

    def __init__(self, node, target: ArmTarget, steps, control_dt, stream_timeout, rate_hz, on_end):
        self.target = target
        self.state = IDLE
        self.chunk_topic = None
        self.gripper_range = None
        self._node = node
        self._steps = steps
        self._control_dt = control_dt
        self._stream_timeout = stream_timeout
        self._rate_hz = rate_hz
        self._on_end = on_end
        self._action = controller_action_name(target.controller, 'chunk_stream')
        self._client = ActionClient(node, ChunkStream, self._action)
        self._publisher = None
        self._gripper_publisher = None
        self._handle = None
        self._seq = 0

    @property
    def active(self) -> bool:
        return self.state == ACTIVE

    def prepare(self, gripper=True, timeout_sec=5.0):
        """Find the server, its chunk topic and the gripper; an error message, or None. Call before spinning."""
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            return f'{self._action} is not available'
        topic = controller_chunk_topic(self._node, self.target.controller)
        if topic is None:
            topic = DEFAULT_CHUNK_TOPIC
            self._node.get_logger().warn(
                f'{self.target.controller} did not report its chunk_topic; publishing on {topic}')
        self.chunk_topic = topic
        self._publisher = self._node.create_publisher(ActionChunk, topic, 10)
        if gripper and self.target.gripper_controller:
            name = self.target.gripper_controller
            values = controller_parameters(self._node, name, ['width_at_open', 'width_at_closed'])
            opened, closed = values.get('width_at_open'), values.get('width_at_closed')
            if (isinstance(opened, float) and isinstance(closed, float)
                    and math.isfinite(opened) and math.isfinite(closed) and opened != closed):
                self.gripper_range = (opened, closed)
                self._gripper_publisher = self._node.create_publisher(Float64, f'/{name}/width_command', 10)
            else:
                self._node.get_logger().warn(
                    f'{name} reported no width_at_open / width_at_closed: '
                    f'{self.target.profile} streams without its gripper')
        return None

    def open(self):
        """Ask for a goal; chunks go out once it is accepted."""
        if self.state != IDLE:
            return
        goal = ChunkStream.Goal()
        goal.model_name = MODEL_NAME
        goal.inference_frequency = float(self._rate_hz)
        goal.stream_timeout = float(self._stream_timeout)
        self.state = OPENING
        self._seq = 0
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def close(self):
        """Cancel the goal (the controller brings the arm to rest and holds)."""
        if self.state == ACTIVE and self._handle is not None:
            self.state = CLOSING
            self._handle.cancel_goal_async()
        elif self.state == OPENING:
            self.state = CLOSING

    def publish(self, position, quaternion):
        if self.state != ACTIVE:
            return
        self._seq += 1
        self._publisher.publish(build_chunk(position, quaternion, self._steps, self._control_dt, self._seq))

    def publish_gripper(self, trigger):
        if self.state != ACTIVE or self._gripper_publisher is None:
            return
        self._gripper_publisher.publish(Float64(data=gripper_width(trigger, *self.gripper_range)))

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            closing = self.state == CLOSING
            self.state = IDLE
            if not closing:
                self._on_end(self, f'{self._action} rejected the goal')
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(self._on_result)
        if self.state == CLOSING:
            handle.cancel_goal_async()
            return
        self.state = ACTIVE

    def _on_result(self, future):
        response = future.result()
        closing = self.state == CLOSING
        self.state = IDLE
        self._handle = None
        if closing:
            return
        status = _STATUS_NAMES.get(response.status, str(response.status)) if response else 'unknown'
        message = response.result.message if response else ''
        self._on_end(self, f'{self._action} goal {status}' + (f': {message}' if message else ''))
