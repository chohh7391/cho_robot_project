#!/usr/bin/env python3

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

"""Expose Cho JointSpace/TaskSpace actions backed by MoveIt plan-and-execute.

A goal is planned (MoveGroup, plan only), slowed down uniformly when the plan
is shorter than the goal's ``duration_sec`` -- which the action contract makes
a minimum, never a planning budget -- and then executed (ExecuteTrajectory).
"""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from cho_interfaces.action import JointSpace, TaskSpace
from cho_robot_config import (blocked_home_joint_goals, controller_action_name,
                              load_robot_config, moveit_bridge_node)
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


# The bridge serves the same actions a controller does, by the same rule
# (cho_interfaces/CONTRACT.md): relative to its own node. Its node name is what
# scopes them to one robot profile -- see cho_robot_config.moveit_bridge_node().
JOINT_ACTION = '~/joint_space'
TASK_ACTION = '~/task_space'

# How far a frame may sit from the planning frame and still count as the same
# frame (m, rad). Static transforms from a URDF are exact; this only absorbs
# float round-off in a composed chain.
COINCIDENT_TOLERANCE = 1e-6

# The longest duration_sec a goal may ask for [s]. No motion this bridge plans
# is meant to take an hour: a larger value is a unit slip (ms for s) or garbage,
# and beyond 2**31 s the stretched trajectory's time_from_start cannot even be
# written (int32 seconds), which used to surface as an AssertionError from the
# message setter mid-goal instead of a rejection.
MAX_GOAL_DURATION_SEC = 3600.0


def _seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def _set_seconds(duration, seconds):
    whole = int(math.floor(seconds))
    nanosec = int(round((seconds - whole) * 1e9))
    if nanosec >= 1_000_000_000:
        whole += 1
        nanosec -= 1_000_000_000
    duration.sec = whole
    duration.nanosec = nanosec


def _scale_twist(twist, factor):
    for vector in (twist.linear, twist.angular):
        vector.x *= factor
        vector.y *= factor
        vector.z *= factor


def stretch_to_minimum_duration(trajectory, minimum_sec):
    """Slow *trajectory* uniformly so it lasts at least *minimum_sec*; never speed it up.

    *trajectory* is a ``moveit_msgs/RobotTrajectory`` and is changed in place.
    Every ``time_from_start`` is multiplied by ``scale = minimum_sec /
    planned``, every velocity divided by it and every acceleration by its
    square: the same path, followed at a uniformly lower speed, so it stays
    within the limits it was planned to. A plan already as long as asked for is
    left exactly as MoveIt timed it (``scale`` 1). A plan with no duration at
    all starts at its goal, so there is nothing to slow down and it is left
    alone too. Returns ``(planned_sec, scale)``.
    """
    minimum_sec = float(minimum_sec)
    if not math.isfinite(minimum_sec) or not 0.0 < minimum_sec <= MAX_GOAL_DURATION_SEC:
        raise ValueError(
            f'the minimum duration must be finite and in (0, {MAX_GOAL_DURATION_SEC:g}] s, '
            f'not {minimum_sec}')
    joint_points = list(trajectory.joint_trajectory.points)
    multi_points = list(trajectory.multi_dof_joint_trajectory.points)
    if not joint_points and not multi_points:
        raise ValueError('the plan has no points')
    times = [_seconds(point.time_from_start) for point in joint_points + multi_points]
    if not all(math.isfinite(value) and value >= 0.0 for value in times):
        raise ValueError('the plan has a negative or non-finite time_from_start')
    planned = max(times)
    if planned >= minimum_sec or planned <= 0.0:
        return planned, 1.0
    scale = minimum_sec / planned
    for point in joint_points:
        _set_seconds(point.time_from_start, _seconds(point.time_from_start) * scale)
        point.velocities = [value / scale for value in point.velocities]
        point.accelerations = [value / (scale * scale) for value in point.accelerations]
    for point in multi_points:
        _set_seconds(point.time_from_start, _seconds(point.time_from_start) * scale)
        for twist in point.velocities:
            _scale_twist(twist, 1.0 / scale)
        for twist in point.accelerations:
            _scale_twist(twist, 1.0 / (scale * scale))
    return planned, scale


class MoveItActionBridge(Node):
    @staticmethod
    def _expected_action_names(robot_type, profile='single'):
        """The absolute (joint, task) names clients look this bridge up by."""
        node = moveit_bridge_node(robot_type, profile)
        return (controller_action_name(node, 'joint_space'),
                controller_action_name(node, 'task_space'))

    def __init__(self):
        super().__init__('moveit_action_bridge')
        self.declare_parameter('robot_type', '')
        self.declare_parameter('profile', 'single')
        self.declare_parameter('planning_group', 'fr5_arm')
        self.declare_parameter('ee_link', 'wrist3_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('joint_names', ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        self.declare_parameter('trajectory_controller', 'joint_trajectory_controller')
        trajectory_controllers = self.declare_parameter(
            'trajectory_controllers', Parameter.Type.STRING_ARRAY)
        # A typed declaration without an override remains NOT_SET in Humble;
        # get_parameter(...).value then raises ParameterUninitializedException.
        # Only the OpenArm launch passes the plural override, so every other
        # robot's bridge needs the array explicitly initialized here.
        if trajectory_controllers.type_ == Parameter.Type.NOT_SET:
            self.set_parameters([Parameter(
                'trajectory_controllers', Parameter.Type.STRING_ARRAY, [])])
        self.declare_parameter('supports_task', True)
        self.declare_parameter('max_velocity_scaling_factor', 0.25)
        self.declare_parameter('max_acceleration_scaling_factor', 0.25)
        # Named explicitly rather than left to move_group's default, so the
        # pipeline a goal is planned with is visible in the launch file. OMPL is
        # the only pipeline this project registers.
        self.declare_parameter('planning_pipeline', 'ompl')
        # MoveIt's allowed_planning_time [s]. Its own parameter: the goal's
        # duration_sec is the motion's minimum length (CONTRACT.md), and it
        # used to be spent here as the planning budget instead.
        self.declare_parameter('planning_time_sec', 5.0)
        self.declare_parameter('move_group_action', '/move_action')
        # The plan is executed separately, after it has been slowed to the
        # goal's duration; move_group serves this by default.
        self.declare_parameter('execute_trajectory_action', '/execute_trajectory')
        self.declare_parameter('ready_service', '/static_scene_ready')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('planning_scene_service', '/get_planning_scene')
        self._robot_type = self.get_parameter('robot_type').value.strip('/')
        self._profile = self.get_parameter('profile').value.strip('/') or 'single'
        self._pipeline = self.get_parameter('planning_pipeline').value
        self._group = self.get_parameter('planning_group').value
        self._ee_link = self.get_parameter('ee_link').value
        self._world_frame = self.get_parameter('world_frame').value
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._trajectory_controller = self.get_parameter('trajectory_controller').value
        self._trajectory_controllers = list(
            self.get_parameter('trajectory_controllers').value)
        if not self._trajectory_controllers:
            self._trajectory_controllers = [self._trajectory_controller]
        self._supports_task = bool(self.get_parameter('supports_task').value)
        self._velocity_scaling = float(
            self.get_parameter('max_velocity_scaling_factor').value)
        self._acceleration_scaling = float(
            self.get_parameter('max_acceleration_scaling_factor').value)
        if not 0.0 < self._velocity_scaling <= 1.0:
            raise ValueError('max_velocity_scaling_factor must be in (0, 1]')
        if not 0.0 < self._acceleration_scaling <= 1.0:
            raise ValueError('max_acceleration_scaling_factor must be in (0, 1]')
        self._planning_time = float(self.get_parameter('planning_time_sec').value)
        if not math.isfinite(self._planning_time) or self._planning_time <= 0.0:
            raise ValueError('planning_time_sec must be finite and positive')
        self._move_group_action = self.get_parameter('move_group_action').value
        self._execute_action = self.get_parameter('execute_trajectory_action').value
        self._ready_service = self.get_parameter('ready_service').value
        controller_manager = self.get_parameter('controller_manager').value.rstrip('/')
        self._planning_scene_service = self.get_parameter('planning_scene_service').value
        if not self._robot_type:
            raise ValueError('robot_type must be non-empty')
        robot_config = load_robot_config(self._robot_type, self._profile)
        self._blocked_joint_goals = blocked_home_joint_goals(robot_config)
        # Accepted as an absolute goal's frame when TF shows it coincides with
        # the planning frame: it is the frame the controllers' own absolute
        # goals are written in on most robots (see _unhonoured_frame).
        self._arm_base_link = robot_config['model']['arm_base_link']
        self._joint_action = self.resolve_topic_name(JOINT_ACTION)
        self._task_action = self.resolve_topic_name(TASK_ACTION)
        expected = self._expected_action_names(self._robot_type, self._profile)
        if (self._joint_action, self._task_action) != expected:
            # The registry's action preferences -- what every client binds to --
            # name the bridge by robot and profile. Served under any other node
            # name, its actions would be found by nobody, or by a client of a
            # different robot.
            raise ValueError(
                f'MoveIt bridge for {self._robot_type}/{self._profile} must run as node '
                f'/{moveit_bridge_node(self._robot_type, self._profile)} so it serves '
                f'{expected[0]}; it is {self.get_fully_qualified_name()} and would serve '
                f'{self._joint_action}')
        if not self._group or not self._ee_link or not self._world_frame:
            raise ValueError('planning_group, ee_link, and world_frame must be non-empty')
        if not self._joint_names or len(set(self._joint_names)) != len(self._joint_names):
            raise ValueError('joint_names must be a non-empty list of unique names')
        self._callbacks = ReentrantCallbackGroup()
        self._ready = False
        self._ready_verified_at = 0.0
        self._ready_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._goal_reserved = False
        self._faulted = False
        self._fault_reason = ''
        self._joint_server = None
        self._task_server = None
        self._move_client = ActionClient(
            self, MoveGroup, self._move_group_action, callback_group=self._callbacks)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, self._execute_action, callback_group=self._callbacks)
        self._ready_client = self.create_client(
            Trigger, self._ready_service, callback_group=self._callbacks)
        self._controllers_client = self.create_client(
            ListControllers, f'{controller_manager}/list_controllers',
            callback_group=self._callbacks)
        self._scene_client = self.create_client(
            GetPlanningScene, self._planning_scene_service, callback_group=self._callbacks)
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self._ready_timer = self.create_timer(
            0.5, self._poll_ready, callback_group=self._callbacks)
        self._ready_query_pending = False
        self.get_logger().info(
            f'Cho MoveIt bridge ({self._robot_type}, {self._group}, {self._ee_link}) '
            'waiting for its floor/JTC identity gate before advertising actions')

    def _advertise_action_servers(self):
        if self._joint_server is not None:
            return
        self._joint_server = ActionServer(
            self, JointSpace, JOINT_ACTION, self._execute_joint,
            goal_callback=self._joint_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callbacks)
        if self._supports_task:
            self._task_server = ActionServer(
                self, TaskSpace, TASK_ACTION, self._execute_task,
                goal_callback=self._task_goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._callbacks)
        self.get_logger().info(
            f'Advertising identity-scoped actions: {self._joint_action}'
            + (f', {self._task_action}' if self._supports_task else ' (joint-only profile)'))
        self.get_logger().info(f'Planning pipeline: {self._pipeline}')

    def _poll_ready(self):
        services = (self._ready_client, self._controllers_client, self._scene_client)
        if self._ready_query_pending or not all(client.service_is_ready() for client in services):
            with self._ready_lock:
                if time.monotonic() - self._ready_verified_at > 1.5:
                    self._ready = False
            return
        self._ready_query_pending = True
        future = self._ready_client.call_async(Trigger.Request())
        future.add_done_callback(self._ready_response)

    def _ready_response(self, future):
        try:
            gate_ready = bool(future.result().success)
        except Exception as error:  # noqa: BLE001 - readiness remains closed
            self.get_logger().warn(f'Floor readiness query failed: {error}')
            gate_ready = False
        if not gate_ready:
            self._finish_ready_query(False)
            return
        future = self._controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._controllers_response)

    def _controllers_response(self, future):
        try:
            states = {item.name: item.state for item in future.result().controller}
            jtc_active = all(
                states.get(name) == 'active' for name in self._trajectory_controllers)
        except Exception as error:  # noqa: BLE001 - readiness remains closed
            self.get_logger().warn(f'Controller readiness query failed: {error}')
            jtc_active = False
        if not jtc_active:
            self._finish_ready_query(False)
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self._scene_client.call_async(request)
        future.add_done_callback(self._scene_response)

    def _scene_response(self, future):
        try:
            floor_present = any(
                item.id == 'floor'
                for item in future.result().scene.world.collision_objects)
        except Exception as error:  # noqa: BLE001 - readiness remains closed
            self.get_logger().warn(f'Planning scene readiness query failed: {error}')
            floor_present = False
        self._finish_ready_query(floor_present)

    def _finish_ready_query(self, ready):
        with self._ready_lock:
            was_ready = self._ready
            self._ready = ready
            if ready:
                self._ready_verified_at = time.monotonic()
        self._ready_query_pending = False
        if ready and not was_ready:
            self._advertise_action_servers()
            self.get_logger().info(
                f'READY: floor exists and {self._trajectory_controllers} are active')

    def _joint_goal_callback(self, request):
        # Everything that can be judged from the goal alone is judged here, so
        # a bad goal is rejected rather than accepted and then aborted
        # (cho_interfaces/CONTRACT.md).
        try:
            self._checked_joint_positions(request.target_joints)
        except ValueError as error:
            self.get_logger().error(f'Goal rejected: {error}')
            return GoalResponse.REJECT
        return self._goal_callback(request)

    def _task_goal_callback(self, request):
        reason = self._unhonoured_frame(request) or self._unusable_pose(request)
        if reason:
            self.get_logger().error(f'Goal rejected: {reason}')
            return GoalResponse.REJECT
        return self._goal_callback(request)

    def _checked_joint_positions(self, target_joints):
        """The goal's positions in joint order, or ValueError for any it cannot be."""
        positions = self._ordered_joint_positions(target_joints)
        if len(positions) != len(self._joint_names):
            raise ValueError(
                f'target_joints has {len(positions)} positions; this bridge drives '
                f'{len(self._joint_names)} joints ({self._joint_names})')
        if not all(math.isfinite(value) for value in positions):
            raise ValueError('target_joints contains a non-finite position')
        return positions

    @staticmethod
    def _unusable_pose(request):
        """Why the goal's pose cannot be used, or '' when it can."""
        pose = request.target_pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            return 'target_pose contains a non-finite value'
        try:
            MoveItActionBridge._normalize_quaternion(pose.orientation)
        except ValueError as error:
            return str(error)
        return ''

    def _ordered_joint_positions(self, target_joints):
        """The goal's positions in this bridge's joint order.

        With ``name`` empty they are already in that order; with names they are
        matched by name, and a goal naming an unknown joint, a joint twice, or
        not every joint is refused (cho_interfaces/CONTRACT.md).
        """
        positions = list(target_joints.position)
        names = list(target_joints.name)
        if not names:
            return positions
        if len(names) != len(positions):
            raise ValueError(
                f'target_joints names {len(names)} joints but gives {len(positions)} positions')
        unknown = sorted(set(names) - set(self._joint_names))
        if unknown:
            raise ValueError(f'target_joints names unknown joints {unknown}')
        if len(set(names)) != len(names):
            raise ValueError('target_joints names a joint more than once')
        missing = [name for name in self._joint_names if name not in names]
        if missing:
            raise ValueError(f'target_joints does not name {missing}')
        by_name = dict(zip(names, positions))
        return [by_name[name] for name in self._joint_names]

    def _unhonoured_frame(self, request):
        """Why the goal's frame cannot be honoured, or '' when it can.

        Absolute goals are planned in ``world_frame`` (the planning frame) and
        relative ones are composed in the EE frame. An absolute goal may
        therefore say '' or the planning frame, or the robot's
        ``model.arm_base_link`` -- the frame a controller's absolute goal is
        written in on most robots -- but only while TF shows it coincides with
        the planning frame, so that one stamped goal means the same pose to
        the bridge and to the controllers. A relative goal may say '' or the
        EE frame. Like the controllers, it does not transform: any other frame
        is refused rather than silently read as one of these.
        """
        frame = request.target_pose.header.frame_id
        if request.relative:
            if frame in ('', self._ee_link):
                return ''
            return (f"target_pose frame '{frame}' is not supported: a relative goal is a "
                    f"displacement in the EE frame, so frame_id must be '' or "
                    f"'{self._ee_link}'")
        if frame in ('', self._world_frame):
            return ''
        if frame == self._arm_base_link:
            apart = self._apart_from_planning_frame(frame)
            if not apart:
                return ''
            return (f"target_pose frame '{frame}' (the robot's arm_base_link) is accepted "
                    f"only where it coincides with the planning frame "
                    f"'{self._world_frame}', and {apart}")
        return (f"target_pose frame '{frame}' is not supported: an absolute goal is a pose "
                f"in the planning frame, so frame_id must be '', '{self._world_frame}' or, "
                f"where it coincides with that, '{self._arm_base_link}'")

    def _apart_from_planning_frame(self, frame):
        """'' when TF has *frame* at the planning frame, else what keeps them apart."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, frame, rclpy.time.Time())
        except TransformException as error:
            return f'TF cannot say whether it does: {error}'
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        offset = math.sqrt(translation.x ** 2 + translation.y ** 2 + translation.z ** 2)
        angle = 2.0 * math.acos(min(1.0, abs(rotation.w)))
        if offset <= COINCIDENT_TOLERANCE and angle <= COINCIDENT_TOLERANCE:
            return ''
        return f'TF has it {offset:.4f} m and {angle:.4f} rad away'

    def _goal_callback(self, request):
        duration = float(request.duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            self.get_logger().error('Goal rejected: duration must be finite and positive')
            return GoalResponse.REJECT
        if duration > MAX_GOAL_DURATION_SEC:
            self.get_logger().error(
                f'Goal rejected: duration_sec {duration:g} s is over the '
                f'{MAX_GOAL_DURATION_SEC:g} s this bridge accepts')
            return GoalResponse.REJECT
        with self._ready_lock:
            ready = self._ready and time.monotonic() - self._ready_verified_at <= 1.5
        if not ready:
            self.get_logger().error('Goal rejected: static floor planning scene is not ready')
            return GoalResponse.REJECT
        if not self._move_client.server_is_ready():
            self.get_logger().error(
                f'Goal rejected: MoveGroup {self._move_group_action} is unavailable')
            return GoalResponse.REJECT
        if not self._execute_client.server_is_ready():
            self.get_logger().error(
                f'Goal rejected: ExecuteTrajectory {self._execute_action} is unavailable')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._faulted:
                self.get_logger().error(
                    f'Goal rejected: MoveIt bridge is faulted: {self._fault_reason}; '
                    'restart the bridge after verifying the robot is stopped')
                return GoalResponse.REJECT
            if self._goal_reserved:
                self.get_logger().warn('Goal rejected: another MoveIt bridge goal is active')
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _release_goal(self):
        with self._goal_lock:
            if not self._faulted:
                self._goal_reserved = False

    def _latch_fault(self, reason):
        with self._goal_lock:
            self._faulted = True
            self._fault_reason = reason
            # Deliberately retain the active reservation: downstream motion is
            # not known to be terminal. Recovery requires a node restart after
            # independently verifying MoveGroup/JTC are idle.
            self._goal_reserved = True
        self.get_logger().fatal(f'FAIL-CLOSED MoveIt bridge fault: {reason}')

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _joint_constraints(self, positions):
        constraints = Constraints()
        for name, value in zip(self._joint_names, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = value
            joint.tolerance_above = 0.001
            joint.tolerance_below = 0.001
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _blocked_joint_goal(self, positions):
        for blocked in self._blocked_joint_goals:
            max_distance = max(
                abs(actual - expected)
                for actual, expected in zip(positions, blocked['positions']))
            if max_distance <= blocked['max_joint_distance']:
                return blocked
        return None

    @staticmethod
    def _rotate(q, vector):
        x, y, z = vector
        return (
            (1 - 2*(q.y*q.y + q.z*q.z))*x
            + 2*(q.x*q.y - q.z*q.w)*y + 2*(q.x*q.z + q.y*q.w)*z,
            2*(q.x*q.y + q.z*q.w)*x
            + (1 - 2*(q.x*q.x + q.z*q.z))*y + 2*(q.y*q.z - q.x*q.w)*z,
            2*(q.x*q.z - q.y*q.w)*x
            + 2*(q.y*q.z + q.x*q.w)*y + (1 - 2*(q.x*q.x + q.y*q.y))*z,
        )

    @staticmethod
    def _normalize_quaternion(q):
        values = (q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(value*value for value in values))
        if not math.isfinite(norm) or norm < 1e-6:
            raise ValueError('target orientation quaternion is invalid')
        return tuple(value / norm for value in values)

    @staticmethod
    def _error_name(code):
        """Name the MoveIt error code so a failure reason reads without a lookup."""
        for name, value in vars(MoveItErrorCodes).items():
            if name.isupper() and isinstance(value, int) and value == code:
                return f'{name}({code})'
        return str(code)

    @staticmethod
    def _compose_quaternion(left, right):
        lx, ly, lz, lw = MoveItActionBridge._normalize_quaternion(left)
        rx, ry, rz, rw = MoveItActionBridge._normalize_quaternion(right)
        values = (
            lw*rx + lx*rw + ly*rz - lz*ry,
            lw*ry - lx*rz + ly*rw + lz*rx,
            lw*rz + lx*ry - ly*rx + lz*rw,
            lw*rw - lx*rx - ly*ry - lz*rz,
        )
        norm = math.sqrt(sum(value*value for value in values))
        return tuple(value / norm for value in values)

    def _task_constraints(self, request):
        # The frame was checked when the goal was accepted: '', world_frame or
        # an arm_base_link that coincides with it for an absolute goal, '' or
        # the EE frame for a relative one. A relative goal is composed against
        # TF's world_frame -> EE, so world_frame has to be in TF; it is on
        # every bringup that starts this bridge (cho_moveit/README.md).
        target = request.target_pose.pose
        if request.relative:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, self._ee_link, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            delta = self._rotate(
                transform.transform.rotation,
                (target.position.x, target.position.y, target.position.z))
            pose = Pose()
            pose.position.x = transform.transform.translation.x + delta[0]
            pose.position.y = transform.transform.translation.y + delta[1]
            pose.position.z = transform.transform.translation.z + delta[2]
            composed = self._compose_quaternion(
                transform.transform.rotation, target.orientation)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = composed
        else:
            pose = Pose()
            pose.position = target.position
            normalized = self._normalize_quaternion(target.orientation)
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = normalized

        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('target pose contains a non-finite value')

        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self._world_frame
        position.link_name = self._ee_link
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0
        constraints.position_constraints.append(position)
        orientation = OrientationConstraint()
        orientation.header.frame_id = self._world_frame
        orientation.link_name = self._ee_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _plan_goal(self, constraints, pipeline):
        """A plan-only MoveGroup goal: the plan is slowed and executed separately."""
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.pipeline_id = pipeline
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = self._planning_time
        # The scaling caps the planned speed. The goal's duration_sec then
        # sets a floor on the motion's length: stretch_to_minimum_duration().
        goal.request.max_velocity_scaling_factor = self._velocity_scaling
        goal.request.max_acceleration_scaling_factor = self._acceleration_scaling
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        return goal

    def _wait_for(self, future, cho_handle):
        """Wait for *future*; False if a cancel or shutdown came first."""
        while rclpy.ok() and not future.done():
            if cho_handle.is_cancel_requested:
                return False
            time.sleep(0.02)
        return future.done()

    @staticmethod
    def _cancel_plan_if_accepted(send_future):
        """Done-callback: a planning request given up on is cancelled once accepted."""
        try:
            handle = send_future.result()
        except Exception:  # noqa: BLE001 - it never reached move_group
            return
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def _plan(self, cho_handle, constraints, pipeline):
        """Plan without executing. Returns (RobotTrajectory or None, reason, canceled).

        Nothing moves while planning, so unlike execution a failure here never
        latches the bridge's fault: there is no motion whose state is unknown.
        """
        try:
            send_future = self._move_client.send_goal_async(
                self._plan_goal(constraints, pipeline))
        except Exception as error:  # noqa: BLE001 - nothing was planned
            return None, f'MoveGroup send_goal transport failed: {error}', False
        if not self._wait_for(send_future, cho_handle):
            send_future.add_done_callback(self._cancel_plan_if_accepted)
            if cho_handle.is_cancel_requested:
                return None, 'canceled while planning; nothing moved', True
            return None, 'ROS shutdown while planning; nothing moved', False
        try:
            plan_handle = send_future.result()
        except Exception as error:  # noqa: BLE001 - nothing was planned
            return None, f'MoveGroup send_goal result failed: {error}', False
        if plan_handle is None or not plan_handle.accepted:
            return None, 'MoveGroup rejected the planning request', False
        try:
            result_future = plan_handle.get_result_async()
        except Exception as error:  # noqa: BLE001 - plan only, nothing moves
            plan_handle.cancel_goal_async()
            return None, f'MoveGroup get_result transport failed: {error}', False
        if not self._wait_for(result_future, cho_handle):
            # A plan-only goal never moves the arm, so there is nothing to wait
            # for: cancel the planning and report at once.
            plan_handle.cancel_goal_async()
            if cho_handle.is_cancel_requested:
                return None, 'canceled while planning; nothing moved', True
            return None, 'ROS shutdown while planning; nothing moved', False
        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001 - nothing was planned
            return None, f'MoveGroup planning result failed: {error}', False
        if wrapped is None or wrapped.result is None:
            return None, 'MoveGroup returned no planning result', False
        error = wrapped.result.error_code.val
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or error != MoveItErrorCodes.SUCCESS:
            return None, (f'MoveIt planning failed: action_status={wrapped.status}, '
                          f'error_code={self._error_name(error)}'), False
        return wrapped.result.planned_trajectory, '', False

    def _run_move_group(self, cho_handle, constraints, duration, feedback_type, pipeline):
        """Plan, slow the plan to *duration*, execute. Return (succeeded, reason).

        *duration* is the goal's duration_sec, a minimum (CONTRACT.md): a plan
        shorter than it is stretched uniformly, a longer one is left as MoveIt
        timed it. reason is '' only on success.
        """
        feedback = feedback_type()
        feedback.percent_complete = 0.0
        cho_handle.publish_feedback(feedback)
        trajectory, reason, canceled = self._plan(cho_handle, constraints, pipeline)
        if trajectory is None:
            if canceled:
                cho_handle.canceled()
                return False, reason
            self.get_logger().error(reason)
            cho_handle.abort()
            return False, reason
        try:
            planned, scale = stretch_to_minimum_duration(trajectory, duration)
        except ValueError as error:
            reason = f'MoveIt returned an unusable plan: {error}'
            self.get_logger().error(reason)
            cho_handle.abort()
            return False, reason
        if scale > 1.0:
            self.get_logger().info(
                f'Planned {planned:.2f}s; slowed to the goal\'s minimum {float(duration):.2f}s')
        if cho_handle.is_cancel_requested:
            cho_handle.canceled()
            return False, 'canceled before execution; nothing moved'
        return self._execute(cho_handle, trajectory, feedback)

    def _execute(self, cho_handle, trajectory, feedback):
        """Execute *trajectory*; return (succeeded, reason) and end *cho_handle*.

        From here the arm can move, so any outcome whose motion state is
        unknown latches the bridge's fault and keeps its reservation.
        """
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        try:
            send_future = self._execute_client.send_goal_async(goal)
        except Exception as error:  # noqa: BLE001 - transport state is unknown
            reason = f'ExecuteTrajectory send_goal transport failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        cancel_requested_before_accept = False
        send_wait_warning_at = time.monotonic() + 10.0
        while rclpy.ok() and not send_future.done():
            if cho_handle.is_cancel_requested:
                cancel_requested_before_accept = True
            if time.monotonic() >= send_wait_warning_at:
                self.get_logger().error(
                    'Still waiting for ExecuteTrajectory send_goal response; reservation '
                    'remains locked because a late acceptance must not escape cancellation')
                send_wait_warning_at = time.monotonic() + 10.0
            time.sleep(0.02)
        if not send_future.done():
            reason = 'ROS shutdown while ExecuteTrajectory goal acceptance was pending'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        try:
            move_handle = send_future.result()
        except Exception as error:  # noqa: BLE001 - late acceptance is possible
            reason = f'ExecuteTrajectory send_goal result failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        if move_handle is None or not move_handle.accepted:
            if cancel_requested_before_accept:
                cho_handle.canceled()
                return False, 'canceled before ExecuteTrajectory accepted the goal'
            reason = 'ExecuteTrajectory rejected the planned trajectory'
            self.get_logger().error(reason)
            cho_handle.abort()
            return False, reason
        try:
            result_future = move_handle.get_result_async()
        except Exception as error:  # noqa: BLE001 - accepted goal may be moving
            reason = f'ExecuteTrajectory get_result transport failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        if cancel_requested_before_accept:
            return self._cancel_downstream(cho_handle, move_handle, result_future)
        while rclpy.ok() and not result_future.done():
            if cho_handle.is_cancel_requested:
                return self._cancel_downstream(cho_handle, move_handle, result_future)
            time.sleep(0.02)
        if not result_future.done():
            reason = 'ROS shutdown while accepted ExecuteTrajectory goal was active'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001 - terminal state is unknown
            reason = f'ExecuteTrajectory result future failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        if wrapped is None or wrapped.result is None:
            reason = 'ExecuteTrajectory returned no result; motion state is unknown'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        error = wrapped.result.error_code.val
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or error != MoveItErrorCodes.SUCCESS):
            reason = (f'MoveIt execution failed: action_status={wrapped.status}, '
                      f'error_code={self._error_name(error)}')
            self.get_logger().error(reason)
            cho_handle.abort()
            return False, reason
        feedback.percent_complete = 100.0
        cho_handle.publish_feedback(feedback)
        cho_handle.succeed()
        return True, ''

    def _cancel_downstream(self, cho_handle, move_handle, result_future):
        try:
            cancel_future = move_handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001 - motion state is unknown
            reason = f'ExecuteTrajectory cancel transport failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not cancel_future.done():
            reason = 'ExecuteTrajectory cancel response timed out; motion state is unknown'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        try:
            response = cancel_future.result()
        except Exception as error:  # noqa: BLE001 - motion state is unknown
            reason = f'ExecuteTrajectory cancel result failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        if (response is None or response.return_code != CancelGoal.Response.ERROR_NONE
                or not response.goals_canceling):
            code = response.return_code if response is not None else 'no response'
            reason = (f'ExecuteTrajectory cancel rejected (return_code={code}); '
                      'motion state is unknown')
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not result_future.done():
            reason = ('ExecuteTrajectory did not reach a terminal state after cancel; '
                      'motion state is unknown')
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001 - terminal state is unknown
            reason = f'ExecuteTrajectory post-cancel result failed: {error}'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        if wrapped is None or wrapped.status != GoalStatus.STATUS_CANCELED:
            status = wrapped.status if wrapped is not None else 'no result'
            reason = f'ExecuteTrajectory terminal status after cancel was not CANCELED ({status})'
            self._latch_fault(reason)
            cho_handle.abort()
            return False, reason
        cho_handle.canceled()
        return False, 'goal canceled'

    @staticmethod
    def _target_summary(constraints):
        """Describe the resolved world-frame target a task goal was planned to.

        A relative goal is composed against live TF, so the pose the operator
        typed is not the pose MoveIt refused. Reporting the resolved target is
        what makes an unreachable `reach` diagnosable from the client alone.
        """
        region = constraints.position_constraints[0].constraint_region
        pose = region.primitive_poses[0]
        return (f'resolved target x={pose.position.x:+.4f} y={pose.position.y:+.4f} '
                f'z={pose.position.z:+.4f}')

    def _execute_joint(self, goal_handle):
        result = JointSpace.Result()
        try:
            try:
                # Already checked when the goal was accepted; repeated so this
                # path never plans a goal it would have rejected.
                positions = self._checked_joint_positions(goal_handle.request.target_joints)
            except ValueError as error:
                result.message = f'Joint goal rejected: {error}'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result
            blocked = self._blocked_joint_goal(positions)
            if blocked is not None:
                result.message = (
                    f"Joint goal rejected: home {blocked['selector']} is disabled for "
                    f"{self._robot_type}: {blocked['reason']}")
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result
            result.is_completed, result.message = self._run_move_group(
                goal_handle, self._joint_constraints(positions),
                goal_handle.request.duration_sec, JointSpace.Feedback,
                self._pipeline)
            return result
        finally:
            self._release_goal()

    def _execute_task(self, goal_handle):
        result = TaskSpace.Result()
        try:
            try:
                constraints = self._task_constraints(goal_handle.request)
            except (ValueError, TransformException) as error:
                result.message = f'Task goal conversion failed: {error}'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result
            result.is_completed, result.message = self._run_move_group(
                goal_handle, constraints, goal_handle.request.duration_sec, TaskSpace.Feedback,
                self._pipeline)
            if not result.is_completed:
                result.message = f'{result.message}; {self._target_summary(constraints)}'
            return result
        finally:
            self._release_goal()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItActionBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
