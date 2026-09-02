#!/usr/bin/env python3
"""Expose Cho JointSpace/TaskSpace actions backed by MoveIt plan-and-execute."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from cho_interfaces.action import JointSpace, TaskSpace
from cho_robot_config import blocked_home_joint_goals, load_robot_config
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
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


ACTION_NAMESPACE = 'controller_action_server'


class MoveItActionBridge(Node):
    @staticmethod
    def _action_names(robot_type, profile='single'):
        robot_type = robot_type.strip('/')
        if not robot_type:
            raise ValueError('robot_type must be non-empty')
        prefix = f'/{robot_type}' if profile == 'single' else f'/{robot_type}/{profile}'
        return (f'{prefix}/{ACTION_NAMESPACE}/moveit_joint',
                f'{prefix}/{ACTION_NAMESPACE}/moveit_task')

    def __init__(self):
        super().__init__('moveit_action_bridge')
        self.declare_parameter('robot_type', '')
        self.declare_parameter('profile', 'single')
        self.declare_parameter('planning_group', 'fr5_arm')
        self.declare_parameter('ee_link', 'wrist3_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('joint_names', ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        self.declare_parameter('trajectory_controller', 'joint_trajectory_controller')
        self.declare_parameter('trajectory_controllers', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('supports_task', True)
        self.declare_parameter('max_velocity_scaling_factor', 0.25)
        self.declare_parameter('max_acceleration_scaling_factor', 0.25)
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('ready_service', '/static_scene_ready')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('planning_scene_service', '/get_planning_scene')
        self._robot_type = self.get_parameter('robot_type').value.strip('/')
        self._profile = self.get_parameter('profile').value.strip('/') or 'single'
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
        self._move_group_action = self.get_parameter('move_group_action').value
        self._ready_service = self.get_parameter('ready_service').value
        controller_manager = self.get_parameter('controller_manager').value.rstrip('/')
        self._planning_scene_service = self.get_parameter('planning_scene_service').value
        if not self._robot_type:
            raise ValueError('robot_type must be non-empty')
        self._blocked_joint_goals = blocked_home_joint_goals(
            load_robot_config(self._robot_type, self._profile))
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
        self._joint_action, self._task_action = self._action_names(
            self._robot_type, self._profile)
        self._move_client = ActionClient(
            self, MoveGroup, self._move_group_action, callback_group=self._callbacks)
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
            self, JointSpace, self._joint_action, self._execute_joint,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callbacks)
        if self._supports_task:
            self._task_server = ActionServer(
                self, TaskSpace, self._task_action, self._execute_task,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._callbacks)
        self.get_logger().info(
            f'Advertising identity-scoped actions: {self._joint_action}'
            + (f', {self._task_action}' if self._supports_task else ' (joint-only profile)'))

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

    def _goal_callback(self, request):
        duration = float(request.duration)
        if not math.isfinite(duration) or duration <= 0.0:
            self.get_logger().error('Goal rejected: duration must be finite and positive')
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
        target = request.target_pose
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

    def _move_goal(self, constraints, duration):
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.pipeline_id = 'ompl'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = max(1.0, min(float(duration), 10.0))
        # Cho duration is used as the planning-time budget. MoveIt trajectory
        # timing remains governed by limits and the conservative speed scaling.
        goal.request.max_velocity_scaling_factor = self._velocity_scaling
        goal.request.max_acceleration_scaling_factor = self._acceleration_scaling
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return goal

    def _run_move_group(self, cho_handle, constraints, duration, feedback_type):
        feedback = feedback_type()
        feedback.percent_complete = 0.0
        cho_handle.publish_feedback(feedback)
        try:
            send_future = self._move_client.send_goal_async(
                self._move_goal(constraints, duration))
        except Exception as error:  # noqa: BLE001 - transport state is unknown
            self._latch_fault(f'MoveGroup send_goal transport failed: {error}')
            cho_handle.abort()
            return False
        cancel_requested_before_accept = False
        send_wait_warning_at = time.monotonic() + 10.0
        while rclpy.ok() and not send_future.done():
            if cho_handle.is_cancel_requested:
                cancel_requested_before_accept = True
            if time.monotonic() >= send_wait_warning_at:
                self.get_logger().error(
                    'Still waiting for MoveGroup send_goal response; reservation remains '
                    'locked because a late acceptance must not escape cancellation')
                send_wait_warning_at = time.monotonic() + 10.0
            time.sleep(0.02)
        if not send_future.done():
            self._latch_fault('ROS shutdown while MoveGroup goal acceptance was pending')
            cho_handle.abort()
            return False
        try:
            move_handle = send_future.result()
        except Exception as error:  # noqa: BLE001 - late acceptance is possible
            self._latch_fault(f'MoveGroup send_goal result failed: {error}')
            cho_handle.abort()
            return False
        if move_handle is None or not move_handle.accepted:
            if cancel_requested_before_accept:
                cho_handle.canceled()
                return False
            self.get_logger().error('MoveGroup rejected translated goal')
            cho_handle.abort()
            return False
        try:
            result_future = move_handle.get_result_async()
        except Exception as error:  # noqa: BLE001 - accepted goal may be moving
            self._latch_fault(f'MoveGroup get_result transport failed: {error}')
            cho_handle.abort()
            return False
        if cancel_requested_before_accept:
            return self._cancel_downstream(cho_handle, move_handle, result_future)
        while rclpy.ok() and not result_future.done():
            if cho_handle.is_cancel_requested:
                return self._cancel_downstream(cho_handle, move_handle, result_future)
            time.sleep(0.02)
        if not result_future.done():
            self._latch_fault('ROS shutdown while accepted MoveGroup goal was active')
            cho_handle.abort()
            return False
        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001 - terminal state is unknown
            self._latch_fault(f'MoveGroup result future failed: {error}')
            cho_handle.abort()
            return False
        if wrapped is None or wrapped.result is None:
            self._latch_fault('MoveGroup returned no result; motion state is unknown')
            cho_handle.abort()
            return False
        error = wrapped.result.error_code.val
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or error != MoveItErrorCodes.SUCCESS):
            self.get_logger().error(
                f'MoveIt plan/execute failed: action_status={wrapped.status}, error_code={error}')
            cho_handle.abort()
            return False
        feedback.percent_complete = 100.0
        cho_handle.publish_feedback(feedback)
        cho_handle.succeed()
        return True

    def _cancel_downstream(self, cho_handle, move_handle, result_future):
        try:
            cancel_future = move_handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001 - motion state is unknown
            self._latch_fault(f'MoveGroup cancel transport failed: {error}')
            cho_handle.abort()
            return False
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not cancel_future.done():
            self._latch_fault('MoveGroup cancel response timed out; motion state is unknown')
            cho_handle.abort()
            return False
        try:
            response = cancel_future.result()
        except Exception as error:  # noqa: BLE001 - motion state is unknown
            self._latch_fault(f'MoveGroup cancel result failed: {error}')
            cho_handle.abort()
            return False
        if (response is None or response.return_code != CancelGoal.Response.ERROR_NONE
                or not response.goals_canceling):
            code = response.return_code if response is not None else 'no response'
            self._latch_fault(
                f'MoveGroup cancel rejected (return_code={code}); motion state is unknown')
            cho_handle.abort()
            return False
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not result_future.done():
            self._latch_fault(
                'MoveGroup did not reach a terminal state after cancel; motion state is unknown')
            cho_handle.abort()
            return False
        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001 - terminal state is unknown
            self._latch_fault(f'MoveGroup post-cancel result failed: {error}')
            cho_handle.abort()
            return False
        if wrapped is None or wrapped.status != GoalStatus.STATUS_CANCELED:
            status = wrapped.status if wrapped is not None else 'no result'
            self._latch_fault(
                f'MoveGroup terminal status after cancel was not CANCELED ({status})')
            cho_handle.abort()
            return False
        cho_handle.canceled()
        return False

    def _execute_joint(self, goal_handle):
        result = JointSpace.Result()
        try:
            positions = list(goal_handle.request.target_joints.position)
            if len(positions) != len(self._joint_names) or not all(
                    math.isfinite(value) for value in positions):
                self.get_logger().error(
                    f'Joint goal must contain {len(self._joint_names)} finite positions')
                goal_handle.abort()
                return result
            blocked = self._blocked_joint_goal(positions)
            if blocked is not None:
                self.get_logger().error(
                    f"Joint goal rejected: home {blocked['selector']} is disabled for "
                    f"{self._robot_type}: {blocked['reason']}")
                goal_handle.abort()
                return result
            result.is_completed = self._run_move_group(
                goal_handle, self._joint_constraints(positions),
                goal_handle.request.duration, JointSpace.Feedback)
            return result
        finally:
            self._release_goal()

    def _execute_task(self, goal_handle):
        result = TaskSpace.Result()
        try:
            try:
                constraints = self._task_constraints(goal_handle.request)
            except (ValueError, TransformException) as error:
                self.get_logger().error(f'Task goal conversion failed: {error}')
                goal_handle.abort()
                return result
            result.is_completed = self._run_move_group(
                goal_handle, constraints, goal_handle.request.duration, TaskSpace.Feedback)
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
