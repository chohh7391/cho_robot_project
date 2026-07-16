import py_trees
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from cho_task_manager.utils.controller_names import valid_controller_action_names


class BaseActionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, action_type, action_name: str, timeout_sec: float = 30.0):
        super().__init__(name)

        valid_controllers = valid_controller_action_names()
        assert str(action_name) in valid_controllers, \
            f"Invalid controller name: '{action_name}'. Must be one of {valid_controllers}"

        self.action_type = action_type
        self.action_name = action_name
        self.timeout_sec = timeout_sec

        self.client = None
        self.node: Node = None
        self.send_goal_future = None
        self.get_result_future = None
        self.goal_handle = None
        self.cb_group = None
        self.server_available = False
        self._deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cb_group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self.node, 
            self.action_type, 
            self.action_name, 
            callback_group=self.cb_group
        )
        self.node.get_logger().info(f"[{self.name}] Waiting for {self.action_name} Server...")
        self.server_available = self.client.wait_for_server(timeout_sec=3.0)
        if not self.server_available:
            self.node.get_logger().warn(
                f"[{self.name}] Action server not available during setup: {self.action_name}"
            )
        return True

    def send_action_goal(self, goal_msg):
        """Called by subclasses from initialise()."""
        if not self.client.wait_for_server(timeout_sec=0.1):
            self.node.get_logger().error(
                f"[{self.name}] Action server not available: {self.action_name}"
            )
            self.server_available = False
            self.send_goal_future = None
            self.get_result_future = None
            self.goal_handle = None
            self._deadline = None
            return

        self.server_available = True
        self.node.get_logger().info(f"[{self.name}] Sending Goal...")
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.get_result_future = None
        self.goal_handle = None
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.timeout_sec)

    def _timed_out(self):
        return self._deadline is not None and self.node.get_clock().now() > self._deadline

    @staticmethod
    def _cancel_late_accepted_goal(send_goal_future):
        """Done-callback: cancel a goal whose acceptance arrived after we gave up on it."""
        goal_handle = send_goal_future.result()
        if goal_handle is not None and goal_handle.accepted:
            goal_handle.cancel_goal_async()

    def update(self):
        if self.send_goal_future is None:
            return py_trees.common.Status.FAILURE

        # Drain completed futures BEFORE the deadline check: a result that is already
        # in must be reported as-is -- returning FAILURE for a motion that actually
        # succeeded (just because the tick landed past the deadline) would abort the
        # mission for nothing. The deadline only fires while genuinely still waiting.
        if self.send_goal_future.done() and self.goal_handle is None:
            self.goal_handle = self.send_goal_future.result()
            if not self.goal_handle.accepted:
                self.node.get_logger().error(f"[{self.name}] Goal Rejected!")
                return py_trees.common.Status.FAILURE
            self.get_result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.get_result_future is not None and self.get_result_future.done():
            result = self.get_result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info(f"[{self.name}] Action Succeeded!")
                return py_trees.common.Status.SUCCESS
            self.node.get_logger().error(f"[{self.name}] Action Failed with status: {result.status}")
            return py_trees.common.Status.FAILURE

        if self._timed_out():
            self.node.get_logger().error(
                f"[{self.name}] Timed out after {self.timeout_sec}s waiting for {self.action_name}"
            )
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            else:
                # Acceptance still pending: the server may accept (and execute) the
                # goal after we've moved on -- make sure it gets cancelled then, or a
                # stale motion could run while the tree is doing something else.
                self.send_goal_future.add_done_callback(self._cancel_late_accepted_goal)
            self.send_goal_future = None
            self.get_result_future = None
            self.goal_handle = None
            self._deadline = None
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.goal_handle is not None:
            if self.get_result_future is not None and not self.get_result_future.done():
                self.node.get_logger().warn(f"[{self.name}] Preempted! Canceling Goal...")
                self.goal_handle.cancel_goal_async()

        self.send_goal_future = None
        self.get_result_future = None
        self.goal_handle = None
        self._deadline = None
