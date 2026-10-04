import threading

import py_trees
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from cho_task_manager.utils.clock import arm, deadline_after
from cho_task_manager.utils.controller_names import valid_controller_action_names


class BaseActionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, action_type, action_name: str, timeout_sec: float = 30.0):
        super().__init__(name)

        # A raise, not an assert: `python -O` strips asserts, and this is the
        # check that keeps a misspelt name from becoming a goal that waits out
        # its timeout against a server nobody runs.
        valid_controllers = valid_controller_action_names()
        if str(action_name) not in valid_controllers:
            raise ValueError(
                f"[{name}] Invalid controller action name: '{action_name}'. "
                f"Must be one of {valid_controllers}")

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
        # Cancels this leaf still owes or is waiting to hear back about, kept
        # so whoever tears the node down can spin until they are out
        # (cancels_outstanding()). Touched from executor threads by the
        # done-callbacks, hence the lock.
        self._cancel_lock = threading.Lock()
        self._owed_cancels = set()      # send_goal futures to cancel on acceptance
        self._cancel_requests = []      # cancel responses not yet received

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
        # None while the node clock has not started (sim time before the first
        # /clock): a deadline from 0 would be missed on the first tick after
        # /clock arrives. _timed_out() takes it once the clock runs.
        self._deadline = deadline_after(self.node.get_clock(), self.timeout_sec)

    def _timed_out(self):
        clock = self.node.get_clock()
        self._deadline = arm(self._deadline, clock, self.timeout_sec)
        return self._deadline is not None and clock.now() > self._deadline

    def _send_cancel(self, goal_handle):
        """Ask the server to cancel *goal_handle*, and remember to wait for its answer."""
        response = goal_handle.cancel_goal_async()
        if response is not None:
            with self._cancel_lock:
                self._cancel_requests.append(response)

    def _cancel_late_accepted_goal(self, send_goal_future):
        """Done-callback: cancel a goal whose acceptance arrived after we gave up on it.

        Every accepted goal is cancelled. Its handle cannot say whether it has
        already finished -- rclpy creates it at the goal response with status
        UNKNOWN and drops status updates until then -- and cancelling a goal
        that has finished costs one refused request. A rejected one has
        nothing to cancel.
        """
        try:
            try:
                goal_handle = send_goal_future.result()
            except Exception:  # noqa: B902 - the request failed; no goal to cancel
                goal_handle = None
            if goal_handle is not None and goal_handle.accepted:
                self._send_cancel(goal_handle)
        finally:
            with self._cancel_lock:
                self._owed_cancels.discard(send_goal_future)

    def _arm_late_cancel(self, send_goal_future):
        """Cancel the goal *send_goal_future* carries as soon as the server accepts it.

        Fires at once when the response is already in but no tick has read it.
        """
        with self._cancel_lock:
            self._owed_cancels.add(send_goal_future)
        send_goal_future.add_done_callback(self._cancel_late_accepted_goal)

    def cancels_outstanding(self):
        """True while a cancel this leaf decided on has not been seen through.

        Either a goal whose acceptance is still in flight and is to be
        cancelled when it lands, or a cancel request with no response yet.
        Both need the executor to spin, so a process that destroys its node
        while this is True drops them (task_manager_node.shutdown_task_manager).
        """
        with self._cancel_lock:
            self._cancel_requests = [
                response for response in self._cancel_requests if not response.done()]
            return bool(self._owed_cancels or self._cancel_requests)

    def _abandon_goal(self, new_status):
        """Cancel the goal this leaf sent if it may still be running.

        Called from terminate() for EVERY status, not only INVALID: a leaf that
        returns SUCCESS or FAILURE while its goal is in flight (a sweep giving
        up mid-motion, say) leaves a motion running that nothing will ever
        cancel. Nothing happens when the goal has finished -- rejected, or its
        result already in -- which is how every plain action leaf ends.
        """
        if self.goal_handle is not None:
            if self.get_result_future is not None and not self.get_result_future.done():
                self.node.get_logger().warn(
                    f"[{self.name}] {self._ending(new_status)} with its goal still running; "
                    "cancelling it")
                self._send_cancel(self.goal_handle)
        elif self.send_goal_future is not None:
            # Acceptance still pending: there is no handle to cancel yet, but the
            # server may accept and execute the goal after the tree has moved on.
            # Same remedy as the timeout path -- cancel it on acceptance.
            self.node.get_logger().warn(
                f"[{self.name}] {self._ending(new_status)} before the goal was accepted; "
                "it will be cancelled on acceptance")
            self._arm_late_cancel(self.send_goal_future)

    @staticmethod
    def _ending(new_status):
        if new_status == py_trees.common.Status.INVALID:
            return 'Preempted'
        return f'Ended {new_status.name}'

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
            # Every cho result carries the server's reason (CONTRACT.md); a
            # FollowJointTrajectory result has error_string instead.
            reason = (getattr(result.result, 'message', '')
                      or getattr(result.result, 'error_string', '') or '')
            self.node.get_logger().error(
                f"[{self.name}] Action Failed with status: {result.status}"
                + (f": {reason}" if reason else ''))
            return py_trees.common.Status.FAILURE

        if self._timed_out():
            self.node.get_logger().error(
                f"[{self.name}] Timed out after {self.timeout_sec}s waiting for {self.action_name}"
            )
            if self.goal_handle is not None:
                self._send_cancel(self.goal_handle)
            else:
                # Acceptance still pending: the server may accept (and execute) the
                # goal after we've moved on -- make sure it gets cancelled then, or a
                # stale motion could run while the tree is doing something else.
                self._arm_late_cancel(self.send_goal_future)
            self.send_goal_future = None
            self.get_result_future = None
            self.goal_handle = None
            self._deadline = None
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._abandon_goal(new_status)
        self.send_goal_future = None
        self.get_result_future = None
        self.goal_handle = None
        self._deadline = None
