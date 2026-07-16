"""
Unit tests for BaseActionBehavior's state machine and timeout handling.

These run without a live ROS graph: rclpy Node/ActionClient are replaced with mocks,
and the clock is faked so timeout tests don't actually sleep.
"""
import py_trees
from action_msgs.msg import GoalStatus
from unittest.mock import MagicMock

from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.utils.controller_names import ControllerNames, controller_action_name


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds / 1e9)

    def __gt__(self, other):
        return self.seconds > other.seconds


class FakeClock:
    def __init__(self, start=0.0):
        self.seconds = start

    def now(self):
        return FakeTime(self.seconds)

    def advance(self, dt):
        self.seconds += dt


def make_behavior(timeout_sec=30.0):
    action_name = controller_action_name(ControllerNames.JOINT_QP)
    behavior = BaseActionBehavior("Test_Action", object, action_name, timeout_sec=timeout_sec)
    behavior.node = MagicMock()
    behavior.clock = FakeClock()
    behavior.node.get_clock.return_value = behavior.clock
    behavior.client = MagicMock()
    behavior.client.wait_for_server.return_value = True
    return behavior


def test_goal_rejected_returns_failure():
    behavior = make_behavior()
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=False)
    send_future.result.return_value = goal_handle
    behavior.client.send_goal_async.return_value = send_future

    behavior.send_action_goal(MagicMock())

    assert behavior.update() == py_trees.common.Status.FAILURE


def test_success_path_returns_success():
    behavior = make_behavior()
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    behavior.client.send_goal_async.return_value = send_future

    result_future = MagicMock()
    result_future.done.return_value = False
    goal_handle.get_result_async.return_value = result_future

    behavior.send_action_goal(MagicMock())
    assert behavior.update() == py_trees.common.Status.RUNNING  # goal accepted, awaiting result
    assert behavior.update() == py_trees.common.Status.RUNNING  # still waiting for result

    result_future.done.return_value = True
    result_future.result.return_value = MagicMock(status=GoalStatus.STATUS_SUCCEEDED)
    assert behavior.update() == py_trees.common.Status.SUCCESS


def test_failed_status_returns_failure():
    behavior = make_behavior()
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    behavior.client.send_goal_async.return_value = send_future

    result_future = MagicMock()
    result_future.done.return_value = False
    goal_handle.get_result_async.return_value = result_future

    behavior.send_action_goal(MagicMock())
    behavior.update()  # consumes goal acceptance, creates get_result_future

    result_future.done.return_value = True
    result_future.result.return_value = MagicMock(status=GoalStatus.STATUS_ABORTED)

    assert behavior.update() == py_trees.common.Status.FAILURE


def test_timeout_before_goal_accepted_arms_late_cancel():
    behavior = make_behavior(timeout_sec=5.0)
    send_future = MagicMock()
    send_future.done.return_value = False  # still waiting for the server
    behavior.client.send_goal_async.return_value = send_future

    behavior.send_action_goal(MagicMock())
    behavior.clock.advance(6.0)

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert behavior.goal_handle is None
    # The goal is already in flight: a done-callback must be armed so a late
    # acceptance still gets cancelled instead of executing a stale motion.
    send_future.add_done_callback.assert_called_once_with(
        behavior._cancel_late_accepted_goal
    )

    # Simulate the late acceptance: invoking the armed callback cancels the goal.
    late_handle = MagicMock(accepted=True)
    send_future.result.return_value = late_handle
    behavior._cancel_late_accepted_goal(send_future)
    late_handle.cancel_goal_async.assert_called_once()


def test_result_arriving_past_deadline_still_reports_success():
    behavior = make_behavior(timeout_sec=5.0)
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    behavior.client.send_goal_async.return_value = send_future

    result_future = MagicMock()
    result_future.done.return_value = False
    goal_handle.get_result_async.return_value = result_future

    behavior.send_action_goal(MagicMock())
    behavior.update()  # accept goal

    # Result completes, but the next tick lands after the deadline: the completed
    # result must win over the timeout -- the motion really succeeded.
    result_future.done.return_value = True
    result_future.result.return_value = MagicMock(status=GoalStatus.STATUS_SUCCEEDED)
    behavior.clock.advance(6.0)

    assert behavior.update() == py_trees.common.Status.SUCCESS
    goal_handle.cancel_goal_async.assert_not_called()


def test_timeout_after_goal_accepted_cancels_goal():
    behavior = make_behavior(timeout_sec=5.0)
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    behavior.client.send_goal_async.return_value = send_future

    result_future = MagicMock()
    result_future.done.return_value = False  # result never arrives
    goal_handle.get_result_async.return_value = result_future

    behavior.send_action_goal(MagicMock())
    behavior.update()  # accept goal, start waiting on result

    behavior.clock.advance(6.0)
    assert behavior.update() == py_trees.common.Status.FAILURE
    goal_handle.cancel_goal_async.assert_called_once()


def test_terminate_invalid_cancels_running_goal():
    behavior = make_behavior()
    goal_handle = MagicMock()
    behavior.goal_handle = goal_handle
    result_future = MagicMock()
    result_future.done.return_value = False
    behavior.get_result_future = result_future

    behavior.terminate(py_trees.common.Status.INVALID)

    goal_handle.cancel_goal_async.assert_called_once()
    assert behavior.send_goal_future is None
    assert behavior.get_result_future is None
    assert behavior.goal_handle is None
