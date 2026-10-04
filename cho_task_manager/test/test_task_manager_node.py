"""Shutting the task manager down while the tree is still driving the arm.

tree.shutdown() only calls each behaviour's shutdown(): it never stops the root,
so a RUNNING action leaf never reached terminate(INVALID) and its goal kept
running after Ctrl-C. These pin the replacement: stopping preempts the running
leaf, a goal still awaiting acceptance is cancelled once accepted, and the node
is destroyed exactly once.
"""

from unittest.mock import MagicMock

import py_trees
import pytest
from rclpy.clock import Clock
from rclpy.task import Future

from cho_task_manager import task_manager_node
from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.task_manager_node import StoppableBehaviourTree, shutdown_task_manager
from cho_task_manager.utils.controller_names import ControllerNames, controller_action_name

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
INVALID = py_trees.common.Status.INVALID


class _GoalLeaf(BaseActionBehavior):
    """An action leaf on a fake client whose send_goal_async returns *send_future*."""

    def __init__(self, name, send_future):
        super().__init__(
            name, object, controller_action_name(ControllerNames.JOINT_QP, 'joint_space'))
        self.node = MagicMock()
        self.node.get_clock.return_value = Clock()
        self.client = MagicMock()
        self.client.wait_for_server.return_value = True
        self.client.send_goal_async.return_value = send_future

    def initialise(self):
        self.send_action_goal(MagicMock())


class _AcceptingExecutor:
    """Spinning delivers the server's acceptance, as the real executor would."""

    def __init__(self, send_future, goal_handle):
        self.send_future = send_future
        self.goal_handle = goal_handle
        self.spins = 0

    def spin_once(self, timeout_sec=None):
        self.spins += 1
        if not self.send_future.done():
            self.send_future.set_result(self.goal_handle)


def _tree(*children):
    return StoppableBehaviourTree(
        root=py_trees.composites.Sequence('Root', memory=True, children=list(children)))


def _accepted_handle():
    handle = MagicMock(accepted=True)
    handle.get_result_async.return_value = Future()  # the result never arrives
    return handle


@pytest.fixture
def no_rclpy_shutdown(monkeypatch):
    monkeypatch.setattr(task_manager_node.rclpy, 'try_shutdown', MagicMock())


def test_stop_cancels_the_goal_of_a_running_leaf():
    handle = _accepted_handle()
    send = Future()
    send.set_result(handle)
    leaf = _GoalLeaf('Move', send)
    tree = _tree(leaf)
    tree.tick()  # initialise() sends the goal, update() takes the acceptance
    assert tree.root.status == RUNNING

    assert tree.stop() is True

    handle.cancel_goal_async.assert_called_once()
    assert leaf.status == INVALID


def test_stop_on_a_finished_tree_preempts_nothing():
    tree = _tree(py_trees.behaviours.Success('Done'))
    tree.tick()
    assert tree.root.status == SUCCESS

    assert tree.stop() is False


def test_a_tick_dispatched_before_stop_does_not_restart_the_tree():
    handle = _accepted_handle()
    send = Future()
    send.set_result(handle)
    leaf = _GoalLeaf('Move', send)
    tree = _tree(leaf)
    tree.tick()
    tree.stop()

    tree.tick()  # a timer callback the executor had already dispatched

    assert leaf.client.send_goal_async.call_count == 1
    assert tree.root.status == INVALID


def test_shutdown_cancels_a_goal_accepted_while_it_waits(no_rclpy_shutdown):
    send = Future()  # acceptance still in flight when Ctrl-C lands
    leaf = _GoalLeaf('Move', send)
    tree = _tree(leaf)
    tree.tick()
    assert tree.root.status == RUNNING

    late = _accepted_handle()
    executor = _AcceptingExecutor(send, late)
    node = MagicMock()
    tree.node = node  # as after setup(): tree.shutdown() would destroy it by default
    shutdown_task_manager(tree, executor, node, flush_sec=0.05)

    assert executor.spins > 0
    late.cancel_goal_async.assert_called_once()
    node.destroy_node.assert_called_once()


def test_shutdown_after_the_task_finished_does_not_wait(no_rclpy_shutdown):
    tree = _tree(py_trees.behaviours.Success('Done'))
    tree.tick()
    executor = MagicMock()
    node = MagicMock()
    tree.node = node

    shutdown_task_manager(tree, executor, node)

    executor.spin_once.assert_not_called()
    node.destroy_node.assert_called_once()
    task_manager_node.rclpy.try_shutdown.assert_called_once()


def test_ctrl_c_is_left_to_python_so_the_cancels_can_still_be_sent(monkeypatch):
    # With rclpy's default handler, SIGINT shuts the context down before the
    # KeyboardInterrupt reaches main(), and every cancel sent after it fails.
    seen = {}

    class Stop(Exception):
        pass

    def record(**kwargs):
        seen.update(kwargs)
        raise Stop

    monkeypatch.setattr(task_manager_node.rclpy, 'init', record)
    with pytest.raises(Stop):
        task_manager_node.main()
    assert seen.get('signal_handler_options') == task_manager_node.SignalHandlerOptions.NO
