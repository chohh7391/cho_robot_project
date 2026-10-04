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

"""Shutting the task manager down while the tree is still driving the arm.

tree.shutdown() only calls each behaviour's shutdown(): it never stops the root,
so a RUNNING action leaf never reached terminate(INVALID) and its goal kept
running after Ctrl-C. These pin the replacement: stopping preempts the running
leaf, a goal still awaiting acceptance is cancelled once accepted -- also when
the tree had already ended and the cancel was armed on its last tick -- a
second Ctrl-C cannot cut the preemption short, and the node is destroyed
exactly once.
"""

import signal
from types import SimpleNamespace
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
FAILURE = py_trees.common.Status.FAILURE
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


class _GivingUpLeaf(_GoalLeaf):
    """Sends its goal and fails on the same tick, before the server answers.

    What a sweep leaf does when its deadline lands right after a waypoint goal
    went out: it ends FAILURE with a cancel armed for the acceptance.
    """

    def update(self):
        return FAILURE


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

    def shutdown(self, timeout_sec=None):
        self.shut_down = True


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
    # Nothing outstanding: no spinning at all.
    tree = _tree(py_trees.behaviours.Success('Done'))
    tree.tick()
    executor = MagicMock()
    node = MagicMock()
    tree.node = node

    shutdown_task_manager(tree, executor, node)

    executor.spin_once.assert_not_called()
    executor.shutdown.assert_called_once()
    node.destroy_node.assert_called_once()
    task_manager_node.rclpy.try_shutdown.assert_called_once()


def test_a_cancel_armed_on_the_last_tick_goes_out_before_the_node_is_destroyed(
        no_rclpy_shutdown):
    # The tree ENDED -- nothing to preempt -- but its last leaf failed with its
    # goal still awaiting acceptance. Destroying the node without spinning used
    # to drop that cancel, and the goal ran once the server accepted it.
    send = Future()
    leaf = _GivingUpLeaf('Sweep', send)
    tree = _tree(leaf)
    tree.tick()
    assert tree.root.status == FAILURE

    late = _accepted_handle()
    executor = _AcceptingExecutor(send, late)
    node = MagicMock()
    tree.node = node
    shutdown_task_manager(tree, executor, node, flush_sec=1.0)

    assert executor.spins > 0
    late.cancel_goal_async.assert_called_once()
    node.destroy_node.assert_called_once()


def test_the_flush_is_bounded_when_the_server_never_answers(no_rclpy_shutdown):
    send = Future()     # never resolves
    leaf = _GivingUpLeaf('Sweep', send)
    tree = _tree(leaf)
    tree.tick()
    executor = MagicMock()
    node = MagicMock()
    tree.node = node

    shutdown_task_manager(tree, executor, node, flush_sec=0.1)

    assert executor.spin_once.call_count > 0
    assert 'may still be running' in node.get_logger().error.call_args[0][0]
    node.destroy_node.assert_called_once()


class _InterruptedLeaf(_GoalLeaf):
    """Ctrl-C lands while this leaf is being terminated."""

    def terminate(self, new_status):
        if new_status == INVALID:
            signal.raise_signal(signal.SIGINT)
        super().terminate(new_status)


def test_a_second_ctrl_c_during_the_preemption_still_terminates_every_leaf(
        no_rclpy_shutdown):
    # tree.stop() terminates the running leaves one after another. A Ctrl-C in
    # the middle used to escape it and be swallowed, so every leaf after the
    # interrupted one kept its goal running.
    first, second = _accepted_handle(), _accepted_handle()
    sends = [Future(), Future()]
    sends[0].set_result(first)
    sends[1].set_result(second)
    interrupted = _InterruptedLeaf('Left', sends[0])
    other = _GoalLeaf('Right', sends[1])
    tree = StoppableBehaviourTree(root=py_trees.composites.Parallel(
        'Root', policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[interrupted, other]))
    tree.tick()
    assert tree.root.status == RUNNING
    executor = MagicMock()
    node = MagicMock()
    tree.node = node

    shutdown_task_manager(tree, executor, node, flush_sec=0.1)

    first.cancel_goal_async.assert_called_once()
    second.cancel_goal_async.assert_called_once()
    node.destroy_node.assert_called_once()


class _ResolvingExecutor:
    """Spinning delivers a pending acceptance; optionally Ctrl-C on the first spin."""

    def __init__(self, send_future, goal_handle, interrupt_first=False):
        self.send_future = send_future
        self.goal_handle = goal_handle
        self.interrupt_first = interrupt_first
        self.spins = 0

    def spin_once(self, timeout_sec=None):
        self.spins += 1
        if self.interrupt_first and self.spins == 1:
            signal.raise_signal(signal.SIGINT)
            return
        if not self.send_future.done():
            self.send_future.set_result(self.goal_handle)

    def shutdown(self, timeout_sec=None):
        pass


def _tree_with_a_goal_awaiting_acceptance(interrupted_sibling=False):
    """A running tree whose second leaf's goal the server has not answered yet."""
    accepted = _accepted_handle()
    sent = Future()
    sent.set_result(accepted)
    first = (_InterruptedLeaf if interrupted_sibling else _GoalLeaf)('Left', sent)
    pending = Future()
    tree = StoppableBehaviourTree(root=py_trees.composites.Parallel(
        'Root', policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[first, _GoalLeaf('Right', pending)]))
    tree.tick()
    assert tree.root.status == RUNNING
    tree.node = MagicMock()
    return tree, pending


def test_a_ctrl_c_held_during_the_preemption_does_not_skip_the_flush(no_rclpy_shutdown):
    # The held interrupt used to be re-raised right after tree.stop() and
    # swallowed, so the cancel armed for the goal still awaiting acceptance
    # never went out: the server accepted it and ran it after the node was gone.
    tree, pending = _tree_with_a_goal_awaiting_acceptance(interrupted_sibling=True)
    late = _accepted_handle()
    executor = _ResolvingExecutor(pending, late)

    shutdown_task_manager(tree, executor, tree.node, flush_sec=1.0)

    assert executor.spins > 0
    late.cancel_goal_async.assert_called_once()
    tree.node.destroy_node.assert_called_once()


def test_a_ctrl_c_during_the_flush_does_not_cut_it_short(no_rclpy_shutdown):
    tree, pending = _tree_with_a_goal_awaiting_acceptance()
    late = _accepted_handle()
    executor = _ResolvingExecutor(pending, late, interrupt_first=True)

    shutdown_task_manager(tree, executor, tree.node, flush_sec=1.0)

    assert executor.spins >= 2
    late.cancel_goal_async.assert_called_once()


def test_a_ctrl_c_before_the_handlers_are_installed_still_stops_the_tree(
        monkeypatch, no_rclpy_shutdown):
    # It used to escape the context manager before tree.stop() and be
    # swallowed: nothing was preempted and nothing cancelled.
    real_signal = signal.signal
    calls = {'n': 0}

    def interrupted_once(signum, handler):
        calls['n'] += 1
        if calls['n'] == 1:
            raise KeyboardInterrupt
        return real_signal(signum, handler)

    monkeypatch.setattr(task_manager_node.signal, 'signal', interrupted_once)
    handle = _accepted_handle()
    sent = Future()
    sent.set_result(handle)
    leaf = _GoalLeaf('Move', sent)
    tree = _tree(leaf)
    tree.tick()
    node = MagicMock()
    tree.node = node

    shutdown_task_manager(tree, MagicMock(), node, flush_sec=0.1)

    handle.cancel_goal_async.assert_called_once()
    assert leaf.status == INVALID
    node.destroy_node.assert_called_once()


def _parameters(**values):
    defaults = dict(task='pick_place', robot_type='franka', arm='single')

    def get_parameter(name):
        value = values.get(name, defaults.get(name))
        return SimpleNamespace(get_parameter_value=lambda: SimpleNamespace(
            string_value=value if isinstance(value, str) else '',
            bool_value=False, double_value=0.0, double_array_value=[0.0, 0.0, 0.0]))
    return get_parameter


def test_ctrl_c_during_setup_exits_cleanly(monkeypatch):
    # Setup waits seconds per action server, so this is where Ctrl-C usually
    # lands. It used to escape main() as a traceback.
    node = MagicMock()
    node.get_parameter.side_effect = _parameters()
    monkeypatch.setattr(task_manager_node.rclpy, 'init', MagicMock())
    monkeypatch.setattr(task_manager_node.rclpy, 'create_node', lambda _name: node)
    monkeypatch.setattr(task_manager_node.rclpy, 'try_shutdown', MagicMock())
    monkeypatch.setattr(task_manager_node.signal, 'signal', MagicMock())
    monkeypatch.setattr(task_manager_node, 'build_task_tree',
                        lambda _task, _config: py_trees.behaviours.Success('Done'))

    def interrupted_setup(self, **_kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(StoppableBehaviourTree, 'setup', interrupted_setup)

    task_manager_node.main()

    node.destroy_node.assert_called_once()
    task_manager_node.rclpy.try_shutdown.assert_called_once()
    assert 'Interrupted during setup' in node.get_logger().error.call_args[0][0]


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
