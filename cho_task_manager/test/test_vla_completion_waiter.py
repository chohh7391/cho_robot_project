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

"""Unit tests for BaseServiceServerBehavior's timeout and VLACompletionWaiterBehavior."""
import py_trees
import pytest
from unittest.mock import MagicMock

from cho_task_manager.behaviors.service.base_service_server_behavior import (
    BaseServiceServerBehavior,
)
from cho_task_manager.behaviors.service.vla_completion_waiter import (
    VLACompletionWaiterBehavior,
)
from std_srvs.srv import Trigger


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


def make_behavior(cls=BaseServiceServerBehavior, timeout_sec=None, **kwargs):
    behavior = cls("Test_Waiter", Trigger, "/test_signal", timeout_sec=timeout_sec, **kwargs)
    behavior.node = MagicMock()
    behavior.clock = FakeClock()
    behavior.node.get_clock.return_value = behavior.clock
    return behavior


def _call(behavior):
    return behavior._service_callback(Trigger.Request(), Trigger.Response())


def test_running_until_signal_received():
    behavior = make_behavior()
    behavior.initialise()
    assert behavior.update() == py_trees.common.Status.RUNNING

    behavior.signal_received = True
    assert behavior.update() == py_trees.common.Status.SUCCESS


def test_no_timeout_stays_running_indefinitely():
    behavior = make_behavior(timeout_sec=None)
    behavior.initialise()
    behavior.clock.advance(10_000.0)
    assert behavior.update() == py_trees.common.Status.RUNNING


def test_timeout_returns_failure():
    behavior = make_behavior(timeout_sec=5.0)
    behavior.initialise()
    behavior.clock.advance(6.0)
    assert behavior.update() == py_trees.common.Status.FAILURE


def test_a_signal_while_waiting_is_accepted():
    behavior = make_behavior()
    behavior.initialise()

    response = _call(behavior)

    assert response.success is True
    assert behavior.update() == py_trees.common.Status.SUCCESS


def test_a_signal_before_the_wait_starts_is_refused():
    # The server exists from setup(), long before the tree reaches this leaf.
    behavior = make_behavior()

    response = _call(behavior)

    assert response.success is False
    assert 'not waiting' in response.message
    # And it is not banked for a later wait to find.
    behavior.initialise()
    assert behavior.update() == py_trees.common.Status.RUNNING


def test_a_signal_after_the_wait_ended_is_refused():
    behavior = make_behavior()
    behavior.initialise()
    _call(behavior)
    assert behavior.update() == py_trees.common.Status.SUCCESS
    behavior.terminate(py_trees.common.Status.SUCCESS)

    assert _call(behavior).success is False


def _vla_waiter(**kwargs):
    behavior = VLACompletionWaiterBehavior(controller='vla_controller', **kwargs)
    behavior.node = MagicMock()
    behavior.node.get_clock.return_value = FakeClock()
    behavior.trigger_success_client = MagicMock()
    behavior.trigger_success_client.service_is_ready.return_value = True
    return behavior


def test_vla_completion_waiter_defaults_to_unbounded_wait():
    # A default deadline would shut the tree down mid-manipulation (no failure branch
    # cancels the VLA goal), so the waiter must wait indefinitely unless opted in.
    behavior = VLACompletionWaiterBehavior(controller='vla_controller')
    assert behavior.timeout_sec is None


def test_vla_completion_waiter_needs_the_robots_vla_controller():
    with pytest.raises(ValueError, match='VLA controller'):
        VLACompletionWaiterBehavior()
    assert VLACompletionWaiterBehavior(controller='left_vla_mit_controller').service_name == (
        '/left_vla_mit_controller/vla/notify_completion')


def test_vla_completion_waiter_triggers_success_reset_on_signal():
    behavior = _vla_waiter()
    behavior.initialise()

    response = _call(behavior)

    behavior.trigger_success_client.call_async.assert_called_once()
    assert response.success is True


def test_vla_completion_waiter_ignores_a_completion_nobody_waits_for():
    # Answering success here used to also reset the VLA controller through
    # the success service, for a goal no step of the tree was waiting on.
    behavior = _vla_waiter()

    response = _call(behavior)

    assert response.success is False
    behavior.trigger_success_client.call_async.assert_not_called()
