"""Unit tests for BaseServiceServerBehavior's timeout and VLACompletionWaiterBehavior."""
import py_trees
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


def test_vla_completion_waiter_defaults_to_unbounded_wait():
    # A default deadline would shut the tree down mid-manipulation (no failure branch
    # cancels the VLA goal), so the waiter must wait indefinitely unless opted in.
    behavior = VLACompletionWaiterBehavior()
    assert behavior.timeout_sec is None


def test_vla_completion_waiter_triggers_success_reset_on_signal():
    behavior = VLACompletionWaiterBehavior()
    behavior.node = MagicMock()
    behavior.trigger_success_client = MagicMock()
    behavior.trigger_success_client.service_is_ready.return_value = True

    response = Trigger.Response()
    behavior.fill_response(Trigger.Request(), response)

    behavior.trigger_success_client.call_async.assert_called_once()
    assert response.success is True
