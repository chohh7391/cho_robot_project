"""Unit tests for BaseServiceBehavior (client-side) response timeout handling."""
import py_trees
import pytest
from unittest.mock import MagicMock

from cho_task_manager.behaviors.service.base_service_behavior import BaseServiceBehavior


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


def make_behavior(response_timeout_sec=30.0):
    behavior = BaseServiceBehavior(
        "Test_Service", object, "/test_service", response_timeout_sec=response_timeout_sec
    )
    behavior.node = MagicMock()
    behavior.clock = FakeClock()
    behavior.node.get_clock.return_value = behavior.clock
    behavior.client = MagicMock()
    behavior.client.wait_for_service.return_value = True
    return behavior


def test_response_success_returns_success():
    behavior = make_behavior()
    future = MagicMock()
    future.done.return_value = False
    behavior.client.call_async.return_value = future

    behavior.send_service_request(object())
    assert behavior.update() == py_trees.common.Status.RUNNING

    future.done.return_value = True
    future.result.return_value = MagicMock()
    assert behavior.update() == py_trees.common.Status.SUCCESS


def test_service_call_exception_returns_failure():
    behavior = make_behavior()
    future = MagicMock()
    future.done.return_value = True
    future.result.side_effect = RuntimeError("boom")
    behavior.client.call_async.return_value = future

    behavior.send_service_request(object())
    assert behavior.update() == py_trees.common.Status.FAILURE


def test_response_timeout_returns_failure():
    behavior = make_behavior(response_timeout_sec=5.0)
    future = MagicMock()
    future.done.return_value = False
    behavior.client.call_async.return_value = future

    behavior.send_service_request(object())
    behavior.clock.advance(6.0)

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert behavior.future is None


def test_server_unavailable_at_request_time_returns_failure():
    behavior = make_behavior()
    behavior.client.wait_for_service.return_value = False

    behavior.send_service_request(object())

    assert behavior.update() == py_trees.common.Status.FAILURE


def test_a_response_already_in_wins_over_a_passed_deadline():
    """The tick that sees the response may land after the deadline.

    Checking the deadline first turned a switch that had succeeded into a
    FAILURE -- and the mission into a safe abort. BaseActionBehavior drains
    its futures first; so does this.
    """
    behavior = make_behavior(response_timeout_sec=5.0)
    future = MagicMock()
    future.done.return_value = False
    behavior.client.call_async.return_value = future

    behavior.send_service_request(object())
    behavior.clock.advance(6.0)
    future.done.return_value = True
    future.result.return_value = MagicMock()

    assert behavior.update() == py_trees.common.Status.SUCCESS
    behavior.client.remove_pending_request.assert_not_called()


# ---------------------------------------------------------------------------
# SwitchControllerServiceBehavior's own deadline
# ---------------------------------------------------------------------------

def _switch(**kwargs):
    from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
    return SwitchControllerServiceBehavior(
        name='Switch', activate=['a'], exclusive_controllers=['a', 'b'], **kwargs)


def test_a_fractional_switch_timeout_reaches_the_request():
    # Duration(sec=2.5) raises: the seconds have to be split.
    timeout = _switch(switch_timeout_sec=2.5).make_request().timeout
    assert (timeout.sec, timeout.nanosec) == (2, 500_000_000)


def test_the_default_switch_timeout_is_unchanged():
    timeout = _switch().make_request().timeout
    assert (timeout.sec, timeout.nanosec) == (2, 0)


def test_the_switch_timeout_no_longer_shadows_the_service_wait():
    # The base class's timeout_sec is how long setup() waits for the service.
    # The switch used to overwrite it with its own 2 s request timeout.
    behaviour = _switch(switch_timeout_sec=7.0)
    assert behaviour.timeout_sec == 3.0
    assert behaviour.switch_timeout_sec == 7.0


@pytest.mark.parametrize('value', [-1.0, float('nan'), float('inf'), True, '2'])
def test_a_meaningless_switch_timeout_is_refused(value):
    with pytest.raises(ValueError, match='switch_timeout_sec'):
        _switch(switch_timeout_sec=value)
