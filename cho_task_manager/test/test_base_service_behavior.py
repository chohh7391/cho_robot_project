"""Unit tests for BaseServiceBehavior (client-side) response timeout handling."""
import py_trees
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
