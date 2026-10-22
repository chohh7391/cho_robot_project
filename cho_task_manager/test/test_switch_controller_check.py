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

"""SwitchControllerServiceBehavior succeeds only once its controllers are active.

The exclusive switch is BEST_EFFORT: switching to a controller the bringup never
loaded returns ok after deactivating the one that held the arm. In the Franka
MuJoCo velocity check that left the arm with no controller for the 3 s hold while
the switch reported SUCCESS.
"""

from unittest.mock import MagicMock

import py_trees
import pytest
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController

from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.controller_names import (
    LIST_CONTROLLERS_SERVICE,
    SWITCH_CONTROLLER_SERVICE,
)

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds
        self.nanoseconds = int(round(seconds * 1e9))

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds / 1e9)

    def __gt__(self, other):
        return self.seconds > other.seconds


class FakeClock:
    def __init__(self, start=100.0):
        self.seconds = start

    def now(self):
        return FakeTime(self.seconds)

    def advance(self, dt):
        self.seconds += dt


def _future(result=None, done=True):
    future = MagicMock()
    future.done.return_value = done
    future.result.return_value = result
    return future


def _switched(ok=True):
    return _future(SwitchController.Response(ok=ok))


def _listed(**states):
    return _future(ListControllers.Response(controller=[
        ControllerState(name=name, state=state) for name, state in states.items()]))


class Rig:
    """A switch behaviour set up on a mock node, one mock client per service."""

    def __init__(self, **kwargs):
        kwargs.setdefault('activate', ['a'])
        if kwargs.get('exclusive', True):
            kwargs.setdefault('exclusive_controllers', ['a', 'b', 'c'])
        self.node = MagicMock()
        self.clock = FakeClock()
        self.node.get_clock.return_value = self.clock
        self.switch_client = MagicMock()
        self.list_client = MagicMock()
        self.node.create_client.side_effect = [self.switch_client, self.list_client]
        self.behaviour = SwitchControllerServiceBehavior(name='Switch', **kwargs)
        self.behaviour.setup(node=self.node)

    def start(self, switch_future):
        self.switch_client.call_async.return_value = switch_future
        self.behaviour.initialise()

    def errors(self):
        return ' '.join(
            str(call.args[0]) for call in self.node.get_logger.return_value.error.call_args_list)


def test_setup_makes_a_list_controllers_client_beside_the_switch_client():
    rig = Rig()
    (switch_type, switch_name), _ = rig.node.create_client.call_args_list[0]
    (list_type, list_name), _ = rig.node.create_client.call_args_list[1]
    assert (switch_type, switch_name) == (SwitchController, SWITCH_CONTROLLER_SERVICE)
    assert (list_type, list_name) == (ListControllers, LIST_CONTROLLERS_SERVICE)


def test_an_ok_switch_keeps_running_until_the_controllers_are_listed():
    rig = Rig()
    rig.list_client.call_async.return_value = _future(done=False)
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    request = rig.list_client.call_async.call_args.args[0]
    assert isinstance(request, ListControllers.Request)
    assert rig.behaviour.update() == RUNNING
    assert rig.list_client.call_async.call_count == 1


def test_an_ok_switch_succeeds_once_the_activated_controller_is_listed_active():
    rig = Rig()
    rig.list_client.call_async.return_value = _listed(a='active', b='inactive')
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    assert rig.behaviour.update() == SUCCESS


def test_an_ok_switch_to_a_controller_that_is_not_loaded_fails():
    """The MuJoCo velocity check: vla_controller was not loaded."""
    rig = Rig()
    rig.list_client.call_async.return_value = _listed(b='inactive', c='inactive')
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    assert rig.behaviour.update() == FAILURE
    assert "'a': 'not loaded'" in rig.errors()


def test_an_ok_switch_that_left_the_controller_inactive_fails():
    rig = Rig()
    rig.list_client.call_async.return_value = _listed(a='inactive')
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    assert rig.behaviour.update() == FAILURE
    assert "'a': 'inactive'" in rig.errors()


def test_every_activated_controller_has_to_be_active():
    rig = Rig(activate=['a', 'b'], exclusive_controllers=['a', 'b', 'c'])
    rig.list_client.call_async.return_value = _listed(a='active', b='unconfigured')
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    assert rig.behaviour.update() == FAILURE
    assert "'b': 'unconfigured'" in rig.errors()
    assert "'a': 'active'" not in rig.errors()


def test_a_strict_switch_is_checked_too():
    rig = Rig(exclusive=False, deactivate=['b'])
    assert rig.behaviour.strict
    rig.list_client.call_async.return_value = _listed(a='inactive', b='inactive')
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    assert rig.behaviour.update() == FAILURE


def test_a_refused_switch_fails_without_listing():
    rig = Rig()
    rig.start(_switched(ok=False))

    assert rig.behaviour.update() == FAILURE
    rig.list_client.call_async.assert_not_called()


def test_a_deactivate_only_switch_has_nothing_to_check():
    rig = Rig(activate=[], exclusive=False, deactivate=['a'])
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == SUCCESS
    rig.list_client.call_async.assert_not_called()


def test_the_check_fails_when_list_controllers_is_unavailable():
    rig = Rig()
    rig.list_client.wait_for_service.return_value = False
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == FAILURE
    assert LIST_CONTROLLERS_SERVICE in rig.errors()


def test_a_list_response_that_never_comes_times_out_on_its_own_client():
    rig = Rig()
    pending = _future(done=False)
    rig.list_client.call_async.return_value = pending
    rig.start(_switched(ok=True))

    assert rig.behaviour.update() == RUNNING
    rig.clock.advance(rig.behaviour.response_timeout_sec + 1.0)
    assert rig.behaviour.update() == FAILURE
    assert LIST_CONTROLLERS_SERVICE in rig.errors()
    rig.list_client.remove_pending_request.assert_called_once_with(pending)
    rig.switch_client.remove_pending_request.assert_not_called()


def test_preempting_the_check_untracks_the_list_request():
    rig = Rig()
    pending = _future(done=False)
    rig.list_client.call_async.return_value = pending
    rig.start(_switched(ok=True))
    assert rig.behaviour.update() == RUNNING

    rig.behaviour.terminate(py_trees.common.Status.INVALID)

    rig.list_client.remove_pending_request.assert_called_once_with(pending)
    rig.switch_client.remove_pending_request.assert_not_called()


@pytest.mark.parametrize('first_states', [{'a': 'inactive'}, {'a': 'active'}])
def test_a_rerun_starts_again_with_the_switch(first_states):
    rig = Rig()
    rig.list_client.call_async.return_value = _listed(**first_states)
    rig.start(_switched(ok=True))
    assert rig.behaviour.update() == RUNNING
    rig.behaviour.update()

    # A mission that re-runs from the top re-initialises the leaf: the new
    # response is the switch's, not a controller list.
    rig.start(_switched(ok=False))
    assert rig.switch_client.call_async.call_count == 2
    assert rig.behaviour.update() == FAILURE
    assert rig.list_client.call_async.call_count == 1
