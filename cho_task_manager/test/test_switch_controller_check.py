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

"""SwitchControllerServiceBehavior checks its controllers before and after the switch.

The exclusive switch is BEST_EFFORT: switching to a controller the bringup never
loaded returns ok after deactivating the one that held the arm. In the Franka
MuJoCo velocity check that left the arm with no controller for the 3 s VLA hold
while the switch reported SUCCESS. The behaviour now lists the controllers first
and refuses such a switch with nothing deactivated, and lists them again after an
ok switch to catch an activation that failed anyway.
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


def _pending():
    return _future(done=False)


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

    def start(self, lists=(), switch=None):
        """Initialise with the responses each call will get, in order."""
        self.list_client.call_async.reset_mock()
        self.switch_client.call_async.reset_mock()
        self.list_client.call_async.side_effect = list(lists)
        self.switch_client.call_async.side_effect = [switch] if switch else []
        self.behaviour.initialise()

    def ticks(self, n):
        return [self.behaviour.update() for _ in range(n)]

    def switch_sent(self):
        return self.switch_client.call_async.call_count

    def lists_sent(self):
        return self.list_client.call_async.call_count

    def errors(self):
        return ' '.join(
            str(call.args[0]) for call in self.node.get_logger.return_value.error.call_args_list)


def test_setup_makes_a_list_controllers_client_beside_the_switch_client():
    rig = Rig()
    (switch_type, switch_name), _ = rig.node.create_client.call_args_list[0]
    (list_type, list_name), _ = rig.node.create_client.call_args_list[1]
    assert (switch_type, switch_name) == (SwitchController, SWITCH_CONTROLLER_SERVICE)
    assert (list_type, list_name) == (ListControllers, LIST_CONTROLLERS_SERVICE)


# ---------------------------------------------------------------------------
# Before the switch
# ---------------------------------------------------------------------------

def test_the_controllers_are_listed_before_anything_is_switched():
    rig = Rig()
    rig.start(lists=[_pending()], switch=_switched())

    assert rig.lists_sent() == 1
    assert isinstance(rig.list_client.call_async.call_args.args[0], ListControllers.Request)
    assert rig.ticks(2) == [RUNNING, RUNNING]
    assert rig.switch_sent() == 0


def test_a_controller_that_is_not_loaded_is_refused_with_nothing_deactivated():
    """The MuJoCo velocity check: vla_controller was not loaded."""
    rig = Rig()
    rig.start(lists=[_listed(b='active', c='inactive')], switch=_switched())

    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0
    assert "'a': 'not loaded'" in rig.errors()
    assert 'nothing was deactivated' in rig.errors()


@pytest.mark.parametrize('state', ['unconfigured', 'finalized'])
def test_a_controller_that_cannot_be_activated_from_its_state_is_refused(state):
    rig = Rig()
    rig.start(lists=[_listed(a=state, b='active')], switch=_switched())

    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0
    assert f"'a': '{state}'" in rig.errors()


def test_every_controller_to_activate_has_to_be_activatable():
    rig = Rig(activate=['a', 'b'])
    rig.start(lists=[_listed(a='inactive', b='unconfigured')], switch=_switched())

    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0
    assert "'b': 'unconfigured'" in rig.errors()
    assert "'a'" not in rig.errors()


def test_no_switch_goes_out_when_list_controllers_is_unavailable():
    rig = Rig()
    rig.list_client.wait_for_service.return_value = False
    rig.start(lists=[], switch=_switched())

    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0
    assert LIST_CONTROLLERS_SERVICE in rig.errors()


def test_a_first_list_that_never_comes_times_out_with_nothing_switched():
    rig = Rig()
    pending = _pending()
    rig.start(lists=[pending], switch=_switched())

    assert rig.ticks(1) == [RUNNING]
    rig.clock.advance(rig.behaviour.response_timeout_sec + 1.0)
    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0
    rig.list_client.remove_pending_request.assert_called_once_with(pending)


def test_preempting_the_first_list_untracks_it_and_switches_nothing():
    rig = Rig()
    pending = _pending()
    rig.start(lists=[pending], switch=_switched())
    assert rig.ticks(1) == [RUNNING]

    rig.behaviour.terminate(py_trees.common.Status.INVALID)

    rig.list_client.remove_pending_request.assert_called_once_with(pending)
    assert rig.switch_sent() == 0


# ---------------------------------------------------------------------------
# The switch, and after it
# ---------------------------------------------------------------------------

def test_an_inactive_controller_is_switched_to_and_found_active_after():
    rig = Rig()
    rig.start(lists=[_listed(a='inactive', b='active'), _listed(a='active', b='inactive')],
              switch=_switched())

    assert rig.ticks(3) == [RUNNING, RUNNING, SUCCESS]
    request = rig.switch_client.call_async.call_args.args[0]
    assert request.activate_controllers == ['a']
    assert request.deactivate_controllers == ['b', 'c']
    assert rig.lists_sent() == 2


def test_an_already_active_controller_is_switched_to():
    # An exclusive switch to the controller already holding the arm: what a
    # mission re-run from the top does. It is not in its own deactivate list.
    rig = Rig()
    rig.start(lists=[_listed(a='active'), _listed(a='active')], switch=_switched())

    assert rig.ticks(3) == [RUNNING, RUNNING, SUCCESS]


def test_a_switch_that_left_the_controller_inactive_fails():
    """on_activate can still fail: the first list cannot know."""
    rig = Rig()
    rig.start(lists=[_listed(a='inactive'), _listed(a='inactive')], switch=_switched())

    assert rig.ticks(3) == [RUNNING, RUNNING, FAILURE]
    assert "left these controllers not active: {'a': 'inactive'}" in rig.errors()


def test_a_controller_unloaded_during_the_switch_fails_after():
    rig = Rig()
    rig.start(lists=[_listed(a='inactive'), _listed(b='inactive')], switch=_switched())

    assert rig.ticks(3) == [RUNNING, RUNNING, FAILURE]
    assert "'a': 'not loaded'" in rig.errors()


def test_every_activated_controller_has_to_be_active_after():
    rig = Rig(activate=['a', 'b'])
    rig.start(lists=[_listed(a='inactive', b='inactive'), _listed(a='active', b='inactive')],
              switch=_switched())

    assert rig.ticks(3) == [RUNNING, RUNNING, FAILURE]
    assert "{'b': 'inactive'}" in rig.errors()


def test_a_strict_switch_is_checked_too():
    rig = Rig(exclusive=False, deactivate=['b'])
    assert rig.behaviour.strict
    rig.start(lists=[_listed(c='inactive')], switch=_switched())

    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0


def test_a_refused_switch_fails_without_a_second_list():
    rig = Rig()
    rig.start(lists=[_listed(a='inactive')], switch=_switched(ok=False))

    assert rig.ticks(2) == [RUNNING, FAILURE]
    assert rig.lists_sent() == 1


def test_a_deactivate_only_switch_has_nothing_to_check():
    rig = Rig(activate=[], exclusive=False, deactivate=['a'])
    rig.start(lists=[], switch=_switched())

    assert rig.ticks(1) == [SUCCESS]
    assert rig.lists_sent() == 0


def test_a_switch_response_that_never_comes_times_out_on_the_switch_client():
    rig = Rig()
    pending = _pending()
    rig.start(lists=[_listed(a='inactive')], switch=pending)

    assert rig.ticks(2) == [RUNNING, RUNNING]
    rig.clock.advance(rig.behaviour.response_timeout_sec + 1.0)
    assert rig.ticks(1) == [FAILURE]
    assert SWITCH_CONTROLLER_SERVICE in rig.errors()
    rig.switch_client.remove_pending_request.assert_called_once_with(pending)
    rig.list_client.remove_pending_request.assert_not_called()


def test_the_second_list_fails_the_run_when_list_controllers_is_gone():
    rig = Rig()
    rig.start(lists=[_listed(a='inactive')], switch=_switched())
    assert rig.ticks(1) == [RUNNING]

    rig.list_client.wait_for_service.return_value = False
    assert rig.ticks(1) == [FAILURE]
    assert LIST_CONTROLLERS_SERVICE in rig.errors()


def test_a_second_list_that_never_comes_times_out_on_its_own_client():
    rig = Rig()
    pending = _pending()
    rig.start(lists=[_listed(a='inactive'), pending], switch=_switched())

    assert rig.ticks(2) == [RUNNING, RUNNING]
    rig.clock.advance(rig.behaviour.response_timeout_sec + 1.0)
    assert rig.ticks(1) == [FAILURE]
    assert LIST_CONTROLLERS_SERVICE in rig.errors()
    rig.list_client.remove_pending_request.assert_called_once_with(pending)
    rig.switch_client.remove_pending_request.assert_not_called()


def test_preempting_the_switch_untracks_it_on_the_switch_client():
    rig = Rig()
    pending = _pending()
    rig.start(lists=[_listed(a='inactive')], switch=pending)
    assert rig.ticks(2) == [RUNNING, RUNNING]

    rig.behaviour.terminate(py_trees.common.Status.INVALID)

    rig.switch_client.remove_pending_request.assert_called_once_with(pending)
    rig.list_client.remove_pending_request.assert_not_called()


@pytest.mark.parametrize('second_list', [{'a': 'inactive'}, {'a': 'active'}])
def test_a_rerun_starts_again_with_the_first_list(second_list):
    rig = Rig()
    rig.start(lists=[_listed(a='inactive'), _listed(**second_list)], switch=_switched())
    rig.ticks(3)

    # A mission that re-runs from the top re-initialises the leaf: the run
    # lists the controllers again before it switches.
    rig.start(lists=[_listed(b='inactive')], switch=_switched())
    assert rig.lists_sent() == 1
    assert rig.ticks(1) == [FAILURE]
    assert rig.switch_sent() == 0
