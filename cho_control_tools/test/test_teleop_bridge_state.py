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

"""teleop_bridge's run state: a press toggles, kill wins, a held button is never a fresh press."""

import pytest

from cho_control_tools.teleop_bridge.state import RUNNING, START, STOP, STOPPED, RunState


TOGGLE = 'right_primary_click'
KILL = 'right_secondary_click'


def buttons(*down):
    return {TOGGLE: TOGGLE in down, KILL: KILL in down}


def started():
    run = RunState(TOGGLE, KILL)
    run.update(buttons())
    assert run.update(buttons(TOGGLE)) == START
    return run


def test_a_press_starts_and_the_next_press_stops():
    run = started()
    assert run.state == RUNNING
    assert run.update(buttons(TOGGLE)) is None
    assert run.update(buttons()) is None
    assert run.update(buttons(TOGGLE)) == STOP
    assert run.state == STOPPED


def test_a_button_down_at_start_does_not_start():
    run = RunState(TOGGLE, KILL)
    assert run.update(buttons(TOGGLE)) is None
    assert run.state == STOPPED
    run.update(buttons())
    assert run.update(buttons(TOGGLE)) == START


def test_kill_stops_and_wins_over_the_toggle():
    run = started()
    run.update(buttons())
    assert run.update(buttons(TOGGLE, KILL)) == STOP
    assert run.update(buttons()) is None
    assert run.state == STOPPED


def test_kill_while_stopped_does_nothing():
    run = RunState(TOGGLE, KILL)
    assert run.update(buttons(KILL)) is None


def test_after_an_input_gap_a_held_toggle_needs_a_release():
    run = started()
    run.update(buttons())
    assert run.update(None) is None
    assert run.state == RUNNING
    assert run.update(buttons(TOGGLE)) is None
    run.update(buttons())
    assert run.update(buttons(TOGGLE)) == STOP


def test_stop_reports_only_a_change():
    run = started()
    assert run.stop() == STOP
    assert run.stop() is None


@pytest.mark.parametrize('toggle,kill', [('', KILL), (TOGGLE, TOGGLE)])
def test_two_different_buttons_are_required(toggle, kill):
    with pytest.raises(ValueError):
        RunState(toggle, kill)
