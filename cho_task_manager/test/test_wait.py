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

"""Settle waits run on the node's clock, not the wall clock.

``py_trees.timers.Timer`` counts wall time. Every wait in these trees is for
something physical to finish, and in simulation physics runs on sim time, so a
simulator slower than real time used to get less settling than a wait was
tuned for. These tests pin the leaf that replaced it and the places it is used.
"""

from pathlib import Path
from unittest.mock import MagicMock

import py_trees
import pytest
from rclpy.time import Time

from cho_task_manager.behaviors.wait import WaitBehavior
from cho_task_manager.subtrees import tare_ft_children

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
PACKAGE = Path(__file__).resolve().parents[1] / 'cho_task_manager'


class NodeClock:
    """A node clock that only moves when the test says so -- like /clock."""

    def __init__(self, seconds=100.0):
        self.seconds = seconds

    def now(self):
        return Time(nanoseconds=int(self.seconds * 1e9))


def _wait(duration_sec):
    behaviour = WaitBehavior('Settle', duration_sec)
    clock = NodeClock()
    node = MagicMock()
    node.get_clock.return_value = clock
    behaviour.setup(node=node)
    return behaviour, clock


def test_the_wait_runs_until_the_node_clock_has_moved_on():
    behaviour, clock = _wait(3.0)
    behaviour.initialise()

    clock.seconds += 2.9
    assert behaviour.update() == RUNNING
    clock.seconds += 0.1
    assert behaviour.update() == SUCCESS


def test_a_stalled_sim_clock_holds_the_wait_however_long_the_wall_clock_runs():
    # The whole point: no /clock, no physics, no settling -- so no SUCCESS.
    behaviour, clock = _wait(0.5)
    behaviour.initialise()
    for _ in range(5):
        assert behaviour.update() == RUNNING


def test_no_clock_yet_holds_the_wait_and_starts_it_from_the_first_reading():
    # Sim time before the first /clock reads 0. A deadline taken from that is
    # already in the past once /clock arrives at the simulator's real time, so
    # the wait used to succeed on the first tick after it -- with no settling.
    behaviour, clock = _wait(3.0)
    clock.seconds = 0.0
    behaviour.initialise()
    for _ in range(3):
        assert behaviour.update() == RUNNING

    clock.seconds = 1234.0        # /clock arrives, far past zero
    assert behaviour.update() == RUNNING
    clock.seconds += 2.9
    assert behaviour.update() == RUNNING
    clock.seconds += 0.1
    assert behaviour.update() == SUCCESS


def test_a_zero_length_wait_still_waits_for_the_clock():
    behaviour, clock = _wait(0.0)
    clock.seconds = 0.0
    behaviour.initialise()
    assert behaviour.update() == RUNNING
    clock.seconds = 5.0
    assert behaviour.update() == SUCCESS


def test_each_entry_waits_the_full_duration_again():
    behaviour, clock = _wait(1.0)
    behaviour.initialise()
    clock.seconds += 1.0
    assert behaviour.update() == SUCCESS
    behaviour.terminate(SUCCESS)

    behaviour.initialise()
    assert behaviour.update() == RUNNING


@pytest.mark.parametrize('value', [-0.1, float('nan'), float('inf'), None, True])
def test_a_meaningless_duration_is_refused(value):
    with pytest.raises(ValueError, match='duration_sec'):
        WaitBehavior('Settle', value)


def test_the_ft_settle_is_a_node_clock_wait():
    tare, settle = tare_ft_children(settle_sec=3.0)
    assert isinstance(settle, WaitBehavior)
    assert (settle.name, settle.duration_sec) == ('Wait_After_Tare', 3.0)


def test_no_tree_waits_on_the_wall_clock():
    offenders = [
        f'{path.relative_to(PACKAGE)}:{number}'
        for path in PACKAGE.rglob('*.py')
        for number, line in enumerate(path.read_text().splitlines(), 1)
        if 'timers.Timer(' in line
    ]
    assert offenders == []
