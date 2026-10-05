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

"""cho_interfaces/CONTRACT.md's Frames table states what the registry stamps.

The table is what a reader checks a goal against, and it had drifted: it still
gave fr5/ur5e/openarm-single relative frames after the registry dropped them
for ''. Each row is compared with ``task_goal_frame()`` for the profiles it
names.
"""

from pathlib import Path
import re

from cho_robot_config import available_profiles, load_robot_config, task_goal_frame
import pytest

CONTRACT = Path(__file__).resolve().parents[2] / 'cho_interfaces' / 'CONTRACT.md'


def _frames_table():
    """{(robot, profile): (absolute, relative)} from the Frames table, cells unquoted."""
    if not CONTRACT.is_file():
        pytest.fail(f'{CONTRACT} is missing; this test reads it from the source tree')
    rows = {}
    for line in CONTRACT.read_text().splitlines():
        match = re.match(r'^\| `(\w+)`( [\w /]+)? \| ([^|]+) \| ([^|]+) \|', line)
        if not match:
            continue
        robot, profiles, absolute, relative = match.groups()
        cells = [cell.strip().strip('`') for cell in (absolute, relative)]
        for profile in (profiles or ' single').strip().split(' / '):
            rows[(robot, profile)] = tuple(
                cell.replace('<side>', profile) if cell != "''" else '' for cell in cells)
    return rows


def _registry_profiles():
    for robot in ('franka', 'fr5', 'ur5e', 'openarm'):
        for profile in available_profiles(robot):
            yield robot, profile


def test_the_table_has_a_row_for_every_registry_profile():
    assert set(_frames_table()) == set(_registry_profiles())


@pytest.mark.parametrize('robot,profile', sorted(_registry_profiles()))
def test_each_row_is_what_the_registry_stamps(robot, profile):
    config = load_robot_config(robot, profile)
    absolute, relative = _frames_table()[(robot, profile)]
    if config.get('supports_task', True) is False:
        assert (absolute, relative) == ('--', '--')
        return
    assert absolute == task_goal_frame(config, relative=False)
    assert relative == task_goal_frame(config, relative=True)
