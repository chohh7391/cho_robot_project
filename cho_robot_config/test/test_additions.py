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

"""A project's arm controllers, added to the registry through CHO_ROBOT_CONFIG_ADDITIONS."""

import os

from cho_robot_config import ADDITIONS_ENV, load_robot_config
import pytest
import yaml


def _core_additional_arm(robot_type, profile=None, monkeypatch=None):
    monkeypatch.delenv(ADDITIONS_ENV, raising=False)
    return load_robot_config(robot_type, profile)['controllers'].get('additional_arm') or []


def _additions(monkeypatch, tmp_path, *documents):
    paths = []
    for index, document in enumerate(documents):
        path = tmp_path / f'additions_{index}.yaml'
        path.write_text(yaml.safe_dump(document), encoding='utf-8')
        paths.append(str(path))
    monkeypatch.setenv(ADDITIONS_ENV, os.pathsep.join(paths))


def test_without_additions_the_registry_is_the_cores(monkeypatch):
    monkeypatch.setenv(ADDITIONS_ENV, '')
    unset = _core_additional_arm('franka', monkeypatch=monkeypatch)
    monkeypatch.setenv(ADDITIONS_ENV, '')
    assert load_robot_config('franka')['controllers']['additional_arm'] == unset


def test_added_controllers_come_after_the_cores(monkeypatch, tmp_path):
    core = _core_additional_arm('franka', monkeypatch=monkeypatch)
    _additions(monkeypatch, tmp_path, {'franka': {'additional_arm': ['sine_joint_controller']}})
    assert load_robot_config('franka')['controllers']['additional_arm'] == core + ['sine_joint_controller']
    # A robot the file does not name is left as it is.
    assert load_robot_config('fr5')['controllers'].get('additional_arm') == \
        (_core_additional_arm('fr5', monkeypatch=monkeypatch) or None)


def test_two_projects_stack_in_the_order_the_variable_lists_them(monkeypatch, tmp_path):
    core = _core_additional_arm('franka', monkeypatch=monkeypatch)
    _additions(monkeypatch, tmp_path,
               {'franka': {'additional_arm': ['first_controller']}},
               {'franka': {'additional_arm': ['second_controller']}})
    assert load_robot_config('franka')['controllers']['additional_arm'] == \
        core + ['first_controller', 'second_controller']


def test_a_profile_gets_only_what_is_added_to_it(monkeypatch, tmp_path):
    core_left = _core_additional_arm('openarm', 'left', monkeypatch=monkeypatch)
    core_single = _core_additional_arm('openarm', monkeypatch=monkeypatch)
    _additions(monkeypatch, tmp_path, {'openarm': {
        'additional_arm': ['wave_controller'],
        'profiles': {'left': {'additional_arm': ['left_wave_controller']}},
    }})
    assert load_robot_config('openarm', 'left')['controllers']['additional_arm'] == \
        core_left + ['left_wave_controller']
    assert load_robot_config('openarm')['controllers']['additional_arm'] == core_single + ['wave_controller']


def test_changing_the_variable_is_not_hidden_by_the_cache(monkeypatch, tmp_path):
    core = _core_additional_arm('franka', monkeypatch=monkeypatch)
    _additions(monkeypatch, tmp_path, {'franka': {'additional_arm': ['sine_joint_controller']}})
    assert 'sine_joint_controller' in load_robot_config('franka')['controllers']['additional_arm']
    monkeypatch.delenv(ADDITIONS_ENV)
    assert load_robot_config('franka')['controllers']['additional_arm'] == core


@pytest.mark.parametrize('documents,message', [
    ([{'frankaa': {'additional_arm': ['x']}}], 'robot types the registry has not'),
    ([{'franka': {'additional_arm': []}}], 'non-empty, unique list'),
    ([{'franka': {'additional_arm': ['x', 'x']}}], 'non-empty, unique list'),
    ([{'franka': {'additional_arm': 'x'}}], 'non-empty, unique list'),
    ([{'franka': {'hold': 'x'}}], 'takes additional_arm and profiles'),
    ([{'franka': {}}], 'takes additional_arm and profiles'),
    ([{'openarm': {'profiles': {'middle': {'additional_arm': ['middle_x']}}}}], 'has no profiles'),
    ([{'openarm': {'profiles': {'left': {'hold': 'left_x'}}}}], 'takes additional_arm only'),
    # A name the core already gives a role, or already lists.
    ([{'franka': {'additional_arm': ['joint_space_position_controller']}}], 'already names'),
    ([{'franka': {'additional_arm': ['task_space_ik_controller']}}], 'already names'),
    ([{'franka': {'additional_arm': ['x']}}, {'franka': {'additional_arm': ['x']}}], 'twice'),
    # The core's own rules still hold for what a project adds.
    ([{'franka': {'additional_arm': ['gripper_controller']}}], 'already names'),
    ([{'openarm': {'profiles': {'left': {'additional_arm': ['wave_controller']}}}}], 'left_'),
])
def test_an_addition_the_registry_cannot_take_is_refused(monkeypatch, tmp_path, documents, message):
    _additions(monkeypatch, tmp_path, *documents)
    robot_type = next(iter(documents[0]))
    profile = 'left' if robot_type == 'openarm' else None
    with pytest.raises(ValueError, match=message):
        load_robot_config(robot_type if robot_type != 'frankaa' else 'franka', profile)


def test_a_listed_file_that_is_not_there_is_refused(monkeypatch, tmp_path):
    monkeypatch.setenv(ADDITIONS_ENV, str(tmp_path / 'missing.yaml'))
    with pytest.raises(ValueError, match='not a file'):
        load_robot_config('franka')
