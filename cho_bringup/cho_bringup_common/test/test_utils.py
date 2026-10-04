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

"""The small helpers, and the search-path environment actions."""

import os

import ament_index_python.packages
from cho_bringup_common import (
    as_bool,
    load_package_utils,
    load_yaml,
    prepend_to_search_paths,
    unique_names,
)
from launch import LaunchContext
from launch.actions import AppendEnvironmentVariable
import pytest


@pytest.mark.parametrize('value', [True, 'true', 'True', '1', 'yes', 'on', 'ON'])
def test_as_bool_true(value):
    assert as_bool(value) is True


@pytest.mark.parametrize('value', [False, 'false', '0', 'no', 'off', '', 'config', 'ture'])
def test_as_bool_is_lenient_everything_else_is_false(value):
    assert as_bool(value) is False


def test_unique_names_keeps_first_occurrence_order():
    assert unique_names(['b', 'a', 'b', 'c', 'a']) == ['b', 'a', 'c']
    assert unique_names([]) == []


def test_load_yaml(tmp_path):
    path = tmp_path / 'x.yaml'
    path.write_text('a: [1, 2]\n')
    assert load_yaml(str(path)) == {'a': [1, 2]}
    with pytest.raises(FileNotFoundError, match='nope.yaml'):
        load_yaml(str(tmp_path / 'nope.yaml'))


def test_load_package_utils_loads_lib_package_utils(tmp_path, monkeypatch):
    share = tmp_path / 'share' / 'cho_bringup_fake'
    share.mkdir(parents=True)
    utils = tmp_path / 'lib' / 'cho_bringup_fake' / 'utils'
    utils.mkdir(parents=True)
    (utils / 'launch_utils.py').write_text('ANSWER = 42\n')
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda package: str(tmp_path / 'share' / package))
    assert load_package_utils('cho_bringup_fake').ANSWER == 42


def test_search_paths_are_prepended_by_launch_actions(monkeypatch):
    monkeypatch.setenv('CHO_TEST_PATH_A', '/existing')
    monkeypatch.delenv('CHO_TEST_PATH_B', raising=False)
    actions = prepend_to_search_paths(['CHO_TEST_PATH_A', 'CHO_TEST_PATH_B'], '/ours')
    assert all(isinstance(action, AppendEnvironmentVariable) for action in actions)
    context = LaunchContext()
    for action in actions:
        action.execute(context)
    assert os.environ['CHO_TEST_PATH_A'] == os.pathsep.join(['/ours', '/existing'])
    assert os.environ['CHO_TEST_PATH_B'] == '/ours'
