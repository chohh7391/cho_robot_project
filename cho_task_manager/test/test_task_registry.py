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

"""Task registration through the ``cho_task_manager.tasks`` entry point group.

This package's tasks come in the way a project package's do, so these run
against the installed package: an entry point exists only once setup.py has
been installed (colcon build).
"""

import importlib.util
import os

import pytest
import yaml

import cho_task_manager.tasks as tasks
from cho_task_manager.tasks import TaskRegistryError, build_task_tree, load_registry
from cho_task_manager.testing import goal_problems, registry_tree
from cho_task_manager.utils.task_params import load_task_params

#: The project template's package (templates/overlay_project/), which no build here includes.
TEMPLATE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        'templates', 'overlay_project', 'project_tasks')

#: This package's tasks and the robots each is built for.
CORE = {
    'peg_insert': {'franka'},
    'gear_mesh': {'franka'},
    'nut_thread': {'franka'},
    'pick_place': {'franka', 'ur5e'},
    'pick_place_position': {'franka'},
    'tag_reach': {'franka'},
    'controller_check_position': {'franka'},
    'controller_check_torque': {'franka', 'openarm'},
    'controller_check_velocity': {'franka'},
    'trajectory_replay': {'franka', 'fr5', 'ur5e', 'openarm'},
    'fjt_handover': {'fr5'},
    'multi_move': {'ur5e'},
    'mit_task_tuning': {'openarm'},
}


def test_this_packages_tasks_come_through_its_own_entry_points():
    assert set(tasks.package_tasks('cho_task_manager')) == set(CORE)
    registry = tasks.registered_tasks()
    assert {name: set(registry[name]) for name in CORE} == CORE


class _Entry:
    """An importlib.metadata EntryPoint as load_registry() reads one."""

    def __init__(self, name, target, package='my_project', value='my_project.tasks:TASK'):
        self.name = name
        self.value = value
        self.dist = type('Dist', (), {'name': package})()
        self._target = target

    def load(self):
        if isinstance(self._target, Exception):
            raise self._target
        return self._target


def _builder(robot_config):
    return robot_config


def test_two_packages_registering_one_name_refuse_to_start():
    entries = [_Entry('wave', {'franka': _builder}, 'first'), _Entry('wave', {'ur5e': _builder}, 'second')]
    with pytest.raises(TaskRegistryError, match="'wave' is registered by both first and second"):
        load_registry(entries)


@pytest.mark.parametrize('target', [None, {}, {'franka': 'build'}, {'': _builder}, [_builder], _builder])
def test_a_registration_that_is_not_a_mapping_of_builders_is_refused(target):
    with pytest.raises(TaskRegistryError, match='not a non-empty mapping'):
        load_registry([_Entry('wave', target)])


def test_a_registration_that_does_not_load_names_its_package():
    error = ModuleNotFoundError("No module named 'my_project.tasks'")
    with pytest.raises(TaskRegistryError, match=r"'wave' of my_project does not load \(my_project.tasks:TASK\)"):
        load_registry([_Entry('wave', error)])


def test_a_refused_registration_is_a_value_error_the_node_reports():
    # The node ends on a ValueError from build_task_tree() with its message.
    assert issubclass(TaskRegistryError, ValueError)


def test_every_builder_finds_its_task_params(monkeypatch):
    seen = []
    monkeypatch.setattr(tasks, 'registered_tasks', lambda: {'wave': {'franka': seen.append}})
    build_task_tree('wave', {'robot_type': 'franka'})
    build_task_tree('wave', {'robot_type': 'franka', 'task_params': {'amplitude': 0.1}})
    assert [config['task_params'] for config in seen] == [{}, {'amplitude': 0.1}]


def test_the_project_template_still_builds_on_this_package(monkeypatch):
    # Its own tests first run in a project's copy (COLCON_IGNORE keeps it out of
    # this workspace), so a change here that breaks what it uses fails this.
    spec = importlib.util.spec_from_file_location(
        'template_joint_wave', os.path.join(TEMPLATE, 'project_tasks', 'joint_wave.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entry = _Entry(module.TASK_NAME, module.TASK, 'project_tasks', 'project_tasks.joint_wave:TASK')
    monkeypatch.setattr(tasks, 'registered_tasks', lambda: load_registry([entry]))
    with open(os.path.join(TEMPLATE, 'config', 'joint_wave.yaml'), encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    tree, config = registry_tree(module.TASK_NAME, 'franka', task_params=params)
    assert goal_problems(tree, config) == []


def test_a_task_of_another_robot_is_not_available(monkeypatch):
    monkeypatch.setattr(tasks, 'registered_tasks', lambda: {'wave': {'franka': _builder}})
    assert tasks.available_tasks('franka') == ['wave']
    assert tasks.available_tasks('ur5e') == []
    with pytest.raises(ValueError, match="No tasks registered for robot_type 'ur5e'"):
        build_task_tree('wave', {'robot_type': 'ur5e'})


def test_task_params_are_the_files_mapping(tmp_path):
    path = tmp_path / 'wave.yaml'
    path.write_text('amplitude: 0.1\njoints: [6]\n')
    assert load_task_params(str(path)) == {'amplitude': 0.1, 'joints': [6]}
    empty = tmp_path / 'empty.yaml'
    empty.write_text('')
    assert load_task_params(str(empty)) == {}
    assert load_task_params('') == {}


@pytest.mark.parametrize('text,message', [
    ('- 1\n- 2\n', 'holds a list, not a mapping'),
    ('amplitude: [0.1\n', 'is not YAML'),
])
def test_task_params_that_are_not_a_mapping_are_refused(tmp_path, text, message):
    path = tmp_path / 'wave.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match=message):
        load_task_params(str(path))


def test_a_missing_task_params_file_is_refused(tmp_path):
    with pytest.raises(ValueError, match='task_params .*missing.yaml'):
        load_task_params(str(tmp_path / 'missing.yaml'))
