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

"""The project's task, checked as cho_task_manager checks its own.

Run against the installed package (colcon test): the task is found through the
entry point setup.py declares, not imported by hand.
"""

import os

import pytest
import yaml

from cho_task_manager.tasks import available_tasks
from cho_task_manager.testing import goal_leaves, goal_problems, registry_tree

TASK = 'example_joint_wave'
EXAMPLE_PARAMS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              'config', 'joint_wave.yaml')


def test_the_task_is_registered_for_franka_only():
    assert TASK in available_tasks('franka')
    assert TASK not in available_tasks('ur5e')


@pytest.mark.parametrize('task_params', [{}, yaml.safe_load(open(EXAMPLE_PARAMS))])
def test_every_goal_names_the_registry_frame_and_joints(task_params):
    tree, config = registry_tree(TASK, 'franka', task_params=task_params)
    assert goal_problems(tree, config) == []


def test_the_example_params_wave_as_often_as_they_say():
    params = yaml.safe_load(open(EXAMPLE_PARAMS))
    tree, _config = registry_tree(TASK, 'franka', task_params=params)
    assert len(goal_leaves(tree)) == 1 + 2 * params['repeats']


@pytest.mark.parametrize('task_params,message', [
    ({'amplitud': 0.1}, "does not take \\['amplitud'\\]"),
    ({'joint': 8}, 'joint must be 1..7'),
    ({'amplitude': 1.0}, 'amplitude must be in'),
    ({'repeats': 0}, 'repeats must be an integer'),
])
def test_a_parameter_it_cannot_run_is_refused_before_anything_moves(task_params, message):
    with pytest.raises(ValueError, match=message):
        registry_tree(TASK, 'franka', task_params=task_params)
