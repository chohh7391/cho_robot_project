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

"""Robot facts live in cho_robot_config; the task manager only reads them.

The home poses, the controllers a leaf addresses and the extra Franka arm
controllers used to be written into the task manager -- as tree constants, as
Franka defaults in robot-independent leaves, and as a Franka-only list plus an
``if robot_type == 'franka'`` branch. These tests pin where each now comes
from, and that moving them changed no value a tree sends.
"""

from pathlib import Path

import pytest

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.action import (
    GripperActionBehavior,
    JointSpaceActionBehavior,
)
from cho_task_manager.subtrees import home_joint_state, home_subtree
from cho_task_manager.tasks import build_task_tree
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    load_robot_config,
    task_home_positions,
)

PACKAGE = Path(__file__).resolve().parents[1] / 'cho_task_manager'

#: What the trees hard-coded before. The registry entries must reproduce them.
FRANKA_TASK_HOME = [0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785]
UR5E_TASK_HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
OPENARM_TASK_HOME = [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0]


def _franka_registry_names():
    """Every controller name the Franka registry entry declares."""
    registry = load_registry_config('franka')
    controllers = registry['controllers']
    names = {value for key, value in controllers.items()
             if key not in ('hold_by_control_mode', 'additional_arm') and value}
    names |= set(controllers['hold_by_control_mode'].values())
    names |= set(controllers['additional_arm'])
    return names


def test_controller_names_is_franka_registry_names_only():
    # ControllerNames stays as readable constants for the Franka trees, and it
    # must not name anything franka.yaml does not declare.
    assert {member.value for member in ControllerNames} <= _franka_registry_names()


@pytest.mark.parametrize('robot_type,expected', [
    ('franka', FRANKA_TASK_HOME), ('ur5e', UR5E_TASK_HOME), ('openarm', OPENARM_TASK_HOME),
])
def test_the_task_home_is_the_registrys(robot_type, expected):
    config = load_robot_config(robot_type)
    assert task_home_positions(config) == expected
    assert list(home_joint_state(config).position) == expected


def _go_home_targets(tree):
    return [list(node.target_joints.position) for node in tree.iterate()
            if isinstance(node, JointSpaceActionBehavior) and node.name.startswith('Go_Home')]


@pytest.mark.parametrize('robot_type,task,expected', [
    ('franka', 'pick_place', FRANKA_TASK_HOME),
    ('franka', 'pick_place_position', FRANKA_TASK_HOME),
    ('franka', 'tag_reach', FRANKA_TASK_HOME),
    ('ur5e', 'pick_place', UR5E_TASK_HOME),
    ('ur5e', 'multi_move', UR5E_TASK_HOME),
])
def test_the_trees_still_home_where_they_did(robot_type, task, expected):
    targets = _go_home_targets(build_task_tree(task, load_robot_config(robot_type)))
    assert targets and all(target == expected for target in targets)


def test_no_tree_spells_out_a_home_pose():
    # The constants the registry replaced must not come back.
    for name in ('FRANKA_HOME_POSITION', 'UR5E_HOME_POSITION', 'HOME_POSE_KEY',
                 'FR5_POSITION_LIMITS', 'EXCLUSIVE_ARM_CONTROLLERS'):
        offenders = [str(path.relative_to(PACKAGE)) for path in PACKAGE.rglob('*.py')
                     if name in path.read_text()]
        assert offenders == [], name


def test_controller_names_has_no_robot_special_case():
    source = (PACKAGE / 'utils' / 'controller_names.py').read_text()
    assert "== 'franka'" not in source and '== "franka"' not in source


@pytest.mark.parametrize('profile,gripper', [
    ('single', 'gripper_controller'),
    ('left', 'left_gripper_controller'),
    ('right', 'right_gripper_controller'),
])
def test_the_home_block_opens_the_profiles_own_gripper(profile, gripper):
    config = load_robot_config('openarm', profile)
    seq = home_subtree(
        config, target_joints=home_joint_state(config),
        controller=config['joint_space'])
    opener = seq.children[-1]
    assert isinstance(opener, GripperActionBehavior)
    assert opener.action_name == f'/{gripper}/gripper'


def test_a_home_block_cannot_open_a_gripper_the_robot_lacks():
    config = dict(load_robot_config('ur5e'), gripper=None)
    with pytest.raises(ValueError, match='open_gripper=False'):
        home_subtree(config, target_joints=home_joint_state(config),
                     controller=config['joint_space'])


def test_every_gripper_leaf_in_the_franka_trees_addresses_the_registry_gripper():
    config = load_robot_config('franka')
    for task in ('pick_place', 'peg_insert', 'gear_mesh', 'controller_check_position'):
        leaves = [node for node in build_task_tree(task, config).iterate()
                  if isinstance(node, GripperActionBehavior)]
        assert leaves, task
        assert {leaf.action_name for leaf in leaves} == {'/gripper_controller/gripper'}, task


def test_the_franka_vla_wait_listens_to_the_registry_vla_controller():
    from cho_task_manager.behaviors.service import VLACompletionWaiterBehavior
    config = load_robot_config('franka')
    for task in ('pick_place', 'pick_place_position', 'peg_insert'):
        waits = [node for node in build_task_tree(task, config).iterate()
                 if isinstance(node, VLACompletionWaiterBehavior)]
        assert [wait.service_name for wait in waits] == [
            '/vla_controller/vla/notify_completion'], task
