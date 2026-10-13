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

"""ST-5 (AC6): every left_X block of a bimanual OpenArm controller YAML equals its right_X block.

Both arms are the same mechanism bolted to the same torso, so after swapping 'left' <-> 'right'
in the string keys and values of one arm's block the other arm's block must come out identical
(key set and values). The controller_manager's left_X/right_X entries (their plugin type) are
held to the same rule. A left_/right_ block without a partner fails.
"""

import re
from pathlib import Path

import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / 'config'

BIMANUAL_FILES = [
    'isaac/controllers_bimanual.yaml',
    'mujoco/controllers_bimanual.yaml',
    'mujoco/controllers_mit_bimanual.yaml',
    'mujoco/controllers_mit_direct_bimanual.yaml',
    'mujoco/controllers_mit_moveit_bimanual.yaml',
    'real/controllers_mit_bimanual.yaml',
]

_STARTUP_POSTURE_REASON = (
    'Each arm starts from its own FK-checked, non-singular posture that converges from the '
    'simulator / hardware initial state; the right arm is not the left posture mirrored.')

# (file, block name without the side prefix, key) -> why the two arms differ on purpose.
# The key is compared by presence only: both sides must carry it, with different values.
INTENDED_DIFFERENCES = {
    ('mujoco/controllers_mit_direct_bimanual.yaml', 'task_space_impedance_mit_controller',
     'startup_posture'): _STARTUP_POSTURE_REASON,
    ('mujoco/controllers_mit_direct_bimanual.yaml', 'vla_mit_controller',
     'startup_posture'): _STARTUP_POSTURE_REASON,
    ('real/controllers_mit_bimanual.yaml', 'task_space_impedance_mit_controller',
     'startup_posture'): _STARTUP_POSTURE_REASON,
    ('real/controllers_mit_bimanual.yaml', 'vla_mit_controller',
     'startup_posture'): _STARTUP_POSTURE_REASON,
}

_SIDE_RE = re.compile(r'left|right')
_SWAP = {'left': 'right', 'right': 'left'}


def swap_sides(text):
    """Exchange every 'left' and 'right' substring of text."""
    return _SIDE_RE.sub(lambda m: _SWAP[m.group(0)], text)


def mirror(node):
    """Return node with 'left' <-> 'right' exchanged in every string key and string value."""
    if isinstance(node, dict):
        return {(swap_sides(k) if isinstance(k, str) else k): mirror(v) for k, v in node.items()}
    if isinstance(node, list):
        return [mirror(v) for v in node]
    if isinstance(node, str):
        return swap_sides(node)
    return node


def load_blocks(relative):
    """The node-parameter blocks of a file: name -> ros__parameters, under '/**' or top level."""
    document = yaml.safe_load((CONFIG / relative).read_text())
    assert isinstance(document, dict) and document, relative
    top = document['/**'] if '/**' in document else document
    blocks = {}
    for name, body in top.items():
        assert isinstance(body, dict) and 'ros__parameters' in body, (relative, name)
        blocks[name] = body['ros__parameters']
    return blocks


def side_pairs(names, where):
    """Group left_X / right_X names into {X: (left_name, right_name)}; a lone side is an error."""
    sides = {}
    for name in names:
        for side in ('left', 'right'):
            if name.startswith(side + '_'):
                sides.setdefault(name[len(side) + 1:], {})[side] = name
    unpaired = sorted(n for base, pair in sides.items() if len(pair) != 2 for n in pair.values())
    assert not unpaired, f'{where}: left_/right_ without a partner: {unpaired}'
    return {base: (pair['left'], pair['right']) for base, pair in sides.items()}


def differences(left, right):
    """Top-level keys of two parameter dicts whose values (or presence) differ."""
    return sorted(k for k in set(left) | set(right)
                  if k not in left or k not in right or left[k] != right[k])


def _symmetry_problems(relative, base, left, right):
    """Keys of one pair that differ and are not allowed to, as readable strings."""
    mirrored_left = mirror(left)
    problems = []
    for key in differences(mirrored_left, right):
        allowed = (relative, base, key) in INTENDED_DIFFERENCES
        if allowed:
            if key not in left or key not in right:
                problems.append(f'{base}.{key}: allowed difference must be present on both arms')
            continue
        problems.append(
            f'{base}.{key}: left (mirrored) = {mirrored_left.get(key, "<absent>")!r}, '
            f'right = {right.get(key, "<absent>")!r}')
    return problems


@pytest.mark.parametrize('relative', BIMANUAL_FILES)
def test_ac6_arm_blocks_are_mirror_images(relative):
    """AC6: each left_X block equals its right_X block with left and right exchanged."""
    blocks = load_blocks(relative)
    pairs = side_pairs(blocks, relative)
    problems = []
    for base, (left_name, right_name) in sorted(pairs.items()):
        problems += [f'{left_name}/{right_name} {p}' for p in _symmetry_problems(
            relative, base, blocks[left_name], blocks[right_name])]
    assert not problems, f'{relative}:\n  ' + '\n  '.join(problems)


@pytest.mark.parametrize('relative', BIMANUAL_FILES)
def test_ac6_controller_manager_arm_entries_are_mirror_images(relative):
    """AC6: controller_manager's left_X / right_X entries (plugin type) match too."""
    manager = load_blocks(relative)['controller_manager']
    pairs = side_pairs(manager, f'{relative} controller_manager')
    assert pairs, f'{relative}: controller_manager lists no left_/right_ controller'
    for base, (left_name, right_name) in sorted(pairs.items()):
        assert 'type' in manager[left_name], (relative, left_name)
        assert manager[left_name]['type'] == manager[right_name]['type'], (
            relative, base, manager[left_name], manager[right_name])
        assert mirror(manager[left_name]) == manager[right_name], (relative, base)


@pytest.mark.parametrize('relative', BIMANUAL_FILES)
def test_ac6_every_arm_controller_in_manager_has_a_block_and_vice_versa(relative):
    """AC6: the pairs declared in controller_manager and the pairs given a block are the same set."""
    blocks = load_blocks(relative)
    declared = set(side_pairs(blocks['controller_manager'], relative))
    configured = set(side_pairs([n for n in blocks if n != 'controller_manager'], relative))
    # A controller may be declared without parameters (e.g. a broadcaster whose block is optional is
    # still listed with one here), but a block for a controller the manager never loads is dead YAML.
    assert configured <= declared, f'{relative}: blocks without a controller_manager entry: ' \
        f'{sorted(configured - declared)}'


@pytest.mark.parametrize('relative', BIMANUAL_FILES)
def test_ac6_file_has_arm_blocks(relative):
    """AC6 guard: the parametrisation really covers arm blocks (an empty pairing would pass vacuously)."""
    blocks = load_blocks(relative)
    pairs = side_pairs([n for n in blocks if n != 'controller_manager'], relative)
    assert pairs, f'{relative}: no left_/right_ parameter blocks found'


def test_ac6_intended_differences_are_real_and_documented():
    """AC6: every allow-list entry names an existing pair, carries a reason, and really differs."""
    for (relative, base, key), reason in INTENDED_DIFFERENCES.items():
        assert relative in BIMANUAL_FILES, (relative, base, key)
        assert len(reason) > 30, (relative, base, key)
        blocks = load_blocks(relative)
        pairs = side_pairs(blocks, relative)
        assert base in pairs, f'{relative}: allow-list names a block that does not exist: {base}'
        left_name, right_name = pairs[base]
        left, right = blocks[left_name], blocks[right_name]
        assert key in left and key in right, f'{relative}:{base}.{key} missing on one arm'
        assert left[key] != right[key], (
            f'{relative}:{base}.{key} is now equal on both arms; drop it from INTENDED_DIFFERENCES')


def test_ac6_all_six_files_exist():
    """AC6: the six bimanual files are the ones the spec names and all of them are present."""
    assert len(BIMANUAL_FILES) == 6
    for relative in BIMANUAL_FILES:
        assert (CONFIG / relative).is_file(), relative


# --- the helpers themselves: a symmetric pair passes, a broken one is caught -----------------------

def test_mirror_swaps_keys_values_and_nested_lists():
    node = {'left_a': ['openarm_left_joint1', {'pose_topic': '/ee_state/right/pose'}], 'n': 3, 'f': 1.5}
    assert mirror(node) == {'right_a': ['openarm_right_joint1', {'pose_topic': '/ee_state/left/pose'}],
                            'n': 3, 'f': 1.5}
    assert mirror(mirror(node)) == node


def test_symmetry_check_catches_a_numeric_difference():
    left = {'joints': ['openarm_left_joint1'], 'kp': [1.0, 2.0]}
    same = {'joints': ['openarm_right_joint1'], 'kp': [1.0, 2.0]}
    off = {'joints': ['openarm_right_joint1'], 'kp': [1.0, 2.5]}
    assert _symmetry_problems('x.yaml', 'foo', left, same) == []
    assert any('kp' in p for p in _symmetry_problems('x.yaml', 'foo', left, off))


def test_symmetry_check_catches_a_joint_list_that_was_not_mirrored():
    left = {'joints': ['openarm_left_joint1']}
    right = {'joints': ['openarm_left_joint1']}  # copy-paste slip: right arm names the left joint
    assert any('joints' in p for p in _symmetry_problems('x.yaml', 'foo', left, right))


def test_symmetry_check_catches_a_key_missing_on_one_arm():
    left = {'kp': [1.0], 'kd': [2.0]}
    right = {'kp': [1.0]}
    assert any('kd' in p for p in _symmetry_problems('x.yaml', 'foo', left, right))


def test_unpaired_side_block_is_reported():
    with pytest.raises(AssertionError, match='left_orphan_controller'):
        side_pairs(['left_orphan_controller', 'left_a', 'right_a'], 'x.yaml')
