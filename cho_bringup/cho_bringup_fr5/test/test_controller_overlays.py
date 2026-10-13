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

"""The FR5 controller parameters are a base file plus one overlay per backend.

Each backend's controller_manager and spawners receive config/common/controllers.yaml
first and config/<backend>/controllers.yaml after it.  rcl merges parameter files
by name: a later value of the same name wins, a dict is merged key by key and a
list is replaced whole.  A key can never be removed by a later file, so the base
holds only what every backend agrees on, and an overlay holds only what differs.

YAML only, no ROS graph.
"""

import copy
from pathlib import Path

import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / 'config'
BASE_FILE = CONFIG / 'common' / 'controllers.yaml'
BACKENDS = ('real', 'mujoco', 'isaac', 'gz')
ARM_JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

# Controller types that need a parameter block but have no `joints` parameter.
NO_JOINTS_TYPES = {
    'cho_controller_gripper/GripperController': 'a gripper names gripper_joint, not the arm joints',
}
# Controller types that are fine without a block of their own.
BLOCKLESS_TYPES = {
    'joint_state_broadcaster/JointStateBroadcaster': 'a broadcaster runs on its defaults',
}


def _load(path):
    assert path.is_file(), f'{path} is missing'
    data = yaml.safe_load(path.read_text())
    assert isinstance(data, dict), f'{path} is not a mapping'
    return data


def _merge(base, over):
    """rcl rule: dicts merge recursively, the later value wins, a list is replaced whole."""
    out = copy.deepcopy(base)
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _flatten(node, prefix=()):
    """{path tuple: leaf value}; a list is one leaf, an empty dict is one leaf."""
    if isinstance(node, dict) and node:
        flat = {}
        for key, value in node.items():
            flat.update(_flatten(value, prefix + (key,)))
        return flat
    return {prefix: node}


def _overlay(backend):
    return _load(CONFIG / backend / 'controllers.yaml')


def _merged(backend):
    return _merge(_load(BASE_FILE), _overlay(backend))


def _dotted(path):
    return '.'.join(str(p) for p in path)


def test_base_and_overlays_exist_AC3():
    """AC3: the base and the four overlays are where the launch files look for them."""
    assert BASE_FILE.is_file()
    for backend in BACKENDS:
        assert (CONFIG / backend / 'controllers.yaml').is_file(), backend


def test_files_use_plain_keys_not_wildcard_AC3():
    """AC3/constraint: a node's plain keys and `/**` must not be mixed (FR5 uses plain keys)."""
    for path in [BASE_FILE] + [CONFIG / b / 'controllers.yaml' for b in BACKENDS]:
        keys = list(_load(path))
        assert '/**' not in keys, f'{path} mixes /** with plain node keys'
        assert all(not str(k).startswith('/') for k in keys), f'{path}: {keys}'


def test_no_key_shared_by_every_overlay_with_the_same_value_AC3():
    """AC3(a): a key that all four overlays carry with one value belongs in the base."""
    flats = [_flatten(_overlay(b)) for b in BACKENDS]
    common = set(flats[0])
    for flat in flats[1:]:
        common &= set(flat)
    shared = sorted(
        _dotted(path) for path in common
        if all(flat[path] == flats[0][path] for flat in flats[1:]))
    assert not shared, f'move to config/common/controllers.yaml: {shared}'


@pytest.mark.parametrize('backend', BACKENDS)
def test_overlay_does_not_restate_base_value_AC3(backend):
    """AC3(b): an overlay key is either absent from the base or different from it."""
    base = _flatten(_load(BASE_FILE))
    over = _flatten(_overlay(backend))
    restated = sorted(
        _dotted(path) for path, value in over.items()
        if path in base and base[path] == value)
    assert not restated, f'{backend} overlay repeats base values: {restated}'


def test_base_is_not_empty_AC3():
    """AC3: the split is real; the base carries the keys the backends agree on."""
    base = _flatten(_load(BASE_FILE))
    assert base
    # The type of an arm controller is the same on every backend.
    assert ('controller_manager', 'ros__parameters', 'joint_trajectory_controller', 'type') in base


@pytest.mark.parametrize('backend', BACKENDS)
def test_every_typed_controller_has_a_block_with_arm_joints_AC3(backend):
    """AC3(c): each controller the manager can load has parameters, and arm ones name j1..j6."""
    merged = _merged(backend)
    manager = merged['controller_manager']['ros__parameters']
    typed = {
        name: value['type'] for name, value in manager.items()
        if isinstance(value, dict) and 'type' in value}
    assert 'joint_trajectory_controller' in typed, backend
    assert 'joint_state_broadcaster' in typed, backend
    for name, ctype in typed.items():
        if ctype in BLOCKLESS_TYPES and name not in merged:
            continue
        assert name in merged, f'{backend}: {name} ({ctype}) has no parameter block'
        params = merged[name].get('ros__parameters')
        assert isinstance(params, dict), f'{backend}: {name} has no ros__parameters'
        if ctype in NO_JOINTS_TYPES:
            continue
        assert params.get('joints') == ARM_JOINTS, f'{backend}: {name} joints = {params.get("joints")}'


@pytest.mark.parametrize('backend', BACKENDS)
def test_merged_update_rate_present_AC3(backend):
    """AC3(d): every backend ends up with a controller_manager update_rate."""
    rate = _merged(backend)['controller_manager']['ros__parameters'].get('update_rate')
    assert isinstance(rate, (int, float)) and not isinstance(rate, bool), backend
    assert rate > 0


def test_update_rates_by_backend_AC2():
    """AC2: the merge keeps the per-backend rates (real 125, mujoco 500, isaac 250, gz 500)."""
    expected = {'real': 125, 'mujoco': 500, 'isaac': 250, 'gz': 500}
    for backend, rate in expected.items():
        assert _merged(backend)['controller_manager']['ros__parameters']['update_rate'] == rate


def test_open_loop_control_only_on_mujoco_AC2():
    """AC2: the merged result keeps open_loop_control true on MuJoCo and absent elsewhere."""
    for backend in BACKENDS:
        jtc = _merged(backend)['joint_trajectory_controller']['ros__parameters']
        if backend == 'mujoco':
            assert jtc['open_loop_control'] is True
        else:
            assert 'open_loop_control' not in jtc, backend


@pytest.mark.parametrize('backend', BACKENDS)
def test_bringup_type_matches_backend_AC2(backend):
    """AC2: task_space_ik_controller still reports its own backend after the merge."""
    params = _merged(backend)['task_space_ik_controller']['ros__parameters']
    assert params['robot_type'] == 'fr5'
    assert params['bringup_type'] == backend


def test_merge_helper_follows_rcl_rules():
    """The merge used above is the rcl one: recursive dicts, later wins, lists replaced."""
    base = {'a': {'b': 1, 'c': [1, 2, 3], 'd': 4}}
    over = {'a': {'c': [9], 'd': 5, 'e': 6}}
    assert _merge(base, over) == {'a': {'b': 1, 'c': [9], 'd': 5, 'e': 6}}
