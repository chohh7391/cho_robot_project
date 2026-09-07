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

"""`gripper` and `load_gripper` resolve to one gripper name, or refuse to.

The real robot bringup and the MoveIt bringup both call resolve_gripper, and
they have to agree: the MoveIt one hands the resolved name to move_group's own
description, so a disagreement is a model mismatch rather than a typo.
"""

import importlib.util
import re
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
UTILS = PACKAGE / 'utils' / 'launch_utils.py'
REAL_CONFIG = PACKAGE / 'config' / 'real' / 'fr5.config.yaml'

spec = importlib.util.spec_from_file_location('fr5_launch_utils_under_test', UTILS)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

resolve = launch_utils.resolve_gripper


@pytest.mark.parametrize('load_arg', ['true', 'True', '1', 'yes', 'on'])
def test_load_gripper_true_selects_the_only_fr5_gripper(load_arg):
    assert resolve('', load_arg, 'none') == 'ag95'


@pytest.mark.parametrize('load_arg', ['false', 'False', '0', 'no', 'off'])
def test_load_gripper_false_selects_no_gripper(load_arg):
    assert resolve('', load_arg, 'ag95') == 'none'


@pytest.mark.parametrize('load_arg', ['config', '', None])
def test_deferring_uses_the_config_file(load_arg):
    assert resolve('', load_arg, 'ag95') == 'ag95'
    assert resolve('', load_arg, 'none') == 'none'


def test_the_name_form_still_works_on_its_own():
    assert resolve('ag95', 'config', 'none') == 'ag95'
    assert resolve('none', 'config', 'ag95') == 'none'


def test_both_spellings_are_accepted_when_they_agree():
    assert resolve('ag95', 'true', 'none') == 'ag95'
    assert resolve('none', 'false', 'ag95') == 'none'


@pytest.mark.parametrize('named,load_arg', [('ag95', 'false'), ('none', 'true')])
def test_contradicting_spellings_are_refused_rather_than_ranked(named, load_arg):
    # Silently honouring one of them is how an operator who asked for no
    # gripper ends up activating one.
    with pytest.raises(RuntimeError, match='contradicts'):
        resolve(named, load_arg, 'none')


def test_an_unknown_gripper_name_is_refused():
    with pytest.raises(RuntimeError, match='Unknown gripper'):
        resolve('robotiq', 'config', 'none')


def test_an_unknown_config_value_is_refused():
    with pytest.raises(RuntimeError, match='config file'):
        resolve('', 'config', 'pgi140')


def test_a_non_boolean_load_gripper_is_refused():
    with pytest.raises(RuntimeError, match='boolean'):
        resolve('', 'maybe', 'none')


def test_the_shipped_config_resolves_and_names_a_known_gripper():
    config = yaml.safe_load(REAL_CONFIG.read_text())['fr5']
    assert config['gripper'] in launch_utils.GRIPPERS
    assert resolve('', 'config', config['gripper']) == config['gripper']
    # load_gripper must be able to override whatever the file happens to say.
    assert resolve('', 'true', config['gripper']) == 'ag95'
    assert resolve('', 'false', config['gripper']) == 'none'


def test_every_launch_file_defers_load_gripper_by_default():
    # The MoveIt bringups resolve the gripper themselves and pass the NAME down
    # to the robot bringup. If a robot bringup defaulted load_gripper to false
    # instead of deferring, that pass-down would read as gripper:=ag95 against
    # load_gripper:=false and resolve_gripper would refuse the launch. That is
    # exactly what happened to bringup_mujoco_robot, so pin it.
    launches = sorted((PACKAGE / 'launch').glob('*.launch.py'))
    assert launches, 'no launch files found'
    # Match the DECLARATION, not the first mention: the MoveIt bringups read
    # LaunchConfiguration('load_gripper') inside their OpaqueFunction, which
    # appears earlier in the file than the DeclareLaunchArgument.
    pattern = re.compile(
        r"DeclareLaunchArgument\(\s*'load_gripper',\s*default_value='([^']*)'")
    declaring = {}
    for f in launches:
        found = pattern.search(f.read_text())
        if found:
            declaring[f.name] = found.group(1)
    # Both real bringups plus both MuJoCo ones.
    assert len(declaring) >= 4, declaring
    assert all(v == 'config' for v in declaring.values()), declaring


def test_resolved_name_passed_down_alone_does_not_contradict_the_default():
    # The shape the MoveIt bringups actually use.
    assert resolve('ag95', 'config', 'none') == 'ag95'
    assert resolve('none', 'config', 'none') == 'none'


def test_the_default_gripper_is_one_the_description_switches_on():
    assert launch_utils.DEFAULT_GRIPPER in launch_utils.GRIPPERS
    assert launch_utils.DEFAULT_GRIPPER != 'none'
