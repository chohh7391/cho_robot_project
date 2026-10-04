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

"""The exclusive-switch set must describe the robot being switched.

The set exists so an exclusive switch is idempotent: it deactivates whatever
holds the arm's command interfaces, whichever controller that happens to be.
A Franka-only list cannot do that for OpenArm or UR, and because the exclusive
path is deliberately BEST_EFFORT the omission fails silently.
"""

import pytest

from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    load_robot_config,
)


# The Franka-only list controller_names.py used to carry, and add to the
# registry-derived set with an `if robot_type == 'franka'` branch. It now comes
# from franka.yaml (roles, per-mode holds, controllers.additional_arm); these
# names must all still be in it, or an exclusive switch on a Franka would leave
# one of them running.
HISTORICAL_FRANKA_SET = [
    'joint_space_impedance_controller',
    'joint_space_qp_controller',
    'joint_space_position_controller',
    'joint_space_velocity_controller',
    'task_space_ik_controller',
    'task_space_velocity_controller',
    'operational_space_controller',
    'task_space_impedance_controller',
    'task_space_qp_controller',
    'vla_controller',
    'gravity_compensation_controller',
]


def _config(robot_type, profile='single'):
    try:
        return load_robot_config(robot_type, profile)
    except (ValueError, ImportError, LookupError) as exc:
        pytest.skip(f'robot registry unavailable for {robot_type}/{profile}: {exc}')


@pytest.mark.parametrize('config', [None, {}])
def test_no_robot_config_is_refused_rather_than_assumed_franka(config):
    # The old no-config default was Franka's list: on any other arm it
    # deactivated nothing that actually held it.
    with pytest.raises(ValueError, match='robot config'):
        exclusive_arm_controllers(config)


def test_openarm_set_contains_its_mit_controllers():
    names = exclusive_arm_controllers(_config('openarm'))

    assert 'task_space_impedance_mit_controller' in names
    assert 'joint_impedance_mit_controller' in names
    # The MoveIt trajectory controller claims the same joints.
    assert 'joint_trajectory_controller' in names
    # The compatibility override the task trees actually activate.
    assert 'joint_space_impedance_controller' in names
    # A Franka controller name is meaningless here and must not appear.
    assert 'task_space_qp_controller' not in names
    # The gripper claims a separate interface and must stay active.
    assert 'gripper_controller' not in names


@pytest.mark.parametrize('profile', ['left', 'right'])
def test_bimanual_profile_yields_its_own_prefixed_controllers(profile):
    other = 'right' if profile == 'left' else 'left'
    names = exclusive_arm_controllers(_config('openarm', profile))

    assert f'{profile}_task_space_impedance_mit_controller' in names
    assert f'{profile}_joint_impedance_mit_controller' in names
    assert f'{profile}_joint_space_position_controller' in names
    assert f'{profile}_joint_trajectory_controller' in names
    # The other arm is an independent 7-axis robot: never take it down.
    assert not any(name.startswith(f'{other}_') for name in names)
    assert f'{profile}_gripper_controller' not in names


def test_franka_set_is_exactly_its_registry_entry():
    """Same set as before the Franka branch moved into franka.yaml.

    The historical list plus the registry-only MoveIt trajectory controller,
    and nothing else: the move must not change what a Franka switch takes down.
    """
    names = exclusive_arm_controllers(_config('franka'))

    assert set(names) == set(HISTORICAL_FRANKA_SET) | {'moveit_joint_trajectory_controller'}
    assert len(names) == len(set(names))
    assert 'gripper_controller' not in names


def test_the_code_has_no_franka_special_case():
    # The extra Franka controllers come from the registry, so a robot that
    # declared the same controllers.additional_arm would get the same set.
    config = _config('ur5e')
    names = exclusive_arm_controllers(config)
    assert 'gravity_compensation_controller' not in names
    assert 'task_space_velocity_controller' not in names


def test_ur_set_is_the_ur_controllers():
    names = exclusive_arm_controllers(_config('ur5e'))

    assert set(names) == {
        'joint_space_position_controller',
        'task_space_ik_controller',
        'joint_trajectory_controller',
    }


def test_switch_behaviour_deactivates_the_robots_own_controllers():
    config = _config('openarm')
    behaviour = SwitchControllerServiceBehavior(
        name='Switch', activate=[config['joint_space']], robot_config=config)

    request = behaviour.make_request()

    assert request.activate_controllers == ['joint_space_impedance_controller']
    assert 'task_space_impedance_mit_controller' in request.deactivate_controllers
    assert 'joint_impedance_mit_controller' in request.deactivate_controllers
    # Never in both lists at once.
    assert 'joint_space_impedance_controller' not in request.deactivate_controllers


def test_exclusive_switch_without_a_robot_config_is_refused():
    with pytest.raises(ValueError, match='robot_config'):
        SwitchControllerServiceBehavior(
            name='Switch', activate=['task_space_qp_controller'])


def test_franka_switch_takes_down_every_other_franka_arm_controller():
    behaviour = SwitchControllerServiceBehavior(
        name='Switch', activate=['task_space_qp_controller'],
        robot_config=_config('franka'))

    deactivate = behaviour.make_request().deactivate_controllers

    assert set(deactivate) == (
        set(HISTORICAL_FRANKA_SET) | {'moveit_joint_trajectory_controller'}) - {
        'task_space_qp_controller'}


def test_explicit_exclusive_controllers_win():
    behaviour = SwitchControllerServiceBehavior(
        name='Switch', activate=['a'], exclusive_controllers=['a', 'b'])

    assert behaviour.make_request().deactivate_controllers == ['b']


def test_non_exclusive_switch_still_uses_the_given_deactivate_list():
    behaviour = SwitchControllerServiceBehavior(
        name='Switch', activate=['a'], deactivate=['b'], exclusive=False)

    request = behaviour.make_request()

    assert request.deactivate_controllers == ['b']
    assert request.strictness == request.STRICT
