"""The exclusive-switch set must describe the robot being switched.

The set exists so an exclusive switch is idempotent: it deactivates whatever
holds the arm's command interfaces, whichever controller that happens to be.
A Franka-only list cannot do that for OpenArm or UR, and because the exclusive
path is deliberately BEST_EFFORT the omission fails silently.
"""

import pytest

from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.controller_names import (
    EXCLUSIVE_ARM_CONTROLLERS,
    exclusive_arm_controllers,
    load_robot_config,
)


HISTORICAL_FRANKA_SET = [str(controller) for controller in EXCLUSIVE_ARM_CONTROLLERS]


def _config(robot_type, profile='single'):
    try:
        return load_robot_config(robot_type, profile)
    except (ValueError, ImportError, LookupError) as exc:
        pytest.skip(f'robot registry unavailable for {robot_type}/{profile}: {exc}')


def test_no_robot_config_keeps_the_historical_franka_set():
    assert exclusive_arm_controllers() == HISTORICAL_FRANKA_SET
    assert exclusive_arm_controllers(None) == HISTORICAL_FRANKA_SET


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


def test_franka_keeps_every_historical_name():
    names = exclusive_arm_controllers(_config('franka'))

    assert set(HISTORICAL_FRANKA_SET) <= set(names)
    # Registry-only names join it; the gripper still does not.
    assert 'moveit_joint_trajectory_controller' in names
    assert 'gripper_controller' not in names


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


def test_switch_behaviour_without_a_robot_config_is_unchanged():
    behaviour = SwitchControllerServiceBehavior(
        name='Switch', activate=['task_space_qp_controller'])

    deactivate = behaviour.make_request().deactivate_controllers

    assert deactivate == [
        name for name in HISTORICAL_FRANKA_SET if name != 'task_space_qp_controller']


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
