"""The shared subtrees, and the failure path the task trees never had.

Before this, any leaf that failed propagated straight to the root, the task
manager node saw a terminal status and shut down -- with whichever controller
the mission was last driving still active and nothing having tried to hold the
arm. These tests pin the two claims the guard rests on: the abort actually
runs, and the mission failure is still reported as a failure afterwards.
"""

import py_trees
import pytest

from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.subtrees import (
    guarded_mission,
    home_subtree,
    safe_abort_subtree,
    tare_ft_children,
)
from cho_task_manager.subtrees import safe_abort as safe_abort_module
from cho_task_manager.utils.controller_names import load_robot_config
from cho_task_manager.utils.msg_utils import make_joint_state

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE


def _config(robot_type, profile='single'):
    try:
        return load_robot_config(robot_type, profile)
    except (ValueError, ImportError, LookupError) as exc:
        pytest.skip(f'robot registry unavailable for {robot_type}/{profile}: {exc}')


class Scripted(py_trees.behaviour.Behaviour):
    """Returns a scripted status per tick and counts how often it was ticked.

    The tick count is the point: it is how "the mission was not re-run" and
    "the abort actually got to finish" are told apart.
    """

    def __init__(self, name, statuses):
        super().__init__(name)
        self.statuses = list(statuses)
        self.ticks = 0

    def update(self):
        self.ticks += 1
        return self.statuses[min(self.ticks - 1, len(self.statuses) - 1)]


def _tick_to_terminal(root, limit=20):
    """Tick like task_manager_node does: stop at the first terminal status.

    That stop is why the non-latching OneShot root never re-runs a failed
    mission, so a test that kept ticking past it would not describe the
    runtime.
    """
    statuses = []
    for _ in range(limit):
        root.tick_once()
        statuses.append(root.status)
        if root.status in (SUCCESS, FAILURE):
            return statuses
    raise AssertionError(f'never reached a terminal status: {statuses}')


# ---------------------------------------------------------------------------
# guarded_mission
# ---------------------------------------------------------------------------

def test_failed_mission_runs_the_abort_and_still_reports_failure(monkeypatch):
    abort = Scripted('Abort', [RUNNING, RUNNING, SUCCESS])
    monkeypatch.setattr(
        safe_abort_module, 'safe_abort_subtree', lambda *args, **kwargs: abort)
    mission = Scripted('Mission', [RUNNING, FAILURE])

    root = guarded_mission(mission, _config('franka'), 'torque')
    statuses = _tick_to_terminal(root)

    # The abort ran to completion instead of the tree dying on the failure.
    assert abort.ticks == 3
    # And the mission was not re-ticked, which would have re-sent the goal
    # that just failed.
    assert mission.ticks == 2
    # A successful abort must not be reported as a successful mission.
    assert statuses[-1] == FAILURE
    assert all(status == RUNNING for status in statuses[:-1])


def test_abort_that_itself_fails_still_reports_mission_failure(monkeypatch):
    """The abort is best-effort: a hold controller that will not activate
    (wrong control mode, crashed controller) must not flip the root to SUCCESS.
    """
    abort = Scripted('Abort', [FAILURE])
    monkeypatch.setattr(
        safe_abort_module, 'safe_abort_subtree', lambda *args, **kwargs: abort)
    mission = Scripted('Mission', [FAILURE])

    root = guarded_mission(mission, _config('franka'), 'torque')

    assert _tick_to_terminal(root)[-1] == FAILURE
    assert abort.ticks == 1


def test_successful_mission_never_runs_the_abort(monkeypatch):
    abort = Scripted('Abort', [SUCCESS])
    monkeypatch.setattr(
        safe_abort_module, 'safe_abort_subtree', lambda *args, **kwargs: abort)
    mission = Scripted('Mission', [RUNNING, SUCCESS])

    root = guarded_mission(mission, _config('franka'), 'torque')

    assert _tick_to_terminal(root)[-1] == SUCCESS
    assert abort.ticks == 0


def test_guard_shape_is_oneshot_over_selector():
    mission = py_trees.composites.Sequence(name='Mission', memory=True)

    root = guarded_mission(mission, _config('franka'), 'torque')

    assert isinstance(root, py_trees.decorators.OneShot)
    guard = root.decorated
    assert isinstance(guard, py_trees.composites.Selector)
    # Without memory the Selector would re-tick the failed mission each tick.
    assert guard.memory
    assert guard.children[0] is mission
    inverter = guard.children[1]
    assert isinstance(inverter, py_trees.decorators.Inverter)
    assert isinstance(inverter.decorated, py_trees.decorators.FailureIsSuccess)


def test_abort_can_be_declined_for_a_bringup_with_no_hold_controller():
    """The OpenArm MIT prototype spawns only the selected MIT controller, so
    there is no hold controller to switch to and the controller owns its own
    SAFE stop. abort=False must reproduce the bare OneShot root exactly.
    """
    mission = py_trees.composites.Sequence(name='Mission', memory=True)

    root = guarded_mission(mission, robot_config=None, abort=False)

    assert isinstance(root, py_trees.decorators.OneShot)
    assert root.decorated is mission


# ---------------------------------------------------------------------------
# safe_abort_subtree
# ---------------------------------------------------------------------------

def test_abort_switches_to_the_modes_hold_and_verifies_it_took():
    config = _config('franka')

    seq = safe_abort_subtree(config, 'torque')
    switch, verify = seq.children

    assert isinstance(switch, SwitchControllerServiceBehavior)
    request = switch.make_request()
    assert request.activate_controllers == ['joint_space_impedance_controller']
    # Exclusive: whatever was driving the arm comes down, whichever it was.
    assert 'task_space_qp_controller' in request.deactivate_controllers
    assert 'joint_space_impedance_controller' not in request.deactivate_controllers
    # BEST_EFFORT, so result.ok alone proves nothing -- hence the verify step.
    assert request.strictness == request.BEST_EFFORT
    assert isinstance(verify, ListControllersServiceBehavior)
    assert verify.require_active == ['joint_space_impedance_controller']


@pytest.mark.parametrize('mode, expected', [
    ('position', 'joint_space_position_controller'),
    ('velocity', 'joint_space_velocity_controller'),
    ('torque', 'joint_space_impedance_controller'),
])
def test_abort_hold_follows_the_control_mode(mode, expected):
    seq = safe_abort_subtree(_config('franka'), mode)

    assert seq.children[0].make_request().activate_controllers == [expected]


def test_operator_control_mode_overrides_the_tasks_default():
    """Only the operator knows how the bringup was actually started."""
    config = dict(_config('franka'), control_mode='position')

    seq = safe_abort_subtree(config, 'torque')

    assert seq.children[0].make_request().activate_controllers == [
        'joint_space_position_controller']


def test_bimanual_abort_holds_that_arms_own_instance():
    config = _config('openarm', 'left')

    seq = safe_abort_subtree(config, 'torque')
    request = seq.children[0].make_request()

    assert request.activate_controllers == ['left_joint_space_impedance_controller']
    # The other arm is an independent robot: never take it down.
    assert not any(
        name.startswith('right_') for name in request.deactivate_controllers)


def test_abort_refuses_a_mode_the_robot_has_no_hold_for():
    with pytest.raises(ValueError, match="no hold controller for control_mode 'torque'"):
        safe_abort_subtree(_config('ur5e'), 'torque')


def test_abort_refuses_an_unknown_control_mode():
    with pytest.raises(ValueError, match='Unknown control_mode'):
        safe_abort_subtree(_config('franka'), 'effort')


def test_abort_refuses_to_guess_when_no_mode_is_available():
    with pytest.raises(ValueError, match='No control_mode available'):
        safe_abort_subtree(_config('franka'), None)


# ---------------------------------------------------------------------------
# home_subtree
# ---------------------------------------------------------------------------

HOME = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])


def test_home_subtree_is_switch_move_open():
    seq = home_subtree(
        _config('franka'), target_joints=HOME,
        controller='joint_space_impedance_controller')

    assert seq.name == '1_Initialize'
    assert [child.name for child in seq.children] == [
        'Switch_To_joint_space_impedance_controller', 'Go_Home', 'Open_Gripper']


def test_home_subtree_suffix_and_gripper_are_per_call():
    seq = home_subtree(
        _config('ur5e'), target_joints=HOME,
        controller='joint_space_position_controller',
        name='3_Finish', suffix='_Final', open_gripper=False)

    assert seq.name == '3_Finish'
    assert [child.name for child in seq.children] == [
        'Switch_To_joint_space_position_controller_Final', 'Go_Home_Final']


def test_home_subtree_derives_the_deactivate_list_from_the_robot():
    """Passing robot_config is the reason there is one copy of this block.

    Four trees carried their own; only the UR ones passed robot_config, so the
    Franka copies fell back to the historical hard-coded Franka name list.
    """
    seq = home_subtree(
        _config('ur5e'), target_joints=HOME,
        controller='joint_space_position_controller')
    deactivate = seq.children[0].make_request().deactivate_controllers

    assert 'task_space_ik_controller' in deactivate
    # A Franka controller name is meaningless on a UR.
    assert not any('_qp_' in name for name in deactivate)


def test_home_subtree_lead_children_go_in_front():
    seq = home_subtree(
        _config('franka'), target_joints=HOME,
        controller='joint_space_impedance_controller',
        lead_children=tare_ft_children())

    assert [child.name for child in seq.children][:2] == [
        'Tare_FT_Sensor', 'Wait_After_Tare']
