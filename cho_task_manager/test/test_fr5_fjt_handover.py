"""The handover tree, and the registry entries it must not drift away from.

Nothing here drives a robot. What it checks is the wiring that decides whether
the arm ends up held or abandoned: that the controller the tree switches in is
the one the exclusive-switch set later takes down, and that the tree never
sends a gripper goal on a build whose gripper may not exist.
"""

from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.fr5 import fjt_handover
from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    load_robot_config,
)
import pytest


def _config():
    return load_robot_config('fr5')


def test_the_task_is_registered_for_fr5_only():
    assert 'fjt_handover' in available_tasks('fr5')
    for other in ('franka', 'ur5e', 'openarm'):
        assert 'fjt_handover' not in available_tasks(other)


def test_the_tree_builds_and_has_the_three_blocks():
    tree = build_task_tree('fjt_handover', _config())
    names = {node.name for node in tree.iterate()}

    assert '1_Initialize' in names
    assert '2_Handover' in names
    assert '3_Finish' in names
    assert 'External_Trajectory_Session' in names
    # The abort branch is what puts the arm on its hold controller when the
    # external session fails; a tree without it walks away from a live arm.
    assert 'Abort_Switch_To_Hold_position' in names


def test_the_handover_controller_comes_from_the_registry():
    config = _config()
    assert fjt_handover.handover_controller(config) == 'joint_trajectory_controller'


def test_the_switch_back_can_take_the_handover_controller_down():
    # If the trajectory controller is not in the exclusive set, switching the
    # hold back in never deactivates it, both claim the same command
    # interfaces, and controller_manager rejects the switch -- leaving the arm
    # under the controller the failed session was driving.
    config = _config()
    assert fjt_handover.handover_controller(config) in exclusive_arm_controllers(config)


def test_the_tree_never_commands_the_gripper():
    # The FR5 gripper controller is only loaded with gripper:=ag95, and the arm
    # may be holding a vessel when this tree homes.
    tree = build_task_tree('fjt_handover', _config())
    assert not [node for node in tree.iterate()
                if 'Gripper' in node.name or 'gripper' in node.name]


def test_the_home_pose_is_the_registry_ready_pose_not_zero():
    home = fjt_handover.home_joint_state(_config())
    assert len(home.position) == 6
    # home 0 is all-zero and recorded as diagnostic-only: it puts the wrist at
    # the floor and j5=0 is a wrist singularity.
    assert any(abs(value) > 1e-9 for value in home.position)


def test_a_session_timeout_inside_the_start_window_is_rejected():
    from cho_task_manager.behaviors.topic import ExternalSessionBehavior
    with pytest.raises(ValueError):
        ExternalSessionBehavior(
            name='Bad_Window', start_timeout_sec=600.0, session_timeout_sec=60.0)
