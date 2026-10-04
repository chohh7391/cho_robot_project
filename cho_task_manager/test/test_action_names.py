"""Every action a tree addresses follows cho_interfaces/CONTRACT.md.

A controller serves its actions under its own node, `/<controller>/<kind>`, and
the trees build those names through one helper. A name built any other way is a
goal sent to a server that does not exist: the leaf waits out its timeout and
fails, which on a real cell reads as a controller that stopped responding.
"""

import pytest

from cho_task_manager.behaviors.action import (
    FollowJointTrajectoryBehavior,
    GripperActionBehavior,
    JointSpaceActionBehavior,
    TaskSpaceActionBehavior,
)
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
    exclusive_arm_controllers,
    load_robot_config,
    moveit_joint_action_name,
    valid_controller_action_names,
    vla_completion_service_name,
)
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose


@pytest.mark.parametrize('kind', [
    'joint_space', 'task_space', 'gripper', 'vla', 'follow_joint_trajectory'])
def test_a_controller_serves_each_kind_under_its_own_node(kind):
    assert controller_action_name('left_vla_mit_controller', kind) == (
        f'/left_vla_mit_controller/{kind}')
    assert controller_action_name(ControllerNames.JOINT_QP, kind) == (
        f'/joint_space_qp_controller/{kind}')


def test_the_pour_action_keeps_its_own_name():
    assert controller_action_name('pouring_controller', 'pour') == (
        '/controller_action_server/pouring_controller')


def test_an_unknown_kind_is_refused_rather_than_guessed():
    with pytest.raises(ValueError, match='unknown action kind'):
        controller_action_name(ControllerNames.JOINT_QP, 'moveit_joint')


def test_the_action_leaves_address_their_controllers_actions():
    joint = JointSpaceActionBehavior(
        'Joint', target_joints=make_joint_state([0.0] * 7),
        controller_name=ControllerNames.JOINT_IMPEDANCE)
    task = TaskSpaceActionBehavior(
        'Task', target_pose=make_pose([0.4, 0.0, 0.4]), controller_name=ControllerNames.IK)
    gripper = GripperActionBehavior('Grip', grasp=True, controller_name='left_gripper_controller')
    replay = FollowJointTrajectoryBehavior(
        'Replay', 'joint_trajectory_controller', ['j1'], [0.0, 1.0], [[0.0], [0.1]])

    assert joint.action_name == '/joint_space_impedance_controller/joint_space'
    assert task.action_name == '/task_space_ik_controller/task_space'
    assert gripper.action_name == '/left_gripper_controller/gripper'
    assert replay.action_name == '/joint_trajectory_controller/follow_joint_trajectory'


def test_the_vla_completion_service_hangs_off_the_vla_action():
    # The controller creates its client as ~/vla/notify_completion.
    assert vla_completion_service_name('vla_controller') == '/vla_controller/vla/notify_completion'
    assert vla_completion_service_name('vla_mit_controller') == (
        '/vla_mit_controller/vla/notify_completion')


@pytest.mark.parametrize('controller', [None, ''])
def test_the_vla_completion_service_has_no_default_robot(controller):
    # The default used to be Franka's: an OpenArm tree that forgot to pass its
    # controller waited forever on a service its controller never calls.
    with pytest.raises(ValueError, match='VLA controller'):
        vla_completion_service_name(controller)


def test_the_gripper_leaf_has_no_default_controller():
    with pytest.raises(ValueError, match='controller_name is required'):
        GripperActionBehavior('Grip', grasp=True)


def test_an_unknown_action_name_raises_even_under_python_O():
    # A ValueError, not an assert: `python -O` strips asserts.
    with pytest.raises(ValueError, match='Invalid controller action name'):
        JointSpaceActionBehavior(
            'Typo', target_joints=make_joint_state([0.0] * 7),
            controller_name='joint_space_qp_controler')


@pytest.mark.parametrize('robot_type,profile,expected', [
    ('fr5', 'single', '/fr5_moveit_action_bridge/joint_space'),
    ('openarm', 'left', '/openarm_left_moveit_action_bridge/joint_space'),
    ('openarm', 'both', '/openarm_both_moveit_action_bridge/joint_space'),
])
def test_the_moveit_joint_action_is_the_bridges_own(robot_type, profile, expected):
    assert moveit_joint_action_name(load_robot_config(robot_type, profile)) == expected


@pytest.mark.parametrize('robot_type,profile', [
    ('franka', 'single'), ('openarm', 'single'), ('openarm', 'left'), ('ur5e', 'single')])
def test_the_bridge_is_never_mistaken_for_a_controller(robot_type, profile):
    # The preferences list the bridge's actions next to the controllers'; the
    # bridge is not something controller_manager could switch.
    names = exclusive_arm_controllers(load_robot_config(robot_type, profile))
    assert not any('moveit_action_bridge' in name for name in names)


def test_only_the_pour_action_is_named_the_old_way():
    legacy = [name for name in valid_controller_action_names()
              if 'controller_action_server' in name]
    assert legacy == ['/controller_action_server/pouring_controller']
