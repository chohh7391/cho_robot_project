"""The pouring controller's registry wiring, which two packages have to agree on.

The controller itself is C++ and is exercised against a running arm. What is
checked here is the part that fails silently: whether the rest of the stack
knows the controller exists. If the pour role does not reach the exclusive set,
switching another controller in never takes the pouring one down, both claim the
same command interfaces, and controller_manager refuses the switch -- leaving
the arm on the controller a failed pour was driving.
"""

from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    load_robot_config,
    valid_controller_action_names,
)


POUR_CONTROLLER = 'pouring_controller'


def test_the_registry_gives_fr5_a_pour_controller():
    assert load_robot_config('fr5').get('pour') == POUR_CONTROLLER


def test_switching_away_takes_the_pour_controller_down():
    config = load_robot_config('fr5')
    assert POUR_CONTROLLER in exclusive_arm_controllers(config)


def test_a_pour_goal_is_addressable_from_a_behaviour():
    # BaseActionBehavior asserts its action name against this list, so a role
    # the compatibility view drops is a behaviour that cannot be constructed.
    assert f'/controller_action_server/{POUR_CONTROLLER}' in valid_controller_action_names()


def test_robots_without_a_scale_declare_no_pour():
    for robot in ('franka', 'ur5e', 'openarm'):
        assert load_robot_config(robot).get('pour') is None
