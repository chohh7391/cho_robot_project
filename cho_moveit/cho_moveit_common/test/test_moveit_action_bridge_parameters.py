"""Real-node construction guards: optional array parameters and the node name."""

import importlib.util
from pathlib import Path

import pytest
import rclpy


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'moveit_action_bridge.py'
SPEC = importlib.util.spec_from_file_location('moveit_action_bridge_under_test', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_single_controller_launches_initialize_the_optional_array():
    # Only the OpenArm launch passes 'trajectory_controllers'; every other robot
    # brings the bridge up with the singular name alone.
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5'])
    node = None
    try:
        node = MODULE.MoveItActionBridge()
        assert node.get_parameter('trajectory_controllers').value == []
        assert node._trajectory_controllers == ['joint_trajectory_controller']
        # ~/joint_space and ~/task_space resolve to what the registry lists.
        assert node._joint_action == '/fr5_moveit_action_bridge/joint_space'
        assert node._task_action == '/fr5_moveit_action_bridge/task_space'
        # The planning budget is a parameter of its own; duration_sec is the
        # motion's minimum length, not a planning budget.
        assert node._planning_time == 5.0
        assert node._execute_action == '/execute_trajectory'
        # An absolute goal may also name the registry's arm_base_link, where TF
        # puts it at the planning frame.
        assert node._arm_base_link == 'base_link'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('remap', [
    [],  # the executable's own default node name
    ['-r', '__node:=ur5e_moveit_action_bridge'],  # another robot's bridge
    ['-r', '__ns:=/elsewhere', '-r', '__node:=fr5_moveit_action_bridge'],
])
def test_a_bridge_under_a_name_no_client_looks_for_refuses_to_start(remap):
    rclpy.init(args=['--ros-args', *remap, '-p', 'robot_type:=fr5'])
    try:
        with pytest.raises(ValueError, match='/fr5_moveit_action_bridge/joint_space'):
            MODULE.MoveItActionBridge()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('value', ['0.0', '-1.0'])
def test_a_planning_budget_that_cannot_plan_is_refused(value):
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5', '-p', f'planning_time_sec:={value}'])
    try:
        with pytest.raises(ValueError, match='planning_time_sec'):
            MODULE.MoveItActionBridge()
    finally:
        rclpy.shutdown()
