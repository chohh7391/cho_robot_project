"""Real-node construction guard for the bridge's optional array parameters."""

import importlib.util
from pathlib import Path

import rclpy


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'moveit_action_bridge.py'
SPEC = importlib.util.spec_from_file_location('moveit_action_bridge_under_test', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_single_controller_launches_initialize_the_optional_array():
    # Only the OpenArm launch passes 'trajectory_controllers'; every other robot
    # brings the bridge up with the singular name alone.
    rclpy.init(args=['--ros-args', '-p', 'robot_type:=fr5'])
    node = None
    try:
        node = MODULE.MoveItActionBridge()
        assert node.get_parameter('trajectory_controllers').value == []
        assert node._trajectory_controllers == ['joint_trajectory_controller']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
