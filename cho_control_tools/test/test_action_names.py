"""The operator tools' copy of the action naming rule, and the PoseLog reader.

The robot-scoped clients cannot import cho_robot_config (they must run in a
workspace that does not install it), so they carry their own copy of the rule in
cho_interfaces/CONTRACT.md. These tests pin that copy to the registry's.
"""

import struct

import pytest
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from cho_control_tools import action_names
from cho_control_tools.plotting.pose_log import decode_pose_log


@pytest.mark.parametrize('space,kind', [
    ('joint', 'joint_space'), ('task', 'task_space'), ('gripper', 'gripper'), ('vla', 'vla')])
def test_a_controller_serves_each_space_under_its_own_node(space, kind):
    expected = f'/left_vla_mit_controller/{kind}'
    assert action_names.controller_action_name('left_vla_mit_controller', space) == expected
    assert action_names.controller_action_name('left_vla_mit_controller', kind) == expected


def test_an_unknown_kind_is_refused():
    with pytest.raises(ValueError, match='unknown action kind'):
        action_names.controller_action_name('gripper_controller', 'moveit_joint')


def test_the_name_parts_round_trip():
    name = action_names.controller_action_name('openarm_left_moveit_action_bridge', 'joint')
    assert action_names.serving_node(name) == 'openarm_left_moveit_action_bridge'
    assert action_names.action_kind(name) == 'joint_space'


@pytest.mark.parametrize('robot_type,profile', [
    ('fr5', 'single'), ('franka', 'single'), ('ur5e', 'single'),
    ('openarm', 'single'), ('openarm', 'left'), ('openarm', 'both')])
def test_the_copy_agrees_with_the_registry(robot_type, profile):
    try:
        import cho_robot_config
    except ImportError:
        pytest.skip('central registry is not installed in this robot workspace')
    assert action_names.moveit_bridge_node(robot_type, profile) == (
        cho_robot_config.moveit_bridge_node(robot_type, profile))
    for space, kind in action_names.ACTION_KINDS.items():
        assert action_names.controller_action_name('a_controller', space) == (
            cho_robot_config.controller_action_name('a_controller', kind))


def _pose_values(base):
    return [base + offset for offset in (0.1, 0.2, 0.3)] + [0.0, 0.0, 0.0, 1.0]


def test_a_stamped_pose_log_is_read_as_is():
    pose_log = get_message('cho_interfaces/msg/PoseLog')
    msg = pose_log()
    msg.header.frame_id = 'fr3_link0'
    msg.pose_des.position.x = 0.4
    msg.pose_curr.position.z = 0.3
    decoded = decode_pose_log(serialize_message(msg), pose_log)
    assert decoded.header.frame_id == 'fr3_link0'
    assert (decoded.pose_des.position.x, decoded.pose_curr.position.z) == (0.4, 0.3)


@pytest.mark.parametrize('encapsulation,endian', [(b'\x00\x01\x00\x00', '<'),
                                                  (b'\x00\x00\x00\x00', '>')])
def test_a_pose_log_recorded_before_the_header_still_reads(encapsulation, endian):
    # Three bare Poses (ref, des, curr), the layout bags recorded before
    # PoseLog was stamped carry.
    values = _pose_values(1.0) + _pose_values(2.0) + _pose_values(3.0)
    data = encapsulation + struct.pack(f'{endian}21d', *values)
    decoded = decode_pose_log(data, get_message('cho_interfaces/msg/PoseLog'))
    assert decoded.pose_ref.position.x == pytest.approx(1.1)
    assert decoded.pose_des.position.y == pytest.approx(2.2)
    assert decoded.pose_curr.position.z == pytest.approx(3.3)
    assert decoded.pose_curr.orientation.w == pytest.approx(1.0)
