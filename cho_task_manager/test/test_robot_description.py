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

"""Joint position limits come from the running robot description.

The FR5 replay used to carry a hand copy of fr5_macro.xacro's limits. It now
reads them from /robot_description through the same Pinocchio reader the
safety monitor uses, and a segment is refused while they are unknown.
"""

from unittest.mock import MagicMock

import pytest
from std_msgs.msg import String

from cho_task_manager.behaviors.action import FollowJointTrajectoryBehavior
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from cho_task_manager.utils.robot_description import (
    LATCHED_QOS,
    DescriptionPositionLimits,
    position_limits,
)

JOINTS = ['j1', 'j2']


def _urdf(limits=((-1.0, 1.0), (-2.0, 0.5)), joint_type='revolute'):
    links = ['<link name="base_link"/>']
    joints = []
    parent = 'base_link'
    for index, (lower, upper) in enumerate(limits):
        child = f'link{index + 1}'
        links.append(
            f'<link name="{child}"><inertial><mass value="1.0"/>'
            '<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>'
            '</inertial></link>')
        joints.append(
            f'<joint name="j{index + 1}" type="{joint_type}">'
            f'<parent link="{parent}"/><child link="{child}"/>'
            '<origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>'
            f'<limit effort="100" velocity="3" lower="{lower}" upper="{upper}"/></joint>')
        parent = child
    return '<robot name="two_joint">' + ''.join(links) + ''.join(joints) + '</robot>'


def test_limits_are_read_per_joint_from_the_urdf():
    assert position_limits(_urdf(), JOINTS) == {'j1': (-1.0, 1.0), 'j2': (-2.0, 0.5)}


def test_a_joint_the_description_does_not_have_is_an_error():
    with pytest.raises(ValueError, match="joint 'j3' is not in"):
        position_limits(_urdf(), ['j1', 'j3'])


def test_a_joint_without_a_bounded_range_is_an_error():
    with pytest.raises(ValueError, match='nq=2'):
        position_limits(_urdf(joint_type='continuous'), JOINTS)


def _source():
    source = DescriptionPositionLimits(JOINTS)
    node = MagicMock()
    source.setup(node)
    return source, node


def test_one_subscription_however_many_segments_share_the_source():
    source, node = _source()
    source.setup(node)
    source.setup(node)
    assert node.create_subscription.call_count == 1
    topic = node.create_subscription.call_args[0][1]
    assert topic == '/robot_description'


def test_the_description_is_subscribed_reliable_and_latched():
    # robot_state_publisher publishes RELIABLE + TRANSIENT_LOCAL. A best-effort
    # reader is not sent the latched sample by every DDS, and then the replay
    # refuses every segment for want of limits.
    assert LATCHED_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert LATCHED_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL
    source = DescriptionPositionLimits(JOINTS)
    node = MagicMock()
    source.setup(node)
    assert node.create_subscription.call_args[0][3] is LATCHED_QOS


def test_unknown_limits_are_a_reason_not_an_empty_dict():
    source, _ = _source()
    limits, reason = source.resolve()
    assert limits is None
    assert 'no /robot_description' in reason


def test_a_description_resolves_the_limits():
    source, _ = _source()
    source.on_description(String(data=_urdf()))
    assert source.resolve() == ({'j1': (-1.0, 1.0), 'j2': (-2.0, 0.5)}, None)


def test_an_unusable_description_is_reported():
    source, _ = _source()
    source.on_description(String(data=_urdf(joint_type='continuous')))
    limits, reason = source.resolve()
    assert limits is None
    assert 'cannot read joint limits' in reason


def _segment(source, positions):
    behaviour = FollowJointTrajectoryBehavior(
        'Replay', 'joint_trajectory_controller', JOINTS, [0.0, 1.0], positions,
        position_limits=source)
    behaviour.node = MagicMock()
    behaviour.send_action_goal = MagicMock()
    return behaviour


def test_a_segment_is_refused_while_the_limits_are_unknown():
    source, _ = _source()
    behaviour = _segment(source, [[0.0, 0.0], [0.1, 0.1]])

    behaviour.initialise()

    behaviour.send_action_goal.assert_not_called()
    assert 'no /robot_description' in behaviour.rejection


def test_a_segment_outside_the_description_limits_is_refused():
    source, _ = _source()
    source.on_description(String(data=_urdf()))
    behaviour = _segment(source, [[0.0, 0.0], [0.0, 0.6]])

    behaviour.initialise()

    behaviour.send_action_goal.assert_not_called()
    assert 'j2' in behaviour.rejection and 'outside its limits' in behaviour.rejection


def test_a_segment_inside_the_limits_is_sent():
    source, _ = _source()
    source.on_description(String(data=_urdf()))
    behaviour = _segment(source, [[0.0, 0.0], [0.5, -1.5]])

    behaviour.initialise()

    assert behaviour.rejection is None
    behaviour.send_action_goal.assert_called_once()
