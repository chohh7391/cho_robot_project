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

"""teleop_bridge's robot side: which controllers an arm resolves to, and what each chunk says."""

import pytest

from cho_control_tools.teleop_bridge.chunk_streamer import (
    ArmTarget, build_chunk, gripper_width, resolve_arm)


def _registry():
    try:
        import cho_robot_config
    except ImportError:
        pytest.skip('cho_robot_config is not installed in this workspace')
    return cho_robot_config


def fake_registry(config):
    def load(robot, profile):
        return config

    def goal_frame(loaded, relative):
        assert relative is False
        return (loaded.get('model') or {}).get('absolute_goal_frame') or ''
    return load, goal_frame


def test_a_chunk_holds_one_absolute_pose_for_every_waypoint():
    msg = build_chunk((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0), 3, 1.0 / 60.0, 7)
    assert msg.seq == 7
    assert msg.action_space == 'task'
    assert msg.relative_mode == 'absolute'
    assert msg.rotation_type == 'quaternion'
    assert msg.chunk_size == 3
    assert msg.control_dt == pytest.approx(1.0 / 60.0)
    assert list(msg.arm_actions) == pytest.approx([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0] * 3)
    assert len(msg.gripper_actions) == 0
    # Unstamped: timed by arrival.
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (0, 0)


@pytest.mark.parametrize('trigger,width', [(0.0, 0.095), (1.0, 0.0), (0.5, 0.0475), (-1.0, 0.095), (2.0, 0.0)])
def test_the_trigger_closes_the_gripper(trigger, width):
    assert gripper_width(trigger, 0.095, 0.0) == pytest.approx(width)


def test_an_arm_resolves_from_its_profile():
    load, goal_frame = fake_registry({
        'controllers': {'action_chunk': 'left_action_chunk_controller', 'gripper': 'left_gripper_controller'},
        'model': {'base_frame': 'world', 'absolute_goal_frame': 'world', 'ee_link': 'left_tcp'},
    })
    assert resolve_arm('robot', 'left', load, goal_frame) == ArmTarget(
        'left', 'left_action_chunk_controller', 'left_gripper_controller', 'world', 'left_tcp')


def test_without_an_absolute_goal_frame_the_base_frame_is_used():
    load, goal_frame = fake_registry({
        'controllers': {'action_chunk': 'action_chunk_controller', 'gripper': None},
        'model': {'base_frame': 'base', 'ee_link': 'tcp'},
    })
    target = resolve_arm('robot', 'single', load, goal_frame)
    assert (target.goal_frame, target.gripper_controller) == ('base', None)


def test_a_profile_of_several_arms_is_refused_naming_its_arms():
    load, goal_frame = fake_registry({'controllers': {'action_chunk': None}, 'arms': ['left', 'right'],
                                      'model': {'base_frame': 'world', 'ee_link': 'tcp'}})
    with pytest.raises(ValueError, match='--arms left right'):
        resolve_arm('robot', 'both', load, goal_frame)


def test_an_arm_without_an_action_chunk_controller_is_refused():
    load, goal_frame = fake_registry({'controllers': {'action_chunk': None},
                                      'model': {'base_frame': 'world', 'ee_link': 'tcp'}})
    with pytest.raises(ValueError, match='controllers.action_chunk'):
        resolve_arm('robot', 'single', load, goal_frame)


def test_franka_from_the_registry():
    registry = _registry()
    target = resolve_arm('franka', 'single', registry.load_robot_config, registry.task_goal_frame)
    assert target == ArmTarget('single', 'action_chunk_controller', 'gripper_controller', 'fr3_link0', 'fr3_hand_tcp')


@pytest.mark.parametrize('side', ['left', 'right'])
def test_each_openarm_arm_from_the_registry(side):
    registry = _registry()
    target = resolve_arm('openarm', side, registry.load_robot_config, registry.task_goal_frame)
    assert target == ArmTarget(side, f'{side}_action_chunk_mit_controller', f'{side}_gripper_controller',
                               'world', f'openarm_{side}_hand_tcp')


def test_openarm_both_is_refused():
    registry = _registry()
    with pytest.raises(ValueError, match='--arms left right'):
        resolve_arm('openarm', 'both', registry.load_robot_config, registry.task_goal_frame)
