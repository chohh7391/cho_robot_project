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

"""Spawner construction and the order the spawners run in."""

from cho_bringup_common import (
    chain_spawners,
    create_controller_spawners,
    make_spawner_node,
    top_level_spawner,
)
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import pytest


def arguments(node):
    """A Node's arguments as plain strings (it keeps them as substitutions)."""
    rendered = []
    for argument in node._Node__arguments:
        pieces = argument if isinstance(argument, (list, tuple)) else [argument]
        rendered.append(''.join(str(getattr(piece, 'text', piece)) for piece in pieces))
    return rendered


def followers(action):
    """The actions an OnProcessExit RegisterEventHandler starts."""
    assert isinstance(action, RegisterEventHandler)
    handler = action.event_handler
    assert isinstance(handler, OnProcessExit)
    return handler._OnActionEventBase__actions_on_event


def test_spawner_arguments():
    node = make_spawner_node(['a', 'b'], '/tmp/p.yaml', active=False, timeout=60,
                             controller_manager='/controller_manager')
    assert arguments(node) == [
        'a', 'b', '-p', '/tmp/p.yaml', '--controller-manager', '/controller_manager',
        '--inactive', '--controller-manager-timeout', '60']
    assert node._Node__package == 'controller_manager'
    assert node._Node__node_executable == 'spawner'


def test_a_bare_spawner_has_no_options_and_no_parameters():
    node = make_spawner_node(['a'])
    assert arguments(node) == ['a']
    assert not node._Node__parameters


def test_use_sim_time_namespace_and_condition_are_forwarded():
    condition = IfCondition('true')
    node = make_spawner_node(['a'], use_sim_time={'use_sim_time': True}, namespace='ns',
                             condition=condition)
    assert node._Node__parameters
    assert node._Node__node_namespace is not None
    assert node.condition is condition


def test_chain_without_followers_is_just_the_spawner():
    first = make_spawner_node(['a'])
    assert chain_spawners(first, []) == [first]
    assert chain_spawners(first, [None]) == [first]


def test_chain_registers_the_handler_before_its_target():
    first, second = make_spawner_node(['a']), make_spawner_node(['b'])
    actions = chain_spawners(first, [second])
    assert actions[1] is first
    assert followers(actions[0]) == [second]
    assert actions[0].event_handler._OnActionEventBase__action_matcher is first


def test_broadcasters_then_requested_controller_active_the_rest_inactive():
    actions = create_controller_spawners(
        always_active=['joint_state_broadcaster', 'ee_state_broadcaster'],
        switchable_controllers=['x', 'y', 'z', 'y'],
        initial_active_controllers='y',
        timeout=60)
    active = top_level_spawner(actions)
    assert arguments(active)[:3] == ['joint_state_broadcaster', 'ee_state_broadcaster', 'y']
    assert '--inactive' not in arguments(active)
    (inactive,) = followers(actions[0])
    assert arguments(inactive)[:2] == ['x', 'z']
    assert '--inactive' in arguments(inactive)


def test_a_requested_controller_that_is_not_switchable_is_not_activated():
    actions = create_controller_spawners(['jsb'], ['x'], ['not_there'])
    assert arguments(top_level_spawner(actions)) == ['jsb']


def test_every_initial_controller_is_active_on_a_bimanual_build():
    actions = create_controller_spawners(
        ['jsb'], ['left_c', 'right_c', 'left_d', 'right_d'], ['left_c', 'right_c'])
    assert arguments(top_level_spawner(actions)) == ['jsb', 'left_c', 'right_c']
    (inactive,) = followers(actions[0])
    assert arguments(inactive)[:2] == ['left_d', 'right_d']


def test_optional_controllers_get_their_own_spawner_after_the_arm():
    # A spawner dies on the first failure in its list, so a gripper sharing the
    # arm's list could keep the arm controller from ever being spawned.
    actions = create_controller_spawners(
        ['jsb'], ['arm'], ['arm'], optional_controllers=['gripper', 'jsb'])
    assert arguments(top_level_spawner(actions)) == ['jsb', 'arm']
    (optional,) = followers(actions[0])
    assert arguments(optional) == ['gripper']
    assert '--inactive' not in arguments(optional)


def test_nothing_to_follow_returns_the_active_spawner_alone():
    actions = create_controller_spawners(['jsb'], ['arm'], ['arm'])
    assert len(actions) == 1 and isinstance(actions[0], Node)


def test_runtime_param_file_and_manager_reach_every_spawner():
    actions = create_controller_spawners(
        ['jsb'], ['a', 'b'], ['a'], runtime_param_file='/tmp/p.yaml',
        controller_manager='/controller_manager', optional_controllers=['g'])
    for node in [top_level_spawner(actions)] + followers(actions[0]):
        rendered = arguments(node)
        assert rendered[rendered.index('-p') + 1] == '/tmp/p.yaml'
        assert rendered[rendered.index('--controller-manager') + 1] == '/controller_manager'


def test_top_level_spawner_needs_exactly_one():
    with pytest.raises(RuntimeError, match='exactly one'):
        top_level_spawner([make_spawner_node(['a']), make_spawner_node(['b'])])
    with pytest.raises(RuntimeError, match='exactly one'):
        top_level_spawner([])
