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

"""Which VLA controller the synthetic-stream client drives, and where its chunks go."""

import sys

import pytest

from cho_control_tools.vla import action_client as MODULE


def _registry():
    try:
        import cho_robot_config
    except ImportError:
        pytest.skip('cho_robot_config is not installed in this workspace')
    return cho_robot_config


def test_without_arguments_it_drives_frankas_vla_controller():
    assert MODULE.resolve_controller() == 'vla_controller'


def test_an_explicit_controller_is_used_as_given():
    assert MODULE.resolve_controller(controller='my_vla') == 'my_vla'


@pytest.mark.parametrize('robot,arm', [
    ('franka', 'single'), ('openarm', 'single'), ('openarm', 'left'), ('openarm', 'right'),
])
def test_a_robot_resolves_to_its_registry_vla_role(robot, arm):
    registry = _registry()
    expected = registry.load_robot_config(robot, arm)['controllers']['vla']
    assert expected
    assert MODULE.resolve_controller(robot=robot, arm=arm) == expected


def test_openarm_resolves_to_its_mit_vla_controller():
    _registry()
    assert MODULE.resolve_controller(robot='openarm') == 'vla_mit_controller'
    assert MODULE.resolve_controller(robot='openarm', arm='left') == 'left_vla_mit_controller'


@pytest.mark.parametrize('robot', ['fr5', 'ur5e'])
def test_a_robot_without_a_vla_controller_is_refused(robot):
    _registry()
    with pytest.raises(ValueError, match='no VLA controller'):
        MODULE.resolve_controller(robot=robot)


def test_the_registry_is_read_through_the_loader_it_is_given():
    seen = []

    def loader(robot, arm):
        seen.append((robot, arm))
        return {'controllers': {'vla': 'right_vla_mit_controller'}}

    assert MODULE.resolve_controller(
        robot='openarm', arm='right', load_robot_config=loader) == 'right_vla_mit_controller'
    assert seen == [('openarm', 'right')]


def test_without_the_registry_a_robot_cannot_be_resolved(monkeypatch):
    monkeypatch.setitem(sys.modules, 'cho_robot_config', None)
    with pytest.raises(ValueError, match='--controller'):
        MODULE.resolve_controller(robot='openarm')


def test_an_arm_needs_a_robot():
    with pytest.raises(ValueError, match='--robot'):
        MODULE.resolve_controller(arm='left')


def test_controller_and_robot_are_exclusive():
    with pytest.raises(SystemExit):
        MODULE.build_parser().parse_args(['--controller', 'x', '--robot', 'openarm'])


def test_main_hands_the_resolved_controller_and_topic_to_the_node(monkeypatch):
    built = {}

    class Tester:
        def __init__(self, controller, chunk_topic):
            built.update(controller=controller, chunk_topic=chunk_topic)

        def destroy_node(self):
            pass

    monkeypatch.setattr(MODULE, 'VLAActionTester', Tester)
    monkeypatch.setattr(MODULE.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(MODULE.rclpy, 'spin', lambda _node: None)
    monkeypatch.setattr(MODULE.rclpy, 'try_shutdown', lambda: None)
    loader = {'controllers': {'vla': 'left_vla_mit_controller'}}
    monkeypatch.setattr(MODULE, '_registry_loader', lambda: (lambda _robot, _arm: loader))

    assert MODULE.main(['--robot', 'openarm', '--arm', 'left']) == 0
    assert built == {'controller': 'left_vla_mit_controller', 'chunk_topic': None}

    assert MODULE.main(['--controller', 'vla_controller', '--chunk-topic', '/x']) == 0
    assert built == {'controller': 'vla_controller', 'chunk_topic': '/x'}


def test_a_refused_resolution_exits_before_ros_starts(monkeypatch, capsys):
    def must_not_run(**_kwargs):
        raise AssertionError('rclpy.init reached with an unresolvable controller')

    monkeypatch.setattr(MODULE.rclpy, 'init', must_not_run)
    monkeypatch.setattr(MODULE, '_registry_loader', lambda: (
        lambda _robot, _arm: {'controllers': {'vla': None}}))
    assert MODULE.main(['--robot', 'fr5']) == 2
    assert 'no VLA controller' in capsys.readouterr().err


# ------------------------------------------------------------ chunk topic

class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ParamClient:
    def __init__(self, values, available=True):
        self.values = values
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, request):
        from rcl_interfaces.srv import GetParameters
        self.requests.append(request)
        response = GetParameters.Response()
        response.values = self.values
        return _Future(response)


class _Node:
    def __init__(self, client):
        self.client = client
        self.services = []
        self.destroyed = []

    def create_client(self, _type, name):
        self.services.append(name)
        return self.client

    def destroy_client(self, client):
        self.destroyed.append(client)


def _string_value(text):
    from rcl_interfaces.msg import ParameterType, ParameterValue
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=text)


def test_the_chunk_topic_is_asked_of_the_controller(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'spin_until_future_complete', lambda *_a, **_k: None)
    client = _ParamClient([_string_value('/vla/action/left')])
    node = _Node(client)
    assert MODULE.controller_chunk_topic(node, 'left_vla_mit_controller') == '/vla/action/left'
    assert node.services == ['/left_vla_mit_controller/get_parameters']
    assert list(client.requests[0].names) == ['chunk_topic']
    assert node.destroyed == [client]


@pytest.mark.parametrize('available', [True, False])
def test_an_unanswered_chunk_topic_query_gives_none(monkeypatch, available):
    # Up with no such parameter (NOT_SET comes back), or not up at all.
    from rcl_interfaces.msg import ParameterValue
    monkeypatch.setattr(MODULE.rclpy, 'spin_until_future_complete', lambda *_a, **_k: None)
    node = _Node(_ParamClient([ParameterValue()], available))
    assert MODULE.controller_chunk_topic(node, 'vla_controller') is None
