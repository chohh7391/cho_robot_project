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


# ------------------------------------------- AC7: the 'both' profile has no VLA role

def test_ac7_the_both_profile_has_no_vla_role_in_the_registry():
    registry = _registry()
    assert registry.load_robot_config('openarm', 'both')['controllers']['vla'] is None


def test_ac7_resolving_openarm_both_is_refused_and_says_vla():
    # Two arms stream two chunk topics to two controllers; "the" VLA controller does not exist.
    _registry()
    with pytest.raises((SystemExit, ValueError)) as error:
        MODULE.resolve_controller(robot='openarm', arm='both')
    assert 'vla' in str(error.value).lower()


def test_ac7_main_for_openarm_both_exits_with_a_vla_message_before_ros_starts(monkeypatch):
    _registry()

    def must_not_run(**_kwargs):
        raise AssertionError('rclpy.init reached with no VLA controller to drive')

    monkeypatch.setattr(MODULE.rclpy, 'init', must_not_run)
    with pytest.raises(SystemExit) as error:
        MODULE.main(['--robot', 'openarm', '--arm', 'both'])
    # A message, not a bare status code: SystemExit(0)/SystemExit(2) would say nothing about why.
    assert isinstance(error.value.code, str), error.value.code
    assert 'vla' in error.value.code.lower()


def test_ac7_the_per_arm_profiles_still_resolve():
    _registry()
    assert MODULE.resolve_controller(robot='openarm', arm='left') == 'left_vla_mit_controller'
    assert MODULE.resolve_controller(robot='openarm', arm='right') == 'right_vla_mit_controller'


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


class _Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def _circle_tester(relative):
    """A VLAActionTester with the stream's own shape and no ROS, publishing into a list."""
    from builtin_interfaces.msg import Time

    class _Clock:
        def now(self):
            return type('Now', (), {'to_msg': staticmethod(Time)})()

    tester = MODULE.VLAActionTester.__new__(MODULE.VLAActionTester)
    tester.goal_accepted = True
    tester.test_rotation_type = 'quaternion'
    tester.is_relative = relative
    tester.count = 0
    tester.chunk_size = MODULE.CHUNK_SIZE
    tester.inference_dt = MODULE.INFERENCE_DT
    tester.dt = MODULE.WAYPOINT_DT
    tester.publisher_ = _Publisher()
    tester.get_clock = _Clock
    tester.get_logger = lambda: type('Logger', (), {'info': staticmethod(lambda _m: None)})()
    return tester


@pytest.mark.parametrize('relative', [True, False])
def test_the_circle_keeps_the_gripper_open(relative):
    # In binary mode > 0 opens and < 0 closes (ActionChunk.msg). The client sent
    # -1.0 as "open", which closed a Franka hand on the first chunk.
    tester = _circle_tester(relative)
    tester.publish_action_chunk()

    (msg,) = tester.publisher_.sent
    assert msg.gripper_mode == 'binary'
    assert len(msg.gripper_actions) == msg.chunk_size
    assert all(value > 0.0 for value in msg.gripper_actions)


@pytest.mark.parametrize('relative', [True, False])
def test_each_chunk_plans_past_the_next_one_and_the_blend_into_it(relative):
    # A chunk that ended as the next one arrived left the controller holding its
    # last waypoint at rest in between, and the controller's 0.1 s blend
    # (chunk_blend_duration) mixed that hold into every chunk: the circle ran
    # stop-and-go at about 5 Hz (2026-10-22, FR3).
    blend = 0.1
    tester = _circle_tester(relative)
    tester.publish_action_chunk()
    tester.publish_action_chunk()
    first, second = tester.publisher_.sent
    assert first.control_dt == pytest.approx(MODULE.WAYPOINT_DT)
    assert (first.chunk_size - 1) * first.control_dt >= MODULE.INFERENCE_DT + blend

    # The second chunk goes on along the same circle: its waypoint i is the
    # first's waypoint i + one inference period, relative chunks each from their
    # own start.
    step = round(MODULE.INFERENCE_DT / MODULE.WAYPOINT_DT)

    def positions(msg):
        values = list(msg.arm_actions)
        return [values[7 * i:7 * i + 3] for i in range(msg.chunk_size)]

    a, b = positions(first), positions(second)
    for i in range(first.chunk_size - step):
        expected = [a[i + step][k] - (a[step][k] if relative else 0.0) for k in range(3)]
        assert b[i] == pytest.approx(expected, abs=1e-12)
