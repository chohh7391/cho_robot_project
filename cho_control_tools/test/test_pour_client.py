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

"""The pour client's command line, and the one thing about its shutdown that matters."""

import pytest

from cho_control_tools.clients.fr5 import pour_client


def test_container_defaults_to_reading_the_empty_vessel():
    args = pour_client.build_parser().parse_args(['--target', '50'])
    assert args.container == 'auto'
    assert args.material == 'liquid'
    assert args.flow_index == 0.0


def test_a_flow_index_outside_its_range_is_refused_before_anything_starts(monkeypatch):
    def must_not_run(**_kwargs):
        raise AssertionError('rclpy.init reached with an invalid flow_index')

    monkeypatch.setattr(pour_client.rclpy, 'init', must_not_run)
    assert pour_client.main(['--target', '50', '--flow-index', '3']) == 2


def test_ctrl_c_is_left_to_python_so_the_cancel_can_still_be_sent(monkeypatch):
    # With rclpy's default handler, SIGINT shuts the context down before the
    # KeyboardInterrupt reaches pour(), and the cancel it then sends goes out on
    # a dead context: the controller never hears it and pours on to target.
    seen = {}

    class Stop(Exception):
        pass

    def record(**kwargs):
        seen.update(kwargs)
        raise Stop

    monkeypatch.setattr(pour_client.rclpy, 'init', record)
    with pytest.raises(Stop):
        pour_client.main(['--target', '50', '--container', '139.15'])
    assert seen.get('signal_handler_options') == pour_client.SignalHandlerOptions.NO


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


def _pour_result(status, completed):
    wrapped = type('Wrapped', (), {})()
    wrapped.status = status
    wrapped.result = pour_client.Pour.Result()
    wrapped.result.is_completed = completed
    wrapped.result.message = '' if completed else 'stopped'
    return wrapped


def _interrupted_pour(monkeypatch, cancel_answer, status):
    """Run pour() with Ctrl-C arriving while the result is awaited."""
    from action_msgs.msg import GoalInfo
    from action_msgs.srv import CancelGoal

    response = CancelGoal.Response()
    response.return_code, canceling = cancel_answer
    response.goals_canceling = [GoalInfo() for _ in range(canceling)]
    result_future = _Future(_pour_result(status, completed=False))

    class Handle:
        accepted = True

        def get_result_async(self):
            return result_future

        def cancel_goal_async(self):
            return _Future(response)

    class Client:
        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal, feedback_callback=None):
            return _Future(Handle())

    calls = {'n': 0}

    def spin_until_future_complete(_node, future, **_kwargs):
        calls['n'] += 1
        if future is result_future and calls['n'] == 2:
            raise KeyboardInterrupt

    monkeypatch.setattr(pour_client.rclpy, 'spin_until_future_complete', spin_until_future_complete)
    client = object.__new__(pour_client.PourClient)
    client._client = Client()
    goal = pour_client.Pour.Goal()
    goal.target_grams = 50.0
    return client.pour(goal)


def test_a_rejected_cancel_is_reported_and_the_pour_is_not_said_to_stop(monkeypatch, capsys):
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal

    assert _interrupted_pour(monkeypatch, (CancelGoal.Response.ERROR_REJECTED, 0),
                             GoalStatus.STATUS_ABORTED) == 1
    out = capsys.readouterr().out
    assert 'Cancel REJECTED' in out
    assert 'still running' in out
    assert 'parks the vessel' not in out
    assert 'ABORTED' in out


def test_an_accepted_cancel_is_reported_with_how_the_goal_ended(monkeypatch, capsys):
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal

    assert _interrupted_pour(monkeypatch, (CancelGoal.Response.ERROR_NONE, 1),
                             GoalStatus.STATUS_CANCELED) == 1
    out = capsys.readouterr().out
    assert 'cancel accepted' in out.lower()
    assert 'CANCELED' in out
