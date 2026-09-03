"""Unit coverage for robot-specific operator action-client entry points."""

import pytest

from cho_control_tools.clients import operator_client
from cho_control_tools.clients import robot_action_client as clients


@pytest.mark.parametrize('entrypoint,robot_type', [
    (clients.openarm_main, 'openarm'),
    (clients.fr5_main, 'fr5'),
    (clients.franka_main, 'franka'),
    (clients.ur5e_main, 'ur5e'),
])
def test_robot_entrypoints_fix_robot_identity(monkeypatch, entrypoint, robot_type):
    calls = []
    monkeypatch.setattr(clients, '_run', lambda *args, **kwargs: calls.append((args, kwargs)))

    entrypoint()

    expected_kwargs = {'allow_arm': True} if robot_type == 'openarm' else {}
    assert calls == [((robot_type,), expected_kwargs)]


def test_openarm_profile_is_the_only_operator_option(monkeypatch):
    calls = []
    monkeypatch.setattr(operator_client, 'RobotActionShell',
                        lambda robot_type, arm, **_kwargs:
                        calls.append((robot_type, arm)) or _Shell())

    assert operator_client.run_robot_client(
        'openarm', ['--arm', 'both'], allow_arm=True) == 0
    assert calls == [('openarm', 'both')]


def test_non_openarm_rejects_robot_and_controller_selectors():
    with pytest.raises(SystemExit):
        clients._run('fr5', ['--robot-type', 'openarm'])
    with pytest.raises(SystemExit):
        clients._run('fr5', ['--task-controller', 'anything'])


def test_operator_shell_reports_readiness_without_endpoint_names(monkeypatch, capsys):
    class BaseShell:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.robot_type = kwargs['robot_type']
            self.task_action_name = None
            self.joint_space_action_client = object()
            self.task_space_action_client = object()
            self.gripper_action_client = None
            self.robotiq_command_publisher = None
            self._send_goal_and_wait = lambda *_args: True

    monkeypatch.setattr(operator_client, '_control_suite_shell', lambda: BaseShell)
    shell = operator_client.RobotActionShell('openarm')
    shell._shell.do_status('')

    output = capsys.readouterr().out
    assert 'joint-space: ready' in output
    assert 'task-space: ready' in output
    assert 'controller_action_server' not in output
    shell._shell.do_use_task('task_space_impedance_mit_controller')
    assert 'selected automatically' in capsys.readouterr().out
    assert 'do_use_task' not in shell._shell.get_names()


@pytest.mark.parametrize('endpoint', [
    '/controller_action_server/task_space_impedance_mit_controller',
    '/controller_action_server/left_task_space_impedance_mit_controller',
    '/controller_action_server/right_task_space_impedance_mit_controller',
])
def test_openarm_mit_startup_retries_only_rejected_task_goal(monkeypatch, capsys, endpoint):
    class BaseShell:
        def __init__(self, **kwargs):
            del kwargs
            self.robot_type = 'openarm'
            self.task_action_name = endpoint
            self.joint_space_action_client = object()
            self.task_space_action_client = object()
            self.gripper_action_client = None
            self.robotiq_command_publisher = None
            self.calls = 0

            def send(client, goal):
                del client, goal
                self.calls += 1
                self._last_goal_rejected = self.calls == 1
                return self.calls == 2

            self._send_goal_and_wait = send

    monkeypatch.setattr(operator_client, '_control_suite_shell', lambda: BaseShell)
    monkeypatch.setattr(operator_client.time, 'sleep', lambda _seconds: None)
    shell = operator_client.RobotActionShell('openarm')._shell

    assert shell._send_goal_and_wait(shell.task_space_action_client, object())
    assert shell.calls == 2
    assert 'Initial posture preparing' in capsys.readouterr().out


def test_openarm_mit_does_not_retry_accepted_task_failure(monkeypatch):
    class BaseShell:
        def __init__(self, **kwargs):
            del kwargs
            self.robot_type = 'openarm'
            self.task_action_name = '/controller_action_server/task_space_impedance_mit_controller'
            self.joint_space_action_client = object()
            self.task_space_action_client = object()
            self.gripper_action_client = None
            self.robotiq_command_publisher = None
            self.calls = 0

            def send(client, goal):
                del client, goal
                self.calls += 1
                self._last_goal_rejected = False
                return False

            self._send_goal_and_wait = send

    monkeypatch.setattr(operator_client, '_control_suite_shell', lambda: BaseShell)
    shell = operator_client.RobotActionShell('openarm')._shell

    assert not shell._send_goal_and_wait(shell.task_space_action_client, object())
    assert shell.calls == 1


def test_openarm_mit_startup_rejection_is_not_retried_after_launch_window(monkeypatch):
    class BaseShell:
        def __init__(self, **kwargs):
            del kwargs
            self.robot_type = 'openarm'
            self.task_action_name = '/controller_action_server/task_space_impedance_mit_controller'
            self.joint_space_action_client = object()
            self.task_space_action_client = object()
            self.gripper_action_client = None
            self.robotiq_command_publisher = None
            self.calls = 0

            def send(client, goal):
                del client, goal
                self.calls += 1
                self._last_goal_rejected = True
                return False

            self._send_goal_and_wait = send

    monkeypatch.setattr(operator_client, '_control_suite_shell', lambda: BaseShell)
    wrapper = operator_client.RobotActionShell('openarm')
    shell = wrapper._shell
    monkeypatch.setattr(operator_client.time, 'monotonic',
                        lambda: wrapper._openarm_task_startup_deadline)

    assert not shell._send_goal_and_wait(shell.task_space_action_client, object())
    assert shell.calls == 1


class _Shell:
    def cmdloop(self):
        return None
