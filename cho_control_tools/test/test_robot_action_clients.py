"""Unit coverage for robot-specific operator action-client entry points."""

import pytest

from cho_control_tools.clients import operator_client
from cho_control_tools.clients.fr5 import action_client as fr5_client
from cho_control_tools.clients.franka import action_client as franka_client
from cho_control_tools.clients.openarm import action_client as openarm_client
from cho_control_tools.clients.openarm import metadata as openarm_metadata
from cho_control_tools.clients.ur5e import action_client as ur5e_client


@pytest.mark.parametrize('module,robot_type,allow_arm', [
    (openarm_client, 'openarm', True),
    (fr5_client, 'fr5', False),
    (franka_client, 'franka', False),
    (ur5e_client, 'ur5e', False),
])
def test_robot_entrypoints_fix_robot_identity(
        monkeypatch, module, robot_type, allow_arm):
    calls = []

    def fake_run(*args, **kwargs):
        calls.append((args, kwargs))
        return 0

    monkeypatch.setattr(module, 'run_robot_client', fake_run)

    assert module.main() == 0

    args, kwargs = calls[0]
    assert args == (robot_type, None)
    if allow_arm:
        assert kwargs['allow_arm'] is True
    else:
        assert 'allow_arm' not in kwargs
    assert callable(kwargs['robot_config_loader'])
    assert callable(kwargs['home_pose_policy_loader'])


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
        operator_client.run_robot_client('fr5', ['--robot-type', 'openarm'])
    with pytest.raises(SystemExit):
        operator_client.run_robot_client('fr5', ['--task-controller', 'anything'])


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


@pytest.mark.parametrize('endpoint,profile,forward_bend', [
    ('/controller_action_server/task_space_impedance_mit_controller', 'single',
     ((0.402, 0.0, 0.3425),
      (0.0, 0.707106781187, 0.0, 0.707106781187))),
    ('/controller_action_server/left_task_space_impedance_mit_controller', 'left',
     ((0.402, 0.153499191895, 0.477999550034),
      (0.707106781185, 0.000001298672,
       0.707106781185, 0.000001298672))),
    ('/controller_action_server/right_task_space_impedance_mit_controller', 'right',
     ((0.402, -0.153499191895, 0.477999550034),
      (0.707106781185, -0.000001298672,
       0.707106781185, -0.000001298672))),
])
def test_openarm_mit_task_endpoint_uses_relative_probes_and_absolute_forward_bend(
        monkeypatch, endpoint, profile, forward_bend):
    class BaseShell:
        def __init__(self, **kwargs):
            self.arm = kwargs.get('arm', 'single')
            self.robot_type = 'openarm'
            self.task_action_name = endpoint
            self.robot_config = openarm_metadata.load(self.arm)
            self.joint_space_action_client = object()
            self.task_space_action_client = object()
            self.gripper_action_client = None
            self.robotiq_command_publisher = None
            self._send_goal_and_wait = lambda *_args: True

    monkeypatch.setattr(operator_client, '_control_suite_shell', lambda: BaseShell)
    shell = operator_client.RobotActionShell('openarm', profile)._shell

    reach = shell.robot_config['motions']['reach']
    expected_relative = {
        '0': {'relative': True, 'position': [0.045, 0.0, 0.0],
              'orientation': [0.0, 0.0, 0.0, 1.0]},
        '1': {'relative': True, 'position': [-0.040, 0.015, 0.0],
              'orientation': [0.0998334166, 0.0, 0.0, 0.9950041653]},
        '2': {'relative': True, 'position': [0.010, 0.040, -0.015],
              'orientation': [0.0, -0.1246747334, 0.0, 0.9921976672]},
    }
    assert set(reach) == {'0', '1', '2', '3'}
    assert {selector: reach[selector] for selector in ('0', '1', '2')} == expected_relative
    assert reach['3'] == {
        'relative': False,
        'position': list(forward_bend[0]),
        'orientation': list(forward_bend[1]),
    }

    # Selectors 0--2 remain TCP-local relative actions. Direct task admission
    # intentionally has no Cartesian translation/orientation/speed limit;
    # runtime DLS clamps every reference to the upstream joint limits instead.
    for selector in ('0', '1', '2'):
        motion = reach[selector]
        assert motion['relative']

    # Selector 3 is an absolute, profile-specific world-frame FK target.
    assert not reach['3']['relative']


def test_openarm_non_mit_task_metadata_keeps_its_absolute_presets(monkeypatch):
    class BaseShell:
        def __init__(self, **kwargs):
            del kwargs
            self.robot_type = 'openarm'
            self.task_action_name = '/openarm/controller_action_server/moveit_task'
            self.robot_config = openarm_metadata.load('single')
            self.joint_space_action_client = object()
            self.task_space_action_client = object()
            self.gripper_action_client = None
            self.robotiq_command_publisher = None
            self._send_goal_and_wait = lambda *_args: True

    monkeypatch.setattr(operator_client, '_control_suite_shell', lambda: BaseShell)
    shell = operator_client.RobotActionShell('openarm')._shell

    assert all(not motion['relative']
               for motion in shell.robot_config['motions']['reach'].values())


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
