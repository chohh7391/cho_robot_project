"""Safety tests for multi-robot Cho action selection in the control-tools package."""

import pytest

from cho_control_tools.clients import action_client as MODULE


def bare_shell(robot_type):
    shell = object.__new__(MODULE.ControlSuiteShell)
    shell.robot_type = robot_type
    return shell


def test_wrong_robot_moveit_action_is_never_selected():
    shell = bare_shell('ur5e')
    wrong = '/fr5/controller_action_server/moveit_joint'
    available = {wrong: [MODULE.ACTION_TYPE_NAMES['joint']]}
    assert shell._select_action_name('joint', available, {
        'joint_trajectory_controller'}, None) is None
    generic = '/controller_action_server/moveit_joint'
    assert not shell._action_belongs_to_robot(generic)


def test_robot_scoped_moveit_action_requires_its_backend():
    shell = bare_shell('franka')
    action = '/franka/controller_action_server/moveit_joint'
    assert shell._action_has_active_backend(
        action, {'moveit_joint_trajectory_controller'})
    assert not shell._action_has_active_backend(
        action, {'joint_trajectory_controller'})


def test_direct_preference_has_no_moveit_startup_delay_dependency():
    shell = bare_shell('ur5e')
    direct = '/controller_action_server/joint_space_position_controller'
    available = {direct: [MODULE.ACTION_TYPE_NAMES['joint']]}
    assert shell._select_action_name(
        'joint', available, {'joint_space_position_controller'}, None) == direct


def test_operator_client_never_selects_available_but_inactive_task_endpoint():
    shell = bare_shell('openarm')
    shell._operator_facing = True
    task = '/controller_action_server/task_space_impedance_mit_controller'
    joint = '/controller_action_server/joint_impedance_mit_controller'
    available = {
        task: [MODULE.ACTION_TYPE_NAMES['task']],
        joint: [MODULE.ACTION_TYPE_NAMES['joint']],
    }
    active = {'joint_impedance_mit_controller'}

    assert shell._select_action_name('task', available, active, None) is None
    assert shell._select_action_name('joint', available, active, None) == joint


def test_openarm_mit_impedance_action_is_selectable_by_canonical_name(monkeypatch):
    shell = bare_shell('openarm')
    action = '/controller_action_server/joint_impedance_mit_controller'
    selected_client = object()
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        action: [MODULE.ACTION_TYPE_NAMES['joint']]})
    monkeypatch.setattr(shell, '_active_controllers', lambda timeout_sec: {
        'joint_impedance_mit_controller'})
    monkeypatch.setattr(shell, '_create_client', lambda *_args: selected_client)
    shell.do_use_joint('joint_impedance_mit_controller')
    assert shell.joint_action_name == action
    assert shell.joint_space_action_client is selected_client


def test_openarm_mit_task_impedance_action_is_selectable_by_canonical_name(monkeypatch):
    shell = bare_shell('openarm')
    action = '/controller_action_server/task_space_impedance_mit_controller'
    selected_client = object()
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        action: [MODULE.ACTION_TYPE_NAMES['task']]})
    monkeypatch.setattr(shell, '_active_controllers', lambda timeout_sec: {
        'task_space_impedance_mit_controller'})
    monkeypatch.setattr(shell, '_create_client', lambda *_args: selected_client)
    shell.do_use_task('task_space_impedance_mit_controller')
    assert shell.task_action_name == action
    assert shell.task_space_action_client is selected_client


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_openarm_bimanual_profile_discovers_its_own_direct_mit_task_endpoint(arm):
    shell = bare_shell('openarm')
    shell.arm = arm
    shell.robot_config = MODULE.load_robot_config('openarm', arm)
    shell.action_preferences = shell.robot_config['actions']['preferences']
    endpoint = f'/controller_action_server/{arm}_task_space_impedance_mit_controller'
    assert endpoint in shell.action_preferences['task']
    assert shell._action_has_active_backend(
        endpoint, {f'{arm}_task_space_impedance_mit_controller'})
    assert not shell._action_has_active_backend(
        endpoint, {f'{"right" if arm == "left" else "left"}_task_space_impedance_mit_controller'})


def test_manual_switch_rejects_wrong_robot_moveit_action(monkeypatch, capsys):
    shell = bare_shell('ur5e')
    wrong = '/fr5/controller_action_server/moveit_joint'
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        wrong: [MODULE.ACTION_TYPE_NAMES['joint']]})
    monkeypatch.setattr(
        shell, '_active_controllers', lambda timeout_sec: {'joint_trajectory_controller'})
    assert not shell._switch_client('joint', wrong)
    assert 'does not belong to robot ur5e' in capsys.readouterr().out


def test_manual_switch_rejects_inactive_moveit_backend(monkeypatch, capsys):
    shell = bare_shell('ur5e')
    action = '/ur5e/controller_action_server/moveit_joint'
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        action: [MODULE.ACTION_TYPE_NAMES['joint']]})
    monkeypatch.setattr(shell, '_active_controllers', lambda timeout_sec: set())
    assert not shell._switch_client('joint', action)
    assert 'no active controller backend' in capsys.readouterr().out


def test_manual_switch_rejects_generic_moveit_action(monkeypatch, capsys):
    shell = bare_shell('ur5e')
    generic = '/controller_action_server/moveit_joint'
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        generic: [MODULE.ACTION_TYPE_NAMES['joint']]})
    monkeypatch.setattr(
        shell, '_active_controllers', lambda timeout_sec: {'joint_trajectory_controller'})
    assert not shell._switch_client('joint', generic)
    assert 'does not belong to robot ur5e' in capsys.readouterr().out


def test_fr5_home_zero_is_rejected_before_sending_goal(capsys):
    shell = bare_shell('fr5')
    shell.joint_space_action_client = object()
    shell._send_goal_and_wait = lambda *_args: (_ for _ in ()).throw(
        AssertionError('unsafe home goal must not be sent'))
    shell.do_home('0')
    output = capsys.readouterr().out
    assert 'home 0 is disabled for fr5' in output
    assert 'floor' in output


@pytest.mark.parametrize('arm', ['single', 'left', 'right'])
def test_openarm_single_arm_reach_keeps_task_space_goal_format(arm):
    shell = bare_shell('openarm')
    shell.arm = arm
    shell.robot_config = MODULE.load_robot_config('openarm', arm)
    shell.task_space_action_client = object()
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append((client, goal)) or True

    shell.do_reach('2')

    client, goal = sent[0]
    motion = shell.robot_config['motions']['reach']['2']
    assert client is shell.task_space_action_client
    assert isinstance(goal, MODULE.TaskSpace.Goal)
    assert goal.relative is motion['relative']
    assert [goal.target_pose.position.x, goal.target_pose.position.y,
            goal.target_pose.position.z] == motion['position']
    assert [goal.target_pose.orientation.x, goal.target_pose.orientation.y,
            goal.target_pose.orientation.z,
            goal.target_pose.orientation.w] == motion['orientation']


@pytest.mark.parametrize('arm', ['single', 'left', 'right'])
def test_openarm_task_reach_is_absolute_and_idempotent_at_action_boundary(arm):
    shell = bare_shell('openarm')
    shell.arm = arm
    shell.robot_config = MODULE.load_robot_config('openarm', arm)
    shell.task_space_action_client = object()
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append((client, goal)) or True

    shell.do_reach('0')
    shell.do_reach('0')

    assert len(sent) == 2
    first, second = (item[1] for item in sent)
    assert first.relative is False
    assert second.relative is False
    assert first.target_pose.position == second.target_pose.position
    assert first.target_pose.orientation == second.target_pose.orientation


def test_openarm_both_reach_sends_all_registered_14_joint_goals(capsys):
    shell = bare_shell('openarm')
    shell.arm = 'both'
    shell.robot_config = MODULE.load_robot_config('openarm', 'both')
    shell.joint_space_action_client = object()
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append((client, goal)) or True

    for selector in ('0', '1', '2', '3'):
        shell.do_reach(selector)

    assert len(sent) == 4
    for selector, (client, goal) in zip(('0', '1', '2', '3'), sent):
        assert client is shell.joint_space_action_client
        assert isinstance(goal, MODULE.JointSpace.Goal)
        assert list(goal.target_joints.position) == (
            shell.robot_config['poses']['reach'][selector])
        assert len(goal.target_joints.position) == 14
    assert capsys.readouterr().out.count('action succeed') == 4


def test_openarm_mit_selected_home_and_reach_keep_joint_space_goal_contract():
    shell = bare_shell('openarm')
    shell.arm = 'single'
    shell.robot_config = MODULE.load_robot_config('openarm', 'single')
    shell.joint_space_action_client = object()
    shell.task_space_action_client = None
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append((client, goal)) or True

    shell.do_home('1')
    shell.do_reach('1')

    assert len(sent) == 2
    for client, goal in sent:
        assert client is shell.joint_space_action_client
        assert isinstance(goal, MODULE.JointSpace.Goal)
        assert goal.duration == 5.0
        assert len(goal.target_joints.position) == 7
    assert list(sent[0][1].target_joints.position) == shell.robot_config['poses']['home']['1']
    assert list(sent[1][1].target_joints.position) == shell.robot_config['poses']['reach']['1']
