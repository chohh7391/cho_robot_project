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

"""Safety tests for multi-robot Cho action selection in the control-tools package."""

import pytest

from cho_control_tools.clients import action_client as MODULE


def bare_shell(robot_type):
    shell = object.__new__(MODULE.ControlSuiteShell)
    shell.robot_type = robot_type
    return shell


def test_wrong_robot_moveit_action_is_never_selected():
    shell = bare_shell('ur5e')
    wrong = '/fr5_moveit_action_bridge/joint_space'
    available = {wrong: [MODULE.ACTION_TYPE_NAMES['joint']]}
    assert shell._select_action_name('joint', available, {
        'joint_trajectory_controller'}, None) is None
    generic = '/moveit_action_bridge/joint_space'
    assert not shell._action_belongs_to_robot(generic)


@pytest.mark.parametrize('stale', [
    '/controller_action_server/joint_space_position_controller',
    '/ur5e/controller_action_server/moveit_joint',
])
def test_a_pre_contract_action_name_is_never_selected(stale):
    shell = bare_shell('ur5e')
    available = {stale: [MODULE.ACTION_TYPE_NAMES['joint']]}
    assert shell._select_action_name(
        'joint', available, {'joint_space_position_controller'}, None) is None


def test_a_controller_name_resolves_to_its_own_action_for_each_space():
    normalize = MODULE.ControlSuiteShell._normalize_action_name
    assert normalize('joint_space_qp_controller', 'joint') == (
        '/joint_space_qp_controller/joint_space')
    assert normalize('task_space_ik_controller', 'task') == '/task_space_ik_controller/task_space'
    assert normalize('left_gripper_controller', 'gripper') == '/left_gripper_controller/gripper'
    assert normalize('/already/absolute', 'joint') == '/already/absolute'


def test_robot_scoped_moveit_action_requires_its_backend():
    shell = bare_shell('franka')
    action = '/franka_moveit_action_bridge/joint_space'
    assert shell._action_has_active_backend(
        action, {'moveit_joint_trajectory_controller'})
    assert not shell._action_has_active_backend(
        action, {'joint_trajectory_controller'})


def test_direct_preference_has_no_moveit_startup_delay_dependency():
    shell = bare_shell('ur5e')
    direct = '/joint_space_position_controller/joint_space'
    available = {direct: [MODULE.ACTION_TYPE_NAMES['joint']]}
    assert shell._select_action_name(
        'joint', available, {'joint_space_position_controller'}, None) == direct


def test_operator_client_never_selects_available_but_inactive_task_endpoint():
    shell = bare_shell('openarm')
    shell._operator_facing = True
    task = '/task_space_impedance_mit_controller/task_space'
    joint = '/joint_impedance_mit_controller/joint_space'
    available = {
        task: [MODULE.ACTION_TYPE_NAMES['task']],
        joint: [MODULE.ACTION_TYPE_NAMES['joint']],
    }
    active = {'joint_impedance_mit_controller'}

    assert shell._select_action_name('task', available, active, None) is None
    assert shell._select_action_name('joint', available, active, None) == joint


def test_openarm_mit_impedance_action_is_selectable_by_canonical_name(monkeypatch):
    shell = bare_shell('openarm')
    action = '/joint_impedance_mit_controller/joint_space'
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
    action = '/task_space_impedance_mit_controller/task_space'
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
    endpoint = f'/{arm}_task_space_impedance_mit_controller/task_space'
    assert endpoint in shell.action_preferences['task']
    assert shell._action_has_active_backend(
        endpoint, {f'{arm}_task_space_impedance_mit_controller'})
    assert not shell._action_has_active_backend(
        endpoint, {f'{"right" if arm == "left" else "left"}_task_space_impedance_mit_controller'})


def test_manual_switch_rejects_wrong_robot_moveit_action(monkeypatch, capsys):
    shell = bare_shell('ur5e')
    wrong = '/fr5_moveit_action_bridge/joint_space'
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        wrong: [MODULE.ACTION_TYPE_NAMES['joint']]})
    monkeypatch.setattr(
        shell, '_active_controllers', lambda timeout_sec: {'joint_trajectory_controller'})
    assert not shell._switch_client('joint', wrong)
    assert 'does not belong to robot ur5e' in capsys.readouterr().out


def test_manual_switch_rejects_inactive_moveit_backend(monkeypatch, capsys):
    shell = bare_shell('ur5e')
    action = '/ur5e_moveit_action_bridge/joint_space'
    monkeypatch.setattr(shell, '_discover_action_servers', lambda timeout_sec: {
        action: [MODULE.ACTION_TYPE_NAMES['joint']]})
    monkeypatch.setattr(shell, '_active_controllers', lambda timeout_sec: set())
    assert not shell._switch_client('joint', action)
    assert 'no active controller backend' in capsys.readouterr().out


def test_manual_switch_rejects_generic_moveit_action(monkeypatch, capsys):
    shell = bare_shell('ur5e')
    generic = '/moveit_action_bridge/joint_space'
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
    assert [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
            goal.target_pose.pose.position.z] == motion['position']
    assert [goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w] == motion['orientation']
    assert goal.duration_sec == 5.0
    # Stamped with the registry's frame: 'world' for an absolute OpenArm goal,
    # the root of every OpenArm controller's model, single arm or torso.
    assert goal.target_pose.header.frame_id == 'world'


def test_task_reach_honors_an_optional_motion_duration():
    shell = bare_shell('openarm')
    shell.arm = 'single'
    shell.robot_config = MODULE.load_robot_config('openarm', 'single')
    shell.task_space_action_client = object()
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append((client, goal)) or True

    shell.do_reach('3')

    assert len(sent) == 1
    assert sent[0][1].duration_sec == 5.0


@pytest.mark.parametrize('arm', ['single', 'left', 'right'])
def test_openarm_registry_task_reach_is_absolute_and_idempotent_at_action_boundary(arm):
    """The registry contract: absolute targets, so a repeat is not a relative move.

    The operator client substitutes relative probes for selectors 0-2 on the
    direct MIT task endpoint only - see
    test_openarm_direct_mit_task_reach_accumulates_at_the_action_boundary.
    """
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
    assert first.target_pose.pose.position == second.target_pose.pose.position
    assert first.target_pose.pose.orientation == second.target_pose.pose.orientation


@pytest.mark.parametrize('arm', ['single', 'left', 'right'])
def test_openarm_direct_mit_task_reach_accumulates_at_the_action_boundary(
        monkeypatch, arm, capsys):
    """The one documented exception to the absolute registry contract.

    Direct MIT task control starts from nominal zero, where the `home 1`-derived
    absolute targets can be unreachable, so the operator client substitutes
    bounded relative TCP probes for selectors 0-2. Two `reach 0` commands
    therefore send two relative goals and the displacement compounds.
    """
    from cho_control_tools.clients import operator_client

    prefix = '' if arm == 'single' else f'{arm}_'
    shell = bare_shell('openarm')
    shell.arm = arm
    shell.robot_config = MODULE.load_robot_config('openarm', arm)
    shell.task_action_name = (
        f'/{prefix}task_space_impedance_mit_controller/task_space')
    shell.task_space_action_client = object()
    shell.joint_space_action_client = None
    shell.gripper_action_client = None
    shell.robotiq_command_publisher = None
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append(goal) or True

    monkeypatch.setattr(operator_client, '_control_suite_shell',
                        lambda: (lambda **_kwargs: shell))
    operator_client.RobotActionShell('openarm', arm)
    assert 'repeats accumulate' in capsys.readouterr().out

    shell.do_reach('0')
    shell.do_reach('0')
    shell.do_reach('3')

    assert [goal.relative for goal in sent] == [True, True, False]
    # Identical relative goals: the arm moves the same delta again, it does not
    # return to a fixed world target.
    assert sent[0].target_pose.pose.position == sent[1].target_pose.pose.position


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
        assert goal.duration_sec == 5.0
        assert len(goal.target_joints.position) == 7
    assert list(sent[0][1].target_joints.position) == shell.robot_config['poses']['home']['1']
    assert list(sent[1][1].target_joints.position) == shell.robot_config['poses']['reach']['1']


def test_server_failure_reason_reaches_the_operator(capsys):
    shell = bare_shell('fr5')
    shell.robot_config = MODULE.load_robot_config('fr5', 'single')
    shell.task_space_action_client = object()
    shell.joint_space_action_client = object()

    def send(client, goal):
        del client, goal
        shell._last_result_message = (
            'MoveIt plan/execute failed: action_status=6, '
            'error_code=NO_IK_SOLUTION(-31); resolved target x=-0.0153')
        return False

    shell._send_goal_and_wait = send
    shell.do_reach('3')

    out = capsys.readouterr().out
    assert 'action failed: MoveIt plan/execute failed' in out
    assert 'NO_IK_SOLUTION(-31)' in out
    assert 'x=-0.0153' in out


def test_failure_without_a_reason_keeps_the_plain_line(capsys):
    shell = bare_shell('fr5')
    shell.robot_config = MODULE.load_robot_config('fr5', 'single')
    shell.task_space_action_client = object()
    shell.joint_space_action_client = object()
    shell._last_result_message = ''
    shell._send_goal_and_wait = lambda client, goal: False

    shell.do_reach('3')

    assert capsys.readouterr().out.strip().endswith('action failed')


# ------------------------------------------ names and frames on the goals

@pytest.mark.parametrize('robot_type,selector,relative,frame', [
    ('franka', '0', False, 'fr3_link0'),
    # Franka's controllers take ee_name at launch, so a relative goal is unstamped.
    ('franka', '2', True, ''),
    ('ur5e', '0', False, 'base_link'),
    ('ur5e', '2', True, ''),
    ('fr5', '0', False, 'base_link'),
])
def test_task_reach_is_stamped_with_the_registry_frame(robot_type, selector, relative, frame):
    shell = bare_shell(robot_type)
    shell.arm = 'single'
    shell.robot_config = MODULE.load_robot_config(robot_type, 'single')
    shell.task_space_action_client = object()
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append(goal) or True
    shell.do_reach(selector)
    assert sent[0].relative is relative
    assert sent[0].target_pose.header.frame_id == frame


@pytest.mark.parametrize('arm,prefix', [
    ('single', 'openarm_joint'), ('left', 'openarm_left_joint'), ('right', 'openarm_right_joint')])
def test_joint_goals_name_the_profiles_own_joints(arm, prefix):
    # Unnamed, a home meant for the left arm would drive whichever arm the
    # selected server is; named, the other arm's server rejects it.
    shell = bare_shell('openarm')
    shell.arm = arm
    shell.robot_config = MODULE.load_robot_config('openarm', arm)
    shell.joint_space_action_client = object()
    shell.task_space_action_client = None
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append(goal) or True
    shell.do_home('1')
    shell.do_reach('1')
    for goal in sent:
        assert list(goal.target_joints.name) == [f'{prefix}{index}' for index in range(1, 8)]


def test_the_bimanual_joint_goal_names_both_arms():
    shell = bare_shell('openarm')
    shell.arm = 'both'
    shell.robot_config = MODULE.load_robot_config('openarm', 'both')
    shell.joint_space_action_client = object()
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append(goal) or True
    shell.do_home('0')
    names = list(sent[0].target_joints.name)
    assert names[:7] == [f'openarm_left_joint{index}' for index in range(1, 8)]
    assert names[7:] == [f'openarm_right_joint{index}' for index in range(1, 8)]


def test_metadata_without_joint_names_sends_an_unnamed_target():
    shell = bare_shell('openarm')
    shell.robot_config = {'poses': {'home': {'0': [0.0] * 7}}, 'actions': {'preferences': {}}}
    shell.joint_space_action_client = object()
    shell._home_pose_policy_loader = lambda _config, _selector: {'enabled': True, 'reason': ''}
    sent = []
    shell._send_goal_and_wait = lambda client, goal: sent.append(goal) or True
    shell.do_home('0')
    assert list(sent[0].target_joints.name) == []


# ------------------------------------------------- the MoveIt scene gate

class _GraphNode:
    def __init__(self, services):
        self.services = services
        self.errors = []

    def get_service_names_and_types(self):
        return [(name, ['std_srvs/srv/Trigger']) for name in self.services]

    def get_logger(self):
        errors = self.errors
        return type('Logger', (), {'error': staticmethod(errors.append)})()


@pytest.mark.parametrize('arm', ['left', 'right', 'both'])
def test_a_bimanual_profile_waits_for_its_own_scene_gate(monkeypatch, arm):
    # The gate's name used to ignore the profile, so for left/right/both it was
    # never found and discovery settled for the direct controllers at once,
    # before the profile's MoveIt bridge was up.
    shell = bare_shell('openarm')
    shell.arm = arm
    shell.robot_config = MODULE.load_robot_config('openarm', arm)
    shell.node = _GraphNode([f'/cho_moveit/openarm/{arm}/static_scene_ready'])
    direct = [name for name in shell.robot_config['actions']['preferences']['joint']
              if 'moveit' not in name]
    monkeypatch.setattr(MODULE, 'get_action_names_and_types', lambda _node: [
        (name, ['cho_interfaces/action/JointSpace']) for name in direct])
    clock = {'now': 0.0}
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock['now'])
    shell._spin_or_sleep = lambda seconds: clock.__setitem__('now', clock['now'] + 5.0)
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)

    shell._discover_action_servers(timeout_sec=3.0)

    # It waited out the gate's own window rather than the 3 s direct grace.
    assert clock['now'] >= 210.0
    assert shell.node.errors and 'MoveIt safety gate' in shell.node.errors[0]


# ----------------------------------------------------- Ctrl-C and timeouts

class _Future:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done
        self.callbacks = []

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self._done:
            callback(self)


def _cancel_response(return_code=None, canceling=1):
    """A CancelGoal answer; ERROR_NONE listing *canceling* goals by default."""
    from action_msgs.msg import GoalInfo
    from action_msgs.srv import CancelGoal
    response = CancelGoal.Response()
    response.return_code = (CancelGoal.Response.ERROR_NONE if return_code is None
                            else return_code)
    response.goals_canceling = [GoalInfo() for _ in range(canceling)]
    return response


class _Handle:
    def __init__(self, cancel_answer=None, answered=True):
        self.accepted = True
        self.cancels = 0
        self.cancel_answer = _cancel_response() if cancel_answer is None else cancel_answer
        self.answered = answered

    def get_result_async(self):
        return _Future(done=False)          # the result never arrives

    def cancel_goal_async(self):
        self.cancels += 1
        return _Future(self.cancel_answer, done=self.answered)


class _GoalClient:
    def __init__(self, handle, accepted_yet=True):
        self.handle = handle
        self.accepted_yet = accepted_yet

    def send_goal_async(self, _goal):
        return _Future(self.handle, done=self.accepted_yet)


def _interrupt_on_first_sleep(monkeypatch):
    calls = {'n': 0}

    def sleep(_seconds):
        calls['n'] += 1
        if calls['n'] == 1:
            raise KeyboardInterrupt
    monkeypatch.setattr(MODULE.time, 'sleep', sleep)


def test_ctrl_c_while_a_goal_runs_cancels_it(monkeypatch, capsys):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    _interrupt_on_first_sleep(monkeypatch)
    shell = bare_shell('fr5')
    handle = _Handle()
    assert shell._send_goal_and_wait(_GoalClient(handle), MODULE.JointSpace.Goal()) is False
    assert handle.cancels == 1
    assert 'interrupted' in shell._last_result_message
    assert 'cancel accepted' in shell._last_result_message
    assert 'Cancel accepted' in capsys.readouterr().out


def _code(name):
    from action_msgs.srv import CancelGoal
    return getattr(CancelGoal.Response, name)


# What each answer must be reported as. Only an ERROR_NONE answer that lists
# the goal means the server is stopping it; every other one used to print
# "Goal cancelled" all the same.
_REFUSED_CANCELS = [
    ('ERROR_REJECTED', 1, 'REJECTED'),
    ('ERROR_REJECTED', 0, 'REJECTED'),
    ('ERROR_UNKNOWN_GOAL_ID', 0, 'does not know this goal'),
    ('ERROR_GOAL_TERMINATED', 0, 'already finished'),
    ('ERROR_NONE', 0, 'cancelling no goal'),
]


@pytest.mark.parametrize('code,canceling,expected', _REFUSED_CANCELS)
def test_ctrl_c_reports_a_refused_cancel_as_refused(monkeypatch, capsys, code, canceling,
                                                    expected):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    _interrupt_on_first_sleep(monkeypatch)
    shell = bare_shell('fr5')
    handle = _Handle(_cancel_response(_code(code), canceling))
    assert shell._send_goal_and_wait(_GoalClient(handle), MODULE.JointSpace.Goal()) is False
    out = capsys.readouterr().out
    assert expected in out
    assert expected in shell._last_result_message
    assert 'Goal cancelled' not in out
    assert 'cancel accepted' not in out.lower()
    assert 'the goal was cancelled' not in shell._last_result_message
    assert 'cancel accepted' not in shell._last_result_message


def test_a_cancel_nobody_answers_says_the_arm_may_still_be_moving(monkeypatch, capsys):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    _interrupt_on_first_sleep(monkeypatch)
    clock = {'now': 0.0}

    def monotonic():
        clock['now'] += 1.0
        return clock['now']
    monkeypatch.setattr(MODULE.time, 'monotonic', monotonic)
    shell = bare_shell('fr5')
    handle = _Handle(answered=False)
    assert shell._send_goal_and_wait(_GoalClient(handle), MODULE.JointSpace.Goal()) is False
    out = capsys.readouterr().out
    assert 'No answer to the cancel; the arm may still be moving' in out
    assert 'no answer to the cancel' in shell._last_result_message


def test_a_timed_out_goal_whose_cancel_is_rejected_does_not_claim_it_was_cancelled(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    clock = {'now': 0.0}
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(MODULE.time, 'sleep',
                        lambda seconds: clock.__setitem__('now', clock['now'] + 10.0))
    shell = bare_shell('fr5')
    handle = _Handle(_cancel_response(_code('ERROR_REJECTED'), 0))
    goal = MODULE.JointSpace.Goal()
    goal.duration_sec = 5.0
    assert shell._send_goal_and_wait(_GoalClient(handle), goal) is False
    assert 'no result within 65s' in shell._last_result_message
    assert 'REJECTED' in shell._last_result_message
    assert 'the goal was cancelled' not in shell._last_result_message


@pytest.mark.parametrize('code,canceling,expected', [
    ('ERROR_NONE', 1, 'cancel accepted'),
    ('ERROR_REJECTED', 0, 'REJECTED'),
])
def test_ctrl_c_before_acceptance_reports_the_late_cancel_answer(monkeypatch, capsys, code,
                                                                 canceling, expected):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    _interrupt_on_first_sleep(monkeypatch)
    shell = bare_shell('fr5')
    handle = _Handle(_cancel_response(_code(code), canceling))
    client = _GoalClient(handle, accepted_yet=False)
    send_future = []
    original = client.send_goal_async
    client.send_goal_async = lambda goal: send_future.append(original(goal)) or send_future[0]
    assert shell._send_goal_and_wait(client, MODULE.JointSpace.Goal()) is False
    # Nothing is known yet, so nothing may be claimed.
    assert 'the goal was cancelled' not in shell._last_result_message
    assert 'before the server answered' in shell._last_result_message
    capsys.readouterr()
    send_future[0]._done = True
    for callback in send_future[0].callbacks:
        callback(send_future[0])
    assert handle.cancels == 1
    assert expected in capsys.readouterr().out


def test_ctrl_c_before_the_goal_is_accepted_cancels_it_on_acceptance(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    _interrupt_on_first_sleep(monkeypatch)
    shell = bare_shell('fr5')
    handle = _Handle()
    client = _GoalClient(handle, accepted_yet=False)
    send_future = []
    original = client.send_goal_async
    client.send_goal_async = lambda goal: send_future.append(original(goal)) or send_future[0]
    assert shell._send_goal_and_wait(client, MODULE.JointSpace.Goal()) is False
    assert handle.cancels == 0
    send_future[0]._done = True
    for callback in send_future[0].callbacks:
        callback(send_future[0])
    assert handle.cancels == 1


def test_a_result_that_never_comes_is_given_up_on_and_cancelled(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    clock = {'now': 0.0}
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(MODULE.time, 'sleep',
                        lambda seconds: clock.__setitem__('now', clock['now'] + 10.0))
    shell = bare_shell('fr5')
    handle = _Handle()
    goal = MODULE.JointSpace.Goal()
    goal.duration_sec = 5.0
    assert shell._send_goal_and_wait(_GoalClient(handle), goal) is False
    assert handle.cancels == 1
    assert 'no result within 65s' in shell._last_result_message
