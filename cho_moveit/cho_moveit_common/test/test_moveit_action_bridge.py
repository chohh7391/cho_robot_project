"""Focused lifecycle/math/configuration tests for the common MoveIt action bridge."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Quaternion
import pytest
from sensor_msgs.msg import JointState


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'moveit_action_bridge.py'
SPEC = importlib.util.spec_from_file_location('moveit_action_bridge', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Future:
    def __init__(self, result, first_pending=False):
        self._result = result
        self._first_pending = first_pending

    def done(self):
        if self._first_pending:
            self._first_pending = False
            return False
        return True

    def result(self):
        return self._result


class ExceptionalFuture(Future):
    def result(self):
        raise RuntimeError('transport lost')


class ChoHandle:
    def __init__(self, cancel_requested=True):
        self.is_cancel_requested = cancel_requested
        self.terminal = None

    def publish_feedback(self, _feedback):
        pass

    def canceled(self):
        self.terminal = 'canceled'

    def succeed(self):
        self.terminal = 'succeeded'

    def abort(self):
        self.terminal = 'aborted'


def bare_bridge():
    bridge = object.__new__(MODULE.MoveItActionBridge)
    bridge.get_logger = lambda: SimpleNamespace(
        error=lambda _text: None, warn=lambda _text: None, fatal=lambda _text: None)
    bridge._goal_lock = MODULE.threading.Lock()
    bridge._goal_reserved = True
    bridge._faulted = False
    bridge._fault_reason = ''
    bridge._joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    bridge._group = 'fr5_arm'
    bridge._velocity_scaling = 0.25
    bridge._acceleration_scaling = 0.25
    bridge._blocked_joint_goals = []
    # Must match the node's parameter default.
    bridge._pipeline = 'ompl'
    return bridge


def test_disabled_home_joint_target_is_identified_independent_of_client():
    bridge = bare_bridge()
    bridge._blocked_joint_goals = [{
        'selector': '0', 'positions': [0.0] * 6, 'reason': 'floor contact',
        'max_joint_distance': 0.01}]
    assert bridge._blocked_joint_goal([0.0] * 6)['selector'] == '0'
    assert bridge._blocked_joint_goal([0.01] * 6)['selector'] == '0'
    assert bridge._blocked_joint_goal([0.010001, 0.0, 0.0, 0.0, 0.0, 0.0]) is None


@pytest.mark.parametrize('joint_names,group', [
    (['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], 'ur_manipulator'),
    ([f'openarm_joint{i}' for i in range(1, 8)], 'openarm_manipulator'),
])
def test_robot_specific_joint_constraints_and_group(joint_names, group):
    bridge = bare_bridge()
    bridge._joint_names = joint_names
    bridge._group = group
    constraints = bridge._joint_constraints([0.0] * len(joint_names))
    assert [item.joint_name for item in constraints.joint_constraints] == bridge._joint_names
    request = bridge._move_goal(constraints, 5.0, 'ompl').request
    assert request.group_name == group
    assert request.max_velocity_scaling_factor == bridge._velocity_scaling
    assert request.max_acceleration_scaling_factor == bridge._acceleration_scaling


def test_robot_identity_scopes_action_names():
    # Served relative to the node, like a controller's; the node name is what
    # carries the robot identity.
    assert (MODULE.JOINT_ACTION, MODULE.TASK_ACTION) == ('~/joint_space', '~/task_space')
    assert MODULE.MoveItActionBridge._expected_action_names('ur5e') == (
        '/ur5e_moveit_action_bridge/joint_space',
        '/ur5e_moveit_action_bridge/task_space')
    assert MODULE.MoveItActionBridge._expected_action_names('openarm', 'left') == (
        '/openarm_left_moveit_action_bridge/joint_space',
        '/openarm_left_moveit_action_bridge/task_space')
    assert MODULE.MoveItActionBridge._expected_action_names('fr5') != (
        MODULE.MoveItActionBridge._expected_action_names('ur5e'))


def test_named_joint_goals_are_reordered_into_the_bridges_joint_order():
    bridge = bare_bridge()
    goal = JointState(name=['j6', 'j5', 'j4', 'j3', 'j2', 'j1'],
                      position=[6.0, 5.0, 4.0, 3.0, 2.0, 1.0])
    assert bridge._ordered_joint_positions(goal) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    # Unnamed goals are already in joint order.
    assert bridge._ordered_joint_positions(JointState(position=[0.5] * 6)) == [0.5] * 6


@pytest.mark.parametrize('names,positions,message', [
    (['j1', 'j2', 'j3', 'j4', 'j5', 'x'], [0.0] * 6, 'unknown'),
    (['j1', 'j1', 'j3', 'j4', 'j5', 'j6'], [0.0] * 6, 'more than once'),
    (['j1', 'j2', 'j3', 'j4', 'j5'], [0.0] * 5, 'does not name'),
    (['j1', 'j2', 'j3', 'j4', 'j5', 'j6'], [0.0] * 5, 'positions'),
])
def test_a_named_joint_goal_that_does_not_name_every_joint_once_is_rejected(
        names, positions, message):
    bridge = bare_bridge()
    goal = JointState(name=names, position=positions)
    with pytest.raises(ValueError, match=message):
        bridge._ordered_joint_positions(goal)
    request = SimpleNamespace(target_joints=goal, duration_sec=5.0)
    assert bridge._joint_goal_callback(request) == MODULE.GoalResponse.REJECT


@pytest.mark.parametrize('relative,frame,honoured', [
    (False, '', True),
    (False, 'world', True),
    (False, 'wrist3_link', False),
    (False, 'camera_link', False),
    (True, '', True),
    (True, 'wrist3_link', True),
    (True, 'world', False),
])
def test_task_goal_frames_follow_the_contract(relative, frame, honoured):
    bridge = bare_bridge()
    bridge._world_frame = 'world'
    bridge._ee_link = 'wrist3_link'
    request = SimpleNamespace(relative=relative, target_pose=PoseStamped(), duration_sec=5.0)
    request.target_pose.header.frame_id = frame
    reason = bridge._unhonoured_frame(request)
    assert (reason == '') is honoured
    if not honoured:
        assert frame in reason
        # Refused before anything is reserved, rather than planned in the
        # wrong frame.
        assert bridge._task_goal_callback(request) == MODULE.GoalResponse.REJECT


def test_relative_quaternion_composition_and_normalization():
    half = 2**-0.5
    current = Quaternion(z=half, w=half)
    relative = Quaternion(x=half, w=half)
    actual = MODULE.MoveItActionBridge._compose_quaternion(current, relative)
    expected = (0.5, 0.5, 0.5, 0.5)
    assert all(abs(a - e) < 1e-9 for a, e in zip(actual, expected))


def test_concurrent_goal_is_rejected():
    bridge = bare_bridge()
    bridge._ready = True
    bridge._ready_verified_at = MODULE.time.monotonic()
    bridge._ready_lock = MODULE.threading.Lock()
    bridge._goal_lock = MODULE.threading.Lock()
    bridge._goal_reserved = True
    bridge._move_client = SimpleNamespace(server_is_ready=lambda: True)
    request = SimpleNamespace(duration_sec=5.0)
    assert bridge._goal_callback(request) == MODULE.GoalResponse.REJECT


def test_non_positive_duration_is_rejected():
    bridge = bare_bridge()
    assert bridge._goal_callback(
        SimpleNamespace(duration_sec=0.0)) == MODULE.GoalResponse.REJECT


def test_cancel_requested_before_movegroup_accept_is_forwarded(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    move_handle = SimpleNamespace(accepted=True, get_result_async=lambda: Future(None))
    bridge._move_client = SimpleNamespace(
        send_goal_async=lambda _goal: Future(move_handle, first_pending=True))
    bridge._move_goal = lambda _constraints, _duration, _pipeline: object()
    called = []
    bridge._cancel_downstream = lambda *_args: (called.append(True), (False, 'canceled'))[1]
    handle = ChoHandle(cancel_requested=True)
    succeeded, reason = bridge._run_move_group(handle, object(), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    assert called == [True]


def test_cancel_while_running_is_forwarded(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    result_future = Future(None, first_pending=True)
    move_handle = SimpleNamespace(
        accepted=True, get_result_async=lambda: result_future)
    bridge._move_client = SimpleNamespace(
        send_goal_async=lambda _goal: Future(move_handle))
    bridge._move_goal = lambda _constraints, _duration, _pipeline: object()
    called = []
    bridge._cancel_downstream = lambda *_args: (called.append(True), (False, 'canceled'))[1]
    handle = ChoHandle(cancel_requested=True)
    succeeded, reason = bridge._run_move_group(handle, object(), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    assert called == [True]


def test_result_future_exception_latches_fault(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    move_handle = SimpleNamespace(
        accepted=True, get_result_async=lambda: ExceptionalFuture(None))
    bridge = bare_bridge()
    bridge._move_client = SimpleNamespace(
        send_goal_async=lambda _goal: Future(move_handle))
    bridge._move_goal = lambda _constraints, _duration, _pipeline: object()
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, object(), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    bridge._release_goal()
    assert handle.terminal == 'aborted'
    assert bridge._faulted and bridge._goal_reserved


def test_none_result_latches_fault(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    move_handle = SimpleNamespace(
        accepted=True, get_result_async=lambda: Future(None))
    bridge = bare_bridge()
    bridge._move_client = SimpleNamespace(
        send_goal_async=lambda _goal: Future(move_handle))
    bridge._move_goal = lambda _constraints, _duration, _pipeline: object()
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, object(), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    bridge._release_goal()
    assert bridge._faulted and bridge._goal_reserved


def cancel_fixture(cancel_response, terminal_status):
    result = SimpleNamespace(status=terminal_status)
    result_future = Future(result)
    move_handle = SimpleNamespace(
        cancel_goal_async=lambda: Future(cancel_response))
    return move_handle, result_future


def test_confirmed_cancel_allows_reservation_release(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    response = SimpleNamespace(
        return_code=MODULE.CancelGoal.Response.ERROR_NONE,
        goals_canceling=[object()])
    move_handle, result_future = cancel_fixture(
        response, MODULE.GoalStatus.STATUS_CANCELED)
    bridge = bare_bridge()
    handle = ChoHandle()
    succeeded, reason = bridge._cancel_downstream(handle, move_handle, result_future)
    assert not succeeded and reason
    bridge._release_goal()
    assert handle.terminal == 'canceled'
    assert not bridge._faulted
    assert not bridge._goal_reserved


def test_cancel_reject_latches_fault_and_retains_reservation(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    response = SimpleNamespace(return_code=1, goals_canceling=[])
    move_handle, result_future = cancel_fixture(
        response, MODULE.GoalStatus.STATUS_ABORTED)
    bridge = bare_bridge()
    handle = ChoHandle()
    succeeded, reason = bridge._cancel_downstream(handle, move_handle, result_future)
    assert not succeeded and reason
    bridge._release_goal()
    assert handle.terminal == 'aborted'
    assert bridge._faulted and bridge._goal_reserved


def test_cancel_timeout_latches_fault_and_retains_reservation(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    ticks = iter([0.0, 0.0, 10.0])
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: next(ticks, 10.0))
    pending = SimpleNamespace(done=lambda: False)
    move_handle = SimpleNamespace(cancel_goal_async=lambda: pending)
    bridge = bare_bridge()
    handle = ChoHandle()
    succeeded, reason = bridge._cancel_downstream(handle, move_handle, Future(None))
    assert not succeeded and reason
    bridge._release_goal()
    assert bridge._faulted and bridge._goal_reserved


def test_non_canceled_terminal_latches_fault_and_retains_reservation(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    response = SimpleNamespace(
        return_code=MODULE.CancelGoal.Response.ERROR_NONE,
        goals_canceling=[object()])
    move_handle, result_future = cancel_fixture(
        response, MODULE.GoalStatus.STATUS_ABORTED)
    bridge = bare_bridge()
    handle = ChoHandle()
    succeeded, reason = bridge._cancel_downstream(handle, move_handle, result_future)
    assert not succeeded and reason
    bridge._release_goal()
    assert bridge._faulted and bridge._goal_reserved


def test_moveit_failure_reason_names_the_error_code(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    wrapped = SimpleNamespace(
        status=MODULE.GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_code=SimpleNamespace(
            val=MODULE.MoveItErrorCodes.NO_IK_SOLUTION)))
    move_handle = SimpleNamespace(
        accepted=True, get_result_async=lambda: Future(wrapped))
    bridge = bare_bridge()
    bridge._move_client = SimpleNamespace(
        send_goal_async=lambda _goal: Future(move_handle))
    bridge._move_goal = lambda _constraints, _duration, _pipeline: object()
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, object(), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded
    assert 'NO_IK_SOLUTION(-31)' in reason
    assert handle.terminal == 'aborted'


def test_successful_run_reports_no_reason(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    wrapped = SimpleNamespace(
        status=MODULE.GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(error_code=SimpleNamespace(
            val=MODULE.MoveItErrorCodes.SUCCESS)))
    move_handle = SimpleNamespace(
        accepted=True, get_result_async=lambda: Future(wrapped))
    bridge = bare_bridge()
    bridge._move_client = SimpleNamespace(
        send_goal_async=lambda _goal: Future(move_handle))
    bridge._move_goal = lambda _constraints, _duration, _pipeline: object()
    handle = ChoHandle(cancel_requested=False)
    assert bridge._run_move_group(
        handle, object(), 5.0, SimpleNamespace, 'ompl') == (True, '')


def test_task_failure_reports_the_resolved_world_target():
    bridge = bare_bridge()
    bridge._world_frame = 'world'
    bridge._ee_link = 'wrist3_link'
    request = SimpleNamespace(
        relative=False,
        target_pose=PoseStamped(),
        duration_sec=5.0)
    request.target_pose.pose.position.x = -0.0153
    request.target_pose.pose.position.y = 0.0040
    request.target_pose.pose.position.z = 0.7249
    request.target_pose.pose.orientation.w = 1.0
    summary = bridge._target_summary(bridge._task_constraints(request))
    assert 'x=-0.0153' in summary and 'y=+0.0040' in summary and 'z=+0.7249' in summary


def test_move_goal_carries_the_requested_pipeline():
    """The pipeline is per-request, not baked in."""
    bridge = bare_bridge()
    constraints = bridge._joint_constraints([0.0] * len(bridge._joint_names))
    for pipeline in ('ompl', 'some_other_pipeline'):
        assert bridge._move_goal(constraints, 5.0, pipeline).request.pipeline_id == pipeline


def test_planning_pipeline_defaults_to_ompl():
    """OMPL is the only pipeline this project registers, and both goal types use it."""
    bridge = bare_bridge()
    assert bridge._pipeline == 'ompl'
