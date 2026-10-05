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

"""Focused lifecycle/math/configuration tests for the common MoveIt action bridge."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import RobotTrajectory
import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'moveit_action_bridge.py'
SPEC = importlib.util.spec_from_file_location('moveit_action_bridge', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Future:
    def __init__(self, result, first_pending=False):
        self._result = result
        self._first_pending = first_pending
        self.callbacks = []

    def done(self):
        if self._first_pending:
            self._first_pending = False
            return False
        return True

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        # Delivered at once, as rclpy does for a future that is already done.
        self.callbacks.append(callback)
        callback(self)


class Handle(SimpleNamespace):
    """A downstream goal handle that records the cancels sent to it."""

    def __init__(self, result_future=None, accepted=True):
        super().__init__(accepted=accepted, cancels=0, result_future=result_future)

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return Future(None)


class Client(SimpleNamespace):
    """An action client whose next goal gets *handle*; records what was sent."""

    def __init__(self, handle, first_pending=False, on_send=None):
        super().__init__(handle=handle, first_pending=first_pending, sent=[], on_send=on_send)

    def send_goal_async(self, goal):
        self.sent.append(goal)
        if self.on_send is not None:
            self.on_send()
        return Future(self.handle, first_pending=self.first_pending)

    @staticmethod
    def server_is_ready():
        return True


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


class StopPublisher:
    """The bridge's "stop" publisher: records what it sent, as move_group would see it."""

    def __init__(self, subscribers=1):
        self.published = []
        self.subscribers = subscribers

    def publish(self, message):
        self.published.append(message.data)

    def get_subscription_count(self):
        return self.subscribers

    def wait_for_all_acked(self, _timeout):
        self.acked = list(self.published)
        return True


class FakeClock:
    """time.monotonic/time.sleep for the bridge's wait loops, without waiting."""

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class PendingFuture(Future):
    """A future that is done only after *polls* calls to done()."""

    def __init__(self, result, polls):
        super().__init__(result)
        self.polls = polls

    def done(self):
        if self.polls > 0:
            self.polls -= 1
            return False
        return True


class Tf:
    """A TF buffer holding one transform from the planning frame, or none."""

    def __init__(self, transforms=None):
        self.transforms = transforms or {}

    def lookup_transform(self, target, source, _time, timeout=None):
        del timeout
        try:
            return self.transforms[(target, source)]
        except KeyError:
            raise MODULE.TransformException(f'{source} does not exist') from None


def transform(x=0.0, yaw_quaternion_z=0.0, y=0.0, z=0.0):
    stamped = TransformStamped()
    stamped.transform.translation.x = x
    stamped.transform.translation.y = y
    stamped.transform.translation.z = z
    stamped.transform.rotation.z = yaw_quaternion_z
    stamped.transform.rotation.w = (1.0 - yaw_quaternion_z ** 2) ** 0.5
    return stamped


def wrapped(error_code, trajectory=None, status=None):
    """A MoveGroup / ExecuteTrajectory result as the action client returns it."""
    status = MODULE.GoalStatus.STATUS_SUCCEEDED if status is None else status
    result = SimpleNamespace(error_code=SimpleNamespace(val=error_code),
                             planned_trajectory=trajectory)
    return SimpleNamespace(status=status, result=result)


def trajectory(seconds=(0.0, 1.0, 2.0), velocity=1.0, acceleration=2.0):
    planned = RobotTrajectory()
    for second in seconds:
        point = JointTrajectoryPoint(positions=[0.0] * 6, velocities=[velocity] * 6,
                                     accelerations=[acceleration] * 6)
        MODULE._set_seconds(point.time_from_start, second)
        planned.joint_trajectory.points.append(point)
    return planned


def planner(plan, first_pending=False, result_pending=False):
    """A MoveGroup client whose plan-only goal returns *plan*."""
    return Client(Handle(Future(plan, first_pending=result_pending)), first_pending)


def executor(result, first_pending=False, result_pending=False, on_send=None):
    return Client(Handle(Future(result, first_pending=result_pending)), first_pending, on_send)


SUCCESS = MODULE.MoveItErrorCodes.SUCCESS


def _constraints(bridge):
    return bridge._joint_constraints([0.0] * len(bridge._joint_names))


def bare_bridge():
    bridge = object.__new__(MODULE.MoveItActionBridge)
    bridge.get_logger = lambda: SimpleNamespace(
        error=lambda _text: None, warn=lambda _text: None, fatal=lambda _text: None,
        info=lambda _text: None)
    bridge._goal_lock = MODULE.threading.Lock()
    bridge._goal_reserved = True
    bridge._faulted = False
    bridge._fault_reason = ''
    bridge._executing = False
    bridge._robot_type = 'fr5'
    bridge._event_topic = '/trajectory_execution_event'
    bridge._stop_publisher = StopPublisher()
    bridge._joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    bridge._group = 'fr5_arm'
    bridge._velocity_scaling = 0.25
    bridge._acceleration_scaling = 0.25
    bridge._blocked_joint_goals = []
    # Must match the node's parameter defaults.
    bridge._pipeline = 'ompl'
    bridge._planning_time = 5.0
    bridge._world_frame = 'world'
    bridge._ee_link = 'wrist3_link'
    bridge._arm_base_link = 'base_link'
    bridge._tf_buffer = Tf({('world', 'base_link'): transform()})
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
    request = bridge._plan_goal(constraints, 'ompl').request
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
    (False, 'base_link', True),        # the arm_base_link, at the planning frame in TF
    (False, 'wrist3_link', False),
    (False, 'camera_link', False),
    (True, '', True),
    (True, 'wrist3_link', True),
    (True, 'world', False),
    (True, 'base_link', False),
])
def test_task_goal_frames_follow_the_contract(relative, frame, honoured):
    bridge = bare_bridge()
    request = SimpleNamespace(relative=relative, target_pose=PoseStamped(), duration_sec=5.0)
    request.target_pose.header.frame_id = frame
    reason = bridge._unhonoured_frame(request)
    assert (reason == '') is honoured
    if not honoured:
        assert frame in reason
        # Refused before anything is reserved, rather than planned in the
        # wrong frame.
        assert bridge._task_goal_callback(request) == MODULE.GoalResponse.REJECT


@pytest.mark.parametrize('tf,why', [
    (Tf({('world', 'base_link'): transform(x=0.25)}), '0.2500 m'),
    (Tf({('world', 'base_link'): transform(yaw_quaternion_z=0.5)}), 'rad away'),
    (Tf(), 'TF cannot say'),
])
def test_an_arm_base_link_apart_from_the_planning_frame_is_refused(tf, why):
    # OpenArm's bimanual torso puts each arm's link0 off the world origin; a
    # goal stamped there means a different pose to the planner than to the
    # numbers in it, and nothing here transforms.
    bridge = bare_bridge()
    bridge._tf_buffer = tf
    request = SimpleNamespace(relative=False, target_pose=PoseStamped(), duration_sec=5.0)
    request.target_pose.header.frame_id = 'base_link'
    reason = bridge._unhonoured_frame(request)
    assert why in reason
    assert bridge._task_goal_callback(request) == MODULE.GoalResponse.REJECT


@pytest.mark.parametrize('position', [
    [0.0] * 5,                          # one joint short
    [0.0] * 7,                          # one too many
    [0.0, 0.0, float('nan'), 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, float('inf'), 0.0, 0.0],
])
def test_a_joint_goal_it_cannot_plan_is_rejected_not_accepted_and_aborted(position):
    bridge = bare_bridge()
    request = SimpleNamespace(target_joints=JointState(position=position), duration_sec=5.0)
    assert bridge._joint_goal_callback(request) == MODULE.GoalResponse.REJECT


@pytest.mark.parametrize('field,value', [
    ('x', float('nan')), ('z', float('inf')), ('qw', float('nan')), ('zero_quaternion', 0.0),
])
def test_a_task_goal_it_cannot_plan_is_rejected_not_accepted_and_aborted(field, value):
    bridge = bare_bridge()
    request = SimpleNamespace(relative=False, target_pose=PoseStamped(), duration_sec=5.0)
    pose = request.target_pose.pose
    pose.orientation.w = 1.0
    if field == 'qw':
        pose.orientation.w = value
    elif field == 'zero_quaternion':
        pose.orientation.w = 0.0
    else:
        setattr(pose.position, field, value)
    assert bridge._unusable_pose(request)
    assert bridge._task_goal_callback(request) == MODULE.GoalResponse.REJECT


def test_a_relative_goal_is_composed_in_the_ee_frame():
    # EE at (0.3, 0.1, 0.5) turned 90 deg about world z; the goal moves it
    # 0.1 m along its own x and turns it 90 deg about its own x. In the world
    # that is +0.1 m along y, and the orientation (z90 * x90) = (0.5, 0.5, 0.5, 0.5).
    half = 2 ** -0.5
    bridge = bare_bridge()
    bridge._tf_buffer = Tf({('world', 'wrist3_link'): transform(
        x=0.3, y=0.1, z=0.5, yaw_quaternion_z=half)})
    request = SimpleNamespace(relative=True, target_pose=PoseStamped(), duration_sec=5.0)
    request.target_pose.pose.position.x = 0.1
    # Not unit length: it is normalized before it is composed.
    request.target_pose.pose.orientation.x = 2.0
    request.target_pose.pose.orientation.w = 2.0
    constraints = bridge._task_constraints(request)
    pose = constraints.position_constraints[0].constraint_region.primitive_poses[0]
    assert (pose.position.x, pose.position.y, pose.position.z) == pytest.approx((0.3, 0.2, 0.5))
    orientation = constraints.orientation_constraints[0].orientation
    assert (orientation.x, orientation.y, orientation.z, orientation.w) == pytest.approx(
        (0.5, 0.5, 0.5, 0.5))


def test_concurrent_goal_is_rejected():
    bridge = bare_bridge()
    bridge._ready = True
    bridge._ready_verified_at = MODULE.time.monotonic()
    bridge._ready_lock = MODULE.threading.Lock()
    bridge._goal_lock = MODULE.threading.Lock()
    bridge._goal_reserved = True
    bridge._move_client = SimpleNamespace(server_is_ready=lambda: True)
    bridge._execute_client = SimpleNamespace(server_is_ready=lambda: True)
    request = SimpleNamespace(duration_sec=5.0)
    assert bridge._goal_callback(request) == MODULE.GoalResponse.REJECT


def test_non_positive_duration_is_rejected():
    bridge = bare_bridge()
    assert bridge._goal_callback(
        SimpleNamespace(duration_sec=0.0)) == MODULE.GoalResponse.REJECT


def _ready_bridge():
    """A bridge whose readiness gate is open and that holds no goal."""
    bridge = bare_bridge()
    bridge._ready = True
    bridge._ready_verified_at = MODULE.time.monotonic()
    bridge._ready_lock = MODULE.threading.Lock()
    bridge._goal_reserved = False
    bridge._move_client = SimpleNamespace(server_is_ready=lambda: True)
    bridge._execute_client = SimpleNamespace(server_is_ready=lambda: True)
    return bridge


def test_a_plausible_duration_is_accepted():
    assert _ready_bridge()._goal_callback(
        SimpleNamespace(duration_sec=5.0)) == MODULE.GoalResponse.ACCEPT


def test_a_goal_is_rejected_while_nothing_could_stop_it():
    # Humble's move_group ignores a cancel of ExecuteTrajectory; the stop
    # event is what stops the arm, so with no one listening a goal could not
    # be cancelled once it moved.
    bridge = _ready_bridge()
    bridge._stop_publisher = StopPublisher(subscribers=0)
    assert bridge._goal_callback(SimpleNamespace(duration_sec=5.0)) == MODULE.GoalResponse.REJECT
    assert not bridge._goal_reserved


def test_a_disabled_home_is_rejected_when_the_goal_arrives():
    # It used to be accepted, reserved, and then aborted by the execute path.
    bridge = _ready_bridge()
    bridge._blocked_joint_goals = [{
        'selector': '0', 'positions': [0.0] * 6, 'reason': 'floor contact',
        'max_joint_distance': 0.01}]
    request = SimpleNamespace(target_joints=JointState(position=[0.0] * 6), duration_sec=5.0)
    assert bridge._joint_goal_callback(request) == MODULE.GoalResponse.REJECT
    assert not bridge._goal_reserved
    request.target_joints.position = [0.5] * 6
    assert bridge._joint_goal_callback(request) == MODULE.GoalResponse.ACCEPT


def test_a_relative_goal_without_the_ee_transform_is_rejected_when_it_arrives():
    bridge = _ready_bridge()
    bridge._tf_buffer = Tf()
    request = SimpleNamespace(relative=True, target_pose=PoseStamped(), duration_sec=5.0)
    request.target_pose.pose.orientation.w = 1.0
    assert bridge._task_goal_callback(request) == MODULE.GoalResponse.REJECT
    assert not bridge._goal_reserved
    bridge._tf_buffer = Tf({('world', 'wrist3_link'): transform()})
    assert bridge._task_goal_callback(request) == MODULE.GoalResponse.ACCEPT


@pytest.mark.parametrize('execute_action,topic', [
    ('/execute_trajectory', '/trajectory_execution_event'),
    ('/ns/execute_trajectory', '/ns/trajectory_execution_event'),
    ('execute_trajectory', 'trajectory_execution_event'),
])
def test_the_stop_goes_to_the_event_topic_of_the_move_group_that_executes(execute_action, topic):
    assert MODULE.execution_event_topic(execute_action) == topic


@pytest.mark.parametrize('duration', [2.0 ** 31, 1e12, MODULE.MAX_GOAL_DURATION_SEC + 1.0])
def test_an_absurd_duration_is_rejected_when_the_goal_arrives(duration):
    # 2**31 s used to be accepted, then failed mid-goal: the stretched
    # time_from_start overflows int32 seconds and the message setter raised an
    # AssertionError past `except ValueError` -- a traceback, an empty abort.
    bridge = _ready_bridge()
    assert bridge._goal_callback(
        SimpleNamespace(duration_sec=duration)) == MODULE.GoalResponse.REJECT
    assert not bridge._goal_reserved


def test_stretching_to_an_absurd_duration_is_a_value_error_not_an_assertion():
    with pytest.raises(ValueError, match='minimum duration'):
        MODULE.stretch_to_minimum_duration(trajectory(), 2.0 ** 31)


# ------------------------------------------------- plan, then execute
#
# A goal is planned plan-only, slowed to its duration_sec, then executed. Only
# the execution can move the arm, so only its unknown outcomes latch a fault.

def test_cancel_before_the_plan_is_accepted_cancels_it_once_accepted(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()), first_pending=True)
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=True)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and 'nothing moved' in reason
    assert handle.terminal == 'canceled'
    assert bridge._move_client.handle.cancels == 1
    assert bridge._execute_client.sent == []
    assert not bridge._faulted


def test_cancel_while_planning_cancels_the_plan_and_moves_nothing(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()), result_pending=True)
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=True)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    assert handle.terminal == 'canceled'
    assert bridge._move_client.handle.cancels == 1
    assert bridge._execute_client.sent == []
    bridge._release_goal()
    assert not bridge._faulted and not bridge._goal_reserved


def _cancel_on_send(handle):
    def on_send():
        handle.is_cancel_requested = True
    return on_send


def test_cancel_requested_before_execution_accept_is_forwarded(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    handle = ChoHandle(cancel_requested=False)
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()))
    bridge._execute_client = executor(
        wrapped(SUCCESS), first_pending=True, on_send=_cancel_on_send(handle))
    called = []
    bridge._cancel_downstream = lambda *_args: (called.append(True), (False, 'canceled'))[1]
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    assert called == [True]


def test_cancel_while_executing_is_forwarded(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    handle = ChoHandle(cancel_requested=False)
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()))
    bridge._execute_client = executor(
        wrapped(SUCCESS), result_pending=True, on_send=_cancel_on_send(handle))
    called = []
    bridge._cancel_downstream = lambda *_args: (called.append(True), (False, 'canceled'))[1]
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    assert called == [True]


def test_execution_result_exception_latches_fault(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()))
    bridge._execute_client = Client(Handle(ExceptionalFuture(None)))
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    bridge._release_goal()
    assert handle.terminal == 'aborted'
    assert bridge._faulted and bridge._goal_reserved


def test_none_execution_result_latches_fault(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()))
    bridge._execute_client = executor(None)
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and reason
    bridge._release_goal()
    assert bridge._faulted and bridge._goal_reserved


def test_a_planning_failure_aborts_without_latching_a_fault(monkeypatch):
    # Nothing moves while planning, so there is no motion whose state is unknown.
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = Client(Handle(ExceptionalFuture(None)))
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and 'planning result failed' in reason
    bridge._release_goal()
    assert handle.terminal == 'aborted'
    assert not bridge._faulted and not bridge._goal_reserved
    assert bridge._execute_client.sent == []


ACCEPTED = SimpleNamespace(return_code=CancelGoal.Response.ERROR_NONE, goals_canceling=[object()])


class MoveHandle:
    """An accepted ExecuteTrajectory goal: records cancels, answers with *cancel_response*."""

    def __init__(self, cancel_response=ACCEPTED, cancel_raises=False):
        self.cancel_response = cancel_response
        self.cancel_raises = cancel_raises
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1
        if self.cancel_raises:
            raise RuntimeError('transport lost')
        return Future(self.cancel_response)


def terminal(status, error_code=None):
    """An ExecuteTrajectory result as the action client returns it."""
    result = None if error_code is None else SimpleNamespace(
        error_code=SimpleNamespace(val=error_code))
    return SimpleNamespace(status=status, result=result)


def cancel(monkeypatch, result_future, move_handle=None):
    """Cancel a running execution through the bridge; returns (bridge, cho handle, outcome)."""
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    clock = FakeClock()
    monkeypatch.setattr(MODULE.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(MODULE.time, 'sleep', clock.sleep)
    bridge = bare_bridge()
    handle = ChoHandle()
    outcome = bridge._cancel_downstream(handle, move_handle or MoveHandle(), result_future)
    bridge._release_goal()
    return bridge, handle, outcome


PREEMPTED = MODULE.MoveItErrorCodes.PREEMPTED


def test_a_cancel_publishes_stop_because_humble_ignores_the_cancel(monkeypatch):
    # move_group 2.5.x accepts the cancel and runs the trajectory to its end;
    # TrajectoryExecutionManager's "stop" event is what calls stopExecution().
    move_handle = MoveHandle()
    bridge, _handle, _outcome = cancel(
        monkeypatch, Future(terminal(MODULE.GoalStatus.STATUS_ABORTED, PREEMPTED)), move_handle)
    assert bridge._stop_publisher.published[:1] == ['stop']
    # Still sent: a MoveIt that honours it stops on it.
    assert move_handle.cancels == 1


def test_the_stopped_execution_humble_reports_is_a_cancel_not_a_fault(monkeypatch):
    # stopExecution() ends the goal ABORTED with PREEMPTED; that is the cancel
    # taking effect, and the arm holds where it was.
    bridge, handle, (succeeded, reason) = cancel(
        monkeypatch, Future(terminal(MODULE.GoalStatus.STATUS_ABORTED, PREEMPTED)))
    assert not succeeded and 'stopped' in reason
    assert handle.terminal == 'canceled'
    assert not bridge._faulted and not bridge._goal_reserved


def test_confirmed_cancel_allows_reservation_release(monkeypatch):
    bridge, handle, (succeeded, reason) = cancel(
        monkeypatch, Future(terminal(MODULE.GoalStatus.STATUS_CANCELED)))
    assert not succeeded and reason
    assert handle.terminal == 'canceled'
    assert not bridge._faulted and not bridge._goal_reserved


def test_the_stop_is_repeated_until_the_execution_is_over(monkeypatch):
    # A stop that reaches move_group before execute() started is a no-op
    # there, so one stop is not enough.
    result = PendingFuture(terminal(MODULE.GoalStatus.STATUS_ABORTED, PREEMPTED), polls=60)
    bridge, handle, _outcome = cancel(monkeypatch, result)
    assert bridge._stop_publisher.published.count('stop') >= 4
    assert handle.terminal == 'canceled' and not bridge._faulted


def test_a_trajectory_that_finished_before_the_stop_succeeded(monkeypatch):
    bridge, handle, outcome = cancel(
        monkeypatch, Future(terminal(MODULE.GoalStatus.STATUS_SUCCEEDED, SUCCESS)))
    assert outcome == (True, '')
    assert handle.terminal == 'succeeded'
    assert not bridge._faulted


def test_an_execution_failure_while_stopping_aborts_without_a_fault(monkeypatch):
    # MoveIt reports the execution over; the motion state is known.
    bridge, handle, (succeeded, reason) = cancel(monkeypatch, Future(terminal(
        MODULE.GoalStatus.STATUS_ABORTED, MODULE.MoveItErrorCodes.CONTROL_FAILED)))
    assert not succeeded and 'CONTROL_FAILED' in reason
    assert handle.terminal == 'aborted'
    assert not bridge._faulted and not bridge._goal_reserved


def test_a_rejected_cancel_is_decided_by_the_execution_not_the_response(monkeypatch):
    rejected = SimpleNamespace(return_code=CancelGoal.Response.ERROR_REJECTED, goals_canceling=[])
    bridge, handle, _outcome = cancel(
        monkeypatch, Future(terminal(MODULE.GoalStatus.STATUS_ABORTED, PREEMPTED)),
        MoveHandle(cancel_response=rejected))
    assert handle.terminal == 'canceled' and not bridge._faulted
    bridge, handle, _outcome = cancel(
        monkeypatch, Future(terminal(MODULE.GoalStatus.STATUS_ABORTED, PREEMPTED)),
        MoveHandle(cancel_raises=True))
    assert handle.terminal == 'canceled' and not bridge._faulted


def test_an_execution_that_never_ends_after_the_stop_latches_fault(monkeypatch):
    result = SimpleNamespace(done=lambda: False)
    bridge, handle, (succeeded, reason) = cancel(monkeypatch, result)
    assert not succeeded and 'motion state is unknown' in reason
    assert handle.terminal == 'aborted'
    assert bridge._faulted and bridge._goal_reserved


@pytest.mark.parametrize('result', [
    Future(None),
    ExceptionalFuture(None),
    Future(terminal(MODULE.GoalStatus.STATUS_UNKNOWN)),
])
def test_an_unknown_outcome_after_the_stop_latches_fault(monkeypatch, result):
    bridge, handle, (succeeded, _reason) = cancel(monkeypatch, result)
    assert not succeeded and handle.terminal == 'aborted'
    assert bridge._faulted and bridge._goal_reserved


def test_a_fault_while_executing_also_asks_move_group_to_stop(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()))
    bridge._execute_client = executor(None)
    handle = ChoHandle(cancel_requested=False)
    bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert bridge._faulted
    assert bridge._stop_publisher.published == ['stop']
    assert not bridge._executing


def test_shutting_down_mid_execution_asks_move_group_to_stop():
    bridge = bare_bridge()
    bridge.stop_active_execution()
    assert bridge._stop_publisher.published == []
    bridge._executing = True
    bridge.stop_active_execution()
    assert bridge._stop_publisher.published == ['stop']
    # Delivered before the context goes away, not just queued.
    assert bridge._stop_publisher.acked == ['stop']


def test_moveit_failure_reason_names_the_error_code(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(
        MODULE.MoveItErrorCodes.NO_IK_SOLUTION, status=MODULE.GoalStatus.STATUS_ABORTED))
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded
    assert 'NO_IK_SOLUTION(-31)' in reason
    assert handle.terminal == 'aborted'


def test_successful_run_reports_no_reason(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory()))
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=False)
    assert bridge._run_move_group(
        handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl') == (True, '')
    assert handle.terminal == 'succeeded'


# --------------------------------------------- duration_sec is a minimum

def test_the_goal_is_planned_only_and_the_planning_budget_is_its_own():
    # duration_sec used to be spent as allowed_planning_time (clamped 1-10 s).
    bridge = bare_bridge()
    bridge._planning_time = 7.5
    goal = bridge._plan_goal(bridge._joint_constraints([0.0] * 6), 'ompl')
    assert goal.planning_options.plan_only is True
    assert goal.request.allowed_planning_time == 7.5


def test_a_plan_shorter_than_the_goal_duration_is_executed_slowed_to_it(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, trajectory((0.0, 1.0, 2.0))))
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=False)
    assert bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')[0]

    executed = bridge._execute_client.sent[0].trajectory.joint_trajectory.points
    times = [MODULE._seconds(point.time_from_start) for point in executed]
    assert times == pytest.approx([0.0, 2.5, 5.0])
    assert executed[1].velocities == pytest.approx([0.4] * 6)        # 1.0 / 2.5
    assert executed[1].accelerations == pytest.approx([0.32] * 6)    # 2.0 / 2.5**2


def test_a_plan_longer_than_the_goal_duration_is_never_sped_up():
    planned = trajectory((0.0, 3.0, 6.0))
    assert MODULE.stretch_to_minimum_duration(planned, 5.0) == (6.0, 1.0)
    points = planned.joint_trajectory.points
    assert [MODULE._seconds(point.time_from_start) for point in points] == [0.0, 3.0, 6.0]
    assert list(points[1].velocities) == [1.0] * 6


def test_a_plan_with_no_duration_is_left_alone():
    # Already at its goal: nothing to slow down.
    planned = trajectory((0.0,))
    assert MODULE.stretch_to_minimum_duration(planned, 5.0) == (0.0, 1.0)


@pytest.mark.parametrize('planned,minimum', [
    (RobotTrajectory(), 5.0),
    (trajectory(), 0.0),
    (trajectory(), float('nan')),
])
def test_stretching_refuses_what_it_cannot_honour(planned, minimum):
    with pytest.raises(ValueError):
        MODULE.stretch_to_minimum_duration(planned, minimum)


def test_an_unusable_plan_aborts_before_anything_executes(monkeypatch):
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: True)
    bridge = bare_bridge()
    bridge._move_client = planner(wrapped(SUCCESS, RobotTrajectory()))
    bridge._execute_client = executor(wrapped(SUCCESS))
    handle = ChoHandle(cancel_requested=False)
    succeeded, reason = bridge._run_move_group(handle, _constraints(bridge), 5.0, SimpleNamespace, 'ompl')
    assert not succeeded and 'unusable plan' in reason
    assert bridge._execute_client.sent == []


def test_task_failure_reports_the_resolved_world_target():
    bridge = bare_bridge()
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


def test_plan_goal_carries_the_requested_pipeline():
    """The pipeline is per-request, not baked in."""
    bridge = bare_bridge()
    constraints = bridge._joint_constraints([0.0] * len(bridge._joint_names))
    for pipeline in ('ompl', 'some_other_pipeline'):
        assert bridge._plan_goal(constraints, pipeline).request.pipeline_id == pipeline


def test_planning_pipeline_defaults_to_ompl():
    """OMPL is the only pipeline this project registers, and both goal types use it."""
    bridge = bare_bridge()
    assert bridge._pipeline == 'ompl'
