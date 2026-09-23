"""One pass over the raster for every object, with no ROS graph and no arm.

The per-object rules are ``test_occlusion``'s; what is tested here is what the
single pass adds: each object latched at the viewpoint it was found from, the
raster never driven twice, and the latch refusing a pose older than the
evidence that it is the wrist's.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus
from cho_task_manager.behaviors.action.single_pass_sweep import (
    SinglePassSweepBehavior,
    SweepTarget,
)
from cho_task_manager.utils import occlusion
from cho_task_manager.utils.controller_names import ControllerNames
import py_trees
import pytest

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE


@pytest.fixture(autouse=True)
def _fresh_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds / 1e9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=(self.seconds - other.seconds) * 1e9)

    def __gt__(self, other):
        return self.seconds > other.seconds

    def __lt__(self, other):
        return self.seconds < other.seconds


class FakeClock:
    def __init__(self, start=0.0):
        self.seconds = start

    def now(self):
        return FakeTime(self.seconds)

    def advance(self, dt):
        self.seconds += dt


RASTER = ('r0c0', 'r0c1', 'r0c2')


def _sweep(name, waypoints=RASTER, **overrides):
    spec = dict(object=name, recovery_camera='wrist',
                waypoints=tuple(occlusion.SweepWaypoint(point, (0.0,) * 6, 3.0)
                                for point in waypoints),
                waypoint_duration=3.0, dwell_sec=1.0, timeout_sec=60.0,
                min_decision_margin=55.0, min_tag_edge_px=0.0, planning_target=True)
    spec.update(overrides)
    return occlusion.SweepSpec(**spec)


def _targets(**overrides):
    return [SweepTarget(_sweep('beaker', **overrides), 'beaker_pose',
                        '/perception/object_pose/beaker'),
            SweepTarget(_sweep('flask', **overrides), 'flask_pose',
                        '/perception/object_pose/flask')]


def _entry(name, wrist='not_in_frame', side_1='ok', publishing=True, wrist_margin=90.0):
    """One ObjectVisibility, as far as the leaf reads one."""
    from cho_interfaces.msg import CameraVisibility

    def camera(camera_name, state, priority, margin):
        # A camera that decoded nothing reports no score, as the real topic does.
        decoded = state in ('ok', 'suppressed', 'rejected')
        return SimpleNamespace(
            camera=camera_name, detail='',
            state=getattr(CameraVisibility, 'STATE_%s' % state.upper()),
            age_sec=0.1, priority=priority,
            decision_margin=margin if decoded else occlusion.NO_SCORE,
            edge_px=55.0 if decoded else occlusion.NO_SCORE)

    return SimpleNamespace(
        name=name, publishing=publishing,
        override_camera='wrist' if wrist == 'ok' else '', status='test',
        cameras=[camera('side_1', side_1, 0, 150.0),
                 camera('wrist', wrist, 10, wrist_margin)])


def _standing(name):
    """Published from the standing camera, the wrist not seeing it."""
    return _entry(name)


def _close(name):
    """The wrist measuring it and overriding the standing camera."""
    return _entry(name, wrist='ok', side_1='suppressed')


def _snapshot(*entries):
    return SimpleNamespace(objects=list(entries))


def _pose(x, frame='base_link'):
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame),
                           pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0, z=0.1)))


def _behaviour(targets=None, **overrides):
    behaviour = SinglePassSweepBehavior(
        'Recover_Vessels', targets or _targets(), required_frame='base_link',
        controller_name=ControllerNames.JOINT_POSITION, **overrides)
    behaviour.node = MagicMock()
    behaviour.clock = FakeClock()
    behaviour.node.get_clock.return_value = behaviour.clock
    behaviour.client = MagicMock()
    behaviour.client.wait_for_server.return_value = True
    return behaviour


def _target(behaviour, name):
    return next(target for target in behaviour.targets if target.name == name)


def _see(behaviour, snapshot, **poses):
    """Deliver a snapshot, then a pose per object -- so the poses are the newer."""
    behaviour._on_visibility(snapshot)
    for name, pose in poses.items():
        behaviour._on_pose(_target(behaviour, name), pose)


def _accept_goal(behaviour):
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    result_future = MagicMock()
    result_future.done.return_value = False
    goal_handle.get_result_async.return_value = result_future
    behaviour.client.send_goal_async.return_value = send_future
    return goal_handle, result_future


def _arrive(behaviour, result_future):
    assert behaviour.update() == RUNNING       # goal accepted
    result_future.done.return_value = True
    result_future.result.return_value = MagicMock(status=GoalStatus.STATUS_SUCCEEDED)
    assert behaviour.update() == RUNNING       # arrived, now dwelling


def _latched(key):
    return py_trees.blackboard.Blackboard.get('/task/%s' % key)


def _start(behaviour):
    """initialise, see both vessels from the standing camera, drive to r0c0."""
    behaviour.initialise()
    _see(behaviour, _snapshot(_standing('beaker'), _standing('flask')))
    _, result_future = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    _arrive(behaviour, result_future)


def _errors(behaviour):
    return ' '.join(str(call) for call in behaviour.node.get_logger().error.call_args_list)


# -------------------------------------------------------------- the pass

def test_each_object_is_latched_where_it_is_found_and_no_waypoint_is_driven_twice():
    # The run that motivated this leaf: beaker at the second waypoint, flask at
    # the third. Per object, the flask's sweep drove the first two again.
    behaviour = _behaviour()
    _start(behaviour)

    behaviour.clock.advance(1.1)
    _see(behaviour, _snapshot(_standing('beaker'), _standing('flask')))
    _, result_future = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING       # nothing at r0c0 -> r0c1
    _arrive(behaviour, result_future)

    behaviour.clock.advance(1.1)
    _see(behaviour, _snapshot(_close('beaker'), _standing('flask')), beaker=_pose(0.585))
    _, result_future = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING       # beaker latched here -> r0c2
    assert _latched('beaker_pose').position.x == 0.585
    assert not py_trees.blackboard.Blackboard.exists('/task/flask_pose')
    _arrive(behaviour, result_future)

    behaviour.clock.advance(1.1)
    _see(behaviour, _snapshot(_standing('beaker'), _close('flask')), flask=_pose(0.763))
    assert behaviour.update() == SUCCESS
    assert _latched('flask_pose').position.x == 0.763
    # The beaker's latch survives the arm moving on, which is the point of it.
    assert _latched('beaker_pose').position.x == 0.585
    assert behaviour.client.send_goal_async.call_count == 3


def test_objects_found_at_the_same_waypoint_end_the_pass_there():
    behaviour = _behaviour()
    _start(behaviour)
    behaviour.clock.advance(1.1)
    _see(behaviour, _snapshot(_close('beaker'), _close('flask')),
         beaker=_pose(0.5), flask=_pose(0.7))
    assert behaviour.update() == SUCCESS
    assert behaviour.client.send_goal_async.call_count == 1


def test_objects_already_measured_well_enough_are_latched_without_moving():
    # Not planning targets, and the standing camera's decode clears the bar:
    # the per-object leaf would skip, and so does the pass.
    behaviour = _behaviour(_targets(planning_target=False))
    behaviour.initialise()
    _see(behaviour, _snapshot(_standing('beaker'), _standing('flask')),
         beaker=_pose(0.5), flask=_pose(0.7))
    assert behaviour.update() == SUCCESS
    behaviour.client.send_goal_async.assert_not_called()
    assert _latched('beaker_pose').position.x == 0.5


def test_one_object_in_hand_at_the_start_leaves_the_pass_for_the_other():
    targets = [SweepTarget(_sweep('beaker', planning_target=False), 'beaker_pose', '/b'),
               SweepTarget(_sweep('flask'), 'flask_pose', '/f')]
    behaviour = _behaviour(targets)
    behaviour.initialise()
    _see(behaviour, _snapshot(_standing('beaker'), _standing('flask')), beaker=_pose(0.5))
    _, result_future = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING       # beaker latched, sweeping for the flask
    assert _latched('beaker_pose').position.x == 0.5
    assert behaviour.client.send_goal_async.call_count == 1


def test_it_waits_for_a_snapshot_before_deciding_anything():
    behaviour = _behaviour()
    behaviour.initialise()
    assert behaviour.update() == RUNNING
    behaviour.client.send_goal_async.assert_not_called()


# ------------------------------------------------------------- the latch

def test_a_pose_older_than_the_recovering_snapshot_is_not_latched():
    # It can be the standing camera's pose from before the wrist's samples
    # filled the window -- the measurement the sweep went to replace.
    behaviour = _behaviour()
    _start(behaviour)
    behaviour.clock.advance(1.1)
    behaviour._on_pose(_target(behaviour, 'beaker'), _pose(0.40))    # stale
    behaviour.clock.advance(0.1)
    behaviour._on_visibility(_snapshot(_close('beaker'), _standing('flask')))
    assert behaviour.update() == RUNNING
    assert not py_trees.blackboard.Blackboard.exists('/task/beaker_pose')
    assert behaviour.client.send_goal_async.call_count == 1          # still here

    behaviour.clock.advance(0.05)
    behaviour._on_pose(_target(behaviour, 'beaker'), _pose(0.585))
    _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    assert _latched('beaker_pose').position.x == 0.585
    assert behaviour.client.send_goal_async.call_count == 2          # moved on


def test_a_recovered_object_whose_pose_never_arrives_names_the_topic():
    behaviour = _behaviour(latch_timeout_sec=2.0)
    _start(behaviour)
    behaviour.clock.advance(1.1)
    behaviour._on_visibility(_snapshot(_close('beaker'), _standing('flask')))
    assert behaviour.update() == RUNNING
    behaviour.clock.advance(2.1)
    assert behaviour.update() == FAILURE
    assert '/perception/object_pose/beaker' in _errors(behaviour)


def test_a_pose_in_another_frame_is_refused_rather_than_latched():
    behaviour = _behaviour()
    _start(behaviour)
    behaviour.clock.advance(1.1)
    _see(behaviour, _snapshot(_close('beaker'), _standing('flask')),
         beaker=_pose(0.5, frame='wrist_infra1_optical_frame'))
    assert behaviour.update() == FAILURE
    assert not py_trees.blackboard.Blackboard.exists('/task/beaker_pose')


# --------------------------------------------------------- before moving

def test_a_refusal_for_any_object_fails_before_anything_moves():
    behaviour = _behaviour()
    behaviour.initialise()
    _see(behaviour, _snapshot(_standing('beaker'),
                              _entry('flask', wrist='no_tf', side_1='not_in_frame',
                                     publishing=False)))
    assert behaviour.update() == FAILURE
    behaviour.client.send_goal_async.assert_not_called()


def test_one_object_that_has_only_just_gone_missing_holds_the_pass():
    behaviour = _behaviour(_targets(min_unseen_sec=2.0))
    behaviour.initialise()
    gone = _entry('flask', side_1='not_in_frame', publishing=False)
    _see(behaviour, _snapshot(_standing('beaker'), gone))
    assert behaviour.update() == RUNNING
    behaviour.client.send_goal_async.assert_not_called()
    behaviour.clock.advance(2.1)
    _see(behaviour, _snapshot(_standing('beaker'), gone))
    _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    assert behaviour.client.send_goal_async.call_count == 1


def test_an_object_the_pose_node_does_not_carry_fails():
    behaviour = _behaviour()
    behaviour.initialise()
    _see(behaviour, _snapshot(_standing('beaker')))
    assert behaviour.update() == FAILURE
    assert "'flask'" in _errors(behaviour)


# -------------------------------------------------------- running out

def _run_out(behaviour, last):
    """Drive every waypoint seeing *last* from the standing view only."""
    _start(behaviour)
    for _ in RASTER[1:]:
        behaviour.clock.advance(1.1)
        _see(behaviour, last)
        _, result_future = _accept_goal(behaviour)
        assert behaviour.update() == RUNNING
        _arrive(behaviour, result_future)
    behaviour.clock.advance(1.1)


def test_running_out_with_a_poor_pose_in_hand_is_best_effort():
    behaviour = _behaviour()
    snapshot = _snapshot(_standing('beaker'), _standing('flask'))
    _run_out(behaviour, snapshot)
    _see(behaviour, snapshot, beaker=_pose(0.5), flask=_pose(0.7))
    assert behaviour.update() == SUCCESS
    assert _latched('flask_pose').position.x == 0.7
    assert behaviour.client.send_goal_async.call_count == len(RASTER)


def test_running_out_with_an_object_nowhere_fails_and_says_where_to_look():
    behaviour = _behaviour()
    gone = _entry('flask', side_1='not_in_frame', publishing=False)
    snapshot = _snapshot(_standing('beaker'), gone)
    _run_out(behaviour, snapshot)
    _see(behaviour, snapshot)
    assert behaviour.update() == FAILURE
    assert 'never decoded' in _errors(behaviour)


# ------------------------------------------------------------ lifecycle

def test_failing_mid_motion_cancels_the_goal():
    behaviour = _behaviour()
    behaviour.initialise()
    _see(behaviour, _snapshot(_standing('beaker'), _standing('flask')))
    goal_handle, _ = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    assert behaviour.update() == RUNNING       # accepted, in flight
    behaviour.clock.advance(61.0)
    assert behaviour.update() == FAILURE
    behaviour.terminate(FAILURE)
    goal_handle.cancel_goal_async.assert_called_once()


def test_a_second_run_starts_from_the_first_waypoint_with_nothing_in_hand():
    behaviour = _behaviour()
    _start(behaviour)
    behaviour.clock.advance(1.1)
    _see(behaviour, _snapshot(_close('beaker'), _close('flask')),
         beaker=_pose(0.5), flask=_pose(0.7))
    assert behaviour.update() == SUCCESS
    behaviour.terminate(SUCCESS)

    behaviour.initialise()
    assert all(not target.latched for target in behaviour.targets)
    _see(behaviour, _snapshot(_standing('beaker'), _standing('flask')))
    _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    goal = behaviour.client.send_goal_async.call_args[0][0]
    assert goal.duration == 3.0
    assert behaviour._index == 0


# ----------------------------------------------------------- build time

def test_objects_on_different_rasters_are_refused_at_build_time():
    targets = [SweepTarget(_sweep('beaker'), 'beaker_pose', '/b'),
               SweepTarget(_sweep('flask', waypoints=('r0c0', 'close')), 'flask_pose', '/f')]
    with pytest.raises(ValueError, match='one object at a time'):
        _behaviour(targets)


def test_an_object_listed_twice_is_refused():
    targets = [SweepTarget(_sweep('beaker'), 'beaker_pose', '/b'),
               SweepTarget(_sweep('beaker'), 'other_pose', '/b')]
    with pytest.raises(ValueError, match='twice'):
        _behaviour(targets)


def test_a_frame_has_to_be_named():
    with pytest.raises(ValueError, match='required_frame'):
        SinglePassSweepBehavior('Recover_Vessels', _targets(), required_frame='')


def test_a_pass_for_nothing_is_refused():
    with pytest.raises(ValueError, match='no targets'):
        SinglePassSweepBehavior('Recover_Vessels', [], required_frame='base_link')
