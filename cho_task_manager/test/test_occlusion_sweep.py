"""The sweep leaf's state machine, with no ROS graph and no arm.

The rules it applies are tested in ``test_occlusion``; what is tested here is
the sequencing around them, which is where a leaf that drives a robot can go
wrong quietly: skipping without moving when there is nothing to recover,
stopping at the first waypoint that works rather than driving the rest, and
never leaving a goal running when it fails.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus
from cho_task_manager.behaviors.action.occlusion_sweep import OcclusionSweepBehavior
from cho_task_manager.utils import occlusion
from cho_task_manager.utils.controller_names import ControllerNames
import py_trees
import pytest

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds / 1e9)

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


def _sweep(waypoints=('survey', 'close'), **overrides):
    spec = dict(object='beaker', recovery_camera='wrist',
                waypoints=tuple(occlusion.SweepWaypoint(name, (0.0,) * 6, 4.0)
                                for name in waypoints),
                waypoint_duration=4.0, dwell_sec=1.0, timeout_sec=60.0,
                min_decision_margin=0.0, min_tag_edge_px=0.0)
    spec.update(overrides)
    return occlusion.SweepSpec(**spec)


def _snapshot(publishing=False, wrist='not_in_frame', oak='not_in_frame',
              wrist_margin=70.0):
    """An ObjectVisibilityArray, as far as this behaviour reads one."""
    from cho_interfaces.msg import CameraVisibility

    def camera(name, state, priority, margin):
        return SimpleNamespace(
            camera=name, detail='',
            state=getattr(CameraVisibility, 'STATE_%s' % state.upper()),
            age_sec=0.1, priority=priority,
            decision_margin=margin, edge_px=45.0)

    return SimpleNamespace(objects=[SimpleNamespace(
        name='beaker', publishing=publishing, override_camera='wrist' if publishing else '',
        status='test', cameras=[camera('oak', oak, 0, 40.0),
                                camera('wrist', wrist, 10, wrist_margin)])])


def _behaviour(sweep=None, **overrides):
    behaviour = OcclusionSweepBehavior(
        'Recover_Beaker', sweep or _sweep(),
        controller_name=ControllerNames.JOINT_POSITION, **overrides)
    behaviour.node = MagicMock()
    behaviour.clock = FakeClock()
    behaviour.node.get_clock.return_value = behaviour.clock
    behaviour.client = MagicMock()
    behaviour.client.wait_for_server.return_value = True
    return behaviour


def _accept_goal(behaviour):
    """Wire the action client so the next goal is accepted and stays running."""
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    result_future = MagicMock()
    result_future.done.return_value = False
    goal_handle.get_result_async.return_value = result_future
    behaviour.client.send_goal_async.return_value = send_future
    return goal_handle, result_future


def _finish_goal(result_future, status=GoalStatus.STATUS_SUCCEEDED):
    result_future.done.return_value = True
    result_future.result.return_value = MagicMock(status=status)


def _arrive(behaviour, result_future):
    """Tick the leaf through goal acceptance and arrival at a waypoint."""
    assert behaviour.update() == RUNNING       # goal accepted
    _finish_goal(result_future)
    assert behaviour.update() == RUNNING       # arrived, now dwelling


# ------------------------------------------------------------ not moving

def test_an_object_already_published_succeeds_without_a_goal():
    # The cheap path, and the reason this leaf sits in front of every
    # detection rather than behind a failed one.
    behaviour = _behaviour()
    behaviour.initialise()
    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok', oak='suppressed'))
    assert behaviour.update() == SUCCESS
    behaviour.client.send_goal_async.assert_not_called()


def test_it_waits_for_a_snapshot_before_deciding_anything():
    # Deciding on no information would mean sweeping every time the pose node
    # is a moment late.
    behaviour = _behaviour()
    behaviour.initialise()
    assert behaviour.update() == RUNNING
    behaviour.client.send_goal_async.assert_not_called()


def test_a_topic_that_never_arrives_fails_at_the_sweep_deadline():
    # The ceiling that bounds the wait above: a pose node that is not running
    # publishes nothing, and this leaf must not wait for it forever.
    behaviour = _behaviour(_sweep(timeout_sec=10.0))
    behaviour.initialise()
    assert behaviour.update() == RUNNING
    behaviour.clock.advance(11.0)
    assert behaviour.update() == FAILURE


def test_a_refusal_fails_without_moving():
    behaviour = _behaviour()
    behaviour.initialise()
    behaviour._on_visibility(_snapshot(wrist='no_tf'))
    assert behaviour.update() == FAILURE
    behaviour.client.send_goal_async.assert_not_called()


def test_an_object_the_pose_node_does_not_carry_fails():
    # The sweep table and the object table are separate files and can name the
    # vessel differently.
    behaviour = _behaviour(_sweep(object='bottle'))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    assert behaviour.update() == FAILURE


# --------------------------------------------------------------- sweeping

def test_it_stops_at_the_first_waypoint_that_works():
    behaviour = _behaviour(_sweep(('survey', 'high', 'close')))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)

    assert behaviour.update() == RUNNING       # assessed, first goal sent
    assert behaviour.client.send_goal_async.call_count == 1
    _arrive(behaviour, result)

    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok', oak='suppressed'))
    behaviour.clock.advance(1.5)
    assert behaviour.update() == SUCCESS
    # The two remaining waypoints are not driven: the arm stops where the
    # camera can see, which is also where the pose has to stay fresh for the
    # latch that follows.
    assert behaviour.client.send_goal_async.call_count == 1


def test_it_moves_on_when_a_waypoint_sees_nothing():
    behaviour = _behaviour(_sweep(('survey', 'close')))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)

    assert behaviour.update() == RUNNING
    _arrive(behaviour, result)
    behaviour.clock.advance(1.5)

    _goal2, result2 = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING       # still nothing: next waypoint
    assert behaviour.client.send_goal_async.call_count == 2
    _arrive(behaviour, result2)
    behaviour.clock.advance(1.5)
    assert behaviour.update() == FAILURE       # out of waypoints


def test_a_pose_that_decodes_badly_sends_it_down_a_rung():
    # THE LADDER'S WHOLE POINT. cho_object_pose publishes from margin 35 and
    # the far camera already clears that, so a sweep that stopped at the first
    # rung willing to publish would have gained nothing by moving.
    behaviour = _behaviour(_sweep(('survey', 'close'), min_decision_margin=55.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    _arrive(behaviour, result)

    # Published, from the wrist, and still not good enough.
    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok', oak='suppressed',
                                       wrist_margin=41.0))
    behaviour.clock.advance(1.5)
    _goal2, result2 = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    assert behaviour.client.send_goal_async.call_count == 2

    _arrive(behaviour, result2)
    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok', oak='suppressed',
                                       wrist_margin=63.0))
    behaviour.clock.advance(1.5)
    assert behaviour.update() == SUCCESS


def test_running_out_of_waypoints_with_a_pose_in_hand_is_best_effort():
    # A sweep that could not improve on what was already there still has what
    # was already there. Failing would turn a task that used to work into one
    # that does not, on the strength of an improvement nothing promised.
    behaviour = _behaviour(_sweep(('close',), min_decision_margin=55.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    _arrive(behaviour, result)
    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok', wrist_margin=44.0))
    behaviour.clock.advance(1.5)
    assert behaviour.update() == SUCCESS
    assert behaviour._best_margin == 44.0


def test_running_out_of_waypoints_with_no_pose_at_all_fails():
    # The other half of the same decision: a sweep sent because the object
    # could not be seen has nothing to hand back, and the diagnosis has to
    # separate 'never in view' from 'in view and never sharp enough'.
    behaviour = _behaviour(_sweep(('close',), min_decision_margin=55.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    _arrive(behaviour, result)
    behaviour._on_visibility(_snapshot(publishing=False, wrist='ok', wrist_margin=44.0))
    behaviour.clock.advance(1.5)
    assert behaviour.update() == FAILURE
    assert 'come closer' in behaviour._diagnosis()


def test_a_poor_published_pose_sends_it_looking_without_being_occluded():
    # The second trigger, end to end through the leaf: nothing is hidden, the
    # pose is streaming, and the arm goes anyway because the decode is not
    # good enough to act on.
    behaviour = _behaviour(_sweep(('survey', 'close'), min_decision_margin=55.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok', oak='ok',
                                       wrist_margin=38.0))
    _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    behaviour.client.send_goal_async.assert_called_once()


def test_never_decoding_at_all_is_reported_as_looking_in_the_wrong_place():
    behaviour = _behaviour(_sweep(('close',), min_decision_margin=55.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot(wrist_margin=occlusion.NO_SCORE))
    _goal, result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    _arrive(behaviour, result)
    behaviour.clock.advance(1.5)
    assert behaviour.update() == FAILURE
    assert 'do not look where the object is' in behaviour._diagnosis()


def test_each_waypoint_may_take_its_own_time():
    # The first rung is a long swing from home; the rest are short descents.
    sweep = _sweep()
    sweep = sweep._replace(waypoints=(
        occlusion.SweepWaypoint('survey', (0.0,) * 6, 12.0),
        occlusion.SweepWaypoint('close', (0.1,) * 6, 4.0)))
    behaviour = _behaviour(sweep)
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    goal = behaviour.client.send_goal_async.call_args[0][0]
    assert goal.duration == 12.0


def test_it_judges_only_after_the_dwell():
    # A viewpoint judged before the arm has settled is judged on frames taken
    # while it was still arriving, and on a window the pose node has not had
    # time to fill.
    behaviour = _behaviour(_sweep(('close',), dwell_sec=2.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    _arrive(behaviour, result)

    behaviour._on_visibility(_snapshot(publishing=True, wrist='ok'))
    assert behaviour.update() == RUNNING       # dwell not over
    behaviour.clock.advance(2.5)
    assert behaviour.update() == SUCCESS


def test_a_waypoint_the_arm_cannot_reach_fails_the_sweep():
    behaviour = _behaviour()
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING       # assessed, goal sent
    assert behaviour.update() == RUNNING       # goal accepted
    _finish_goal(result, GoalStatus.STATUS_ABORTED)
    assert behaviour.update() == FAILURE


# ------------------------------------------------------------- cleaning up

def test_failing_mid_motion_cancels_the_goal():
    # THE ONE THAT MATTERS FOR THE ARM. guarded_mission takes a failure to the
    # abort branch, which switches to the hold controller -- and a sweep goal
    # still running through that switch is a motion nobody is watching.
    behaviour = _behaviour(_sweep(timeout_sec=5.0))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    goal_handle, _result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    assert behaviour.update() == RUNNING       # goal accepted, in flight

    behaviour.clock.advance(6.0)
    assert behaviour.update() == FAILURE
    behaviour.terminate(FAILURE)
    goal_handle.cancel_goal_async.assert_called()


def test_preemption_cancels_the_goal_as_every_action_leaf_does():
    # watched_mission invalidates the sibling branch on a monitor trip, and
    # BaseActionBehavior's own contract is that INVALID cancels.
    behaviour = _behaviour()
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    goal_handle, _result = _accept_goal(behaviour)
    assert behaviour.update() == RUNNING
    assert behaviour.update() == RUNNING
    behaviour.terminate(py_trees.common.Status.INVALID)
    goal_handle.cancel_goal_async.assert_called()


def test_a_second_run_starts_from_the_first_waypoint_again():
    # py_trees re-enters a leaf after a Selector retry or a repeated mission,
    # and a sweep that resumed at waypoint three would judge the bench from
    # wherever the last run left off.
    behaviour = _behaviour(_sweep(('survey', 'close')))
    behaviour.initialise()
    behaviour._on_visibility(_snapshot())
    _goal, result = _accept_goal(behaviour)
    behaviour.update()
    _arrive(behaviour, result)
    behaviour.clock.advance(1.5)
    behaviour.update()
    assert behaviour._index == 1

    behaviour.terminate(py_trees.common.Status.INVALID)
    behaviour.initialise()
    assert behaviour._index == 0
    assert behaviour._phase == 'assess'
    # And it decides on a snapshot from THIS run, not the one it was holding.
    assert behaviour._latest is None


def test_a_sweep_with_no_waypoints_is_refused_at_build_time():
    with pytest.raises(ValueError):
        OcclusionSweepBehavior('Nowhere', _sweep(waypoints=()))
