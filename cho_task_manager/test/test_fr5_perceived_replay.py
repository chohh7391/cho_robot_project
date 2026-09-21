"""The replay that measures the cell instead of taking a file's word for it.

``trajectory_replay`` gates on a layout YAML someone keeps in step with the
bench by hand. This tree gates on what three cameras actually see, and can keep
watching while the arm runs. What is worth testing is therefore the seam, not
the motion -- the motion is imported from the tree beside it, and these tests
pin that reuse so a rename there fails here rather than in a pour:

* the declared layout becomes optional, and still refuses at BUILD time when given,
* only vessels that are both tracked and assumed are measured, and the rest are
  reported as unverified rather than silently passed,
* the check sits after the home move and before the handover,
* the drift watchdog is opt-in, and naming a vessel no camera tracks raises.

Nothing here needs a ROS graph: the trees are built, not ticked, and the two
behaviours are driven with a stub node.
"""

import json
from unittest.mock import MagicMock

from geometry_msgs.msg import Pose, PoseStamped
import py_trees
import pytest

from cho_task_manager.behaviors.topic import (
    ObjectLayoutCheckBehavior,
    ObjectLayoutMonitorBehavior,
)
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.fr5 import perceived_replay, trajectory_replay
from cho_task_manager.tasks.fr5.common import VESSELS
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, write_client
from cho_task_manager.utils.controller_names import load_robot_config
from cho_task_manager.utils.trajectory_recording import LayoutMismatch

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE

HEADER = 't_s,j1,j2,j3,j4,j5,j6,operation\n'

#: Where the recording says the vessels were. Same names the cameras publish.
ASSUMED = {
    'beaker': {'xy': [0.48, 0.13]},
    'flask': {'xy': [0.53, 0.35]},
}


def _config(**overrides):
    config = load_robot_config('fr5')
    config.update(overrides)
    return config


def _write(tmp_path, count=12, meta=None, name='trial'):
    csv_path = tmp_path / ('%s_waypoints.csv' % name)
    meta_path = tmp_path / ('%s_meta.json' % name)
    rows = [([i * 0.1, 0.0, i * 0.001, 0.0, 0.0, 0.0, 0.0], 'Pick')
            for i in range(count)]
    csv_path.write_text(HEADER + ''.join(
        '%s,%s\n' % (','.join('%.6f' % v for v in row), op) for row, op in rows))
    base = {
        'seed': '5',
        'tool': 'fr5_ag95',
        'joint_names': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
        'waypoints': len(rows),
        'home_arm_rad': list(rows[0][0][1:]),
        'gripper_events': [],
        'layout_the_trajectory_assumes': dict(ASSUMED),
    }
    base.update(meta or {})
    meta_path.write_text(json.dumps(base))
    return str(csv_path), str(meta_path)


def _layout_file(tmp_path, layout, name='cell.yaml'):
    path = tmp_path / name
    lines = ['layout:']
    for obj, entry in layout.items():
        lines.append('  %s:' % obj)
        lines.append('    xy: [%r, %r]' % (entry['xy'][0], entry['xy'][1]))
    path.write_text('\n'.join(lines) + '\n')
    return str(path)


def _tree(tmp_path, **overrides):
    csv_path, meta_path = _write(tmp_path)
    config = _config(replay_trajectory=csv_path, replay_meta=meta_path,
                     home_via='direct', **overrides)
    return build_task_tree('perceived_replay', config)


def _named(tree):
    return {node.name: node for node in tree.iterate()}


# --------------------------------------------------------------- the task

def test_the_task_is_registered_for_fr5_only():
    assert 'perceived_replay' in available_tasks('fr5')
    for robot in ('franka', 'ur5e', 'openarm'):
        assert 'perceived_replay' not in available_tasks(robot)


def test_the_motion_half_is_imported_rather_than_restated(tmp_path):
    # If either of these is renamed in trajectory_replay, this fails here
    # instead of the perceived replay quietly drifting away from the numbers
    # measured on the arm (position limits, gripper settle, velocity ceiling).
    assert callable(trajectory_replay._home_block)
    assert callable(trajectory_replay._replay_children)
    assert perceived_replay.create_fr5_perceived_replay_tree is not None


def test_it_builds_with_no_declared_layout_at_all(tmp_path):
    # The point of the task: the cameras are the gate, so replay_layout stops
    # being required.
    tree = _tree(tmp_path)
    assert 'Check_Cell_Matches_Recording' in _named(tree)
    assert tree.replay_summary['declared_layout'] is None


def test_a_declared_layout_still_refuses_at_build_time(tmp_path):
    # Strictly better than the measured check when you have one: nothing is
    # spun up and the arm has not moved.
    moved = {'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.60]}}
    csv_path, meta_path = _write(tmp_path)
    config = _config(replay_trajectory=csv_path, replay_meta=meta_path,
                     home_via='direct',
                     replay_layout=_layout_file(tmp_path, moved))
    with pytest.raises(LayoutMismatch):
        build_task_tree('perceived_replay', config)


def test_a_matching_declared_layout_builds_and_is_recorded(tmp_path):
    path = _layout_file(tmp_path, ASSUMED)
    tree = _tree(tmp_path, replay_layout=path)
    assert tree.replay_summary['declared_layout'] == path


# ------------------------------------------------------------ what is measured

def test_every_tracked_vessel_the_recording_assumes_is_detected(tmp_path):
    tree = _tree(tmp_path)
    by_name = _named(tree)
    for vessel in VESSELS:
        detect = by_name['Detect_%s' % vessel.name.capitalize()]
        assert detect.topic == vessel.topic
        assert detect.record_as == vessel.key
        assert detect.required_frame == _config()['arm_base_link']
        assert detect.namespace == TASK_NAMESPACE


def test_an_assumed_object_no_camera_tracks_is_reported_unverified(tmp_path):
    assumed = dict(ASSUMED)
    assumed['stirrer'] = {'xy': [0.20, 0.40]}
    csv_path, meta_path = _write(
        tmp_path, meta={'layout_the_trajectory_assumes': assumed})
    tree = build_task_tree('perceived_replay', _config(
        replay_trajectory=csv_path, replay_meta=meta_path, home_via='direct'))
    # Not a failure: the arm still goes there, so it is named out loud and left
    # to the declared layout file rather than passed over.
    assert tree.replay_summary['unverified'] == ['stirrer']
    assert tree.replay_summary['measured'] == ['beaker', 'flask']
    assert 'Detect_Stirrer' not in _named(tree)


def test_a_recording_this_bench_cannot_measure_is_refused(tmp_path):
    csv_path, meta_path = _write(tmp_path, meta={
        'layout_the_trajectory_assumes': {'stirrer': {'xy': [0.2, 0.4]}}})
    with pytest.raises(ValueError, match='nothing to perceive'):
        build_task_tree('perceived_replay', _config(
            replay_trajectory=csv_path, replay_meta=meta_path, home_via='direct'))


def test_the_cell_is_measured_after_homing_and_before_the_handover(tmp_path):
    # After: the arm is then at the recording's own start pose, a repeatable
    # viewpoint rather than wherever it was left leaning over the bench.
    # Before: a refusal lands while the arm is still held and nothing is
    # streaming at the trajectory controller.
    tree = _tree(tmp_path)
    mission = _named(tree)['FR5_Perceived_Replay_Sequence']
    assert [child.name for child in mission.children] == [
        '1_Initialize', '2_Verify_Cell', '3_Handover', '4_Replay', '5_Finish']
    verify = _named(tree)['2_Verify_Cell']
    assert [child.name for child in verify.children][-1] == 'Check_Cell_Matches_Recording'


# ----------------------------------------------------------------- the watchdog

def test_no_watchdog_unless_it_is_asked_for(tmp_path):
    # Off by default because a transfer recording MOVES a vessel on purpose.
    tree = _tree(tmp_path)
    assert 'Watch_Cell_Layout' not in _named(tree)
    assert tree.replay_summary['watched'] == []


def test_replay_watch_adds_a_parallel_watchdog_beside_the_mission(tmp_path):
    tree = _tree(tmp_path, replay_watch='flask')
    by_name = _named(tree)
    monitor = by_name['Watch_Cell_Layout']
    assert list(monitor.topics) == ['flask']
    watched = by_name['Mission_Under_Watch']
    # Parallel, not a decorator: only a Parallel invalidates the sibling, which
    # is what cancels the goal in flight.
    assert isinstance(watched, py_trees.composites.Parallel)
    assert [child.name for child in watched.children][0] == 'Watch_Cell_Layout'
    assert tree.replay_summary['watched'] == ['flask']


def test_replay_watch_accepts_several_names(tmp_path):
    tree = _tree(tmp_path, replay_watch='flask, beaker')
    assert sorted(_named(tree)['Watch_Cell_Layout'].topics) == ['beaker', 'flask']


def test_watching_a_vessel_no_camera_tracks_raises(tmp_path):
    with pytest.raises(ValueError, match='no camera tracks'):
        _tree(tmp_path, replay_watch='stirrer')


# ------------------------------------------------------- the check behaviour

def _check(expected=None, tolerance=0.015, keys=None):
    keys = keys or {'beaker': 'beaker_pose'}
    behaviour = ObjectLayoutCheckBehavior(
        name='Check', expected=expected or dict(ASSUMED), keys=keys,
        position_tolerance_m=tolerance)
    behaviour.node = MagicMock()
    return behaviour


def _latch(key, x, y):
    board = write_client('producer', [key], TASK_NAMESPACE)
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    setattr(board, key, pose)


def test_the_check_passes_when_the_vessel_is_where_the_recording_assumes():
    _latch('beaker_pose', 0.482, 0.129)
    assert _check().update() == SUCCESS


def test_the_check_fails_when_the_vessel_has_moved():
    _latch('beaker_pose', 0.48, 0.13 + 0.05)
    assert _check().update() == FAILURE


def test_the_check_fails_rather_than_passing_on_a_missing_detection():
    behaviour = _check(keys={'beaker': 'never_written_pose'})
    assert behaviour.update() == FAILURE


def test_the_check_refuses_to_be_built_with_nothing_to_measure():
    with pytest.raises(ValueError, match='no object is measured'):
        ObjectLayoutCheckBehavior(name='Check', expected=dict(ASSUMED), keys={})


def test_the_check_refuses_an_object_the_plan_does_not_assume():
    with pytest.raises(ValueError, match='no such object'):
        ObjectLayoutCheckBehavior(
            name='Check', expected={'flask': {'xy': [0.0, 0.0]}},
            keys={'beaker': 'beaker_pose'})


# ----------------------------------------------------- the monitor behaviour

def _monitor(tolerance=0.015):
    behaviour = ObjectLayoutMonitorBehavior(
        name='Watch', expected=dict(ASSUMED), topics={'flask': '/flask'},
        required_frame='base_link', position_tolerance_m=tolerance)
    behaviour.node = MagicMock()
    return behaviour


def _pose(x, y, frame='base_link'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y = x, y
    return msg


def test_the_monitor_holds_its_opinion_while_nothing_is_visible():
    # The arm occludes the bench on purpose here -- it is reaching across it --
    # so a monitor that tripped on silence would abort the runs it protects.
    # This is the one place it deliberately differs from SafetyMonitorBehavior,
    # whose staleness IS a trip.
    assert _monitor().update() == RUNNING


def test_the_monitor_stays_running_while_the_vessel_is_where_it_should_be():
    behaviour = _monitor()
    behaviour._on_pose('flask', _pose(0.531, 0.348))
    assert behaviour.update() == RUNNING


def test_the_monitor_trips_when_the_vessel_moves():
    behaviour = _monitor()
    behaviour._on_pose('flask', _pose(0.53, 0.35 + 0.04))
    assert behaviour.update() == FAILURE


def test_the_monitor_never_succeeds():
    # A watchdog has no success condition; watched_mission puts the Parallel's
    # success on the mission branch alone.
    behaviour = _monitor()
    behaviour._on_pose('flask', _pose(0.53, 0.35))
    assert behaviour.update() != SUCCESS


def test_the_monitor_trips_on_a_pose_from_the_wrong_frame():
    behaviour = _monitor()
    behaviour._on_pose('flask', _pose(0.53, 0.35, frame='camera_link'))
    assert behaviour.update() == FAILURE


def test_the_monitor_refuses_to_be_built_watching_nothing():
    with pytest.raises(ValueError, match='watching nothing'):
        ObjectLayoutMonitorBehavior(
            name='Watch', expected=dict(ASSUMED), topics={},
            required_frame='base_link')
