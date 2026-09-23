"""Handing a recording's pour to the pouring controller, in the middle of a replay.

The replay goes as far as the pour's first waypoint, the controller pours by
weight and returns the arm to that same configuration, and the replay resumes
from the pour's last waypoint. That is only seamless if the recording puts the
two at one place, the pour is one span, and the jaws do not move inside it --
each of which is refused here, at tree build time, when it is not so.

No ROS graph: trees are built, not ticked, and the latch runs on a fake clock.
"""

import json
import os
from unittest.mock import MagicMock

import py_trees
import pytest

from cho_task_manager.behaviors.action import PourActionBehavior
from cho_task_manager.behaviors.topic import (
    GraspMarkerSampleBehavior,
    JointStateCheckBehavior,
    ScaleLatchBehavior,
)
from cho_task_manager.behaviors.topic.grasp_marker import GRASP_JOINTS_KEY, GRASP_MARKER_KEY
from cho_task_manager.subtrees.pour import (
    CONTAINER_AUTO,
    CONTAINER_KEY,
    DEFAULT_MARKER_TOPIC,
    parse_pour_request,
)
from cho_task_manager.utils.blackboard import write_client
from cho_task_manager.tasks import build_task_tree
from cho_task_manager.utils.blackboard import TASK_NAMESPACE
from cho_task_manager.utils.controller_names import load_robot_config
from cho_task_manager.utils.pour_splice import PourSegment, splice_pour
from cho_task_manager.utils.trajectory_recording import (
    RecordingRejected,
    load_recording,
    plan_segments,
)

HEADER = 't_s,j1,j2,j3,j4,j5,j6,operation\n'
LAYOUT = {'beaker': {'xy': [0.584, -0.298]}, 'flask': {'xy': [0.52, 0.32]}}
REAL = os.path.join(os.path.dirname(__file__), '..', 'config', 'replay', 'real')


def _rows(pour_end_j2=0.1, pour_ops=None):
    """Approach, pour (j6 out and back), place: 30 rows at 0.1 s.

    Rows 10..19 are the pour. It starts where the approach stopped (j2 0.1) and,
    unless told otherwise, ends there too.
    """
    rows = []
    for i in range(30):
        j2, j6 = 0.1, 0.0
        if i < 10:
            op, j2 = 'Move_to_Surface', 0.01 * i + 0.01
        elif i < 20:
            op = 'pouring'
            j6 = -1.2 * (1.0 - abs(i - 14.5) / 4.5) if 10 < i < 19 else 0.0
            if i == 19:
                j2 = pour_end_j2
        else:
            op, j2 = 'Place', 0.1 - 0.005 * (i - 19)
        if pour_ops is not None:
            op = pour_ops.get(i, op)
        rows.append(([0.1 * i, 0.0, j2, 0.0, 0.0, 0.0, j6], op))
    return rows


def _write(tmp_path, rows, events=({'t_s': 0.05, 'event': 'close'},
                                   {'t_s': 2.85, 'event': 'open'})):
    csv_path = tmp_path / 'trial_waypoints.csv'
    meta_path = tmp_path / 'trial_meta.json'
    csv_path.write_text(HEADER + ''.join(
        '%s,%s\n' % (','.join('%.6f' % v for v in row), op) for row, op in rows))
    meta_path.write_text(json.dumps({
        'seed': '9', 'tool': 'fr5_ag95',
        'joint_names': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
        'waypoints': len(rows), 'home_arm_rad': list(rows[0][0][1:]),
        'gripper_events': list(events),
        'layout_the_trajectory_assumes': LAYOUT,
    }))
    layout_path = tmp_path / 'cell.yaml'
    layout_path.write_text('layout:\n' + ''.join(
        '  %s:\n    xy: [%r, %r]\n' % (name, e['xy'][0], e['xy'][1]) for name, e in LAYOUT.items()))
    return str(csv_path), str(meta_path), str(layout_path)


def _tree(tmp_path, rows=None, **pour):
    csv_path, meta_path, layout_path = _write(tmp_path, rows or _rows())
    config = load_robot_config('fr5')
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout_path, 'home_via': 'direct'})
    config.update(pour)
    return build_task_tree('trajectory_replay', config)


def _replay_names(tree):
    replay = [node for node in tree.iterate() if node.name == '3_Replay'][0]
    return [child.name for child in replay.children]


# --- the splice ---------------------------------------------------------------

def test_the_pour_is_cut_out_of_the_move_it_sat_in(tmp_path):
    csv_path, meta_path, _ = _write(tmp_path, _rows())
    recording = load_recording(csv_path, meta_path)
    spliced = splice_pour(recording, plan_segments(recording))

    assert [s.kind for s in spliced] == [
        'move', 'gripper', 'move', 'pour', 'move', 'gripper', 'move']
    before, pour, after = spliced[2], spliced[3], spliced[4]
    assert isinstance(pour, PourSegment)
    # The replay stops at the pour's first waypoint and resumes from its last,
    # which is the same configuration: the controller comes back to where it
    # was handed the arm.
    assert before.positions[-1] == recording.positions[10]
    assert after.positions[0] == recording.positions[19]
    assert before.positions[-1] == after.positions[0] == pour.start
    assert pour.source_t0 == pytest.approx(1.0)
    assert pour.recorded_duration == pytest.approx(0.9)
    # The deepest tilt: j6 furthest out, at row 14/15.
    assert pour.reference == recording.positions[14]
    # Nothing of the recorded tilt is replayed.
    assert all(abs(q[5]) < 1e-12 for q in before.positions + after.positions)
    # Each remaining piece is timed from its own start.
    assert after.times[0] == 0.0


def test_a_recording_with_no_pour_is_refused_when_one_is_asked_for(tmp_path):
    rows = _rows(pour_ops={i: 'Place' for i in range(10, 20)})
    csv_path, meta_path, _ = _write(tmp_path, rows)
    recording = load_recording(csv_path, meta_path)
    with pytest.raises(RecordingRejected) as excinfo:
        splice_pour(recording, plan_segments(recording))
    assert "labelled 'pouring'" in str(excinfo.value)


def test_a_pour_that_ends_somewhere_else_is_refused(tmp_path):
    # The replay would resume from a configuration the controller never goes to.
    csv_path, meta_path, _ = _write(tmp_path, _rows(pour_end_j2=0.13))
    recording = load_recording(csv_path, meta_path)
    with pytest.raises(RecordingRejected) as excinfo:
        splice_pour(recording, plan_segments(recording))
    assert 'from where it started (j2)' in str(excinfo.value)


def test_a_pour_the_jaws_move_inside_is_refused(tmp_path):
    csv_path, meta_path, _ = _write(tmp_path, _rows(), events=(
        {'t_s': 0.05, 'event': 'close'}, {'t_s': 1.45, 'event': 'open'}))
    recording = load_recording(csv_path, meta_path)
    with pytest.raises(RecordingRejected) as excinfo:
        splice_pour(recording, plan_segments(recording))
    assert 'inside the recorded pour' in str(excinfo.value)


def test_two_pours_are_refused(tmp_path):
    rows = _rows(pour_ops={14: 'Move_to_Surface'})
    csv_path, meta_path, _ = _write(tmp_path, rows)
    recording = load_recording(csv_path, meta_path)
    with pytest.raises(RecordingRejected) as excinfo:
        splice_pour(recording, plan_segments(recording))
    assert 'not one contiguous run' in str(excinfo.value)


def _real_recordings():
    """Every recording in config/replay/real, whatever it is called."""
    if not os.path.isdir(REAL):
        return []
    return sorted(os.path.join(REAL, name) for name in os.listdir(REAL) if name.endswith('.csv'))


@pytest.mark.parametrize('csv_path', _real_recordings() or [None])
def test_every_real_cell_recording_splices_cleanly(csv_path):
    if csv_path is None:
        pytest.skip('no real-cell recordings in this checkout')
    recording = load_recording(csv_path, csv_path[:-len('.csv')] + '.meta.json')
    spliced = splice_pour(recording, plan_segments(recording))
    assert [s.kind for s in spliced].count('pour') == 1
    pour = [s for s in spliced if s.kind == 'pour'][0]
    index = spliced.index(pour)
    assert spliced[index - 1].positions[-1] == spliced[index + 1].positions[0]
    # The reference really is a tilt, not the start again.
    assert max(abs(a - b) for a, b in zip(pour.reference, pour.start)) > 0.2


# --- the tree -------------------------------------------------------------

def test_without_a_pour_target_the_recorded_pour_is_replayed(tmp_path):
    names = _replay_names(_tree(tmp_path))
    assert not any('Pour' in name for name in names)
    assert any('pouring' in name for name in names)


def test_the_hand_over_sits_between_the_two_halves_of_the_move(tmp_path):
    tree = _tree(tmp_path, replay_pour_grams=20.0)
    names = _replay_names(tree)
    at = names.index('3_Read_Empty_Receiver')
    assert names[at - 1] == '2_Replay_Move_to_Surface'
    assert names[at:at + 7] == [
        '3_Read_Empty_Receiver',
        '3_Switch_To_pouring_controller',
        '3_Verify_pouring_controller_Active',
        '3_Pour_Result_Logged',
        '3_Verify_Back_At_Pour_Start',
        '3_Switch_Back_To_joint_trajectory_controller',
        '3_Verify_joint_trajectory_controller_Active_Again',
    ]
    assert names[at + 7] == '4_Replay_Place'
    assert tree.replay_summary['pour'] == {
        'target_grams': 20.0, 'container': CONTAINER_AUTO, 'material': 'liquid',
        'flow_index': 0.0, 'timeout': 0.0, 'marker_topic': DEFAULT_MARKER_TOPIC,
        'required': False}


def test_the_replay_resumes_only_from_where_the_pour_was_handed_over(tmp_path):
    tree = _tree(tmp_path, replay_pour_grams=20.0)
    check = [n for n in tree.iterate() if isinstance(n, JointStateCheckBehavior)][0]
    csv_path, meta_path, _ = _write(tmp_path, _rows())
    assert check.target == load_recording(csv_path, meta_path).positions[10]


def test_a_required_pour_that_fails_stops_the_replay(tmp_path):
    names = _replay_names(_tree(tmp_path, replay_pour_grams=20.0, replay_pour_required='true'))
    assert '3_Pour_20g' in names
    assert '3_Pour_Result_Logged' not in names
    with pytest.raises(ValueError):
        parse_pour_request({'replay_pour_grams': 5.0, 'replay_pour_required': 'maybe'})


def test_the_grasp_is_measured_once_the_jaws_have_settled_on_the_vessel(tmp_path):
    names = _replay_names(_tree(tmp_path, replay_pour_grams=20.0))
    # Right after the settle of the close that picks the vessel up, before the
    # arm moves on -- and nowhere else.
    at = names.index('1_Measure_Grasp_Or_At_Pour')
    assert names[at - 2:at] == ['1_Gripper_Close_Move_to_Surface', '1_Gripper_Settle']
    assert names[at + 1] == '2_Replay_Move_to_Surface'
    assert sum('Measure_Grasp' in name for name in names) == 1
    tree = _tree(tmp_path, replay_pour_grams=20.0)
    sampler = [n for n in tree.iterate() if isinstance(n, GraspMarkerSampleBehavior)][0]
    assert sampler.name == '1_Measure_Grasp'
    assert sampler.marker_topic == DEFAULT_MARKER_TOPIC
    assert sampler.required_frame == 'base_link'
    assert sampler.joint_names == ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    pour = [n for n in tree.iterate() if isinstance(n, PourActionBehavior)][0]
    assert (pour.grasp_joints_key, pour.grasp_marker_key) == (GRASP_JOINTS_KEY, GRASP_MARKER_KEY)
    # How to tip is SHOWN to the controller -- the recording's deepest tilt --
    # not a direction or a joint index: recordings differ in both.
    assert pour.pour_direction == 0
    assert pour.pour_reference_joints[5] == pytest.approx(-1.2 * (1.0 - 0.5 / 4.5))


def test_a_grasp_that_goes_unmeasured_does_not_stop_the_replay(tmp_path):
    # Measured on the real cell: the jaws turned the beaker as they closed and
    # its marker left side_2's frame. The pour goal then goes without a grasp
    # and the controller measures the marker at the pour start -- from side_1.
    tree = _tree(tmp_path, replay_pour_grams=20.0, replay_pour_required='true')
    sampler = [n for n in tree.iterate() if isinstance(n, GraspMarkerSampleBehavior)][0]
    assert isinstance(sampler.parent, py_trees.decorators.FailureIsSuccess)
    assert sampler.parent.name == '1_Measure_Grasp_Or_At_Pour'


def test_a_pour_that_turns_another_joint_is_shown_as_it_is(tmp_path):
    # The riser recordings pour by turning j4, not j6. Nothing here decides
    # which joint a pour may use; the reference carries it to the controller.
    rows = [([t, q[0], q[1], q[2], q[5], q[4], 0.0], op) for (t, *q), op in _rows()]
    tree = _tree(tmp_path, rows=rows, replay_pour_grams=20.0)
    pour = [n for n in tree.iterate() if isinstance(n, PourActionBehavior)][0]
    assert pour.pour_reference_joints[3] == pytest.approx(-1.2 * (1.0 - 0.5 / 4.5))
    assert pour.pour_reference_joints[5] == 0.0


def test_none_leaves_the_measuring_to_the_controller(tmp_path):
    tree = _tree(tmp_path, replay_pour_grams=20.0, replay_pour_marker_topic='none')
    assert not [n for n in tree.iterate() if isinstance(n, GraspMarkerSampleBehavior)]
    pour = [n for n in tree.iterate() if isinstance(n, PourActionBehavior)][0]
    assert pour.grasp_joints_key is None


def test_auto_reads_the_empty_receiver_and_hands_it_to_the_pour(tmp_path):
    tree = _tree(tmp_path, replay_pour_grams=20.0)
    latch = [n for n in tree.iterate() if isinstance(n, ScaleLatchBehavior)][0]
    pour = [n for n in tree.iterate() if isinstance(n, PourActionBehavior)][0]
    assert latch.record_as == CONTAINER_KEY
    assert pour.container_grams_key == CONTAINER_KEY


def test_a_given_container_weight_is_used_as_is(tmp_path):
    tree = _tree(tmp_path, replay_pour_grams=20.0, replay_pour_container='139.15')
    assert not [n for n in tree.iterate() if isinstance(n, ScaleLatchBehavior)]
    pour = [n for n in tree.iterate() if isinstance(n, PourActionBehavior)][0]
    assert pour.container_grams == pytest.approx(139.15)


def test_a_recording_that_cannot_hand_its_pour_over_refuses_the_whole_replay(tmp_path):
    with pytest.raises(RecordingRejected):
        _tree(tmp_path, rows=_rows(pour_end_j2=0.13), replay_pour_grams=20.0)


@pytest.mark.parametrize('config, fragment', [
    ({'replay_pour_grams': -5.0}, 'positive amount'),
    ({'replay_pour_grams': 5.0, 'replay_pour_container': 'flask'}, "EMPTY receiver"),
    ({'replay_pour_grams': 5.0, 'replay_pour_material': 'honey'}, 'material'),
    ({'replay_pour_grams': 5.0, 'replay_pour_flow_index': 3.0}, 'flow_index'),
])
def test_a_malformed_pour_request_is_refused_by_name(config, fragment):
    with pytest.raises(ValueError) as excinfo:
        parse_pour_request(config)
    assert fragment in str(excinfo.value)


def test_no_pour_target_means_no_pour():
    assert parse_pour_request({}) is None
    assert parse_pour_request({'replay_pour_grams': 0.0}) is None


# --- the scale latch --------------------------------------------------------

class _Time:
    def __init__(self, seconds):
        self.seconds = seconds
        self.nanoseconds = int(seconds * 1e9)

    def __add__(self, duration):
        return _Time(self.seconds + duration.nanoseconds / 1e9)

    def __sub__(self, other):
        return _Time(self.seconds - other.seconds)

    def __gt__(self, other):
        return self.seconds > other.seconds


class _Clock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return _Time(self.seconds)


def _reading(grams, stable=True):
    msg = MagicMock()
    msg.grams = grams
    msg.stable = stable
    return msg


@pytest.fixture
def latch():
    py_trees.blackboard.Blackboard.clear()
    behaviour = ScaleLatchBehavior('latch', record_as=CONTAINER_KEY, settle_sec=2.0,
                                   timeout_sec=6.0)
    behaviour.node = MagicMock()
    behaviour.clock = _Clock()
    behaviour.node.get_clock.return_value = behaviour.clock
    behaviour.initialise()
    yield behaviour
    py_trees.blackboard.Blackboard.clear()


def _feed(latch, grams, seconds, stable=True, dt=0.2):
    status = None
    for _ in range(int(round(seconds / dt))):
        latch.clock.seconds += dt
        latch._on_reading(_reading(grams, stable))
        status = latch.update()
        if status != py_trees.common.Status.RUNNING:
            break
    return status


def _latched():
    return py_trees.blackboard.Client(name='reader', namespace=TASK_NAMESPACE)


def test_the_latch_takes_a_reading_that_held_still_for_settle_sec(latch):
    assert _feed(latch, 139.15, 3.0) == py_trees.common.Status.SUCCESS
    reader = _latched()
    reader.register_key(CONTAINER_KEY, access=py_trees.common.Access.READ)
    assert getattr(reader, CONTAINER_KEY) == pytest.approx(139.15)


def test_the_latch_waits_out_a_pan_that_is_still_moving(latch):
    for grams in (139.0, 139.1, 139.2, 139.3, 139.4):
        latch.clock.seconds += 0.2
        latch._on_reading(_reading(grams))
        assert latch.update() == py_trees.common.Status.RUNNING
    # Unchanged but not flagged stable does not count either.
    assert _feed(latch, 139.4, 1.0, stable=False) == py_trees.common.Status.RUNNING


def test_the_latch_gives_up_and_says_why(latch):
    assert _feed(latch, 139.4, 7.0, stable=False) == py_trees.common.Status.FAILURE
    assert 'still moving' in latch.node.get_logger().error.call_args[0][0]


# --- the grasp, measured after the jaws close ----------------------------------

@pytest.fixture
def sampler():
    py_trees.blackboard.Blackboard.clear()
    behaviour = GraspMarkerSampleBehavior(
        'grasp', marker_topic=DEFAULT_MARKER_TOPIC, required_frame='base_link',
        joint_names=['j1', 'j2', 'j3', 'j4', 'j5', 'j6'], timeout_sec=5.0)
    behaviour.node = MagicMock()
    behaviour.clock = _Clock()
    behaviour.node.get_clock.return_value = behaviour.clock
    behaviour.initialise()
    yield behaviour
    py_trees.blackboard.Blackboard.clear()


def _marker(x, y, z, frame='base_link'):
    msg = MagicMock()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    return msg


def _joint_state(names, positions):
    msg = MagicMock()
    msg.name, msg.position = list(names), list(positions)
    return msg


def _read(key):
    reader = py_trees.blackboard.Client(name='reader-%s' % key, namespace=TASK_NAMESPACE)
    reader.register_key(key, access=py_trees.common.Access.READ)
    return getattr(reader, key)


def test_the_grasp_is_the_marker_and_the_joints_in_the_controllers_order(sampler):
    assert sampler.update() == py_trees.common.Status.RUNNING
    sampler._on_marker(_marker(0.58, -0.30, 0.12))
    assert sampler.update() == py_trees.common.Status.RUNNING   # no joints yet
    # joint_state_broadcaster does not promise an order; the controller does.
    sampler._on_joints(_joint_state(['j6', 'j1', 'j2', 'j3', 'j4', 'j5', 'gripper_finger_joint'],
                                    [0.6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.02]))
    assert sampler.update() == py_trees.common.Status.SUCCESS
    assert _read(GRASP_JOINTS_KEY) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert _read(GRASP_MARKER_KEY) == [0.58, -0.30, 0.12]


def test_a_marker_in_another_frame_is_refused(sampler):
    sampler._on_marker(_marker(0.1, 0.2, 0.3, frame='side_2_link'))
    sampler._on_joints(_joint_state(['j1', 'j2', 'j3', 'j4', 'j5', 'j6'], [0.0] * 6))
    assert sampler.update() == py_trees.common.Status.FAILURE


def test_no_marker_fails_and_names_the_topic(sampler):
    sampler._on_joints(_joint_state(['j1', 'j2', 'j3', 'j4', 'j5', 'j6'], [0.0] * 6))
    sampler.clock.seconds = 6.0
    assert sampler.update() == py_trees.common.Status.FAILURE
    assert DEFAULT_MARKER_TOPIC in sampler.node.get_logger().error.call_args[0][0]


def test_the_pour_goal_carries_the_grasp_the_sampler_left():
    py_trees.blackboard.Blackboard.clear()
    board = write_client('producer', [GRASP_JOINTS_KEY, GRASP_MARKER_KEY], TASK_NAMESPACE)
    setattr(board, GRASP_JOINTS_KEY, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    setattr(board, GRASP_MARKER_KEY, [0.58, -0.30, 0.12])
    pour = PourActionBehavior('pour', 'pouring_controller', target_grams=20.0,
                              container_grams=139.15, grasp_joints_key=GRASP_JOINTS_KEY,
                              grasp_marker_key=GRASP_MARKER_KEY)
    pour.node = MagicMock()
    pour.send_action_goal = MagicMock()
    pour.initialise()
    goal = pour.send_action_goal.call_args[0][0]
    assert goal.pour_direction == 0
    assert list(goal.pour_reference_joints) == []
    assert list(goal.grasp_joints) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert (goal.grasp_marker.x, goal.grasp_marker.y, goal.grasp_marker.z) == (0.58, -0.30, 0.12)
    py_trees.blackboard.Blackboard.clear()


def test_a_pour_with_no_grasp_measured_leaves_it_to_the_controller():
    py_trees.blackboard.Blackboard.clear()
    pour = PourActionBehavior('pour', 'pouring_controller', target_grams=20.0,
                              container_grams=139.15, grasp_joints_key=GRASP_JOINTS_KEY,
                              grasp_marker_key=GRASP_MARKER_KEY)
    pour.node = MagicMock()
    pour.send_action_goal = MagicMock()
    pour.initialise()
    assert list(pour.send_action_goal.call_args[0][0].grasp_joints) == []
    py_trees.blackboard.Blackboard.clear()


# --- the arm is back before the replay resumes ---------------------------------

def _check(target=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)):
    behaviour = JointStateCheckBehavior('back', ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'], list(target))
    behaviour.node = MagicMock()
    behaviour.clock = _Clock()
    behaviour.node.get_clock.return_value = behaviour.clock
    behaviour.initialise()
    return behaviour


def test_the_resume_waits_for_the_arm_to_be_back():
    check = _check()
    check._on_joints(_joint_state(['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
                                  [0.1, 0.2, 0.3, 0.4, 0.5, 0.605]))
    assert check.update() == py_trees.common.Status.SUCCESS


def test_an_arm_left_elsewhere_stops_the_resume_and_names_the_joint():
    check = _check()
    check._on_joints(_joint_state(['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
                                  [0.1, 0.2, 0.3, 0.4, 0.5, 1.6]))
    assert check.update() == py_trees.common.Status.RUNNING
    check.clock.seconds = 4.0
    assert check.update() == py_trees.common.Status.FAILURE
    assert 'j6 is 1.0000 rad' in check.node.get_logger().error.call_args[0][0]
