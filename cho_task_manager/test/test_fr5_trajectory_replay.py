"""What must be true before a recorded trajectory is replayed at this arm.

A replay is position control that senses nothing, so the checks here are not
hygiene -- each one stands between a recording and an arm that will go where the
recording says whether or not the cell agrees:

* a cell laid out differently REFUSES the replay rather than warning,
* the gripper is commanded at the recorded events, from the meta, because the
  CSV has no gripper column at all,
* the recording is cut at those events and NOTHING else, so the replayed
  motion is the recorded one,
* the recorded waypoints are replayed unchanged; only the clock may be stretched.

Nothing here needs a ROS graph: the trees are built, not ticked, and the
recording layer is deliberately ROS-free.
"""

import json
import os

import pytest

from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.fr5 import trajectory_replay
from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    load_robot_config,
)
from cho_task_manager.utils.trajectory_recording import (
    LayoutMismatch,
    RecordingRejected,
    compare_layout,
    load_recording,
    plan_segments,
    required_time_scale,
)

HEADER = 't_s,j1,j2,j3,j4,j5,j6,operation\n'

#: A layout the fixtures below assume, and the cell file that matches it.
ASSUMED = {
    'beaker': {'xy': [0.48, 0.13], 'yaw_deg': -162.4},
    'flask': {'xy': [0.53, 0.35], 'yaw_deg': -95.6},
}


def _config():
    return load_robot_config('fr5')


def _write(tmp_path, rows, meta=None, name='trial'):
    """Write a CSV/meta pair the way the exporter names them."""
    csv_path = tmp_path / ('%s_waypoints.csv' % name)
    meta_path = tmp_path / ('%s_meta.json' % name)
    csv_path.write_text(HEADER + ''.join(
        '%s,%s\n' % (','.join('%.6f' % v for v in row), op) for row, op in rows))

    base = {
        'seed': '5',
        'tool': 'fr5_ag95',
        'joint_names': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
        'waypoints': len(rows),
        'home_arm_rad': list(rows[0][0][1:]),
        'gripper_events': [],
        'layout_the_trajectory_assumes': ASSUMED,
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
        if 'yaw_deg' in entry:
            lines.append('    yaw_deg: %r' % entry['yaw_deg'])
    path.write_text('\n'.join(lines) + '\n')
    return str(path)


def _row(t, op='Pick', j2=0.0):
    return ([t, 0.0, j2, 0.0, 0.0, 0.0, 0.0], op)


def _ramp(count, op='Pick', step=0.0, start=0.0, dt=0.1):
    return [_row(start + i * dt, op, j2=i * step) for i in range(count)]


# --- registration ----------------------------------------------------------

def test_the_task_is_registered_for_fr5_only():
    assert 'trajectory_replay' in available_tasks('fr5')
    for other in ('franka', 'ur5e', 'openarm'):
        assert 'trajectory_replay' not in available_tasks(other)


def test_the_replay_controller_comes_from_the_registry():
    assert trajectory_replay.replay_controller(_config()) == 'joint_trajectory_controller'


def test_the_replay_controller_can_be_switched_back_out():
    # If it is not in the exclusive set, switching the hold controller back in
    # never deactivates it, both claim the same command interfaces, and the
    # switch is rejected -- leaving the arm under the replay controller.
    config = _config()
    assert trajectory_replay.replay_controller(config) in exclusive_arm_controllers(config)


# --- the layout gate -------------------------------------------------------

def test_a_cell_that_matches_passes():
    assert compare_layout(ASSUMED, ASSUMED) == []


def test_a_displaced_object_is_reported_with_the_distance():
    moved = {'beaker': {'xy': [0.48, 0.20]}, 'flask': {'xy': [0.53, 0.35]}}
    problems = compare_layout(ASSUMED, moved)
    assert len(problems) == 1
    assert 'beaker' in problems[0]
    assert '70.0 mm' in problems[0]


def test_an_object_the_cell_does_not_declare_is_a_problem():
    # The arm still goes there, so "not declared" is not "not in the way".
    problems = compare_layout(ASSUMED, {'beaker': {'xy': [0.48, 0.13]}})
    assert len(problems) == 1
    assert 'flask' in problems[0]
    assert 'declares none' in problems[0]


def test_an_object_with_no_declared_yaw_is_not_yaw_checked():
    # A round vessel has no yaw to match; checking it would refuse a correct cell.
    round_cell = {'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}}
    assert compare_layout(ASSUMED, round_cell) == []


def test_a_rotated_object_that_declares_yaw_is_refused():
    turned = {
        'beaker': {'xy': [0.48, 0.13], 'yaw_deg': -100.0},
        'flask': {'xy': [0.53, 0.35]},
    }
    problems = compare_layout(ASSUMED, turned)
    assert len(problems) == 1
    assert 'deg from the yaw' in problems[0]


def test_building_the_tree_refuses_a_mismatched_cell(tmp_path):
    """The gate is a refusal, not a warning, and it happens before anything moves."""
    csv_path, meta_path = _write(tmp_path, _ramp(20))
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.30]},      # 170 mm away
        'flask': {'xy': [0.53, 0.35]},
    })
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout})

    with pytest.raises(LayoutMismatch) as caught:
        build_task_tree('trajectory_replay', config)
    assert 'beaker' in str(caught.value)
    # A ValueError, so task_manager_node reports it and exits instead of ticking.
    assert isinstance(caught.value, ValueError)


def test_building_the_tree_accepts_a_matching_cell(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(20))
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}})
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout})

    tree = build_task_tree('trajectory_replay', config)
    names = {node.name for node in tree.iterate()}
    assert '1_Initialize' in names
    assert '2_Handover' in names
    assert '3_Replay' in names
    assert '4_Finish' in names
    # The abort branch is what parks the arm when a segment fails.
    assert any('Abort' in name for name in names)


def test_a_replay_without_its_paths_says_which_one(tmp_path):
    with pytest.raises(ValueError, match='replay_trajectory'):
        build_task_tree('trajectory_replay', _config())


# --- gripper events --------------------------------------------------------

def test_the_gripper_is_commanded_at_the_recorded_event(tmp_path):
    """The CSV has no gripper column: the event time comes from the meta."""
    csv_path, meta_path = _write(tmp_path, _ramp(10), {
        'gripper_events': [{'t_s': 0.45, 'event': 'close'}]})
    segments = plan_segments(load_recording(csv_path, meta_path))

    kinds = [segment.kind for segment in segments]
    assert kinds == ['move', 'gripper', 'move']
    assert segments[1].grasp is True
    # The cut falls between the waypoints either side of the event, so the
    # approximate event time stops mattering: what the recording says is "the
    # gripper closed between these two waypoints".
    assert len(segments[0].times) == 5          # t = 0.0 .. 0.4
    assert len(segments[2].times) == 5          # t = 0.5 .. 0.9
    assert segments[1].source_t0 == pytest.approx(0.45)


def test_both_events_of_a_pick_and_place_come_out_in_time_order(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(10), {
        # Deliberately out of order in the file.
        'gripper_events': [{'t_s': 0.75, 'event': 'open'},
                           {'t_s': 0.25, 'event': 'close'}]})
    segments = plan_segments(load_recording(csv_path, meta_path))

    assert [s.kind for s in segments] == [
        'move', 'gripper', 'move', 'gripper', 'move']
    assert segments[1].grasp is True
    assert segments[3].grasp is False


def test_the_tree_puts_a_gripper_behaviour_at_the_event(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(10), {
        'gripper_events': [{'t_s': 0.45, 'event': 'close'}]})
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}})
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout})

    tree = build_task_tree('trajectory_replay', config)
    replay = [node for node in tree.iterate() if node.name == '3_Replay'][0]
    kinds = [child.name for child in replay.children]
    assert len(kinds) == 3
    assert 'Gripper_Close' in kinds[1]


def test_an_unknown_gripper_event_is_refused(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(4), {
        'gripper_events': [{'t_s': 0.1, 'event': 'wiggle'}]})
    with pytest.raises(RecordingRejected, match='neither'):
        plan_segments(load_recording(csv_path, meta_path))


# --- operation boundaries --------------------------------------------------

def test_every_move_segment_restarts_its_clock(tmp_path):
    """A trajectory controller times a goal from when it accepts it."""
    rows = _ramp(5, 'Pick') + _ramp(5, 'pouring', start=0.5)
    csv_path, meta_path = _write(tmp_path, rows, {
        'gripper_events': [{'t_s': 0.45, 'event': 'close'}]})

    second = plan_segments(load_recording(csv_path, meta_path))[2]

    assert second.times[0] == pytest.approx(0.0)
    assert second.source_t0 == pytest.approx(0.5)


def test_operation_labels_are_not_cut_on_but_are_carried_through(tmp_path):
    """Cutting on them was measured to change nothing; the labels still inform.

    On the transfer recording the arm is already at a full stop at every
    operation boundary, so cutting there produced identical motion in more
    steps. The labels are kept because a segment that says what it spans is the
    one useful thing they give a log line.
    """
    rows = (_ramp(5, 'Pick') + _ramp(5, 'Move_to_Surface', start=0.5)
            + _ramp(5, 'pouring', start=1.0) + _ramp(5, 'Place', start=1.5))
    csv_path, meta_path = _write(tmp_path, rows)

    segments = plan_segments(load_recording(csv_path, meta_path))

    assert len(segments) == 1
    assert len(segments[0].times) == 20
    assert segments[0].operations == (
        'Pick', 'Move_to_Surface', 'pouring', 'Place')
    assert segments[0].operation == 'Pick+Move_to_Surface+pouring+Place'


def test_a_blank_label_is_skipped_rather_than_recorded(tmp_path):
    rows = [_row(0.0, 'Pick'), _row(0.1, ''), _row(0.2, 'Pick'), _row(0.3, 'Pick')]
    csv_path, meta_path = _write(tmp_path, rows)

    segments = plan_segments(load_recording(csv_path, meta_path))

    assert len(segments) == 1
    assert len(segments[0].times) == 4
    assert segments[0].operations == ('Pick',)


def test_a_recording_inside_the_ceiling_is_not_stretched(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(10, step=0.001))
    recording = load_recording(csv_path, meta_path)

    scale, joint, _, _ = required_time_scale(recording, {'j2': 1.575}, scaling=0.25)

    assert scale == pytest.approx(1.0)
    assert joint is None


def test_a_recording_over_the_ceiling_is_stretched_and_says_which_joint(tmp_path):
    # 0.1 rad per 0.1 s = 1.0 rad/s, against a 1.575 * 0.25 = 0.394 rad/s ceiling.
    csv_path, meta_path = _write(tmp_path, _ramp(10, step=0.1))
    recording = load_recording(csv_path, meta_path)

    scale, joint, rate, ceiling = required_time_scale(
        recording, {'j2': 1.575}, scaling=0.25)

    assert joint == 'j2'
    assert rate == pytest.approx(1.0, abs=1e-6)
    assert ceiling == pytest.approx(0.39375)
    assert scale > 2.5
    # Strictly over, so the stretched trajectory lands inside the ceiling rather
    # than on it: point times are quantised to nanoseconds.
    assert scale > rate / ceiling


def test_the_speed_scale_is_refused_above_one(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(10))
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}})
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout, 'replay_speed_scale': 2.0})

    with pytest.raises(ValueError, match='replay_speed_scale'):
        build_task_tree('trajectory_replay', config)


def test_the_default_speed_scale_is_conservative():
    assert trajectory_replay.DEFAULT_SPEED_SCALE <= 0.25


def test_waypoints_are_replayed_exactly_as_recorded(tmp_path):
    """Only the clock may change: no re-planning, no re-interpolation."""
    rows = _ramp(6, step=0.01)
    csv_path, meta_path = _write(tmp_path, rows)

    segment = plan_segments(load_recording(csv_path, meta_path))[0]

    assert len(segment.positions) == len(rows)
    for recorded, replayed in zip(rows, segment.positions):
        assert replayed == pytest.approx(list(recorded[0][1:]))


# --- the recording itself --------------------------------------------------

def test_a_duplicate_timestamp_sample_is_dropped(tmp_path):
    # A trajectory controller rejects points whose times do not strictly
    # increase, so the stale half of a same-instant pair has to go.
    rows = [_row(0.0), _row(0.04), _row(0.0401), _row(0.08)]
    csv_path, meta_path = _write(tmp_path, rows, {'waypoints': 4})

    recording = load_recording(csv_path, meta_path)

    assert len(recording.times) == 3
    assert recording.dropped == [(1, pytest.approx(0.04))]
    assert recording.times[1] == pytest.approx(0.0401)


def test_a_recording_in_other_joints_is_refused(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(4), {
        'joint_names': ['a', 'b', 'c', 'd', 'e', 'f']})
    with pytest.raises(RecordingRejected, match='but this arm is'):
        load_recording(csv_path, meta_path, joint_names=['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])


def test_a_recording_without_its_meta_is_refused(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(4))
    os.unlink(meta_path)
    with pytest.raises(RecordingRejected, match='no meta JSON'):
        load_recording(csv_path)


def test_the_home_pose_comes_from_the_meta(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(4), {
        'home_arm_rad': [0.1, -0.9, -2.1, -1.6, 1.4, -0.04]})
    recording = load_recording(csv_path, meta_path)
    assert recording.home == pytest.approx([0.1, -0.9, -2.1, -1.6, 1.4, -0.04])


# --- how the arm gets to the start pose ------------------------------------

def _replay_config(tmp_path, **extra):
    csv_path, meta_path = _write(tmp_path, _ramp(20))
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}})
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout})
    config.update(extra)
    return config


def test_the_default_home_is_direct_and_needs_no_moveit():
    # MuJoCo has no collisions to check (every FR5 geom is contype 0), so
    # starting move_group for it would be a dependency paid for nothing.
    assert trajectory_replay.DEFAULT_HOME_VIA == 'direct'


def test_a_direct_home_uses_the_hold_controllers_own_action(tmp_path):
    tree = build_task_tree('trajectory_replay', _replay_config(tmp_path))
    names = {node.name for node in tree.iterate()}

    assert 'Go_Home' in names
    assert 'Go_Home_MoveIt' not in names
    # The direct home runs on the hold controller, so the arm has to change
    # hands before the replay.
    assert 'Switch_To_joint_trajectory_controller' in names


def test_a_moveit_home_plans_through_the_bridge(tmp_path):
    tree = build_task_tree(
        'trajectory_replay', _replay_config(tmp_path, home_via='moveit'))
    names = {node.name for node in tree.iterate()}

    assert 'Go_Home_MoveIt' in names
    assert 'Go_Home' not in names
    # It goes to the bridge's endpoint, not to a controller's own action server.
    home = [n for n in tree.iterate() if n.name == 'Go_Home_MoveIt'][0]
    assert home.action_name == '/fr5/controller_action_server/moveit_joint'


def test_a_moveit_home_does_one_switch_fewer(tmp_path):
    """Planning the home runs on the replay's own controller, so the arm stays put.

    The direct path homes on the hold controller and must then hand the arm
    over; the MoveIt path hands it over once and keeps it.
    """
    direct = build_task_tree('trajectory_replay', _replay_config(tmp_path))
    moveit = build_task_tree(
        'trajectory_replay', _replay_config(tmp_path, home_via='moveit'))

    def switches(tree):
        return [n.name for n in tree.iterate() if n.name.startswith('Switch_To_')]

    assert len(switches(moveit)) < len(switches(direct))


def test_a_moveit_home_still_parks_the_arm_on_the_hold_controller(tmp_path):
    # Ending a session on the controller an external executor was driving is
    # how an arm is left unattended under a live goal.
    tree = build_task_tree(
        'trajectory_replay', _replay_config(tmp_path, home_via='moveit'))
    names = {node.name for node in tree.iterate()}
    assert 'Park_On_joint_space_position_controller_Final' in names


def test_the_replay_controller_is_verified_active_either_way(tmp_path):
    for home_via in ('direct', 'moveit'):
        tree = build_task_tree(
            'trajectory_replay', _replay_config(tmp_path, home_via=home_via))
        names = {node.name for node in tree.iterate()}
        assert 'Verify_Replay_Controller_Active' in names, home_via


def test_an_unknown_home_via_is_refused(tmp_path):
    with pytest.raises(ValueError, match='home_via'):
        build_task_tree(
            'trajectory_replay', _replay_config(tmp_path, home_via='teleport'))


def test_the_moveit_action_name_comes_from_the_registry():
    # The registry VALIDATES that the first joint preference is exactly the
    # bridge's endpoint, so reading it there cannot drift from what is served.
    from cho_task_manager.utils.controller_names import moveit_joint_action_name
    assert moveit_joint_action_name(_config()) == \
        '/fr5/controller_action_server/moveit_joint'


# --- where the arm is left --------------------------------------------------

def _home_targets(tree):
    """{behaviour name: target joint positions} for every home behaviour."""
    return {node.name: list(node.target_joints.position)
            for node in tree.iterate()
            if node.name.startswith('Go_Home')}


def test_it_starts_at_the_recordings_pose_and_ends_at_the_robots(tmp_path):
    """The two homes are different poses, and deliberately so.

    Going to the recording's start is what makes the first replayed segment not
    a lunge. Coming back to it would leave the arm wherever one trial happened
    to begin -- often low over the bench -- instead of the pose the arm is meant
    to be left in and the next task will assume.
    """
    csv_path, meta_path = _write(tmp_path, _ramp(20), {
        'home_arm_rad': [0.106, -0.9425, -2.1747, -1.6448, 1.4143, -0.0407]})
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}})
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout})

    targets = _home_targets(build_task_tree('trajectory_replay', config))
    ready = list(trajectory_replay.ready_pose(config).position)

    assert targets['Go_Home'] == pytest.approx(
        [0.106, -0.9425, -2.1747, -1.6448, 1.4143, -0.0407])
    assert targets['Go_Home_Final'] == pytest.approx(ready)
    assert targets['Go_Home'] != pytest.approx(targets['Go_Home_Final'])


def test_the_moveit_path_ends_at_the_robots_pose_too(tmp_path):
    csv_path, meta_path = _write(tmp_path, _ramp(20), {
        'home_arm_rad': [0.106, -0.9425, -2.1747, -1.6448, 1.4143, -0.0407]})
    layout = _layout_file(tmp_path, {
        'beaker': {'xy': [0.48, 0.13]}, 'flask': {'xy': [0.53, 0.35]}})
    config = _config()
    config.update({'replay_trajectory': csv_path, 'replay_meta': meta_path,
                   'replay_layout': layout, 'home_via': 'moveit'})

    targets = _home_targets(build_task_tree('trajectory_replay', config))
    ready = list(trajectory_replay.ready_pose(config).position)

    assert targets['Go_Home_MoveIt_Final'] == pytest.approx(ready)


def test_the_ready_pose_is_the_registry_pose_not_zero():
    # home '0' is all-zero and recorded as diagnostic-only: it puts the wrist at
    # the floor and j5 = 0 is a wrist singularity.
    ready = trajectory_replay.ready_pose(_config())
    assert len(ready.position) == 6
    assert any(abs(value) > 1e-9 for value in ready.position)
