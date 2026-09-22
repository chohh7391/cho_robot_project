"""When a sweep is worth doing, and where it is allowed to go.

No ROS anywhere in here, which is the point: these are the rules that decide
whether a robot moves, and a rule like that deserves a test that runs in a
second rather than one that needs a bench.
"""

import os

from ament_index_python.packages import get_package_share_directory
from cho_task_manager.utils import occlusion
import pytest
import yaml

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']


def _camera(name='side_1', state='ok', detail='', age_sec=0.1, priority=0,
            decision_margin=70.0, edge_px=45.0):
    return occlusion.CameraView(name, state, detail, age_sec, priority,
                                decision_margin, edge_px)


def _view(name='beaker', publishing=False, override_camera='', status='',
          cameras=()):
    return occlusion.ObjectView(name, publishing, override_camera, status,
                                tuple(cameras) or (_camera(),))


# ----------------------------------------------------------- when to sweep

def test_an_object_already_being_published_needs_no_sweep():
    # The common case, and the reason this leaf is cheap to put in front of
    # every detection: no motion, one snapshot, done.
    assessment = occlusion.assess(
        _view(publishing=True, status='publishing, spread 1.2 mm'), 'wrist')
    assert assessment.action == occlusion.SATISFIED


def test_a_published_but_poor_pose_is_also_worth_going_to_look_at():
    # THE SECOND TRIGGER. A standing camera watching a bench from a metre away
    # sees every tag obliquely: the pose is present and not accurate. If "a
    # pose exists" were good enough the recovery would never fire for it, and
    # the case would be unrecoverable by construction.
    view = _view(publishing=True, cameras=[
        _camera('side_1', 'ok', decision_margin=38.0),
        _camera('wrist', 'not_in_frame', priority=10)])
    assert occlusion.assess(view, 'wrist').action == occlusion.SATISFIED
    poor = occlusion.assess(view, 'wrist', min_decision_margin=55.0)
    assert poor.action == occlusion.SWEEP
    assert 'margin 38' in poor.reason and 'is being published' in poor.reason


def test_a_published_and_good_pose_is_left_alone():
    view = _view(publishing=True, cameras=[
        _camera('side_1', 'ok', decision_margin=64.0),
        _camera('wrist', 'not_in_frame', priority=10)])
    assert occlusion.assess(
        view, 'wrist', min_decision_margin=55.0).action == occlusion.SATISFIED


def test_only_contributing_cameras_count_toward_the_best_decode():
    # A suppressed camera's sample was thrown away and a stale one's is out of
    # the window: neither is holding up the pose being published, so neither
    # should be able to satisfy a requirement on it.
    view = _view(publishing=True, cameras=[
        _camera('side_1', 'suppressed', 'wrist', decision_margin=90.0),
        _camera('rs', 'stale', 'ok', decision_margin=95.0),
        _camera('wrist', 'ok', priority=10, decision_margin=40.0)])
    assert occlusion.best_decode(view) == 40.0
    assert occlusion.assess(
        view, 'wrist', min_decision_margin=55.0).action == occlusion.SWEEP


def test_a_bench_with_nothing_to_score_reports_no_best_decode():
    assert occlusion.best_decode(_view(cameras=[
        _camera('side_1', 'not_in_frame',
                decision_margin=occlusion.NO_SCORE)])) == occlusion.NO_SCORE


def test_a_tag_no_camera_can_see_is_worth_going_to_look_at():
    # This is what occlusion looks like from the perception side_1, and it is
    # also what a tag outside the field of view looks like. Neither side_1 can
    # tell them apart, and going to look is the right answer to both.
    assessment = occlusion.assess(_view(cameras=[
        _camera('side_1', 'not_in_frame'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')
    assert assessment.action == occlusion.SWEEP


def test_a_tag_that_decodes_badly_is_also_worth_a_closer_look():
    # Too far, too oblique, motion-blurred: exactly what a close view fixes.
    assessment = occlusion.assess(_view(cameras=[
        _camera('side_1', 'rejected', 'decision margin 18.0 < 35.0'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')
    assert assessment.action == occlusion.SWEEP


def test_a_sweep_camera_with_no_tf_is_refused_rather_than_driven():
    # Moving the camera moves the same broken chain somewhere else. For a
    # wrist camera this is usually the robot's TF not being up -- and driving
    # the arm is a peculiar response to that.
    assessment = occlusion.assess(_view(cameras=[
        _camera('side_1', 'not_in_frame'),
        _camera('wrist', 'no_tf', 'base_link <- wrist_tag_0: no transform',
                priority=10)]), 'wrist')
    assert assessment.action == occlusion.REFUSE
    assert 'wrist' in assessment.reason


def test_another_cameras_broken_tf_does_not_refuse_the_sweep():
    # `side_1`'s extrinsic being wrong is a real problem and not this one's:
    # the wrist has its own chain and can still do the looking.
    assessment = occlusion.assess(_view(cameras=[
        _camera('side_1', 'no_tf', 'base_link <- side_tag_0: no transform'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')
    assert assessment.action == occlusion.SWEEP


def test_a_bench_where_nothing_is_running_is_refused():
    # Every camera quiet means the detectors did not start, or are publishing
    # where the pose node is not listening. An arm that sweeps because a node
    # failed to launch is answering the wrong question.
    assessment = occlusion.assess(_view(cameras=[
        _camera('side_1', 'stale', 'last said: ok', age_sec=42.0),
        _camera('wrist', 'unknown', age_sec=-1.0, priority=10)]), 'wrist')
    assert assessment.action == occlusion.REFUSE
    assert 'detectors' in assessment.reason


def test_naming_a_camera_the_pose_node_does_not_have_is_refused():
    # The sweep table and the camera table are separate files. This is the
    # failure that produces: a sweep that drives four waypoints and then waits
    # for a camera that was never in the fusion.
    assessment = occlusion.assess(
        _view(cameras=[_camera('side_1', 'not_in_frame')]), 'wrist')
    assert assessment.action == occlusion.REFUSE
    assert 'cameras.yaml' in assessment.reason


# --------------------------------------------------------- when it worked

def test_a_good_decode_is_required_when_one_is_asked_for():
    # The point of the raster. cho_object_pose publishes from margin 35, and
    # the far view that prompted the recovery already clears it -- so stopping
    # at the first rung that merely publishes trades one marginal measurement
    # for another. The requirement is what makes the sweep keep descending.
    cameras = [_camera('side_1', 'suppressed', 'wrist'),
               _camera('wrist', 'ok', priority=10, decision_margin=42.0)]
    view = _view(publishing=True, override_camera='wrist', cameras=cameras)
    assert not occlusion.recovered(view, 'wrist', min_decision_margin=55.0)
    # ...and it stops descending once it gets one.
    better = [_camera('side_1', 'suppressed', 'wrist'),
              _camera('wrist', 'ok', priority=10, decision_margin=64.0)]
    assert occlusion.recovered(
        _view(publishing=True, override_camera='wrist', cameras=better),
        'wrist', min_decision_margin=55.0)


def test_asking_for_no_particular_score_accepts_whatever_publishes():
    # The old behaviour, and right for a bench whose only problem is line of
    # sight rather than range.
    cameras = [_camera('wrist', 'ok', priority=10, decision_margin=36.0)]
    assert occlusion.recovered(_view(publishing=True, cameras=cameras), 'wrist')


def test_a_camera_that_reports_no_score_cannot_clear_a_threshold():
    # A publisher predating the score field reports -1. Passing it silently
    # would turn a quality requirement into no requirement at all.
    cameras = [_camera('wrist', 'ok', priority=10,
                       decision_margin=occlusion.NO_SCORE)]
    view = _view(publishing=True, cameras=cameras)
    assert not occlusion.recovered(view, 'wrist', min_decision_margin=55.0)
    assert occlusion.recovered(view, 'wrist')


def test_recovery_needs_both_a_pose_and_the_sweep_camera_behind_it():
    cameras = [_camera('side_1', 'suppressed', 'wrist'),
               _camera('wrist', 'ok', priority=10)]
    assert occlusion.recovered(
        _view(publishing=True, override_camera='wrist', cameras=cameras), 'wrist')


def test_the_standing_camera_getting_its_view_back_is_not_a_recovery():
    # A fine outcome for the task, but the sweep did not cause it -- and a leaf
    # that took credit would hide one whose waypoints look nowhere useful.
    assert not occlusion.recovered(_view(publishing=True, cameras=[
        _camera('side_1', 'ok'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')


def test_the_sweep_camera_seeing_the_tag_is_not_yet_a_pose():
    # 'ok' means a sample went into the window; the node still has min_samples
    # and the spread gate to satisfy before anything is published.
    assert not occlusion.recovered(_view(publishing=False, cameras=[
        _camera('wrist', 'ok', priority=10)]), 'wrist')


def test_describe_cameras_names_every_camera_its_reason_and_its_score():
    text = occlusion.describe_cameras(_view(cameras=[
        _camera('side_1', 'rejected', 'too oblique', decision_margin=21.0, edge_px=18.0),
        _camera('wrist', 'not_in_frame', priority=10,
                decision_margin=occlusion.NO_SCORE)]))
    assert 'side_1: rejected (too oblique) [margin 21, edge 18px]' in text
    # Nothing to score reads as nothing, not as a margin of -1.
    assert 'wrist: not_in_frame' in text
    assert '-1' not in text


# --------------------------------------------------------- the sweep table

def _table(**overrides):
    entry = {'object': 'beaker',
             'recovery_camera': 'wrist',
             'waypoints': [{'name': 'over', 'joints': [0.0] * 6}]}
    entry.update(overrides)
    return {'sweeps': [entry]}


def test_a_sweep_is_parsed_with_its_waypoints_and_defaults():
    sweeps = occlusion.parse_sweeps(_table(), joint_names=JOINTS)
    sweep = sweeps['beaker']
    assert sweep.recovery_camera == 'wrist'
    assert [point.name for point in sweep.waypoints] == ['over']
    assert sweep.dwell_sec == occlusion.DEFAULT_DWELL_SEC
    assert sweep.waypoint_duration == occlusion.DEFAULT_WAYPOINT_DURATION
    assert sweep.min_decision_margin == occlusion.DEFAULT_MIN_DECISION_MARGIN
    assert sweep.waypoints[0].duration == occlusion.DEFAULT_WAYPOINT_DURATION


def test_a_waypoint_may_take_longer_than_the_rest():
    # The first waypoint of a sweep is a long swing from wherever the arm was and
    # the rest are small descents. One duration for both would either break the
    # cell's rate ceiling or make every descent crawl.
    sweeps = occlusion.parse_sweeps(_table(waypoints=[
        {'name': 'survey', 'joints': [0.0] * 6, 'duration': 12.0},
        {'name': 'close', 'joints': [0.1] * 6}]), joint_names=JOINTS)
    assert [point.duration for point in sweeps['beaker'].waypoints] == [
        12.0, occlusion.DEFAULT_WAYPOINT_DURATION]


def test_the_score_requirement_may_be_switched_off_but_not_made_negative():
    assert occlusion.parse_sweeps(
        _table(min_decision_margin=0.0), joint_names=JOINTS
    )['beaker'].min_decision_margin == 0.0
    with pytest.raises(ValueError, match='min_decision_margin'):
        occlusion.parse_sweeps(_table(min_decision_margin=-1.0), joint_names=JOINTS)


def test_defaults_apply_and_an_entry_may_override_them():
    document = _table(dwell_sec=0.5)
    document['defaults'] = {'recovery_camera': 'wrist', 'dwell_sec': 3.0,
                            'timeout_sec': 30.0}
    document['sweeps'].append({'object': 'flask',
                               'waypoints': [{'name': 'over', 'joints': [0.1] * 6}]})
    sweeps = occlusion.parse_sweeps(document, joint_names=JOINTS)
    assert sweeps['beaker'].dwell_sec == 0.5
    assert sweeps['flask'].dwell_sec == 3.0
    assert sweeps['flask'].recovery_camera == 'wrist'
    assert sweeps['flask'].timeout_sec == 30.0


def test_a_misspelled_default_is_rejected_rather_than_ignored():
    # A silently ignored `dwell` would leave the sweep judging viewpoints
    # before the arm had settled, and nothing would say so.
    document = _table()
    document['defaults'] = {'dwell': 3.0}
    with pytest.raises(ValueError, match='no such setting'):
        occlusion.parse_sweeps(document, joint_names=JOINTS)


def test_the_wrong_number_of_joints_is_a_build_time_error():
    # The one error this can see. A goal is filled in by POSITION in a list, so
    # a table written for a 7-axis arm would drive six joints to the wrong
    # angles and the seventh value would vanish.
    with pytest.raises(ValueError, match='has 7 values'):
        occlusion.parse_sweeps(
            _table(waypoints=[{'name': 'over', 'joints': [0.0] * 7}]),
            joint_names=JOINTS)


def test_a_sweep_must_look_somewhere():
    with pytest.raises(ValueError, match='waypoints'):
        occlusion.parse_sweeps(_table(waypoints=[]), joint_names=JOINTS)


def test_joint_values_must_be_finite_numbers():
    with pytest.raises(ValueError, match='finite'):
        occlusion.parse_sweeps(
            _table(waypoints=[{'name': 'over', 'joints': [0.0, float('nan'),
                                                          0.0, 0.0, 0.0, 0.0]}]),
            joint_names=JOINTS)


def test_a_recovery_camera_is_required():
    document = _table()
    del document['sweeps'][0]['recovery_camera']
    with pytest.raises(ValueError, match='recovery_camera'):
        occlusion.parse_sweeps(document, joint_names=JOINTS)


def test_one_object_may_not_have_two_sweeps():
    document = _table()
    document['sweeps'].append(dict(document['sweeps'][0]))
    with pytest.raises(ValueError, match='already has a sweep'):
        occlusion.parse_sweeps(document, joint_names=JOINTS)


def test_durations_must_be_positive_but_a_dwell_may_be_zero():
    with pytest.raises(ValueError, match='waypoint_duration'):
        occlusion.parse_sweeps(_table(waypoint_duration=0.0), joint_names=JOINTS)
    with pytest.raises(ValueError, match='timeout_sec'):
        occlusion.parse_sweeps(_table(timeout_sec=-1.0), joint_names=JOINTS)
    assert occlusion.parse_sweeps(
        _table(dwell_sec=0.0), joint_names=JOINTS)['beaker'].dwell_sec == 0.0


# ------------------------------------------------------ the shipped table

def _shipped():
    try:
        path = os.path.join(get_package_share_directory('cho_task_manager'),
                            'config', 'sweep', 'fr5_bench.yaml')
        if os.path.exists(path):
            return path
    except LookupError:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'sweep', 'fr5_bench.yaml')


def test_the_fr5_bench_table_parses_against_the_fr5s_own_joints():
    with open(_shipped(), encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    assert set(sweeps) == {'beaker', 'flask'}
    for sweep in sweeps.values():
        names = [point.name for point in sweep.waypoints]
        # A raster, not a single viewpoint and not a straight descent. The
        # motion is there for PARALLAX -- a different line of sight to the same
        # tag -- which is the only thing that helps when something is standing
        # in the way; a column of viewpoints over one spot samples one line of
        # sight repeatedly.
        rows = {name.rsplit('c', 1)[0] for name in names if name.startswith('survey')}
        assert len(rows) > 1, names
        columns = {name for name in names if name.startswith('survey_r0')}
        assert len(columns) > 1, names
        # Search pass first, resolution pass after: high sees a wide area and
        # is what FINDS the tag; low is for pixels.
        assert names[0].startswith('survey')
        assert names[-1] == 'close'
        # Asking for a decode better than cho_object_pose's own 35 gate is what
        # makes the second pass mean anything.
        assert sweep.min_decision_margin > 35.0
        # The first waypoint is a long swing from the home pose, not a raster
        # step, and this cell has a rate ceiling.
        assert sweep.waypoints[0].duration > sweep.waypoints[-1].duration


def _survey_rows(sweep):
    """{row: [column, ...] in the order they are driven}."""
    rows = {}
    for point in sweep.waypoints:
        if not point.name.startswith('survey_r'):
            continue
        row, column = point.name[len('survey_r'):].split('c')
        rows.setdefault(int(row), []).append(int(column))
    return rows


def test_the_bench_raster_alternates_direction_row_by_row():
    # What makes it a boustrophedon rather than a set of scan lines: the arm
    # never flies back across the bench to start the next row. A table
    # regenerated with the turn dropped would double the travel across a bench
    # with glassware on it, and nothing else would notice.
    #
    # Opposite DIRECTION rather than exact reversal, because a row with an
    # unreachable cell in it is shorter than its neighbours.
    with open(_shipped(), encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    for sweep in sweeps.values():
        rows = _survey_rows(sweep)
        assert len(rows) > 1
        directions = [columns[-1] - columns[0] for _, columns in sorted(rows.items())]
        assert all(value != 0 for value in directions), rows
        for first, second in zip(directions, directions[1:]):
            assert first * second < 0, rows


def test_the_bench_raster_goes_across_at_height_then_straight_down():
    # GAMMA-SHAPED, NEVER L-SHAPED, and this is the test that says so. Every
    # lateral move belongs to the survey pass, at the top height; the close
    # look is ONE cell reached by an approach at that same height and then a
    # vertical drop. A row of cells down at the close height would be lateral
    # motion at low clearance over a bench with other glassware on it -- the
    # shape the whole pattern exists to avoid.
    with open(_shipped(), encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    for sweep in sweeps.values():
        names = [point.name for point in sweep.waypoints]
        assert names[-2:] == ['close_approach', 'close'], names
        # ...and nothing else down there. Exactly one cell at the low height.
        assert [name for name in names if name.startswith('close')] == [
            'close_approach', 'close']
        # Every search cell comes first, so the arm has finished traversing
        # before it descends at all.
        assert all(name.startswith('survey') for name in names[:-2]), names


def test_the_bench_raster_sweeps_far_to_near():
    # The order the operator asked for, and the one that keeps the arm out of
    # its own way: start at the far edge and work back toward the robot.
    with open(_shipped(), encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    for sweep in sweeps.values():
        rows = sorted(_survey_rows(sweep))
        assert rows == list(range(len(rows))), rows


def test_the_bench_raster_is_generated_and_says_so():
    # The table is 24 joint configurations that all have to lie in one IK
    # branch. A hand-edited one leaves that branch silently, so the file has to
    # carry the command that rebuilds it.
    with open(_shipped(), encoding='utf-8') as stream:
        text = stream.read()
    assert 'GENERATED' in text
    assert 'solve_sweep_raster.py' in text


def test_the_bench_table_sweeps_with_the_camera_that_outranks_the_other():
    # The join between two files: a recovery_camera that is not the
    # higher-priority one in cho_object_pose's cameras.yaml would have its
    # close-up view MEDIANED into `side_1`'s far one instead of replacing it.
    cameras = pytest.importorskip('cho_object_pose.cameras')
    share = get_package_share_directory('cho_object_pose')
    with open(os.path.join(share, 'config', 'cameras.yaml'), encoding='utf-8') as stream:
        table = {camera.name: camera
                 for camera in cameras.parse_cameras(yaml.safe_load(stream))}
    with open(_shipped(), encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    for sweep in sweeps.values():
        assert sweep.recovery_camera in table, sweep.recovery_camera
        assert table[sweep.recovery_camera].priority == max(
            camera.priority for camera in table.values())


def _camera_table(package, *parts):
    """A camera table from an installed package, or None in a source-only tree."""
    try:
        path = os.path.join(get_package_share_directory(package), *parts)
    except LookupError:
        return None
    if not os.path.exists(path):
        return None
    cameras = pytest.importorskip('cho_object_pose.cameras')
    with open(path, encoding='utf-8') as stream:
        return {camera.name: camera
                for camera in cameras.parse_cameras(yaml.safe_load(stream))}


@pytest.mark.parametrize('sweep_table,package,camera_parts', [
    ('fr5_bench.yaml', 'cho_object_pose', ('config', 'cameras.yaml')),
    ('fr5_mujoco_cell.yaml', 'cho_bringup_fr5',
     ('config', 'mujoco', 'cameras.yaml')),
    ('fr5_mujoco_cell.yaml', 'cho_bringup_fr5',
     ('config', 'mujoco', 'cameras_wrist_only.yaml')),
])
def test_every_sweep_table_names_a_camera_its_bench_actually_has(
        sweep_table, package, camera_parts):
    # THE ONE THAT BIT. The MuJoCo cell names its camera `wrist_cam`, after the
    # MJCF; the physical bench names its `wrist`. A sweep table carrying the
    # wrong one of those does not fail quietly -- assess() REFUSES and says so
    # -- but it refuses on the bench, after a bringup, instead of here.
    table = _camera_table(package, *camera_parts)
    if table is None:
        pytest.skip(f'{package} is not installed in this tree')
    path = os.path.join(os.path.dirname(_shipped()), sweep_table)
    with open(path, encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    for sweep in sweeps.values():
        assert sweep.recovery_camera in table, (
            sweep_table, sweep.recovery_camera, sorted(table))
        # And it has to be the one that outranks the others, or its close view
        # is medianed into the far one instead of replacing it. A table with
        # only one camera passes this trivially, which is correct.
        assert table[sweep.recovery_camera].priority == max(
            camera.priority for camera in table.values())


def test_every_state_has_a_message_constant():
    # Strings in the rules above, uint8 on the wire, mapped BY NAME in
    # behaviors/action/occlusion_sweep.py. This is what keeps that mapping
    # total from this side_1.
    message = pytest.importorskip('cho_interfaces.msg')
    declared = {name[len('STATE_'):].lower()
                for name in dir(message.CameraVisibility) if name.startswith('STATE_')}
    assert declared == set(occlusion.STATES)


# ------------------------------------------------- a planning target's rule

def test_a_planning_target_is_swept_even_while_it_is_being_published():
    # The trigger the standing pair cannot produce a score for. Both cameras
    # here are publishing happily; the object is still recovered, because the
    # arm is about to touch it and no camera on the arm has measured it.
    assessment = occlusion.assess(
        _view(publishing=True, status='publishing, spread 6.4 mm',
              cameras=(_camera('sdl_cam', decision_margin=237.3, edge_px=32.8),
                       _camera('sdl_cam_side', decision_margin=237.5, edge_px=33.4),
                       _camera('wrist_cam', state='not_in_frame'))),
        'wrist_cam', planning_target=True)
    assert assessment.action == occlusion.SWEEP
    assert 'planning target' in assessment.reason


def test_the_same_object_is_left_alone_when_it_is_not_a_target():
    # The other half of the rule, and the reason it is not just "always sweep":
    # an obstacle is allowed to be known to 10 mm, because the planner inflates
    # it by more than that.
    view = _view(publishing=True, status='publishing, spread 6.4 mm',
                 cameras=(_camera('sdl_cam', decision_margin=237.3, edge_px=32.8),
                          _camera('sdl_cam_side', decision_margin=237.5, edge_px=33.4),
                          _camera('wrist_cam', state='not_in_frame')))
    assert occlusion.assess(view, 'wrist_cam').action == occlusion.SATISFIED


def test_a_planning_target_the_wrist_has_already_measured_needs_no_sweep():
    # Entry and exit have to agree, or the leaf declines to start for a reason
    # it would not have stopped for.
    view = _view(publishing=True, status='publishing, wrist_cam overrides',
                 override_camera='wrist_cam',
                 cameras=(_camera('sdl_cam', state='suppressed'),
                          _camera('wrist_cam', decision_margin=238.3, edge_px=83.4)))
    assessment = occlusion.assess(view, 'wrist_cam', planning_target=True)
    assert assessment.action == occlusion.SATISFIED
    assert 'measured by wrist_cam' in assessment.reason
    assert occlusion.recovered(view, 'wrist_cam')


def test_a_planning_target_naming_an_absent_camera_still_refuses():
    # The flag must not turn a misconfiguration into motion: a sweep towards a
    # camera nobody has cannot end.
    assessment = occlusion.assess(
        _view(publishing=True, cameras=(_camera('sdl_cam'),)),
        'wrist_cam', planning_target=True)
    assert assessment.action == occlusion.REFUSE
    assert 'wrist_cam' in assessment.reason


def test_planning_target_is_read_from_the_table_and_defaults_off():
    document = {'defaults': {'recovery_camera': 'wrist'},
                'sweeps': [{'object': 'beaker',
                            'waypoints': [{'name': 'a', 'joints': [0.0] * 6}]},
                           {'object': 'flask', 'planning_target': True,
                            'waypoints': [{'name': 'a', 'joints': [0.0] * 6}]}]}
    sweeps = occlusion.parse_sweeps(document)
    assert sweeps['beaker'].planning_target is False
    assert sweeps['flask'].planning_target is True


def test_a_planning_target_written_as_a_string_is_refused():
    # 'no' is a non-empty string and therefore true, which would send the arm
    # looking at everything on the bench. YAML parses the real spellings.
    document = {'defaults': {'recovery_camera': 'wrist'},
                'sweeps': [{'object': 'beaker', 'planning_target': 'no',
                            'waypoints': [{'name': 'a', 'joints': [0.0] * 6}]}]}
    with pytest.raises(ValueError, match='planning_target'):
        occlusion.parse_sweeps(document)


# ------------------------------------------------- occlusion is a duration

def _gone(**kwargs):
    """An object nobody is publishing, with the standing camera blind to it."""
    return _view(cameras=[_camera('side_1', 'not_in_frame'),
                          _camera('wrist', 'not_in_frame')], **kwargs)


def test_a_pose_that_has_only_just_stopped_is_waited_on_not_swept_for():
    # One empty aggregation window is a dropped frame, a blurred tag or a slow
    # TF lookup. Driving an arm across the cell for it is the wrong answer, and
    # it is the answer the instant trigger gives.
    assessment = occlusion.assess(_gone(), 'wrist', unseen_sec=0.4, min_unseen_sec=2.0)
    assert assessment.action == occlusion.WAIT
    assert '0.4s ago' in assessment.reason
    assert '2.0s' in assessment.reason


def test_a_pose_gone_for_longer_than_the_threshold_is_occlusion():
    assessment = occlusion.assess(_gone(), 'wrist', unseen_sec=3.5, min_unseen_sec=2.0)
    assert assessment.action == occlusion.SWEEP
    assert 'has not been for 3.5s' in assessment.reason


def test_the_threshold_off_keeps_the_instant_trigger():
    # The older behaviour, and still the right one for a bench whose pose node
    # publishes continuously enough that any gap means something.
    assert occlusion.assess(_gone(), 'wrist', unseen_sec=0.0,
                            min_unseen_sec=0.0).action == occlusion.SWEEP


def test_a_caller_that_does_not_time_it_gets_the_old_behaviour():
    # None means "not measured", which must not be read as "zero seconds" --
    # that would make every untimed caller wait for a threshold it can never
    # report reaching.
    assert occlusion.assess(_gone(), 'wrist', unseen_sec=None,
                            min_unseen_sec=2.0).action == occlusion.SWEEP


def test_the_clock_does_not_delay_a_quality_trigger():
    # A view that is too oblique is exactly as oblique a second later. Waiting
    # only postpones the sweep that was always going to be needed.
    view = _view(publishing=True, cameras=[_camera('side_1', 'ok', edge_px=18.0),
                                           _camera('wrist', 'not_in_frame')])
    assessment = occlusion.assess(view, 'wrist', min_tag_edge_px=40.0,
                                  unseen_sec=None, min_unseen_sec=5.0)
    assert assessment.action == occlusion.SWEEP


def test_a_refusal_still_beats_the_clock():
    # Waiting cannot fix a broken TF chain, so the refusal has to be found
    # first -- otherwise the leaf sits in WAIT until the sweep deadline and
    # reports a timeout instead of the real fault.
    view = _view(cameras=[_camera('side_1', 'not_in_frame'),
                          _camera('wrist', 'no_tf', 'base_link <- wrist_tag_0: no transform')])
    assessment = occlusion.assess(view, 'wrist', unseen_sec=0.1, min_unseen_sec=5.0)
    assert assessment.action == occlusion.REFUSE


def test_the_sweep_table_carries_the_threshold():
    sweeps = occlusion.parse_sweeps({
        'defaults': {'recovery_camera': 'wrist', 'min_unseen_sec': 2.5},
        'sweeps': [{'object': 'beaker',
                    'waypoints': [{'name': 'survey', 'joints': [0.0] * 6}]}]})
    assert sweeps['beaker'].min_unseen_sec == 2.5


def test_a_negative_threshold_is_refused():
    with pytest.raises(ValueError, match='min_unseen_sec'):
        occlusion.parse_sweeps({
            'defaults': {'recovery_camera': 'wrist', 'min_unseen_sec': -1.0},
            'sweeps': [{'object': 'beaker',
                        'waypoints': [{'name': 'survey', 'joints': [0.0] * 6}]}]})
