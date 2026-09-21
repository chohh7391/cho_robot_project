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


def _camera(name='oak', state='ok', detail='', age_sec=0.1, priority=0,
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


def test_a_tag_no_camera_can_see_is_worth_going_to_look_at():
    # This is what occlusion looks like from the perception side, and it is
    # also what a tag outside the field of view looks like. Neither side can
    # tell them apart, and going to look is the right answer to both.
    assessment = occlusion.assess(_view(cameras=[
        _camera('oak', 'not_in_frame'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')
    assert assessment.action == occlusion.SWEEP


def test_a_tag_that_decodes_badly_is_also_worth_a_closer_look():
    # Too far, too oblique, motion-blurred: exactly what a close view fixes.
    assessment = occlusion.assess(_view(cameras=[
        _camera('oak', 'rejected', 'decision margin 18.0 < 35.0'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')
    assert assessment.action == occlusion.SWEEP


def test_a_sweep_camera_with_no_tf_is_refused_rather_than_driven():
    # Moving the camera moves the same broken chain somewhere else. For a
    # wrist camera this is usually the robot's TF not being up -- and driving
    # the arm is a peculiar response to that.
    assessment = occlusion.assess(_view(cameras=[
        _camera('oak', 'not_in_frame'),
        _camera('wrist', 'no_tf', 'base_link <- wrist_tag_0: no transform',
                priority=10)]), 'wrist')
    assert assessment.action == occlusion.REFUSE
    assert 'wrist' in assessment.reason


def test_another_cameras_broken_tf_does_not_refuse_the_sweep():
    # The OAK's extrinsic being wrong is a real problem and not this one's:
    # the wrist has its own chain and can still do the looking.
    assessment = occlusion.assess(_view(cameras=[
        _camera('oak', 'no_tf', 'base_link <- oak_tag_0: no transform'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')
    assert assessment.action == occlusion.SWEEP


def test_a_bench_where_nothing_is_running_is_refused():
    # Every camera quiet means the detectors did not start, or are publishing
    # where the pose node is not listening. An arm that sweeps because a node
    # failed to launch is answering the wrong question.
    assessment = occlusion.assess(_view(cameras=[
        _camera('oak', 'stale', 'last said: ok', age_sec=42.0),
        _camera('wrist', 'unknown', age_sec=-1.0, priority=10)]), 'wrist')
    assert assessment.action == occlusion.REFUSE
    assert 'detectors' in assessment.reason


def test_naming_a_camera_the_pose_node_does_not_have_is_refused():
    # The sweep table and the camera table are separate files. This is the
    # failure that produces: a sweep that drives four waypoints and then waits
    # for a camera that was never in the fusion.
    assessment = occlusion.assess(
        _view(cameras=[_camera('oak', 'not_in_frame')]), 'wrist')
    assert assessment.action == occlusion.REFUSE
    assert 'cameras.yaml' in assessment.reason


# --------------------------------------------------------- when it worked

def test_a_good_decode_is_required_when_one_is_asked_for():
    # The point of the raster. cho_object_pose publishes from margin 35, and
    # the far view that prompted the recovery already clears it -- so stopping
    # at the first rung that merely publishes trades one marginal measurement
    # for another. The requirement is what makes the sweep keep descending.
    cameras = [_camera('oak', 'suppressed', 'wrist'),
               _camera('wrist', 'ok', priority=10, decision_margin=42.0)]
    view = _view(publishing=True, override_camera='wrist', cameras=cameras)
    assert not occlusion.recovered(view, 'wrist', min_decision_margin=55.0)
    # ...and it stops descending once it gets one.
    better = [_camera('oak', 'suppressed', 'wrist'),
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
    cameras = [_camera('oak', 'suppressed', 'wrist'),
               _camera('wrist', 'ok', priority=10)]
    assert occlusion.recovered(
        _view(publishing=True, override_camera='wrist', cameras=cameras), 'wrist')


def test_the_standing_camera_getting_its_view_back_is_not_a_recovery():
    # A fine outcome for the task, but the sweep did not cause it -- and a leaf
    # that took credit would hide one whose waypoints look nowhere useful.
    assert not occlusion.recovered(_view(publishing=True, cameras=[
        _camera('oak', 'ok'),
        _camera('wrist', 'not_in_frame', priority=10)]), 'wrist')


def test_the_sweep_camera_seeing_the_tag_is_not_yet_a_pose():
    # 'ok' means a sample went into the window; the node still has min_samples
    # and the spread gate to satisfy before anything is published.
    assert not occlusion.recovered(_view(publishing=False, cameras=[
        _camera('wrist', 'ok', priority=10)]), 'wrist')


def test_describe_cameras_names_every_camera_its_reason_and_its_score():
    text = occlusion.describe_cameras(_view(cameras=[
        _camera('oak', 'rejected', 'too oblique', decision_margin=21.0, edge_px=18.0),
        _camera('wrist', 'not_in_frame', priority=10,
                decision_margin=occlusion.NO_SCORE)]))
    assert 'oak: rejected (too oblique) [margin 21, edge 18px]' in text
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
        assert names[-1].startswith('close')
        # Asking for a decode better than cho_object_pose's own 35 gate is what
        # makes the second pass mean anything.
        assert sweep.min_decision_margin > 35.0
        # The first waypoint is a long swing from the home pose, not a raster
        # step, and this cell has a rate ceiling.
        assert sweep.waypoints[0].duration > sweep.waypoints[-1].duration


def test_the_bench_raster_alternates_direction_row_by_row():
    # What makes it a boustrophedon rather than a set of scan lines: the arm
    # never flies back across the bench to start the next row. A table
    # reordered by hand -- or regenerated with the turn dropped -- would double
    # the travel and nothing else would notice.
    with open(_shipped(), encoding='utf-8') as stream:
        sweeps = occlusion.parse_sweeps(yaml.safe_load(stream), joint_names=JOINTS)
    for sweep in sweeps.values():
        rows = {}
        for index, point in enumerate(sweep.waypoints):
            if not point.name.startswith('survey_r'):
                continue
            row, column = point.name[len('survey_r'):].split('c')
            rows.setdefault(row, []).append((int(column), index))
        assert len(rows) > 1
        # Consecutive rows run their columns in opposite orders.
        orders = [[column for column, _ in sorted(cells, key=lambda item: item[1])]
                  for _, cells in sorted(rows.items())]
        for first, second in zip(orders, orders[1:]):
            assert first == list(reversed(second)), orders


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
    # close-up view MEDIANED into the OAK's far one instead of replacing it.
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


def test_every_state_has_a_message_constant():
    # Strings in the rules above, uint8 on the wire, mapped BY NAME in
    # behaviors/action/occlusion_sweep.py. This is what keeps that mapping
    # total from this side.
    message = pytest.importorskip('cho_interfaces.msg')
    declared = {name[len('STATE_'):].lower()
                for name in dir(message.CameraVisibility) if name.startswith('STATE_')}
    assert declared == set(occlusion.STATES)
