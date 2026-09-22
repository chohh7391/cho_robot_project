"""The suppression rule, and the vocabulary that reports it.

No camera, no node, no clock -- the same reason ``test_geometry`` needs none.
What is worth testing here is the decision that changes which samples reach the
median, and the two ways a camera can be quiet that must never be confused:
outranked by a better view, and simply gone.
"""

from cho_object_pose import visibility
import pytest


# ------------------------------------------------------------- suppression

def test_equal_priority_cameras_are_all_kept():
    # The historical bench: peers, fused, nothing suppressed. A `priority`
    # nobody set must not change what any existing setup does.
    selection = visibility.select_by_priority(['side_1', 'rs_left'], {})
    assert selection.kept == ('rs_left', 'side_1')
    assert selection.suppressed == ()


def test_a_higher_priority_camera_suppresses_the_others():
    selection = visibility.select_by_priority(
        ['side_1', 'wrist'], {'wrist': 10, 'side_1': 0})
    assert selection.kept == ('wrist',)
    assert selection.suppressed == ('side_1',)
    assert selection.priority == 10


def test_a_camera_that_contributed_nothing_suppresses_nothing():
    # THE WHOLE REASON THE OVERRIDE NEEDS NO EXPIRY. Suppression is decided by
    # what is in the window, so the moment the wrist loses the tag its samples
    # age out and the standing camera is believed again -- with no lifetime
    # concept anywhere and no state to reset.
    selection = visibility.select_by_priority(['side_1'], {'wrist': 10, 'side_1': 0})
    assert selection.kept == ('side_1',)
    assert selection.suppressed == ()


def test_several_cameras_can_share_the_winning_tier():
    selection = visibility.select_by_priority(
        ['left', 'right', 'side_1'], {'left': 5, 'right': 5, 'side_1': 0})
    assert selection.kept == ('left', 'right')
    assert selection.suppressed == ('side_1',)


def test_repeated_samples_from_one_camera_count_once():
    # The node passes every sample in the window, not a set of names.
    selection = visibility.select_by_priority(
        ['side_1', 'side_1', 'side_1', 'wrist'], {'wrist': 1})
    assert selection.kept == ('wrist',)
    assert selection.suppressed == ('side_1',)


def test_a_camera_missing_from_the_table_counts_as_zero():
    # "Declares no priority" is what a default of 0 means, so an entry that
    # simply left the field out must not win or lose by accident.
    selection = visibility.select_by_priority(['side_1', 'wrist'], {'wrist': 3})
    assert selection.suppressed == ('side_1',)


def test_negative_priorities_only_have_to_order():
    selection = visibility.select_by_priority(
        ['bench', 'ceiling'], {'bench': -1, 'ceiling': -5})
    assert selection.kept == ('bench',)


def test_an_empty_window_selects_nothing():
    selection = visibility.select_by_priority([], {'wrist': 10})
    assert selection.kept == ()
    assert selection.suppressed == ()


# ------------------------------------------------------------- the reasons

def test_a_reason_goes_stale_once_it_is_older_than_the_window():
    # Derived from the window rather than from a lifetime of its own: a sample
    # that old has already been pruned, so the reason describes nothing that
    # could still be contributing.
    fresh = visibility.current_reason(visibility.ok(), 0.2, 0.5)
    assert fresh.state == 'ok'
    gone = visibility.current_reason(visibility.ok(), 0.9, 0.5)
    assert gone.state == 'stale'
    # What it last managed to say is kept, because 'stale' alone does not say
    # whether the camera was working when it went quiet.
    assert 'ok' in gone.detail


def test_never_having_been_heard_from_is_not_staleness():
    # A detector that was never launched and one that died look identical
    # otherwise, and they have different fixes.
    assert visibility.current_reason(
        visibility.UNKNOWN, 1e6, 0.5).state == 'unknown'


def test_staleness_wins_over_suppression():
    # A camera that has gone quiet was not outranked, it left. Reporting it as
    # suppressed would say the pipeline is working when it is not.
    reason = visibility.current_reason(
        visibility.ok(), 9.0, 0.5, suppressed_by='wrist')
    assert reason.state == 'stale'


def test_a_fresh_outranked_camera_reads_as_suppressed():
    reason = visibility.current_reason(
        visibility.ok(), 0.1, 0.5, suppressed_by='wrist')
    assert reason.state == 'suppressed'
    assert reason.detail == 'wrist'


def test_occlusion_and_a_bad_decode_stay_distinguishable():
    # The one distinction the recovery turns on: a tag that is not in the frame
    # is something to go and look at, a tag that decoded badly is not.
    assert visibility.NOT_IN_FRAME.state == 'not_in_frame'
    assert visibility.rejected('decision margin 12.0 < 35.0').state == 'rejected'
    assert visibility.no_tf('base_link <- wrist_tag_0: lookup failed').state == 'no_tf'


@pytest.mark.parametrize('reason,expected', [
    (visibility.UNKNOWN, 'no detection yet'),
    (visibility.ok(), 'ok'),
    (visibility.NOT_IN_FRAME, 'tag not in frame'),
    (visibility.rejected('too oblique'), 'rejected: too oblique'),
    (visibility.no_tf('a <- b: no'), 'no TF a <- b: no'),
])
def test_the_report_still_says_what_it_always_said(reason, expected):
    # These strings predate the topic and are what anyone commissioning a bench
    # greps the log for.
    assert visibility.describe(reason) == expected


def test_describe_refuses_a_state_it_does_not_know():
    with pytest.raises(ValueError):
        visibility.describe(visibility.Reason('sideways', ''))


# ------------------------------------------------------------- decode score

def test_the_decode_score_travels_with_the_state():
    # The state says a detection passed the gate, which is set for 'good
    # enough to publish'. A recovery sweep exists to beat that, so it needs the
    # number and not just the verdict.
    reason = visibility.ok(66.0, 41.0)
    assert reason.decision_margin == 66.0
    assert reason.edge_px == 41.0
    assert visibility.describe_score(reason) == 'margin 66, edge 41px'


def test_a_rejected_detection_is_scored_too():
    # 'margin 33 against a gate of 35' and 'margin 4' are a lower waypoint and
    # a wrong lens respectively, and the state alone cannot tell them apart.
    reason = visibility.rejected('decision_margin 33.0 < 35.0', 33.0, 48.0)
    assert reason.decision_margin == 33.0
    assert 'margin 33' in visibility.describe_score(reason)


def test_nothing_to_score_reports_no_score_rather_than_zero():
    # Zero is a real, terrible margin; 'never measured' is not a margin at all,
    # and a consumer thresholding on it must be able to tell.
    assert visibility.UNKNOWN.decision_margin == visibility.NO_SCORE
    assert visibility.NOT_IN_FRAME.decision_margin == visibility.NO_SCORE
    assert visibility.describe_score(visibility.NOT_IN_FRAME) == ''


def test_the_score_survives_both_overlays():
    # A camera that was outranked still measured what it measured, and that is
    # what says whether the override was an improvement or just a preference.
    scored = visibility.ok(66.0, 41.0)
    suppressed = visibility.current_reason(scored, 0.1, 0.5, suppressed_by='wrist')
    assert suppressed.state == 'suppressed'
    assert suppressed.decision_margin == 66.0
    stale = visibility.current_reason(scored, 9.0, 0.5)
    assert stale.state == 'stale'
    assert stale.decision_margin == 66.0


# ------------------------------------------------------------ the wire form

def test_every_state_has_a_message_constant():
    # The states are strings here and uint8 on the wire, and node.py maps one
    # to the other BY NAME. This is what makes that mapping total: add a state
    # without adding STATE_<UPPER> to CameraVisibility.msg and this fails,
    # rather than the node failing at import on a bench.
    message = pytest.importorskip('cho_interfaces.msg')
    declared = {name[len('STATE_'):].lower()
                for name in dir(message.CameraVisibility) if name.startswith('STATE_')}
    assert declared == set(visibility.STATES)
