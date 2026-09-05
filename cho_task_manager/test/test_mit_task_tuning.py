"""Unit tests for the OpenArm MIT task-space tuning probe."""

import py_trees
import pytest

from cho_task_manager.behaviors.service.mit_task_diagnostics import parse_diagnostics
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.openarm.mit_task_tuning import (
    DEFAULT_PROBE_DURATION_SEC,
    DEFAULT_PROBE_TRANSLATION,
)


TASK_DIAGNOSTICS_MESSAGE = (
    'last_pose_error=[0.000294463,-1.14801e-06,-2.04606e-05,5.95537e-06,0.00114411,-2.24283e-05] '
    'peak_wrench=[2.97111,0.555616,0.710019,0.0550937,0.468375,0.0835039] '
    'peak_tau_ff=[0.0744137,0.377507,0.0745415,0.816182,0.0716371,0.195585,0.311662] '
    'q_ref=[0,0,0,0.001,0,0,0]'
)
PROTOCOL_STATUS_MESSAGE = 'session=1 ack=26427 safe_generation=1 safe_ack=1 status=1'


def _config(**overrides):
    config = {'robot_type': 'openarm', 'task_space': 'task_space_impedance_mit_controller'}
    config.update(overrides)
    return config


def _named(root, name):
    for node in root.iterate():
        if node.name == name:
            return node
    return None


def test_parses_the_task_diagnostics_array_message():
    parsed = parse_diagnostics(TASK_DIAGNOSTICS_MESSAGE)
    assert len(parsed['last_pose_error']) == 6
    assert len(parsed['peak_wrench']) == 6
    assert len(parsed['peak_tau_ff']) == 7
    assert len(parsed['q_ref']) == 7
    assert parsed['peak_wrench'][0] == pytest.approx(2.97111)
    # Joint 4 is the elbow: the value the friction-limited probe is judged on.
    assert parsed['peak_tau_ff'][3] == pytest.approx(0.816182)
    assert parsed['q_ref'][3] == pytest.approx(0.001)


def test_parses_the_scalar_protocol_status_message():
    parsed = parse_diagnostics(PROTOCOL_STATUS_MESSAGE)
    assert parsed == {
        'session': 1.0, 'ack': 26427.0, 'safe_generation': 1.0,
        'safe_ack': 1.0, 'status': 1.0,
    }


def test_malformed_message_yields_nothing_instead_of_raising():
    assert parse_diagnostics('') == {}
    assert parse_diagnostics('peak_wrench=[not,a,number]') == {}


def test_task_is_registered_for_openarm_only():
    assert 'mit_task_tuning' in available_tasks('openarm')
    assert 'mit_task_tuning' not in available_tasks('franka')
    assert 'mit_task_tuning' not in available_tasks('ur5e')


def test_default_probe_is_forward_and_slightly_down():
    # A purely tangential forward probe leaves the reach sphere within
    # millimetres near full extension, so the default carries a negative Z.
    assert DEFAULT_PROBE_TRANSLATION[0] > 0.0
    assert DEFAULT_PROBE_TRANSLATION[2] < 0.0
    assert DEFAULT_PROBE_DURATION_SEC >= 0.25


def test_tree_uses_the_configured_probe_and_reverses_it():
    root = build_task_tree(
        'mit_task_tuning',
        _config(probe_translation=[0.04, 0.0, -0.008], probe_duration=6.0),
    )
    out = _named(root, 'Probe_Out')
    back = _named(root, 'Probe_Return')
    assert out is not None and back is not None
    assert (out.target_pose.position.x, out.target_pose.position.z) == (0.04, -0.008)
    assert (back.target_pose.position.x, back.target_pose.position.z) == (-0.04, 0.008)
    assert out.duration == 6.0 and back.duration == 6.0
    # Relative, so each leg composes onto the reference the controller already
    # holds rather than rebasing on the measured pose.
    assert out.relative and back.relative


def test_probe_legs_tolerate_a_stalled_or_unreachable_goal():
    # The controller is friction-limited before it is gain-limited, so an
    # aborted probe is a datapoint. If it failed the sequence, the run would
    # stop before the return leg brought the arm back.
    root = build_task_tree('mit_task_tuning', _config())
    for leg in ('Probe_Out', 'Probe_Return'):
        wrapper = _named(root, f'Tolerate_{leg}')
        assert isinstance(wrapper, py_trees.decorators.FailureIsSuccess)


def test_bimanual_profile_uses_its_own_broadcaster_and_pose_topic():
    # A bimanual build prefixes every per-arm resource. Checking for the
    # single-arm names there fails before the probe can run.
    root = build_task_tree(
        'mit_task_tuning',
        _config(profile='right', task_space='right_task_space_impedance_mit_controller'),
    )
    active = _named(root, 'Controllers_Active').require_active
    assert 'right_ee_state_broadcaster' in active
    assert 'ee_state_broadcaster' not in active
    assert _named(root, 'TCP_Before').topic == '/ee_state/right/pose'
    assert _named(root, 'TCP_After_Probe').topic == '/ee_state/right/pose'


def test_single_profile_keeps_the_unprefixed_names():
    root = build_task_tree('mit_task_tuning', _config())
    assert 'ee_state_broadcaster' in _named(root, 'Controllers_Active').require_active
    assert _named(root, 'TCP_Before').topic == '/ee_state/pose'


def test_return_leg_can_be_disabled():
    root = build_task_tree('mit_task_tuning', _config(probe_return=False))
    assert _named(root, 'Probe_Out') is not None
    assert _named(root, 'Probe_Return') is None


def test_probe_measures_against_a_baseline_taken_before_the_move():
    # peak_wrench / peak_tau_ff never decrease, so a lone reading says nothing
    # about one probe; the baseline is what makes them comparable across runs.
    root = build_task_tree('mit_task_tuning', _config())
    assert _named(root, 'Diagnostics_Baseline').record_as == 'baseline'
    assert _named(root, 'Diagnostics_After_Probe').compare_to == 'baseline'


def test_rejects_a_probe_the_action_server_would_refuse():
    with pytest.raises(ValueError, match='at least'):
        build_task_tree('mit_task_tuning', _config(probe_duration=0.1))
    with pytest.raises(ValueError, match='exactly 3'):
        build_task_tree('mit_task_tuning', _config(probe_translation=[0.03, 0.0]))
