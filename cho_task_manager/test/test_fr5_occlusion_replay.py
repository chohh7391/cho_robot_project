"""Locate the vessels, then replay: the seam between two trees that already work.

Both halves are imported -- the locate blocks from ``occlusion_recovery``, the
motion blocks from ``trajectory_replay`` -- and are tested there. What is worth
pinning here is how they join:

* everything that can refuse still refuses at BUILD time, the declared layout
  gate included, before the recovery half has moved the arm;
* every vessel is located before the arm goes to the recording's start pose,
  and the start pose is the recording's, not the recovery's home;
* each half drives its own controller, and the recovery never touches the jaws
  the replay is about to use.

Nothing here needs a ROS graph: the trees are built, not ticked.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from cho_task_manager.behaviors.action import (
    FollowJointTrajectoryBehavior,
    GripperActionBehavior,
    JointSpaceActionBehavior,
    OcclusionSweepBehavior,
    SinglePassSweepBehavior,
)
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.fr5.common import VESSELS
from cho_task_manager.utils.controller_names import load_robot_config
from cho_task_manager.utils.trajectory_recording import LayoutMismatch
import py_trees
import pytest
import yaml

HEADER = 't_s,j1,j2,j3,j4,j5,j6,operation\n'

#: The recording's assumed layout, and a cell declared to match it.
ASSUMED = {
    'beaker': {'xy': [0.48, 0.13]},
    'flask': {'xy': [0.53, 0.35]},
}

#: Where the recording starts. Deliberately not the registry home pose, so a
#: start-pose move that went to the wrong one is visible.
START = [0.1, -1.1, -2.0, -1.7, 1.6, 0.08]


def _sweep_table():
    try:
        path = os.path.join(get_package_share_directory('cho_task_manager'),
                            'config', 'sweep', 'fr5_bench.yaml')
        if os.path.exists(path):
            return path
    except LookupError:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'sweep', 'fr5_bench.yaml')


def _write(tmp_path, cell=None, gripper_events=()):
    csv_path = tmp_path / 'trial_waypoints.csv'
    meta_path = tmp_path / 'trial_meta.json'
    layout_path = tmp_path / 'cell.yaml'
    rows = [[i * 0.1] + [value + i * 0.001 for value in START] for i in range(12)]
    csv_path.write_text(HEADER + ''.join(
        '%s,Pick\n' % ','.join('%.6f' % value for value in row) for row in rows))
    meta_path.write_text(json.dumps({
        'seed': '2',
        'tool': 'fr5_ag95',
        'joint_names': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
        'waypoints': len(rows),
        'home_arm_rad': START,
        'gripper_events': list(gripper_events),
        'layout_the_trajectory_assumes': ASSUMED,
    }))
    layout_path.write_text(yaml.safe_dump({'layout': cell if cell is not None else ASSUMED}))
    return str(csv_path), str(meta_path), str(layout_path)


def _config(tmp_path, cell=None, gripper_events=(), **overrides):
    csv_path, meta_path, layout_path = _write(tmp_path, cell, gripper_events)
    config = load_robot_config('fr5')
    config.update({
        'sweep_config': _sweep_table(),
        'replay_trajectory': csv_path,
        'replay_meta': meta_path,
        'replay_layout': layout_path,
        'home_via': 'direct',
    })
    config.update(overrides)
    return config


def _tree(tmp_path, **kwargs):
    return build_task_tree('occlusion_replay', _config(tmp_path, **kwargs))


def _leaves(root):
    return [node for node in root.iterate() if not node.children]


def _index(leaves, predicate):
    return next(i for i, leaf in enumerate(leaves) if predicate(leaf))


def test_the_task_is_registered_for_the_fr5_and_only_the_fr5():
    assert 'occlusion_replay' in available_tasks('fr5')
    assert 'occlusion_replay' not in available_tasks('franka')


@pytest.mark.parametrize('missing', ['replay_trajectory', 'replay_layout', 'sweep_config'])
def test_every_input_file_is_required_and_named(tmp_path, missing):
    # The recording, the cell it is gated against, and where to look. Each one
    # missing would otherwise surface halfway through a run.
    with pytest.raises(ValueError, match=missing):
        _tree(tmp_path, **{missing: ''})


def test_a_cell_that_does_not_match_the_recording_refuses_before_anything_moves(tmp_path):
    # The declared gate runs at build time, exactly as in trajectory_replay --
    # a refusal after the recovery sweeps had already driven the arm would be
    # a worse place to learn it.
    moved = dict(ASSUMED, beaker={'xy': [0.60, 0.13]})
    with pytest.raises(LayoutMismatch, match='beaker'):
        _tree(tmp_path, cell=moved)


def test_every_vessel_is_located_before_the_replay_starts(tmp_path):
    # The whole point of the order: the vessels are on screen, and latched,
    # before the arm goes anywhere near them for the pour.
    leaves = _leaves(_tree(tmp_path))
    start = _index(leaves, lambda leaf: leaf.name == 'Go_Home_Replay_Start')
    recover = _index(leaves, lambda leaf: leaf.name == 'Recover_Vessels')
    back = _index(leaves, lambda leaf: leaf.name == 'Return_Vessels')
    assert recover < back < start


def test_per_object_locates_each_vessel_before_the_replay_starts(tmp_path):
    leaves = _leaves(_tree(tmp_path, sweep_mode='per_object'))
    start = _index(leaves, lambda leaf: leaf.name == 'Go_Home_Replay_Start')
    for vessel in VESSELS:
        name = vessel.name.capitalize()
        recover = _index(leaves, lambda leaf, n=name: leaf.name == 'Recover_%s' % n)
        detect = _index(leaves, lambda leaf, n=name: leaf.name == 'Detect_%s' % n)
        back = _index(leaves, lambda leaf, n=name: leaf.name == 'Return_%s' % n)
        assert recover < detect < back < start


def test_the_replay_starts_from_the_recordings_own_start_pose(tmp_path):
    # Not from the recovery's home pose: the first recorded segment is timed
    # from START, and starting it anywhere else makes its first second a lunge.
    leaves = _leaves(_tree(tmp_path))
    start = leaves[_index(leaves, lambda leaf: leaf.name == 'Go_Home_Replay_Start')]
    assert isinstance(start, JointSpaceActionBehavior)
    assert list(start.target_joints.position) == pytest.approx(START)
    first_segment = _index(leaves, lambda leaf: isinstance(leaf, FollowJointTrajectoryBehavior))
    assert leaves.index(start) < first_segment


def test_the_recovery_does_not_home_twice_before_the_replay(tmp_path):
    # Its own closing home move would put a third pose between the last return
    # and the start pose for no reason: the return already ends at home.
    names = [node.name for node in _tree(tmp_path).iterate()]
    assert 'Go_Home_Final' in names          # the replay's closing move, once
    assert names.count('Go_Home_Final') == 1


def test_each_half_drives_its_own_controller(tmp_path):
    config = _config(tmp_path)
    leaves = _leaves(build_task_tree('occlusion_replay', config))
    for leaf in leaves:
        if isinstance(leaf, (OcclusionSweepBehavior, SinglePassSweepBehavior,
                             JointSpaceActionBehavior)):
            assert config['joint_space'] in leaf.action_name
        if isinstance(leaf, FollowJointTrajectoryBehavior):
            assert 'joint_trajectory_controller' in leaf.action_name
    names = [leaf.name for leaf in leaves]
    # The direct start-pose move ran on the hold controller, so the arm has to
    # change hands before the first segment, and the switch has to be checked.
    assert names.index('Switch_To_joint_trajectory_controller') < names.index(
        'Verify_Replay_Controller_Active')


def test_the_recovery_never_touches_the_jaws(tmp_path):
    # The first gripper command of the run is the replay's own grasp.
    events = [{'t_s': 0.55, 'event': 'close'}]
    leaves = _leaves(_tree(tmp_path, gripper_events=events))
    first_gripper = _index(leaves, lambda leaf: isinstance(leaf, GripperActionBehavior))
    first_segment = _index(leaves, lambda leaf: isinstance(leaf, FollowJointTrajectoryBehavior))
    assert first_gripper > first_segment
    assert leaves[first_gripper].grasp is True


def test_the_latches_land_where_occlusion_recovery_puts_them(tmp_path):
    keys = {target.record_as for leaf in _leaves(_tree(tmp_path))
            if isinstance(leaf, SinglePassSweepBehavior) for target in leaf.targets}
    assert keys == {vessel.key for vessel in VESSELS}
    keys = {leaf.record_as for leaf in _leaves(_tree(tmp_path, sweep_mode='per_object'))
            if isinstance(leaf, PoseTargetBehavior)}
    assert keys == {vessel.key for vessel in VESSELS}


def test_the_moveit_variant_homes_through_the_bridge(tmp_path):
    leaves = _leaves(_tree(tmp_path, home_via='moveit'))
    start = leaves[_index(leaves, lambda leaf: leaf.name == 'Go_Home_MoveIt_Replay_Start')]
    assert start.action_name == '/fr5/controller_action_server/moveit_joint'


def test_both_halves_report_what_they_covered(tmp_path):
    root = _tree(tmp_path)
    assert root.recovery_summary['vessels'] == [vessel.name for vessel in VESSELS]
    assert root.replay_summary['home_via'] == 'direct'
    assert root.replay_summary['declared_layout'].endswith('cell.yaml')


def test_a_failure_still_reaches_the_hold_controller(tmp_path):
    root = _tree(tmp_path)
    assert isinstance(root, py_trees.decorators.OneShot)
    names = [node.name for node in root.iterate()]
    assert 'Mission_Or_Safe_Abort' in names
    assert 'Abort_Verify_Hold_Active' in names
