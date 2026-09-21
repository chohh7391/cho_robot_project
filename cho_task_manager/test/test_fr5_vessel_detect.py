"""The perception-only task, and the one thing that makes it worth having.

It commands nothing, so what is worth testing is that it commands nothing --
every other FR5 tree switches a controller and sends a goal, and the value of
this one is precisely that it does neither while a camera setup is being
commissioned. The rest is the contract it shares with its object table: the
topics, the frame, and the blackboard keys a motion leaf would read.
"""

import os

from ament_index_python.packages import get_package_share_directory
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.fr5 import vessel_detect
from cho_task_manager.tasks.fr5.common import VESSELS
from cho_task_manager.utils.blackboard import TASK_NAMESPACE
from cho_task_manager.utils.controller_names import load_robot_config
import pytest
import yaml


def _table():
    """The installed task-owned object table, or the source copy in a fresh tree."""
    try:
        path = os.path.join(get_package_share_directory('cho_task_manager'),
                            'config', 'perception', 'vessel_detect.yaml')
        if os.path.exists(path):
            with open(path, encoding='utf-8') as stream:
                return yaml.safe_load(stream)
    except LookupError:
        pass
    source = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'config', 'perception', 'vessel_detect.yaml')
    with open(source, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _entries():
    return {entry['name']: entry for entry in _table()['objects']}


def _config():
    return load_robot_config('fr5')


def _tree():
    return build_task_tree('vessel_detect', _config())


# --------------------------------------------------------- it moves nothing

def test_the_tree_commands_no_motion_at_all():
    # The whole point. Until the tag ids, the offsets and the extrinsics are
    # checked, driving an arm at a detected pose is driving at a number nobody
    # has measured.
    from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
    assert not [node for node in _tree().iterate()
                if isinstance(node, BaseActionBehavior)]


def test_the_tree_switches_no_controller():
    from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
    assert not [node for node in _tree().iterate()
                if isinstance(node, SwitchControllerServiceBehavior)]


def test_there_is_no_safe_abort_branch_because_nothing_was_taken():
    # abort=False is not a shortcut: the abort exists to put an arm back on a
    # hold controller, and this tree never took it off one. A switch here would
    # be its only act of claiming the robot.
    names = [node.name for node in _tree().iterate()]
    assert not [name for name in names if 'Abort' in name]


def test_every_leaf_is_a_detection():
    leaves = [node for node in _tree().iterate() if not node.children]
    assert leaves, 'the tree has no leaves at all'
    assert all(isinstance(leaf, PoseTargetBehavior) for leaf in leaves)


# ------------------------------------------------------------- the contract

def test_the_task_is_registered_for_fr5_only():
    assert 'vessel_detect' in available_tasks('fr5')
    for robot in ('franka', 'ur5e', 'openarm'):
        assert 'vessel_detect' not in available_tasks(robot)


def test_the_table_publishes_the_topics_the_tree_subscribes_to():
    # The vessel name is the joining key across three files: this table, the
    # tree, and the layout a recording assumes (see perceived_replay).
    entries = _entries()
    for vessel in VESSELS:
        assert vessel.name in entries, f'the tree waits for the object named {vessel.name}'
        assert entries[vessel.name]['topic'] == vessel.topic


def test_each_vessel_is_latched_under_the_key_a_motion_leaf_would_read():
    by_name = {node.name: node for node in _tree().iterate()}
    for vessel in VESSELS:
        detect = by_name['Detect_%s' % vessel.name.capitalize()]
        assert detect.topic == vessel.topic
        assert detect.record_as == vessel.key
        assert detect.namespace == TASK_NAMESPACE


def test_the_detection_frame_comes_from_the_registry_not_a_literal():
    config = _config()
    by_name = {node.name: node for node in build_task_tree('vessel_detect', config).iterate()}
    for vessel in VESSELS:
        # arm_base_link, not base_frame: PoseTargetBehavior transforms nothing,
        # so a pose in any other frame has to fail rather than be obeyed.
        assert by_name['Detect_%s' % vessel.name.capitalize()].required_frame == \
            config['arm_base_link']


def test_the_timeout_allows_for_a_cold_start():
    # cho_object_pose publishes nothing until it has a full agreement window,
    # and on a cold start that waits on TF and the exposure settling too.
    assert vessel_detect.DETECT_TIMEOUT_SEC >= 15.0


# ----------------------------------------------------------- the table itself

def test_every_vessel_is_offset_sideways_from_its_tag():
    # The tag stands on a stalk BESIDE the vessel. A table that lost the
    # sideways component would point at the tag instead, which is the one
    # failure the numbers alone cannot show.
    for entry in _entries().values():
        x, y, _z = entry['grasp_offset']['position']
        assert max(abs(x), abs(y)) > 0.02, entry['name']


def test_the_table_stops_above_the_bench_not_in_it():
    for entry in _entries().values():
        assert entry['grasp_offset']['position'][2] > 0.0, entry['name']


def test_the_two_vessels_do_not_share_a_tag_or_a_topic():
    entries = _entries()
    assert len({entry['tag_id'] for entry in entries.values()}) == len(entries)
    assert len({entry['topic'] for entry in entries.values()}) == len(entries)


def test_every_vessel_declares_a_body_to_draw():
    # The marker is how a wrong offset becomes visible instead of deduced.
    for entry in _entries().values():
        assert 'shape' in entry, entry['name']
        assert len(entry['shape']['size']) == 3


def test_each_body_is_drawn_standing_on_the_bench():
    # shape.origin is relative to the PUBLISHED pose, which is the standoff. So
    # origin_z + standoff_z must come back to half the vessel's own height, or
    # the marker floats above the bench (or sinks into it).
    for entry in _entries().values():
        standoff_z = entry['grasp_offset']['position'][2]
        origin_z = entry['shape']['origin'][2]
        half_height = entry['shape']['size'][2] / 2.0
        assert origin_z + standoff_z == pytest.approx(half_height, abs=1e-9), entry['name']


def test_every_vessel_declares_the_size_its_tag_is_printed_at():
    # The detector is started FROM this table, so this number IS the edge
    # length it estimates range from. Omitting it silently falls back to the
    # detector's default, and a 39 mm tag detected as 40 mm is a 2.5% range
    # error nothing downstream can see.
    for entry in _entries().values():
        assert 'tag_size' in entry, entry['name']
        assert 0.005 < entry['tag_size'] < 0.5, entry['name']


def test_the_table_parses_as_an_object_table():
    # The same parser the pose node and the detector launch use, so a typo here
    # fails a test rather than a bench session.
    from cho_object_pose.objects import parse_objects
    specs = parse_objects(_table())
    assert {spec.name for spec in specs} == {vessel.name for vessel in VESSELS}
    assert all(spec.tag_size is not None for spec in specs)


def test_the_detector_derives_its_tag_list_from_this_table():
    # The ownership that matters: adding an object here must not need an edit
    # in cho_object_pose. The launch reads the same file and builds tag.ids,
    # tag.sizes and tag.frames from it.
    from cho_object_pose.geometry import tag_frame_name
    specs = __import__('cho_object_pose.objects', fromlist=['parse_objects']) \
        .parse_objects(_table())
    assert [tag_frame_name(spec.tag_id) for spec in specs] == \
        ['tag_%d' % spec.tag_id for spec in specs]


def test_every_mesh_body_names_a_file_that_is_actually_there():
    """A missing mesh fails in rviz's log and nowhere else, which is invisible.

    The resource is a package:// URI resolved through the ament index at
    display time, so a typo -- or a mesh that was never added to setup.py's
    data_files -- draws nothing at all and looks exactly like a tag that is not
    being detected.
    """
    import os
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for entry in _entries().values():
        shape = entry['shape']
        if shape['type'] != 'mesh':
            continue
        prefix = 'package://cho_task_manager/'
        assert shape['resource'].startswith(prefix), entry['name']
        assert os.path.isfile(os.path.join(root, shape['resource'][len(prefix):])), \
            entry['name']
