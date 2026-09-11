"""The perception-driven task, and the one string it shares with its config.

The tree subscribes to a topic by name and the object table declares the same
topic; they live in one package but in two files, so this asserts they still
agree. Everything else about the task is checked by building it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.franka import tag_reach
from cho_task_manager.utils.blackboard import TASK_NAMESPACE
import pytest
import yaml


def _table():
    """The installed task-owned object table, or the source copy in a fresh tree."""
    try:
        path = os.path.join(get_package_share_directory('cho_task_manager'),
                            'config', 'perception', 'tag_reach.yaml')
        if os.path.exists(path):
            with open(path, encoding='utf-8') as stream:
                return yaml.safe_load(stream)
    except LookupError:
        pass
    source = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'config', 'perception', 'tag_reach.yaml')
    with open(source, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_the_table_publishes_the_topic_the_tree_subscribes_to():
    entries = {entry['name']: entry for entry in _table()['objects']}
    assert 'target' in entries, 'the tree waits for the object named target'
    assert entries['target']['topic'] == tag_reach.TARGET_POSE_TOPIC


def test_the_table_stops_at_a_standoff_not_on_the_tag():
    # A zero offset would drive the TCP onto the tag face. Until the
    # intrinsics are calibrated the detected distance is only as good as the
    # configured tag size, so the task keeps its distance on purpose.
    entry = {e['name']: e for e in _table()['objects']}['target']
    assert entry['grasp_offset']['position'][2] > 0.0


def test_the_task_is_registered_for_franka_only():
    assert 'tag_reach' in available_tasks('franka')
    assert 'tag_reach' not in available_tasks('ur5e')
    assert 'tag_reach' not in available_tasks('openarm')


def test_the_tree_builds_and_wires_the_detection_into_the_motion():
    from cho_task_manager.utils.controller_names import load_robot_config
    tree = build_task_tree('tag_reach', load_robot_config('franka'))

    by_name = {node.name: node for node in tree.iterate()}
    assert 'Detect_Tag' in by_name
    assert 'Move_To_Tag_Standoff' in by_name

    # The task-space action server only answers while its controller is
    # active, and home_subtree leaves joint impedance running -- so the switch
    # has to be between the detection and the motion.
    assert 'Switch_To_Task_Space' in by_name
    reach = by_name['2_Reach_Detected_Tag']
    order = [child.name for child in reach.children]
    assert order == ['Detect_Tag', 'Switch_To_Task_Space', 'Move_To_Tag_Standoff']

    detect = by_name['Detect_Tag']
    move = by_name['Move_To_Tag_Standoff']
    # Producer and consumer must agree on the key AND its namespace, or the
    # motion leaf reads an entry nobody ever writes.
    assert detect.record_as == tag_reach.TARGET_POSE_KEY
    assert move.target_key is not None
    assert move.target_key.key == tag_reach.TARGET_POSE_KEY
    assert detect.namespace == TASK_NAMESPACE == move.target_key.namespace


def test_the_detection_frame_comes_from_the_registry_not_a_literal():
    from cho_task_manager.utils.controller_names import load_robot_config
    config = load_robot_config('franka')
    tree = build_task_tree('tag_reach', config)
    detect = {node.name: node for node in tree.iterate()}['Detect_Tag']
    # arm_base_link, not base_frame: the latter is 'world' for Franka and does
    # not exist in the published TF tree.
    assert detect.required_frame == config['arm_base_link']


@pytest.mark.parametrize('task', ['tag_reach'])
def test_the_task_declares_the_control_mode_it_needs(task):
    assert tag_reach.CONTROL_MODE == 'torque'
