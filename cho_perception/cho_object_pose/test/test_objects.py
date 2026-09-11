"""Unit tests for the tag-to-object table."""

from cho_object_pose.objects import parse_objects
import numpy as np
import pytest


def document(**overrides):
    entry = {'name': 'cube', 'tag_id': 9}
    entry.update(overrides)
    return {'objects': [entry]}


def test_defaults_fill_in_a_minimal_entry():
    spec, = parse_objects(document())
    assert spec.topic == '/perception/object_pose/cube'
    assert np.allclose(spec.offset_position, [0.0, 0.0, 0.0])
    assert np.allclose(spec.offset_orientation, [0.0, 0.0, 0.0, 1.0])
    assert spec.top_down_yaw is True
    assert np.allclose(spec.yaw_axis, [1.0, 0.0, 0.0])


def test_grasp_offset_is_read_and_the_quaternion_normalised():
    spec, = parse_objects(document(grasp_offset={
        'position': [0.0, 0.0, -0.03], 'orientation': [0.0, 0.0, 0.0, 2.0]}))
    assert np.allclose(spec.offset_position, [0.0, 0.0, -0.03])
    assert np.allclose(spec.offset_orientation, [0.0, 0.0, 0.0, 1.0])


def test_yaw_axis_is_normalised():
    spec, = parse_objects(document(yaw_axis=[0.0, 5.0, 0.0]))
    assert np.allclose(spec.yaw_axis, [0.0, 1.0, 0.0])


@pytest.mark.parametrize('entry', [
    {'tag_id': 9},
    {'name': 'cube'},
    {'name': 'cube', 'tag_id': -1},
    {'name': 'cube', 'tag_id': True},
    {'name': 'cube', 'tag_id': 9, 'top_down_yaw': 'yes'},
    {'name': 'cube', 'tag_id': 9, 'yaw_axis': [0.0, 0.0, 0.0]},
    {'name': 'cube', 'tag_id': 9, 'grasp_offset': {'position': [0.0, 0.0]}},
    {'name': 'cube', 'tag_id': 9, 'grasp_offset': {'position': [0.0, 0.0, float('nan')]}},
])
def test_malformed_entries_are_rejected(entry):
    with pytest.raises(ValueError):
        parse_objects({'objects': [entry]})


def test_an_empty_table_is_rejected():
    with pytest.raises(ValueError):
        parse_objects({'objects': []})


def test_two_objects_on_one_tag_are_rejected():
    with pytest.raises(ValueError, match='tag ids'):
        parse_objects({'objects': [{'name': 'a', 'tag_id': 9},
                                   {'name': 'b', 'tag_id': 9}]})


def test_two_objects_on_one_topic_are_rejected():
    with pytest.raises(ValueError, match='topics'):
        parse_objects({'objects': [{'name': 'a', 'tag_id': 9, 'topic': '/same'},
                                   {'name': 'b', 'tag_id': 10, 'topic': '/same'}]})
