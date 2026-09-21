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


# ---------------------------------------------------------------- display shape

def _entry_with_shape(**shape):
    return {'objects': [{'name': 'beaker', 'tag_id': 0, 'shape': shape}]}


def test_an_object_without_a_shape_is_simply_not_drawn():
    spec, = parse_objects({'objects': [{'name': 'beaker', 'tag_id': 0}]})
    assert spec.shape is None


def test_a_shape_carries_its_size_origin_and_colour():
    spec, = parse_objects(_entry_with_shape(
        type='cylinder', size=[0.05, 0.05, 0.07], origin=[0.0, 0.0, -0.115],
        color=[0.2, 0.7, 1.0, 0.45]))
    assert spec.shape.type == 'cylinder'
    assert list(spec.shape.size) == [0.05, 0.05, 0.07]
    # Where the body sits relative to the PUBLISHED pose, which is a standoff
    # above it -- so a negative z is the normal case, not a mistake.
    assert spec.shape.origin[2] < 0.0


def test_a_shape_without_an_origin_sits_on_the_published_pose():
    spec, = parse_objects(_entry_with_shape(type='sphere', size=[0.08, 0.08, 0.08]))
    assert list(spec.shape.origin) == [0.0, 0.0, 0.0]


def test_an_unknown_shape_type_is_rejected():
    with pytest.raises(ValueError, match='shape.type'):
        parse_objects(_entry_with_shape(type='teapot', size=[0.05, 0.05, 0.07]))


def test_a_shape_needs_three_positive_extents():
    for size in ([0.05, 0.05], [0.05, 0.0, 0.07], [0.05, -0.05, 0.07]):
        with pytest.raises(ValueError):
            parse_objects(_entry_with_shape(type='box', size=size))


def test_a_shape_colour_must_be_four_unit_values():
    with pytest.raises(ValueError, match='shape.color'):
        parse_objects(_entry_with_shape(
            type='box', size=[0.05, 0.05, 0.07], color=[255, 0, 0, 1]))


def test_a_mesh_shape_carries_the_resource_rviz_has_to_load():
    spec, = parse_objects(_entry_with_shape(
        type='mesh', resource='package://cho_task_manager/meshes/beaker.stl',
        size=[0.05, 0.05, 0.07]))
    assert spec.shape.type == 'mesh'
    assert spec.shape.resource.endswith('beaker.stl')
    # A mesh is held to the same contract as a primitive: size is the bounding
    # box in metres, not a scale factor.
    assert list(spec.shape.size) == [0.05, 0.05, 0.07]


def test_a_mesh_without_a_resource_is_rejected():
    with pytest.raises(ValueError, match='shape.resource'):
        parse_objects(_entry_with_shape(type='mesh', size=[0.05, 0.05, 0.07]))


def test_a_primitive_may_not_carry_a_mesh_resource():
    """A resource on a cylinder is a type someone forgot to change."""
    with pytest.raises(ValueError, match='shape.resource'):
        parse_objects(_entry_with_shape(
            type='cylinder', size=[0.05, 0.05, 0.07], resource='package://x/y.stl'))
