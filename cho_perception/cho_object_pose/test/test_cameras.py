"""Unit tests for the camera table.

The table is read by two things -- the launch that starts one detector per
camera, and the node that subscribes to all of them -- so what is worth testing
is the part that makes them agree, and the duplicates that would break TF
silently rather than loudly.
"""

from cho_object_pose.cameras import DEFAULT_DETECTIONS_TOPIC, parse_cameras, single_camera
import pytest


def _document(*cameras):
    return {'cameras': list(cameras)}


def _camera(name='side_1', **overrides):
    entry = {'name': name,
             'image_topic': f'/{name}/left/image_raw',
             'frame_prefix': f'{name}_',
             'detections_topic': f'/{name}/detections'}
    entry.update(overrides)
    return entry


def test_a_camera_is_parsed_with_its_four_fields():
    camera, = parse_cameras(_document(_camera()))
    assert camera.name == 'side_1'
    assert camera.frame_prefix == 'side_1_'
    assert camera.detections_topic == '/side_1/detections'
    assert camera.image_topic == '/side_1/left/image_raw'


def test_camera_info_defaults_to_the_images_sibling():
    # image_transport's convention. Spelling it out in every entry would be
    # three more strings that can be typed wrong.
    camera, = parse_cameras(_document(_camera()))
    assert camera.camera_info_topic == '/side_1/left/camera_info'


def test_camera_info_can_be_given_explicitly():
    camera, = parse_cameras(_document(_camera(camera_info_topic='/elsewhere/camera_info')))
    assert camera.camera_info_topic == '/elsewhere/camera_info'


def test_rectify_defaults_to_off_and_must_be_a_bool():
    camera, = parse_cameras(_document(_camera()))
    assert camera.rectify is False
    with pytest.raises(ValueError):
        parse_cameras(_document(_camera(rectify='yes')))


def test_two_cameras_may_not_share_a_frame_prefix():
    # The failure this exists for: both detectors publish tag_0, one TF child
    # gains two parents, and transforms resolve through whichever camera
    # published last. No error anywhere -- just wrong poses, sometimes.
    with pytest.raises(ValueError, match='frame prefixes'):
        parse_cameras(_document(
            _camera('side_1', frame_prefix='cam_'),
            _camera('rs_left', frame_prefix='cam_')))


def test_two_cameras_may_not_share_a_detections_topic():
    with pytest.raises(ValueError, match='detections topics'):
        parse_cameras(_document(
            _camera('side_1', detections_topic='/detections'),
            _camera('rs_left', detections_topic='/detections')))


def test_two_cameras_may_not_share_a_name():
    with pytest.raises(ValueError, match='names'):
        parse_cameras(_document(_camera('side_1'), _camera('side_1', frame_prefix='other_')))


def test_a_camera_without_an_image_topic_is_rejected():
    # The detector launch reads this same file; an entry with no image is a
    # detector started against nothing.
    entry = _camera()
    del entry['image_topic']
    with pytest.raises(ValueError, match='image_topic'):
        parse_cameras(_document(entry))


def test_an_empty_table_is_rejected():
    for document in ({'cameras': []}, {}, [], None):
        with pytest.raises(ValueError):
            parse_cameras(document)


def test_a_camera_without_a_visual_is_simply_not_drawn():
    camera, = parse_cameras(_document(_camera()))
    assert camera.visual is None


def test_a_visual_carries_the_frame_mesh_and_mounting_pose():
    camera, = parse_cameras(_document(_camera(visual={
        'frame': 'side_1_model_origin',
        'mesh': 'package://depthai_descriptions/urdf/models/OAK-D-PRO-W.stl',
        'xyz': [0.0043, -0.0175, 0.0],
        'rpy': [1.5708, 0.0, 1.5708]})))
    assert camera.visual.frame == 'side_1_model_origin'
    assert camera.visual.mesh.endswith('OAK-D-PRO-W.stl')
    assert camera.visual.position == (0.0043, -0.0175, 0.0)
    # Fixed-axis rpy, kept as written so it can be checked against the vendor's
    # xacro rather than against a quaternion.
    assert camera.visual.orientation == (1.5708, 0.0, 1.5708)
    assert camera.visual.scale == (1.0, 1.0, 1.0)


def test_a_visual_needs_a_frame_and_a_mesh():
    for missing in ('frame', 'mesh'):
        visual = {'frame': 'side_1_model_origin', 'mesh': 'package://x/y.stl'}
        del visual[missing]
        with pytest.raises(ValueError, match=f'visual.{missing}'):
            parse_cameras(_document(_camera(visual=visual)))


def test_the_bench_table_ships_three_distinct_cameras_all_drawn():
    import os

    import yaml
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'cameras.yaml')
    cameras = parse_cameras(yaml.safe_load(open(path, encoding='utf-8')))
    # Two standing observers and the wrist recovery instrument. A fourth entry
    # here means a detector nobody started.
    assert {camera.name for camera in cameras} == {'side_1', 'side_2', 'wrist'}
    assert len({camera.frame_prefix for camera in cameras}) == 3
    # The OAK-D's mono streams are unrectified and the D435s' infra1 is not.
    assert {camera.name for camera in cameras if camera.rectify} == {'side_1'}
    # Crossing lines of sight needs to know where each line STARTS, so every
    # camera on this bench declares its optical centre. Without it the node
    # falls back to a weaker rule, and says so rather than failing.
    assert all(camera.optical_frame for camera in cameras)
    # Both are drawn, because a camera in the wrong place is what the picture
    # is for.
    assert all(camera.visual for camera in cameras)


def test_the_single_camera_fallback_is_the_old_behaviour():
    camera, = single_camera()
    assert camera.frame_prefix == ''
    assert camera.detections_topic == DEFAULT_DETECTIONS_TOPIC


def test_priority_defaults_to_zero_so_cameras_are_peers():
    # Every bench that predates this field must keep fusing exactly as it did.
    camera, = parse_cameras(_document(_camera()))
    assert camera.priority == 0


def test_priority_is_read_and_may_be_negative():
    # Only ever compared, never scaled, so nothing turns on the sign.
    high, low = parse_cameras(_document(
        _camera('wrist', priority=10), _camera('bench', priority=-1)))
    assert (high.priority, low.priority) == (10, -1)


def test_priority_must_be_an_integer_and_not_a_bool():
    # `priority: true` is a typo, not a 1 -- and bool is an int in Python, so
    # nothing but an explicit check catches it.
    with pytest.raises(ValueError, match='priority'):
        parse_cameras(_document(_camera(priority=True)))
    with pytest.raises(ValueError, match='priority'):
        parse_cameras(_document(_camera(priority='high')))
    with pytest.raises(ValueError, match='priority'):
        parse_cameras(_document(_camera(priority=1.5)))


def test_the_single_camera_fallback_has_no_priority_contest():
    camera, = single_camera()
    assert camera.priority == 0


def test_the_shipped_table_makes_the_wrist_outrank_both_standing_cameras():
    # The bench's stated role split -- side_1 and side_2 the standing pair,
    # wrist the recovery instrument -- was a comment until this field existed.
    # If these ever come out equal the recovery sweep silently goes back to
    # being averaged into the far views.
    import os
    import yaml
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'cameras.yaml')
    with open(path, encoding='utf-8') as stream:
        cameras = {camera.name: camera for camera in parse_cameras(yaml.safe_load(stream))}
    assert cameras['wrist'].priority > cameras['side_1'].priority
    assert cameras['wrist'].priority > cameras['side_2'].priority
    # And the standing pair are PEERS, or they would not be fused with each
    # other at all -- which is the whole reason there are two of them.
    assert cameras['side_1'].priority == cameras['side_2'].priority
