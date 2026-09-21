"""The simulated bench's visibility model, and the file that describes one.

No ROS, no camera, no arm. What is worth testing is the handful of decisions
that make a fake detector useful rather than merely quiet: whether a tag is in
the picture, whether something is standing in the way, and whether a scenario
someone typed matches the scene they typed it against.
"""

import math
import os

from cho_object_pose import fake_scene
import numpy as np
import pytest
import yaml

IDENTITY = np.eye(3)


def _document(**overrides):
    document = {
        'base_frame': 'base_link',
        'tag_size_m': 0.039,
        'focal_px': 447.0,
        'margin_per_px': 1.45,
        'blocker_radius_m': 0.10,
        'blockers': ['forearm_link'],
        'cameras': [{'name': 'wrist', 'frame': 'wrist_link', 'frame_prefix': 'wrist_',
                     'detections_topic': '/wrist/detections', 'fov_deg': 50.0,
                     'max_range_m': 1.0}],
        'tags': [{'id': 0, 'name': 'beaker', 'xyz': [1.0, 0.0, 0.0]}],
    }
    document.update(overrides)
    return document


def _scene(**overrides):
    return fake_scene.parse_scene(_document(**overrides))


# ------------------------------------------------------------ apparent size

def test_a_tag_shrinks_with_range_the_way_the_gate_reads_it():
    # This is the number cho_object_pose's min_edge_px gates on, so it has to
    # be the real relation and not a fudge: 39 mm at 447 px focal is about
    # 39 px at 0.45 m, which is the range the shipped sweep raster works at.
    assert fake_scene.apparent_edge_px(0.039, 447.0, 0.45) == pytest.approx(38.7, abs=0.2)
    assert fake_scene.apparent_edge_px(0.039, 447.0, 0.62) == pytest.approx(28.1, abs=0.2)
    far = fake_scene.apparent_edge_px(0.039, 447.0, 1.0)
    near = fake_scene.apparent_edge_px(0.039, 447.0, 0.5)
    assert near == pytest.approx(2 * far)


# ------------------------------------------------------------ field of view

def test_a_tag_on_the_optical_axis_is_seen():
    scene = _scene()
    seen = fake_scene.sight(scene, scene.cameras[0], np.zeros(3), IDENTITY,
                            np.array([0.5, 0.0, 0.0]))
    assert seen.visible and seen.reason is None
    assert seen.range_m == pytest.approx(0.5)
    assert seen.decision_margin == pytest.approx(seen.edge_px * scene.margin_per_px)


def test_a_tag_outside_the_cone_is_not():
    scene = _scene()
    # 45 degrees off, against a 50 degree full cone (25 degree half-angle).
    seen = fake_scene.sight(scene, scene.cameras[0], np.zeros(3), IDENTITY,
                            np.array([0.5, 0.5, 0.0]))
    assert not seen.visible
    assert 'off axis' in seen.reason


def test_a_camera_with_no_fov_is_not_gated_on_one():
    # The honest setting for a wide lens pointed at a whole bench: a cone would
    # only add a number nobody measured.
    document = _document()
    del document['cameras'][0]['fov_deg']
    document['cameras'][0]['max_range_m'] = 3.0
    scene = fake_scene.parse_scene(document)
    assert scene.cameras[0].half_fov_rad is None
    seen = fake_scene.sight(scene, scene.cameras[0], np.zeros(3), IDENTITY,
                            np.array([0.0, 1.0, 0.0]))
    assert seen.visible


def test_range_is_checked_before_anything_else():
    scene = _scene()
    seen = fake_scene.sight(scene, scene.cameras[0], np.zeros(3), IDENTITY,
                            np.array([2.0, 0.0, 0.0]))
    assert not seen.visible and 'out of range' in seen.reason


def test_the_optical_axis_turns_with_the_camera():
    # The wrist camera is carried by the arm; a model that ignored its
    # orientation would see through the back of its own head.
    scene = _scene()
    # Rotate the camera 90 degrees about z, so its +x points along base +y.
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert not fake_scene.sight(scene, scene.cameras[0], np.zeros(3), rotation,
                                np.array([0.5, 0.0, 0.0])).visible
    assert fake_scene.sight(scene, scene.cameras[0], np.zeros(3), rotation,
                            np.array([0.0, 0.5, 0.0])).visible


# --------------------------------------------------------------- occlusion

def test_something_on_the_line_of_sight_blocks_it():
    # The failure the whole recovery exists for, and here it comes out of the
    # arm's actual configuration rather than a flag.
    eye, tag = np.zeros(3), np.array([1.0, 0.0, 0.0])
    assert fake_scene.blocking_point(eye, tag, [np.array([0.5, 0.05, 0.0])], 0.1) == 0
    assert fake_scene.blocking_point(eye, tag, [np.array([0.5, 0.5, 0.0])], 0.1) is None


def test_the_camera_and_the_tag_never_block_their_own_view():
    # A link at the very start of the segment is the camera's own mount, and
    # one at the very end is what the tag is stuck to. Either would make every
    # camera permanently blind.
    eye, tag = np.zeros(3), np.array([1.0, 0.0, 0.0])
    assert fake_scene.blocking_point(eye, tag, [eye], 0.1) is None
    assert fake_scene.blocking_point(eye, tag, [tag], 0.1) is None


def test_a_blocker_beyond_the_tag_does_not_block():
    eye, tag = np.zeros(3), np.array([1.0, 0.0, 0.0])
    assert fake_scene.blocking_point(eye, tag, [np.array([1.5, 0.0, 0.0])], 0.1) is None


def test_the_blocking_link_is_named_in_the_reason():
    # So a run that sees nothing says which part of the arm is in the way.
    scene = _scene()
    seen = fake_scene.sight(scene, scene.cameras[0], np.zeros(3), IDENTITY,
                            np.array([1.0, 0.0, 0.0]),
                            blockers=[np.array([0.5, 0.02, 0.0])])
    assert not seen.visible and 'forearm_link' in seen.reason


# ----------------------------------------------------------------- parsing

def test_two_cameras_may_not_share_a_tag_frame_prefix():
    # The same rule the real camera table enforces, for the same reason: one
    # TF child would gain two parents.
    document = _document()
    document['cameras'].append(dict(document['cameras'][0], name='oak',
                                    frame='oak_link',
                                    detections_topic='/oak/detections'))
    with pytest.raises(ValueError, match='prefixes'):
        fake_scene.parse_scene(document)


def test_the_same_tag_may_not_appear_twice():
    document = _document()
    document['tags'].append(dict(document['tags'][0]))
    with pytest.raises(ValueError, match='twice'):
        fake_scene.parse_scene(document)


def test_a_tag_needs_three_finite_numbers():
    with pytest.raises(ValueError, match='xyz'):
        fake_scene.parse_scene(_document(tags=[{'id': 0, 'xyz': [1.0, 0.0]}]))
    with pytest.raises(ValueError, match='xyz'):
        fake_scene.parse_scene(
            _document(tags=[{'id': 0, 'xyz': [1.0, float('nan'), 0.0]}]))


def test_a_scenario_is_checked_against_the_scene_it_names():
    # A typo here silently produces the run you were not trying to test.
    scene = _scene()
    assert fake_scene.parse_blind(['wrist:0'], scene) == {('wrist', 0)}
    with pytest.raises(ValueError, match='no camera'):
        fake_scene.parse_blind(['oak:0'], scene)
    with pytest.raises(ValueError, match='no tag'):
        fake_scene.parse_blind(['wrist:7'], scene)
    with pytest.raises(ValueError, match='<camera>:<tag id>'):
        fake_scene.parse_blind(['wrist'], scene)


# ------------------------------------------------------------ the FR5 bench

def _shipped():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'fake_scene.yaml')
    with open(path, encoding='utf-8') as stream:
        return fake_scene.parse_scene(yaml.safe_load(stream))


def test_the_shipped_scene_matches_the_camera_table_it_stands_in_for():
    # The fake detectors have to publish on the topics and tag prefixes the real
    # pose node subscribes to and looks up, or the whole harness is quietly
    # testing nothing.
    from cho_object_pose.cameras import parse_cameras
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'cameras.yaml')
    with open(path, encoding='utf-8') as stream:
        real = {camera.name: camera for camera in parse_cameras(yaml.safe_load(stream))}
    for camera in _shipped().cameras:
        assert camera.name in real
        assert camera.detections_topic == real[camera.name].detections_topic
        assert camera.frame_prefix == real[camera.name].frame_prefix


def test_the_shipped_scene_walks_the_fr5_raster_without_satisfying_it_at_once():
    # What margin_per_px is FOR. cho_task_manager's fr5_bench raster asks for a
    # decode margin of 55; the harness has to FAIL the far row (about 0.59 m to
    # the tag) and PASS in the middle one (about 0.55 m), or the sweep either
    # succeeds on its first waypoint and the raster is never exercised, or
    # never succeeds at all and only the failure path is.
    scene = _shipped()

    def margin(range_m):
        return fake_scene.apparent_edge_px(
            scene.tag_size_m, scene.focal_px, range_m) * scene.margin_per_px

    assert margin(0.59) < 55.0
    assert margin(0.55) >= 55.0
    # And the far row still clears cho_object_pose's own min_edge_px of 25, so
    # 'too far to decode at all' and 'not good enough yet' stay distinguishable
    # -- they are a sweep aimed wrong and a sweep that has to come closer.
    assert fake_scene.apparent_edge_px(scene.tag_size_m, scene.focal_px, 0.59) > 25.0


def test_the_harness_cone_is_the_cameras_shorter_angle():
    # Nothing constrains the image roll, so the angle guaranteed to be in frame
    # is the short one. A cone at the long one would have the harness see tags
    # a real run would miss -- the one direction a test must not be wrong in.
    cameras = {camera.name: camera for camera in _shipped().cameras}
    assert cameras['wrist'].half_fov_rad == pytest.approx(math.radians(29.0))


def test_the_shipped_scene_puts_the_tags_where_the_object_table_expects_them():
    # The tag is not at the vessel: it is on a stalk beside it, and the object
    # table records that as a negative x offset. Get this backwards and the
    # whole pipeline publishes a confident pose 140 mm from the glass, which
    # nothing downstream can see. The check is that tag + offset lands on the
    # vessel position the cell layout declares.
    scene = _shipped()
    for name, vessel_x, stalk in (('beaker', 0.49, 0.070), ('flask', 0.48383, 0.090)):
        tag = next(entry for entry in scene.tags if entry.name == name)
        assert tag.position[0] - stalk == pytest.approx(vessel_x, abs=1e-3)


def test_the_shipped_scene_and_the_sweep_raster_aim_at_the_same_tags():
    # The raster looks where this scene puts the tag. Two files with two
    # opinions about that is a sweep that drives twelve waypoints past the
    # thing it went to find.
    raster = pytest.importorskip('yaml')
    share = pytest.importorskip(
        'ament_index_python.packages').get_package_share_directory
    path = os.path.join(share('cho_task_manager'), 'config', 'sweep',
                        'fr5_bench.raster.yaml')
    with open(path, encoding='utf-8') as stream:
        aimed = {entry['object']: entry['tag_xy']
                 for entry in raster.safe_load(stream)['objects']}
    for tag in _shipped().tags:
        assert tag.name in aimed
        assert tag.position[0] == pytest.approx(aimed[tag.name][0], abs=1e-3)
        assert tag.position[1] == pytest.approx(aimed[tag.name][1], abs=1e-3)


def test_the_wrist_is_cone_gated_and_the_oak_is_not():
    cameras = {camera.name: camera for camera in _shipped().cameras}
    assert cameras['wrist'].half_fov_rad is not None
    assert cameras['oak'].half_fov_rad is None
