# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The wrist-camera search, and the scene its default viewpoints were written for.

What each viewpoint shows is worked out here rather than taken from the
docstring: the FR5 description (xacro, pinocchio) puts the wrist camera where
cho_bringup_fr5's measured extrinsic mounts it, and cho_object_pose's fake
scene says what that camera makes of the tag from there -- the same geometry
``fake_detections`` runs in MuJoCo.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cho_interfaces.msg import CameraVisibility, ObjectVisibility
from cho_object_pose import fake_scene
import numpy as np
import pinocchio as pin
import pytest
import xacro
import yaml

from cho_task_manager.behaviors.topic import ObjectVisibilityBehavior, PoseTargetBehavior
from cho_task_manager.behaviors.topic.object_visibility import describe, seen_well_enough
from cho_task_manager.tasks import available_tasks
from cho_task_manager.tasks.fr5 import tag_search
from cho_task_manager.testing import goal_problems, registry_tree
from cho_task_manager.utils.controller_names import load_robot_config, task_home_positions

TASK = 'tag_search'
TABLE = Path(__file__).resolve().parents[1] / 'config' / 'perception' / 'tag_search.yaml'
#: Read from the source tree, as test_goal_frames reads the bringups.
EXTRINSICS = (Path(__file__).resolve().parents[2] / 'cho_bringup' / 'cho_bringup_fr5'
              / 'config' / 'real' / 'camera_extrinsics.yaml')

#: cho_object_pose's own gate (node.py, min_decision_margin and min_edge_px):
#: a view that clears both is OK on the visibility topic and publishes a pose.
NODE_MIN_MARGIN = 35.0
NODE_MIN_EDGE_PX = 25.0

#: The vessels fake_scene.yaml puts the tags beside, in base_link.
VESSELS_XY = {'beaker': (0.490, 0.17475), 'flask': (0.48383, 0.33166)}


def _table():
    with open(TABLE, encoding='utf-8') as stream:
        return {entry['name']: entry for entry in yaml.safe_load(stream)['objects']}


def _scene():
    path = os.path.join(get_package_share_directory('cho_object_pose'), 'config', 'fake_scene.yaml')
    with open(path, encoding='utf-8') as stream:
        return fake_scene.parse_scene(yaml.safe_load(stream))


# ------------------------------------------------------------ the task

def test_the_task_is_registered_for_fr5_only():
    assert TASK in available_tasks('fr5')
    for robot in ('franka', 'ur5e', 'openarm'):
        assert TASK not in available_tasks(robot)


def test_every_goal_names_the_registry_joints():
    tree, config = registry_tree(TASK, 'fr5')
    assert goal_problems(tree, config) == []


def test_the_search_looks_from_home_first_then_from_each_viewpoint_in_order():
    tree, _config = registry_tree(TASK, 'fr5')
    by_name = {node.name: node for node in tree.iterate()}
    search = by_name['2_Find_beaker']
    assert [child.name for child in search.children] == (
        ['Seen_From_Home'] + [f'Viewpoint_{i}' for i in (1, 2, 3)])
    home_look = by_name['Seen_From_Home']
    assert isinstance(home_look, ObjectVisibilityBehavior)
    assert home_look.camera == ''      # any camera will do from home
    for index in (1, 2, 3):
        look = by_name[f'Seen_From_Viewpoint_{index}']
        assert look.camera == 'wrist'  # from a viewpoint, only the camera it aims
        assert look.min_decision_margin == home_look.min_decision_margin == 55.0


def test_the_latch_reads_the_topic_the_table_publishes():
    tree, _config = registry_tree(TASK, 'fr5')
    latch = next(node for node in tree.iterate() if isinstance(node, PoseTargetBehavior))
    assert latch.topic == _table()['beaker']['topic']
    assert latch.record_as == tag_search.FOUND_POSE_KEY
    assert latch.required_frame == 'base_link'


def test_task_params_choose_the_object_and_the_viewpoints():
    viewpoints = tag_search.VIEWPOINTS[1:]
    tree, config = registry_tree(TASK, 'fr5', task_params={'object': 'flask', 'viewpoints': viewpoints})
    names = [node.name for node in tree.iterate()]
    assert '2_Find_flask' in names and 'Latch_flask' in names
    assert 'Viewpoint_2' in names and 'Viewpoint_3' not in names
    assert goal_problems(tree, config) == []


@pytest.mark.parametrize('task_params,message', [
    ({'objects': 'beaker'}, "does not take \\['objects'\\]"),
    ({'object': ''}, 'object must be a non-empty name'),
    ({'min_decision_margin': 0.0}, 'min_decision_margin must be a number > 0'),
    ({'dwell_sec': True}, 'dwell_sec must be a number > 0'),
    ({'viewpoints': []}, 'viewpoints must be a non-empty list'),
    ({'viewpoints': [[0.0] * 5]}, 'viewpoint 1 must be 6 finite joint positions'),
    ({'viewpoints': [[0.0] * 5 + [float('nan')]]}, 'viewpoint 1 must be 6 finite'),
])
def test_a_parameter_it_cannot_run_is_refused_before_anything_moves(task_params, message):
    with pytest.raises(ValueError, match=message):
        registry_tree(TASK, 'fr5', task_params=task_params)


# ------------------------------------------------------------ the visibility wait

def _entry(*views):
    entry = ObjectVisibility(name='beaker')
    for camera, state, margin in views:
        entry.cameras.append(CameraVisibility(
            camera=camera, state=state, decision_margin=margin, edge_px=margin / 1.8))
    return entry


def test_a_view_counts_only_when_ok_and_decoded_well_enough():
    ok, rejected = CameraVisibility.STATE_OK, CameraVisibility.STATE_REJECTED
    entry = _entry(('side_2', rejected, 42.5), ('wrist', ok, 50.6))
    assert seen_well_enough(entry, '', 55.0) is None
    assert seen_well_enough(entry, '', 50.0).camera == 'wrist'
    # A rejected view never counts, however it scored.
    assert seen_well_enough(_entry(('side_2', rejected, 80.0)), '', 55.0) is None
    # Asked about one camera, another camera's good view does not answer.
    entry = _entry(('side_1', ok, 59.2), ('wrist', CameraVisibility.STATE_NOT_IN_FRAME, -1.0))
    assert seen_well_enough(entry, 'wrist', 55.0) is None
    assert seen_well_enough(entry, '', 55.0).camera == 'side_1'


def test_the_log_line_says_what_each_camera_saw():
    entry = _entry(('side_1', CameraVisibility.STATE_NOT_IN_FRAME, -1.0),
                   ('side_2', CameraVisibility.STATE_REJECTED, 42.5))
    assert describe(entry) == 'side_1 NOT_IN_FRAME, side_2 REJECTED (margin 42.5, edge 23.6 px)'


# ------------------------------------------------------------ the scene

def test_the_table_names_the_fake_scenes_tags():
    scene = _scene()
    table = _table()
    assert {tag.name: tag.id for tag in scene.tags} == {
        name: entry['tag_id'] for name, entry in table.items()}
    for entry in table.values():
        assert entry['tag_size'] == pytest.approx(scene.tag_size_m)


def test_the_published_pose_is_over_the_vessel_not_the_tag():
    # The tag stands on a stalk beside its vessel. Get the offset's sign wrong
    # and the pipeline publishes a confident pose 140-180 mm from the glass.
    table = _table()
    for tag in _scene().tags:
        offset = table[tag.name]['grasp_offset']['position']
        assert tag.position[0] + offset[0] == pytest.approx(VESSELS_XY[tag.name][0], abs=1e-3)
        assert tag.position[1] + offset[1] == pytest.approx(VESSELS_XY[tag.name][1], abs=1e-3)


@pytest.fixture(scope='module')
def sights():
    """What each camera makes of the beaker's tag at home and at each viewpoint."""
    scene = _scene()
    path = os.path.join(get_package_share_directory('cho_description_fr5'), 'urdf', 'fr5.urdf.xacro')
    model = pin.buildModelFromXML(xacro.process_file(path, mappings={'hardware': 'mujoco'}).toxml())
    with open(EXTRINSICS, encoding='utf-8') as stream:
        for mount in yaml.safe_load(stream)['transforms']:
            qx, qy, qz, qw = mount['quaternion']
            parent = model.getFrameId(mount['parent_frame'])
            placement = pin.SE3(pin.Quaternion(qw, qx, qy, qz).normalized().matrix(),
                                np.array(mount['xyz'], dtype=float))
            model.addFrame(pin.Frame(mount['child_frame'], model.frames[parent].parentJoint, parent,
                                     model.frames[parent].placement * placement, pin.FrameType.OP_FRAME))
    data = model.createData()
    tag = next(entry for entry in scene.tags if entry.name == 'beaker')
    poses = {'home': task_home_positions(load_robot_config('fr5'))}
    poses.update({f'viewpoint_{i}': q for i, q in enumerate(tag_search.VIEWPOINTS, start=1)})

    result = {}
    for pose, q in poses.items():
        pin.framesForwardKinematics(model, data, np.array(q, dtype=float))
        base = data.oMf[model.getFrameId('base_link')].inverse()
        # A blocker the description does not have (the gripper's links on a
        # gripper-less build) blocks nothing, as in fake_detections.
        blockers = [(base * data.oMf[model.getFrameId(name)]).translation if model.existFrame(name)
                    else np.full(3, 1e6) for name in scene.blockers]
        for camera in scene.cameras:
            eye = base * data.oMf[model.getFrameId(camera.frame)]
            result[pose, camera.name] = fake_scene.sight(
                scene, camera, eye.translation, eye.rotation, tag.position, blockers)
    return result


def _publishes(seen):
    return seen.visible and seen.edge_px >= NODE_MIN_EDGE_PX and seen.decision_margin >= NODE_MIN_MARGIN


def test_from_home_only_the_standing_camera_side_1_sees_the_beaker_well(sights):
    # Which is why the run blinds side_1: without that, the look from home
    # succeeds and nothing moves.
    margin = tag_search.DEFAULTS['min_decision_margin']
    assert not sights['home', 'wrist'].visible
    assert _publishes(sights['home', 'side_1'])
    assert sights['home', 'side_1'].decision_margin >= margin
    # side_2 sees it, but too small for cho_object_pose's edge gate: REJECTED.
    assert sights['home', 'side_2'].visible and not _publishes(sights['home', 'side_2'])


def test_the_viewpoints_walk_not_in_frame_then_not_good_enough_then_found(sights):
    margin = tag_search.DEFAULTS['min_decision_margin']
    first, second, third = (sights[f'viewpoint_{i}', 'wrist'] for i in (1, 2, 3))
    assert not first.visible and 'off axis' in first.reason
    # cho_object_pose publishes this one; the task holds out for its margin.
    assert _publishes(second) and second.decision_margin < margin
    assert third.visible and third.decision_margin >= margin
    # Far enough on either side of the threshold that MuJoCo's servo sag
    # (about 20 mrad on j2, 1 cm at the camera) does not change the answer.
    assert second.decision_margin < margin - 3.0
    assert third.decision_margin > margin + 5.0
