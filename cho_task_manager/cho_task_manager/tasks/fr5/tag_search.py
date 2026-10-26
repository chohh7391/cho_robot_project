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

r"""Find a tag the standing cameras do not see well, with the wrist camera.

The example of what cho_object_pose's camera ``priority`` and its
``/perception/object_visibility`` topic are for. A standing camera is the one
most likely to lose an object -- the arm stands in its way, or the tag is too
far for a good decode -- and the arm's own camera can go and look. The tree:

1. homes on the hold controller;
2. asks whether any camera already sees the object with a decode margin of at
   least ``min_decision_margin``, and if one does, skips the search;
3. otherwise moves the wrist camera through ``viewpoints`` in order and waits
   ``dwell_sec`` at each for that camera to see the object that well;
4. latches the pose cho_object_pose then publishes -- the wrist camera's: its
   higher priority replaces the standing cameras' samples while it sees the
   tag -- and homes again. The pose stops being published once the arm leaves
   the viewpoint; the blackboard keeps it (``FOUND_POSE_KEY``).

A search that runs out of viewpoints fails, and ``guarded_mission`` ends it on
the hold controller.

The default viewpoints are written for cho_object_pose's fake scene
(``config/fake_scene.yaml``): the beaker's tag at (0.560, 0.175, -0.014) in
base_link, and the wrist camera where cho_bringup_fr5's
``config/real/camera_extrinsics.yaml`` mounts it. In order, they get the three
answers a camera gives:

* viewpoint 1 looks down at the bench beside the tag: NOT_IN_FRAME, the tag
  43 deg off the optical axis, outside the 29 deg half-cone;
* viewpoint 2 is aimed at the tag from 0.62 m: OK, so a pose is published, but
  the decode margin is 50.6 and the task holds out for 55;
* viewpoint 3 is 0.50 m from it on the same line of sight: margin 62.8, found.

test_tag_search.py works these out from the description, the extrinsic and the
scene, so a change to any of them that breaks the story fails there.

In MuJoCo against the fake detector, with the standing camera side_1 blind to
the beaker (side_2 sees it at 23.6 px, under cho_object_pose's 25 px edge gate):

    ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py
    ros2 launch cho_bringup_fr5 camera_extrinsics.launch.py
    ros2 run cho_object_pose fake_detections --ros-args -p use_sim_time:=true -p blind:="['side_1:0']"
    ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=fr5 task:=tag_search use_sim_time:=true \
        object_pose_config:=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_search.yaml \
        object_pose_cameras_config:=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml

Without ``blind``, side_1 sees the beaker from home at margin 59, and the tree
latches that pose without moving the camera.
"""

import math

import py_trees

from cho_task_manager.behaviors.action import JointSpaceActionBehavior
from cho_task_manager.behaviors.topic import ObjectVisibilityBehavior, PoseTargetBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.tasks.fr5.common import CONTROL_MODE, home_joint_state
from cho_task_manager.utils.controller_names import arm_joint_names, load_robot_config
from cho_task_manager.utils.msg_utils import make_joint_state

TASK_NAME = 'tag_search'

#: Where the object's pose is published: cho_object_pose's default topic for
#: an object, which config/perception/tag_search.yaml spells out for every
#: entry (test_tag_search.py checks).
POSE_TOPIC_PREFIX = '/perception/object_pose/'

#: Blackboard key the found pose is latched into, under the /task namespace.
FOUND_POSE_KEY = 'found_pose'

#: FR5 joints j1..j6 [rad], solved for the fake scene's beaker tag: where the
#: wrist camera is in base_link, and what it is pointed at.
VIEWPOINTS = [
    # camera (0.350, -0.200, 0.450), looking straight down at the bench beside the tag
    [-0.2097, -1.5282, -1.3286, -1.9945, 1.5433, -0.2931],
    # camera (0.310, 0.097, 0.548), aimed at the tag from 0.62 m, 25 deg off vertical
    [0.6589, -1.2121, -1.3791, -1.8433, 1.4710, 0.2567],
    # camera (0.358, 0.112, 0.439), 0.50 m from the tag on the same line of sight
    [0.6062, -1.2290, -1.6699, -1.5309, 1.4856, 0.2059],
]

#: What task_params may set, and its value when it does not.
DEFAULTS = {
    'object': 'beaker',            # an object in the table cho_object_pose was given
    'camera': 'wrist',             # the camera the viewpoints aim (its cameras.yaml name)
    'min_decision_margin': 55.0,   # what counts as seen well enough
    'look_sec': 5.0,               # [s] the look from home, before anything moves
    'dwell_sec': 3.0,              # [s] the look from each viewpoint
    'move_duration': 4.0,          # [s] per move, a minimum (cho_interfaces/CONTRACT.md)
    'viewpoints': VIEWPOINTS,      # joint positions [rad], one list per viewpoint
}

HOME_DURATION_SEC = 4.0

#: How long the latch waits for the pose once a camera sees the object well:
#: cho_object_pose publishes within a window of the samples arriving.
LATCH_TIMEOUT_SEC = 3.0


def tag_search_params(robot_config):
    """The task's parameters: DEFAULTS, overridden by ``robot_config['task_params']``.

    Raises ValueError, which the node reports before anything moves, for a
    parameter it does not take or a value it cannot run.
    """
    given = robot_config['task_params']
    unknown = sorted(set(given) - set(DEFAULTS))
    if unknown:
        raise ValueError(f'{TASK_NAME} does not take {unknown}; it takes {sorted(DEFAULTS)}')
    params = {**DEFAULTS, **given}
    for key in ('object', 'camera'):
        if not isinstance(params[key], str) or not params[key]:
            raise ValueError(f'{TASK_NAME}: {key} must be a non-empty name, got {params[key]!r}')
    for key in ('min_decision_margin', 'look_sec', 'dwell_sec', 'move_duration'):
        value = params[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0.0:
            raise ValueError(f'{TASK_NAME}: {key} must be a number > 0, got {value!r}')
    joints = len(arm_joint_names(robot_config))
    viewpoints = params['viewpoints']
    if not isinstance(viewpoints, list) or not viewpoints:
        raise ValueError(f'{TASK_NAME}: viewpoints must be a non-empty list, got {viewpoints!r}')
    for index, viewpoint in enumerate(viewpoints, start=1):
        if (not isinstance(viewpoint, list) or len(viewpoint) != joints
                or not all(isinstance(q, (int, float)) and not isinstance(q, bool)
                           and math.isfinite(q) for q in viewpoint)):
            raise ValueError(
                f'{TASK_NAME}: viewpoint {index} must be {joints} finite joint positions, '
                f'got {viewpoint!r}')
    return params


def create_fr5_tag_search_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Home, look, search the viewpoints until the camera sees the object well, latch it, home."""
    robot_config = robot_config or load_robot_config('fr5')
    params = tag_search_params(robot_config)
    hold = robot_config['joint_space']
    home = home_joint_state(robot_config)
    names = arm_joint_names(robot_config)
    target = params['object']

    def look(name, camera, timeout_sec):
        return ObjectVisibilityBehavior(
            name=name,
            object_name=target,
            min_decision_margin=params['min_decision_margin'],
            timeout_sec=timeout_sec,
            camera=camera,
        )

    # memory=True: a viewpoint that did not see the object is not revisited.
    search = py_trees.composites.Selector(name=f'2_Find_{target}', memory=True)
    search.add_child(look('Seen_From_Home', '', params['look_sec']))
    for index, viewpoint in enumerate(params['viewpoints'], start=1):
        step = py_trees.composites.Sequence(name=f'Viewpoint_{index}', memory=True)
        step.add_children([
            JointSpaceActionBehavior(
                name=f'Move_To_Viewpoint_{index}',
                target_joints=make_joint_state(viewpoint, names),
                controller_name=hold,
                duration=params['move_duration'],
                joint_names=names,
            ),
            look(f'Seen_From_Viewpoint_{index}', params['camera'], params['dwell_sec']),
        ])
        search.add_child(step)

    mission = py_trees.composites.Sequence(name='FR5_Tag_Search_Sequence', memory=True)
    mission.add_children([
        home_subtree(robot_config, target_joints=home, controller=hold,
                     duration=HOME_DURATION_SEC, open_gripper=False),
        search,
        PoseTargetBehavior(
            name=f'Latch_{target}',
            record_as=FOUND_POSE_KEY,
            topic=POSE_TOPIC_PREFIX + target,
            required_frame=robot_config['arm_base_link'],
            timeout_sec=LATCH_TIMEOUT_SEC,
        ),
        home_subtree(robot_config, target_joints=home, controller=hold,
                     duration=HOME_DURATION_SEC, name='3_Finish', suffix='_Final',
                     open_gripper=False),
    ])
    return guarded_mission(mission, robot_config, CONTROL_MODE, name='FR5_Tag_Search_Root')
