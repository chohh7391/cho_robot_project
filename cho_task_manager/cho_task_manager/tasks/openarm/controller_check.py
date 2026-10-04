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

"""OpenArm controller smoke check.

Proves the whole chain end to end - bringup, controller_manager, action server,
behaviour tree - for one effort controller. Deliberately smaller than the Franka
equivalent: OpenArm has a single switchable controller so far, so there is
nothing to sweep.

    ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py
    ros2 launch cho_task_manager run_task_manager.launch.py \
        robot_type:=openarm task:=controller_check_torque
"""

import py_trees

from cho_task_manager.behaviors.action import JointSpaceActionBehavior
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_joint_state
from cho_task_manager.tasks.openarm.common import ee_state_names, require_one_arm
from cho_task_manager.utils.controller_names import arm_joint_names
from cho_task_manager.utils.msg_utils import make_joint_state

# The legacy effort controller this check drives only exists on a
# control_mode:=torque bringup, which is also where openarm.yaml's torque hold
# controller lives.
CONTROL_MODE = 'torque'

# The return leg goes to the registry's poses.task_home, which for OpenArm is
# home 0 -- the pose the controller homes to on activation (home_position in the
# bringup controllers.yaml), so the return leg ends where the arm started.
# joint4's lower limit is 0.0, hence the 0.3 offset: at 0.0 it would sit on the
# stop and a small undershoot would read as a limit violation rather than a
# tracking error.
POSE_AWAY = make_joint_state([0.3, 0.2, 0.0, 0.8, 0.0, 0.2, 0.0])

MOVE_DURATION_SEC = 3.0


def create_openarm_controller_check_torque_tree(robot_config):
    """Switch to the joint-space controller, move away, come back.

    One arm: on the bimanual torso it runs per arm (arm:=left, arm:=right),
    each with its own controller, broadcaster and joints.
    """
    require_one_arm(robot_config, 'controller_check_torque')
    controller = robot_config['joint_space']
    ee_broadcaster, _ee_topic = ee_state_names(robot_config)

    seq = py_trees.composites.Sequence(name='OpenArm_Controller_Check_Torque', memory=True)
    seq.add_children([
        SwitchControllerServiceBehavior(
            name=f'Switch_{controller}',
            activate=[controller],
            # The exclusive set is this robot's, from its registry entry; an
            # exclusive switch without robot_config raises (there is no
            # robot-independent set to fall back to).
            robot_config=robot_config,
        ),
        ListControllersServiceBehavior(
            name='Broadcasters_Active',
            require_active=[
                'joint_state_broadcaster',
                # Per arm: left_/right_ee_state_broadcaster on the torso.
                ee_broadcaster,
                controller,
            ],
        ),
        JointSpaceActionBehavior(
            name=f'{controller}_Move',
            target_joints=POSE_AWAY,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
            # Per profile: a bimanual arm's goals name its own left_/right_
            # joints, so one arm's target can never drive the other.
            joint_names=arm_joint_names(robot_config),
        ),
        JointSpaceActionBehavior(
            name=f'{controller}_Return',
            target_joints=home_joint_state(robot_config),
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
            joint_names=arm_joint_names(robot_config),
        ),
        # Cheap insurance against a controller that crashed mid-motion: the
        # action would still report success on the last goal it managed.
        ListControllersServiceBehavior(
            name=f'{controller}_Still_Active',
            require_active=[controller],
        ),
    ])

    # A check that fails mid-motion leaves this effort controller active with a
    # half-executed goal; the abort re-asserts the torque hold and proves it
    # took. On this profile the hold is the same controller, so a healthy
    # controller makes the abort a no-op switch and the verify still reports.
    return guarded_mission(seq, robot_config, CONTROL_MODE)
