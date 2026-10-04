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

import py_trees
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_joint_state, home_subtree
from cho_task_manager.utils.controller_names import ControllerNames, load_robot_config

# joint_space_impedance_controller is a torque controller, so this tree only
# runs on a control_mode:=torque bringup. The safe-abort branch needs to know
# that to pick a hold controller that bringup actually loaded.
CONTROL_MODE = 'torque'


# ==========================================
# Franka pick-and-place (VLA) tree assembly
# ==========================================
def create_franka_pick_place_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    # Fill in the registry entry when built directly (tests, one-off scripts):
    # both the exclusive-switch set and the abort's hold controller derive from
    # it, and an exclusive switch built without one raises.
    robot_config = robot_config or load_robot_config('franka')
    # The registry's poses.task_home: TCP forward and down, gripper pointing
    # straight down (cho_robot_config/config/franka.yaml).
    home = home_joint_state(robot_config)
    vla = robot_config['vla']

    mission_sequence = py_trees.composites.Sequence(name="Franka_Pick_And_Place_Sequence", memory=True)

    # ----------------------------------------------------
    # 1. Init sequence (go home via torque-based joint impedance)
    # ----------------------------------------------------
    init_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=ControllerNames.JOINT_IMPEDANCE,
        duration=3.0,
    )

    # ----------------------------------------------------
    # 2. VLA sequence
    # ----------------------------------------------------
    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[vla],
            robot_config=robot_config,
        ),
        VLACompletionWaiterBehavior(
            name="Wait_For_VLA_Completion", controller=vla,
        ),
    ])

    # ----------------------------------------------------
    # 3. Finish sequence (return home after VLA success)
    # ----------------------------------------------------
    finish_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=ControllerNames.JOINT_IMPEDANCE,
        duration=5.0,
        name="3_Finish",
        suffix="_Final",
    )

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    return guarded_mission(mission_sequence, robot_config, CONTROL_MODE)
