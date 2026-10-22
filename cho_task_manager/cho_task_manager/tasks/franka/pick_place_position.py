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
    ChunkStreamCompletionWaiterBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_joint_state, home_subtree
from cho_task_manager.utils.controller_names import ControllerNames, load_robot_config

CONTROL_MODE = 'position'


# Same flow as pick_place.py, but homes via the position-interface controller
# (joint_space_position_controller) instead of the torque-based
# joint_space_impedance_controller: a control_mode:=position bringup only makes
# POSITION_CONTROLLERS switchable (launch_utils.py), so joint_space_impedance_controller
# is never loaded there.
def create_franka_pick_place_position_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    robot_config = robot_config or load_robot_config('franka')
    # Same pose as pick_place.py: the registry's poses.task_home.
    home = home_joint_state(robot_config)
    chunk_controller = robot_config['action_chunk']

    mission_sequence = py_trees.composites.Sequence(
        name="Franka_Pick_And_Place_Position_Sequence", memory=True
    )

    init_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=ControllerNames.JOINT_POSITION,
        duration=3.0,
    )

    action_chunk_seq = py_trees.composites.Sequence(name="2_Start_Chunk_Stream", memory=True)
    action_chunk_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Action_Chunk",
            activate=[chunk_controller],
            robot_config=robot_config,
        ),
        ChunkStreamCompletionWaiterBehavior(name="Wait_For_Chunk_Stream_Completion", controller=chunk_controller),
    ])

    finish_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=ControllerNames.JOINT_POSITION,
        duration=5.0,
        name="3_Finish",
        suffix="_Final",
    )

    mission_sequence.add_children([init_seq, action_chunk_seq, finish_seq])

    return guarded_mission(mission_sequence, robot_config, CONTROL_MODE)
