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

"""The tasks this package registers: {robot_type: tree builder} per task.

Registered the way any package registers its own, through the
``cho_task_manager.tasks`` entry point group (setup.py names each table below);
nothing reads this module directly. A builder takes the robot's registry
config and returns the tree's root.
"""

from cho_task_manager.tasks.fr5 import (
    create_fr5_fjt_handover_tree,
    create_fr5_tag_search_tree,
    create_fr5_trajectory_replay_tree,
)
from cho_task_manager.tasks.franka import (
    create_franka_controller_check_position_tree,
    create_franka_controller_check_torque_tree,
    create_franka_controller_check_velocity_tree,
    create_franka_pick_place_position_tree,
    create_franka_pick_place_tree,
    create_franka_tag_reach_tree,
    create_franka_trajectory_replay_tree,
)
from cho_task_manager.tasks.openarm import (
    create_openarm_controller_check_torque_tree,
    create_openarm_mit_task_tuning_tree,
    create_openarm_trajectory_replay_tree,
)
from cho_task_manager.tasks.ur import (
    create_ur_multi_move_tree,
    create_ur_pick_place_tree,
    create_ur_trajectory_replay_tree,
)

PICK_PLACE = {'franka': create_franka_pick_place_tree, 'ur5e': create_ur_pick_place_tree}
PICK_PLACE_POSITION = {'franka': create_franka_pick_place_position_tree}

# Perception-driven: the target comes from an AprilTag at run time.
TAG_REACH = {'franka': create_franka_tag_reach_tree}
# The wrist camera searches for a tag the standing cameras do not see well
# (tasks/fr5/tag_search.py).
TAG_SEARCH = {'fr5': create_fr5_tag_search_tree}

# Controller smoke checks, one per bringup control_mode.
CONTROLLER_CHECK_POSITION = {'franka': create_franka_controller_check_position_tree}
CONTROLLER_CHECK_TORQUE = {
    'franka': create_franka_controller_check_torque_tree,
    'openarm': create_openarm_controller_check_torque_tree,
}
CONTROLLER_CHECK_VELOCITY = {'franka': create_franka_controller_check_velocity_tree}

# Replays a recording through the trajectory controller; every robot builds the
# same tree (tasks/trajectory_replay.py).
TRAJECTORY_REPLAY = {
    'franka': create_franka_trajectory_replay_tree,
    'fr5': create_fr5_trajectory_replay_tree,
    'ur5e': create_ur_trajectory_replay_tree,
    'openarm': create_openarm_trajectory_replay_tree,
}

# The plan is produced and executed by another workspace; this tree owns the
# arm's controller state around it (tasks/fr5/fjt_handover.py).
FJT_HANDOVER = {'fr5': create_fr5_fjt_handover_tree}

MULTI_MOVE = {'ur5e': create_ur_multi_move_tree}

MIT_TASK_TUNING = {'openarm': create_openarm_mit_task_tuning_tree}
