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

"""trajectory_replay on the UR5e: the common defaults.

The tree is the robot-independent one (``tasks/trajectory_replay.py``). It
replays through ``joint_trajectory_controller``, which the MuJoCo bringup
loads with ``load_moveit_controller:=true``. The UR MuJoCo bringup has no
gripper, so the example recording has no gripper events.
"""

from cho_task_manager.tasks.trajectory_replay import (
    create_trajectory_replay_tree,
    ReplayDefaults,
)
from cho_task_manager.utils.controller_names import load_robot_config

DEFAULTS = ReplayDefaults()


def create_ur_trajectory_replay_tree(robot_config=None):
    """Build the replay tree with the UR5e's defaults."""
    return create_trajectory_replay_tree(
        robot_config or load_robot_config('ur5e'), DEFAULTS, name='UR5e_Trajectory_Replay')


__all__ = ['DEFAULTS', 'create_ur_trajectory_replay_tree']
