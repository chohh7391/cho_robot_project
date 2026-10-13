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

"""trajectory_replay on the FR5: the measured defaults.

The tree is the robot-independent one (``tasks/trajectory_replay.py``). What
is FR5-specific is :data:`DEFAULTS`, and each value was set on the real arm:

* ``home_via='moveit'``: the replay's start pose is not a neutral pose, and
  the arm starts wherever the last run left it -- on seed 5 the straight move
  from the registry's ready pose is 2.99 rad (171 deg) on j5 alone -- so the
  default plans and collision-checks it. The direct home also calls the move
  done once the six-joint error norm is under 0.05 rad, which on a long move it
  reaches by running out of duration rather than by converging: homing that
  started far from the pose finished 0.0494 and 0.0497 rad out, against 0.0001
  when it started near. A bringup without MoveIt (MuJoCo, where every geom in
  the FR5 model is ``contype/conaffinity 0`` and there is nothing to collide
  with) passes ``home_via:=direct``.
* ``speed_scale=0.25``: the recordings this replays were timed by a planner
  for a simulator, and 1.0 has been measured above this arm's commissioning
  ceiling.
* ``gripper_settle_sec=4.0``: see :data:`GRIPPER_SETTLE_SEC`.

Run the bundled example in MuJoCo (the bringup always loads the trajectory
controller; the AG-95 is what the example's gripper events drive)::

    ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py gripper:=ag95
    ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=fr5 task:=trajectory_replay home_via:=direct

A recording of your own goes in ``config/replay/fr5/`` beside it and is passed
as ``replay_trajectory:=``. One that assumes a cell layout, as
``config/replay/fr5/traj_level_seed2.csv`` does, also needs
``replay_layout:=`` (the cell's own layout file). The real path is this same
tree: only the bringup underneath it changes.
"""

from cho_task_manager.tasks.trajectory_replay import (
    create_trajectory_replay_tree,
    HOME_DURATION_SEC,
    HOME_VIA_CHOICES,
    HOME_VIA_DIRECT,
    HOME_VIA_MOVEIT,
    replay_controller,
    ReplayDefaults,
    velocity_scaling,
)
from cho_task_manager.utils.controller_names import (
    load_robot_config,
    moveit_joint_action_name,
)
from cho_task_manager.utils.trajectory_recording import RecordingRejected

#: How the arm gets to the recording's start pose. See the module docstring.
DEFAULT_HOME_VIA = HOME_VIA_MOVEIT

#: Default playback speed, as a fraction of the recorded clock. Deliberately
#: conservative: see the module docstring.
DEFAULT_SPEED_SCALE = 0.25

#: Seconds the tree stands still after a gripper event before moving the arm.
#:
#: The FR5 will not run the gripper and the arm at once. It accepts the servo
#: stream while the jaws move and executes almost none of it, so the trajectory
#: controller's reference -- which advances on the wall clock -- walks away from
#: an arm that is standing still, and the goal aborts on state tolerance once the
#: gap passes 0.1 rad. Measured on the real robot: 5.7 s of frozen arm at the
#: grasp, every time.
#:
#: Waiting for the gripper's own action is not enough. It reports success while
#: the jaws are still travelling, because the stroke register it judges by
#: reports a fixed wrong value for about 0.8 s after each command. From the
#: action returning to the jaws actually stopping measured 2.05 s and 2.62 s.
#:
#: So this is deliberately over-provisioned rather than tuned: it only has to
#: exceed the jaw travel, which is bounded (a full stroke is ~2.6 s), and paying
#: a few seconds twice in a 150 s replay is cheap next to an aborted run. The
#: exact fix is for the gripper's own "motion done" register to gate its action;
#: until that exists this is the honest substitute.
GRIPPER_SETTLE_SEC = 4.0

DEFAULTS = ReplayDefaults(
    home_via=DEFAULT_HOME_VIA,
    speed_scale=DEFAULT_SPEED_SCALE,
    gripper_settle_sec=GRIPPER_SETTLE_SEC,
)


def create_fr5_trajectory_replay_tree(robot_config=None):
    """Build the replay tree with the FR5's defaults."""
    return create_trajectory_replay_tree(
        robot_config or load_robot_config('fr5'), DEFAULTS, name='FR5_Trajectory_Replay')


__all__ = [
    'create_fr5_trajectory_replay_tree',
    'replay_controller',
    'velocity_scaling',
    'moveit_joint_action_name',
    'RecordingRejected',
    'DEFAULTS',
    'DEFAULT_SPEED_SCALE',
    'DEFAULT_HOME_VIA',
    'GRIPPER_SETTLE_SEC',
    'HOME_VIA_CHOICES',
    'HOME_VIA_DIRECT',
    'HOME_VIA_MOVEIT',
    'HOME_DURATION_SEC',
]
