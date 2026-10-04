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

"""What the OpenArm trees share: per-arm resource names, and the profiles they run on."""

from cho_task_manager.utils.controller_names import arm_joint_names

#: Joints of one OpenArm arm. A profile with more is the bimanual 'both'.
ARM_JOINTS = 7


def ee_state_names(robot_config):
    """The ee_state_broadcaster and the pose topic it publishes, for this arm profile.

    A bimanual build prefixes every per-arm resource (``left_ee_state_broadcaster``,
    ``/ee_state/left/pose``), so the single-arm names do not exist on it and a
    tree that hard-coded them would fail before it moved anything.
    """
    profile = robot_config.get('profile', 'single')
    if profile == 'single':
        return 'ee_state_broadcaster', '/ee_state/pose'
    return f'{profile}_ee_state_broadcaster', f'/ee_state/{profile}/pose'


def require_one_arm(robot_config, task):
    """Refuse, at build time, a profile that is not one arm.

    The OpenArm trees drive one arm's controller with one arm's targets. The
    bimanual 'both' profile is the whole torso -- 14 joints and no controller
    of its own that a 7-joint check or a single TCP probe could address -- so
    the tree is run once per arm instead.
    """
    joints = arm_joint_names(robot_config)
    if len(joints) != ARM_JOINTS:
        profile = robot_config.get('profile', 'single')
        raise ValueError(
            f"{task} drives one OpenArm arm, and profile '{profile}' has {len(joints)} "
            f'joints. On the bimanual torso run it once per arm: arm:=left, then '
            f'arm:=right.')
