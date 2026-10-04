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

"""Validated access to the Cho robot metadata registry."""

from .registry import (ACTION_KINDS, CONTROL_MODES, POUR_ACTION_KIND,
                       PREFERENCE_ACTION_KINDS, available_profiles, available_robot_types,
                       blocked_home_joint_goals, controller_action_name,
                       declared_hold_control_modes, hold_controllers_for_control_mode,
                       home_pose_policy, load_moveit_metadata,
                       load_robot_config, moveit_bridge_node, static_scene_ready_service,
                       task_goal_frame, task_home_pose, validate_robot_config)
from .motion_limits import motion_limit_parameters

__all__ = [
    'ACTION_KINDS', 'CONTROL_MODES', 'POUR_ACTION_KIND', 'PREFERENCE_ACTION_KINDS',
    'available_profiles', 'available_robot_types', 'blocked_home_joint_goals',
    'controller_action_name', 'declared_hold_control_modes', 'hold_controllers_for_control_mode',
    'home_pose_policy', 'load_moveit_metadata', 'load_robot_config', 'motion_limit_parameters',
    'moveit_bridge_node', 'static_scene_ready_service', 'task_goal_frame', 'task_home_pose',
    'validate_robot_config',
]
