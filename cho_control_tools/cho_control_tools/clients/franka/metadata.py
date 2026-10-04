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

"""Franka action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy

from cho_control_tools.action_names import controller_action_name, moveit_bridge_node


_CONFIG = {
    'robot_type': 'franka',
    'supports_task': True,
    # See clients/fr5/metadata.py. No relative_goal_frame: the Franka bringups
    # take ee_name as a launch argument, so relative goals go out with ''.
    'model': {
        'joints': [f'fr3_joint{index}' for index in range(1, 8)],
        'ee_link': 'fr3_hand_tcp',
        'absolute_goal_frame': 'fr3_link0',
    },
    'controllers': {'moveit_trajectory': 'moveit_joint_trajectory_controller'},
    'moveit': {},
    'actions': {'preferences': {
        'joint': [controller_action_name(node, 'joint') for node in (
            moveit_bridge_node('franka'), 'joint_space_qp_controller',
            'joint_space_impedance_controller')],
        'task': [controller_action_name(node, 'task') for node in (
            moveit_bridge_node('franka'), 'task_space_qp_controller',
            'task_space_impedance_controller', 'operational_space_controller',
            'task_space_ik_controller')],
        'gripper': [controller_action_name('gripper_controller', 'gripper')],
    }},
    'poses': {'home': {
        '0': [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0,
              1.5707963267948966, 0.7853981633974483],
        '1': [0.0, 0.0, 0.0, -1.57, 0.0, 2.355, 0.0],
        '2': [-0.3202889859676361, 0.5399062633514404, 0.3390618860721588,
              -1.862808346748352, -0.24342849850654602, 2.361226797103882,
              0.30928418040275574],
        '3': [-0.46396875381469727, 0.6291446089744568, 0.4975337088108063,
              -1.9110225439071655, -0.4653533399105072, 2.424884796142578,
              0.85429847240448],
    }},
    'motions': {'reach': {
        '0': {'relative': False, 'position': [0.2, -0.2, 0.5], 'orientation': [1.0, 0.0, 0.0, 0.0]},
        '1': {'relative': False, 'position': [0.2, 0.2, 0.6], 'orientation': [1.0, 0.0, 0.0, 0.0]},
        '2': {'relative': True, 'position': [0.0, 0.0, -0.2], 'orientation': [0.0, 0.0, 0.0, 1.0]},
        '3': {'relative': False, 'position': [0.6, 0.0, 0.1], 'orientation': [1.0, 0.0, 0.0, 0.0]},
    }},
}


def load(profile='single'):
    if profile != 'single':
        raise ValueError('Franka action client supports only the single profile')
    return deepcopy(_CONFIG)
