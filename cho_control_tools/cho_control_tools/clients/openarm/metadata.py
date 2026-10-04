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

"""OpenArm action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy

from cho_control_tools.action_names import controller_action_name, moveit_bridge_node


_HOME = {
    '0': [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
    '1': [0.0, -0.5, 0.0, 1.2, 0.0, 0.4, 0.0],
    '2': [0.3, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0],
    '3': [-0.3, -0.4, -0.2, 1.0, -0.2, 0.2, 0.0],
}
_REACH = {
    '0': [0.0, -0.5, 0.0, 1.2, 0.0, 0.4, 0.0],
    '1': [0.15, -0.5, 0.1, 1.1, 0.1, 0.35, 0.0],
    '2': [-0.15, -0.45, -0.1, 1.1, -0.1, 0.35, 0.0],
    '3': [0.0, -0.35, 0.0, 0.95, 0.0, 0.30, 0.0],
}
def _model(prefix, ee_link, relative_goal_frame):
    """See clients/fr5/metadata.py; *prefix* is the bimanual arm's, '' for one arm."""
    model = {
        'joints': [f'openarm_{prefix}joint{index}' for index in range(1, 8)],
        'ee_link': ee_link,
        'absolute_goal_frame': 'world',
    }
    if relative_goal_frame:
        model['relative_goal_frame'] = relative_goal_frame
    return model


_CONFIG = {
    'robot_type': 'openarm',
    'supports_task': True,
    'model': _model('', 'openarm_hand_tcp', None),
    'controllers': {'moveit_trajectory': 'joint_trajectory_controller'},
    'moveit': {},
    'actions': {'preferences': {
        'joint': [controller_action_name(node, 'joint') for node in (
            moveit_bridge_node('openarm'), 'joint_space_position_controller',
            'joint_impedance_mit_controller')],
        'task': [controller_action_name(node, 'task') for node in (
            moveit_bridge_node('openarm'), 'task_space_impedance_mit_controller')],
        'gripper': [controller_action_name('gripper_controller', 'gripper')],
        'vla': [controller_action_name('vla_mit_controller', 'vla')],
    }},
    'poses': {'home': _HOME, 'reach': _REACH},
    'motions': {'reach': {
        '0': {'relative': False, 'position': [0.446841389, -0.286500255, 0.414628896],
              'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
        '1': {'relative': False, 'position': [0.275148419, -0.186149069, 0.393389049],
              'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
        '2': {'relative': False, 'position': [0.397230680, -0.191640328, 0.322214847],
              'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
        '3': {'relative': False, 'position': [0.397290216, -0.162259069, 0.460550447],
              'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
    }},
}
_SIDE_REACH = {
    'left': {
        '0': [0.360994904, 0.389823628, 0.466489710],
        '1': [0.360994904, 0.389823628, 0.416489710],
        '2': [0.390994904, 0.389823628, 0.416489710],
        '3': [0.360994904, 0.419823628, 0.416489710],
    },
    'right': {
        '0': [0.353214573, -0.028147131, 0.380270917],
        '1': [0.353214573, -0.028147131, 0.330270917],
        '2': [0.383214573, -0.028147131, 0.330270917],
        '3': [0.353214573, 0.001852869, 0.330270917],
    },
}
_SIDE_ORIENTATION = {
    'left': [0.743171722, -0.028219326, 0.563936869, -0.358991182],
    'right': [0.812511913, 0.061920006, 0.536067776, -0.220503161],
}
_BOTH_HOME = {
    '0': [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0] * 2,
    '1': [0.0, -0.5, 0.0, 1.2, 0.0, 0.4, 0.0] * 2,
    '2': [0.3, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0] * 2,
    '3': [-0.3, -0.4, -0.2, 1.0, -0.2, 0.2, 0.0] * 2,
}
_BOTH_REACH = {
    '0': [0.3, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0] * 2,
    '1': [0.2, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0,
          0.4, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0],
    '2': [0.3, -0.35, 0.25, 0.9, 0.15, 0.25, 0.0] * 2,
    '3': [0.3, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0,
          0.4, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0],
}


def _side_config(profile):
    config = deepcopy(_CONFIG)
    tcp = f'openarm_{profile}_hand_tcp'
    config['model'] = _model(f'{profile}_', tcp, tcp)
    config['controllers'] = {'moveit_trajectory': f'{profile}_joint_trajectory_controller'}
    config['moveit'] = {'controllers': [
        'left_joint_trajectory_controller', 'right_joint_trajectory_controller']}
    bridge = moveit_bridge_node('openarm', profile)
    config['actions']['preferences'] = {
        'joint': [controller_action_name(node, 'joint') for node in (
            bridge, f'{profile}_joint_impedance_mit_controller')],
        'task': [controller_action_name(node, 'task') for node in (
            bridge, f'{profile}_task_space_impedance_mit_controller')],
        'gripper': [controller_action_name(f'{profile}_gripper_controller', 'gripper')],
        'vla': [controller_action_name(f'{profile}_vla_mit_controller', 'vla')],
    }
    config['motions']['reach'] = {
        selector: {'relative': False, 'position': position,
                   'orientation': _SIDE_ORIENTATION[profile]}
        for selector, position in _SIDE_REACH[profile].items()
    }
    return config


def _both_config():
    config = deepcopy(_CONFIG)
    config['supports_task'] = False
    config['model'] = _model('left_', 'openarm_left_hand_tcp', None)
    config['model']['joints'] += [f'openarm_right_joint{index}' for index in range(1, 8)]
    config['controllers'] = {'moveit_trajectory': 'left_joint_trajectory_controller'}
    config['moveit'] = {'controllers': [
        'left_joint_trajectory_controller', 'right_joint_trajectory_controller']}
    config['actions']['preferences'] = {
        'joint': [controller_action_name(moveit_bridge_node('openarm', 'both'), 'joint')],
        'task': [], 'gripper': [],
    }
    config['poses'] = {'home': _BOTH_HOME, 'reach': _BOTH_REACH}
    return config


def load(profile='single'):
    if profile == 'single':
        return deepcopy(_CONFIG)
    if profile in ('left', 'right'):
        return _side_config(profile)
    if profile == 'both':
        return _both_config()
    raise ValueError(f'Unknown OpenArm profile: {profile}')
