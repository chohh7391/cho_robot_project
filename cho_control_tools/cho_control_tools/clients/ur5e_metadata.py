"""UR5e action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy


_CONFIG = {
    'robot_type': 'ur5e',
    'supports_task': True,
    'controllers': {'moveit_trajectory': 'joint_trajectory_controller'},
    'moveit': {},
    'actions': {
        'preferences': {
            'joint': ['/ur5e/controller_action_server/moveit_joint',
                      '/controller_action_server/joint_space_position_controller'],
            'task': ['/ur5e/controller_action_server/moveit_task',
                     '/controller_action_server/task_space_ik_controller'],
            'gripper': [],
        },
        'gripper_command': {
            'topic': '/robotiq_controller/commands', 'open': 0.0, 'close': 0.7929,
        },
    },
    'poses': {'home': {
        '0': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        '1': [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
        '2': [0.2, -1.4, 1.4, -1.6, -1.5, 0.2],
        '3': [-0.2, -1.4, 1.4, -1.6, -1.5, -0.2],
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
        raise ValueError('UR5e action client supports only the single profile')
    return deepcopy(_CONFIG)
