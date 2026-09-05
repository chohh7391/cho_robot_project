"""OpenArm action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy


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
_CONFIG = {
    'robot_type': 'openarm',
    'supports_task': True,
    'controllers': {'moveit_trajectory': 'joint_trajectory_controller'},
    'moveit': {},
    'actions': {'preferences': {
        'joint': ['/openarm/controller_action_server/moveit_joint',
                  '/controller_action_server/joint_space_position_controller',
                  '/controller_action_server/joint_impedance_mit_controller'],
        'task': ['/openarm/controller_action_server/moveit_task',
                 '/controller_action_server/task_space_impedance_mit_controller'],
        'gripper': ['/controller_action_server/gripper_controller'],
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
    config['controllers'] = {'moveit_trajectory': f'{profile}_joint_trajectory_controller'}
    config['moveit'] = {'controllers': [
        'left_joint_trajectory_controller', 'right_joint_trajectory_controller']}
    config['actions']['preferences'] = {
        'joint': [f'/openarm/{profile}/controller_action_server/moveit_joint',
                  f'/controller_action_server/{profile}_joint_impedance_mit_controller'],
        'task': [f'/openarm/{profile}/controller_action_server/moveit_task',
                 f'/controller_action_server/{profile}_task_space_impedance_mit_controller'],
        'gripper': [f'/controller_action_server/{profile}_gripper_controller'],
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
    config['controllers'] = {'moveit_trajectory': 'left_joint_trajectory_controller'}
    config['moveit'] = {'controllers': [
        'left_joint_trajectory_controller', 'right_joint_trajectory_controller']}
    config['actions']['preferences'] = {
        'joint': ['/openarm/both/controller_action_server/moveit_joint'],
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
