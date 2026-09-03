"""Franka action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy


_CONFIG = {
    'robot_type': 'franka',
    'supports_task': True,
    'controllers': {'moveit_trajectory': 'moveit_joint_trajectory_controller'},
    'moveit': {},
    'actions': {'preferences': {
        'joint': ['/franka/controller_action_server/moveit_joint',
                  '/controller_action_server/joint_space_qp_controller',
                  '/controller_action_server/joint_space_impedance_controller'],
        'task': ['/franka/controller_action_server/moveit_task',
                 '/controller_action_server/task_space_qp_controller',
                 '/controller_action_server/task_space_impedance_controller',
                 '/controller_action_server/operational_space_controller',
                 '/controller_action_server/task_space_ik_controller'],
        'gripper': ['/controller_action_server/gripper_controller'],
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
