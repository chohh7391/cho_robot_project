"""FR5 action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy


_CONFIG = {
    'robot_type': 'fr5',
    'supports_task': True,
    'controllers': {'moveit_trajectory': 'joint_trajectory_controller',
                    'gripper': 'gripper_controller'},
    'moveit': {},
    'actions': {'preferences': {
        'joint': ['/fr5/controller_action_server/moveit_joint',
                  '/controller_action_server/joint_space_position_controller'],
        'task': ['/fr5/controller_action_server/moveit_task',
                 '/controller_action_server/task_space_ik_controller'],
        'gripper': ['/controller_action_server/gripper_controller'],
    }},
    'poses': {
        'home': {
            '0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            '1': [0.0, -0.7853981634, -1.5707963268, 0.7853981634, -1.5707963268, 0.0],
            '2': [0.4, -0.7, -1.8, 1.2, 0.4, 1.0],
            '3': [-0.4, -0.7, -1.8, 1.2, -0.4, 1.0],
        },
        'home_safety': {'0': {
            'enabled': False,
            'reason': 'zero pose places the FR5 wrist at the floor and is diagnostic-only',
            'max_joint_distance': 0.01,
        }},
    },
    # Absolute world-frame wrist3_link poses; see cho_robot_config/config/fr5.yaml
    # for why these are fixed endpoints and why -x/-y rather than +x/+y.
    'motions': {'reach': {
        '0': {'relative': False, 'position': [-0.123206132, -0.102101755, 0.831834257],
              'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
        '1': {'relative': False, 'position': [-0.123206132, -0.102101755, 0.631834257],
              'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
        '2': {'relative': False, 'position': [-0.223206132, -0.102101755, 0.731834257],
              'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
        '3': {'relative': False, 'position': [-0.123206132, -0.202101755, 0.731834257],
              'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
    }},
}


def load(profile='single'):
    if profile != 'single':
        raise ValueError('FR5 action client supports only the single profile')
    return deepcopy(_CONFIG)
