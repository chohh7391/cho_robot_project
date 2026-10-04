"""FR5 action-client metadata, intentionally independent of cho_robot_config."""

from copy import deepcopy

from cho_control_tools.action_names import controller_action_name, moveit_bridge_node


_CONFIG = {
    'robot_type': 'fr5',
    'supports_task': True,
    'controllers': {'moveit_trajectory': 'joint_trajectory_controller',
                    'gripper': 'gripper_controller'},
    'moveit': {},
    'actions': {'preferences': {
        'joint': [controller_action_name(node, 'joint') for node in (
            moveit_bridge_node('fr5'), 'joint_space_position_controller')],
        'task': [controller_action_name(node, 'task') for node in (
            moveit_bridge_node('fr5'), 'task_space_ik_controller')],
        'gripper': [controller_action_name('gripper_controller', 'gripper')],
    }},
    'poses': {
        'home': {
            '0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            '1': [-0.0836, -1.1209, -2.0723, -1.7125, 1.6049, 0.0798],
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
