import os
from enum import Enum
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory


ACTION_SERVER_NAMESPACE = '/controller_action_server'
CONTROLLER_MANAGER_NAMESPACE = '/controller_manager'

SWITCH_CONTROLLER_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/switch_controller'
LIST_CONTROLLERS_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/list_controllers'


class ControllerNames(str, Enum):
    """Franka controller names (used directly by the Franka task trees)."""

    # Joint Space Controllers
    JOINT_IMPEDANCE = 'joint_space_impedance_controller'
    JOINT_QP = 'joint_space_qp_controller'

    # Task Space Controllers
    IK = 'task_space_ik_controller'
    OPERATIONAL_SPACE = 'operational_space_controller'
    TASK_IMPEDANCE = 'task_space_impedance_controller'
    TASK_QP = 'task_space_qp_controller'

    # VLA
    VLA = 'vla_controller'

    # Others
    GRAVITY_COMPENSATION = 'gravity_compensation_controller'
    GRIPPER = 'gripper_controller'

    def __str__(self):
        return self.value


# ---------------------------------------------------------------------------
# Config-backed controller registry (single source of truth: config/robots/*.yaml)
# ---------------------------------------------------------------------------

_config_cache: dict = {}


def _robot_config_dir() -> str:
    return os.path.join(
        get_package_share_directory('cho_task_manager'), 'config', 'robots'
    )


def available_robot_types() -> List[str]:
    """Robot types discoverable from config/robots/*.yaml."""
    config_dir = _robot_config_dir()
    if not os.path.isdir(config_dir):
        return []
    return sorted(
        os.path.splitext(f)[0]
        for f in os.listdir(config_dir)
        if f.endswith('.yaml')
    )


def load_robot_config(robot_type: str) -> dict:
    """
    Load the controller config for *robot_type* from config/robots/<robot_type>.yaml.

    Returns a flat dict, e.g.::

        {'robot_type': 'ur5e', 'joint_space': 'joint_space_position_controller',
         'task_space': 'task_space_ik_controller', 'gripper': None, 'vla': None}

    Raises ValueError for unknown robot types.
    """
    if robot_type in _config_cache:
        return dict(_config_cache[robot_type])

    path = os.path.join(_robot_config_dir(), f'{robot_type}.yaml')
    if not os.path.exists(path):
        raise ValueError(
            f"Unknown robot_type '{robot_type}'. "
            f"Valid options: {available_robot_types()}"
        )

    with open(path, 'r') as f:
        raw = yaml.safe_load(f) or {}

    config = dict(raw.get('controllers', {}) or {})
    config['robot_type'] = raw.get('robot_type', robot_type)
    _config_cache[robot_type] = config
    return dict(config)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def controller_name_value(controller):
    if isinstance(controller, ControllerNames):
        return controller.value
    return str(controller)


def controller_action_name(controller):
    return f'{ACTION_SERVER_NAMESPACE}/{controller_name_value(controller)}'


def _config_controller_names() -> List[str]:
    """All non-null controller names referenced by any robot config yaml."""
    names: List[str] = []
    for robot_type in available_robot_types():
        try:
            config = load_robot_config(robot_type)
        except ValueError:
            continue
        for role, controller in config.items():
            if role == 'robot_type' or controller is None:
                continue
            if controller not in names:
                names.append(controller)
    return names


def valid_controller_action_names() -> List[str]:
    """Action names accepted by BaseActionBehavior (enum + all robot configs)."""
    names = [controller_action_name(c) for c in ControllerNames]
    for controller in _config_controller_names():
        action_name = controller_action_name(controller)
        if action_name not in names:
            names.append(action_name)
    return names


def vla_completion_service_name():
    return f'{controller_action_name(ControllerNames.VLA)}/notify_completion'
