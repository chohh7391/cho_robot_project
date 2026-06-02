from enum import Enum
from typing import Optional


ACTION_SERVER_NAMESPACE = '/controller_action_server'
CONTROLLER_MANAGER_NAMESPACE = '/controller_manager'

SWITCH_CONTROLLER_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/switch_controller'
LIST_CONTROLLERS_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/list_controllers'


class ControllerNames(str, Enum):
    """Franka controller names (legacy, retained for backward compatibility)."""

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
# Config-backed controller registry
# ---------------------------------------------------------------------------

_ROBOT_CONFIGS = {
    'franka': {
        'joint_space': 'joint_space_qp_controller',
        'task_space': 'task_space_qp_controller',
        'gripper': 'gripper_controller',
        'vla': 'vla_controller',
    },
    'ur5e': {
        'joint_space': 'joint_space_controller',
        'task_space': 'task_space_ik_controller',
        'gripper': None,
        'vla': None,
    },
}

_loaded_config: Optional[dict] = None


def load_robot_config(robot_type: str) -> dict:
    """
    Return the controller config dict for *robot_type*.

    Raises ValueError for unknown robot types.
    """
    global _loaded_config
    if robot_type not in _ROBOT_CONFIGS:
        raise ValueError(
            f"Unknown robot_type '{robot_type}'. "
            f"Valid options: {list(_ROBOT_CONFIGS.keys())}"
        )
    _loaded_config = dict(_ROBOT_CONFIGS[robot_type])
    _loaded_config['robot_type'] = robot_type
    return _loaded_config


def get_controller_name(role: str, robot_type: Optional[str] = None) -> Optional[str]:
    """Return the controller name for *role* ('joint_space', 'task_space', etc.)."""
    if robot_type is not None:
        cfg = _ROBOT_CONFIGS.get(robot_type, {})
    elif _loaded_config is not None:
        cfg = _loaded_config
    else:
        raise RuntimeError("No robot config loaded. Call load_robot_config() first.")
    return cfg.get(role)


def get_action_name(role: str, robot_type: Optional[str] = None) -> Optional[str]:
    """Return the full action server name for *role*."""
    name = get_controller_name(role, robot_type)
    if name is None:
        return None
    return f'{ACTION_SERVER_NAMESPACE}/{name}'


# ---------------------------------------------------------------------------
# Legacy helpers (unchanged API)
# ---------------------------------------------------------------------------

def controller_name_value(controller):
    if isinstance(controller, ControllerNames):
        return controller.value
    return str(controller)


def controller_action_name(controller):
    return f'{ACTION_SERVER_NAMESPACE}/{controller_name_value(controller)}'


def valid_controller_action_names():
    names = [controller_action_name(c) for c in ControllerNames]
    for config in _ROBOT_CONFIGS.values():
        for controller in config.values():
            if controller is None:
                continue
            action_name = controller_action_name(controller)
            if action_name not in names:
                names.append(action_name)
    return names


def vla_completion_service_name():
    return f'{controller_action_name(ControllerNames.VLA)}/notify_completion'
