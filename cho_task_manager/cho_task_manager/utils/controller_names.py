from enum import Enum
from typing import List

from cho_robot_config import available_robot_types as registry_robot_types
from cho_robot_config import load_robot_config as load_registry_config


ACTION_SERVER_NAMESPACE = '/controller_action_server'
CONTROLLER_MANAGER_NAMESPACE = '/controller_manager'

SWITCH_CONTROLLER_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/switch_controller'
LIST_CONTROLLERS_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/list_controllers'


class ControllerNames(str, Enum):
    """Franka controller names (used directly by the Franka task trees)."""

    # Joint Space Controllers
    JOINT_IMPEDANCE = 'joint_space_impedance_controller'
    # OpenArm MuJoCo MIT adapter. It deliberately has the same JointSpace
    # action contract as JOINT_IMPEDANCE, so the existing action_client
    # home/reach commands need only select this controller name.
    JOINT_IMPEDANCE_MIT = 'joint_impedance_mit_controller'
    JOINT_QP = 'joint_space_qp_controller'
    JOINT_POSITION = 'joint_space_position_controller'
    JOINT_VELOCITY = 'joint_space_velocity_controller'

    # Task Space Controllers
    IK = 'task_space_ik_controller'
    TASK_VELOCITY = 'task_space_velocity_controller'
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


# Arm controllers that all claim the same effort (torque) command interfaces:
# at most one may be active at a time. A switch that activates one of these must
# deactivate every other one, regardless of which was running before, so that
# re-runs after a failure (memory Sequence + OneShot re-tick) don't leave a
# conflicting controller active and fail the STRICT switch.
# GRIPPER uses a separate interface, so it is intentionally excluded.
EXCLUSIVE_ARM_CONTROLLERS = [
    ControllerNames.JOINT_IMPEDANCE,
    ControllerNames.JOINT_QP,
    ControllerNames.JOINT_POSITION,
    ControllerNames.JOINT_VELOCITY,
    ControllerNames.IK,
    ControllerNames.TASK_VELOCITY,
    ControllerNames.OPERATIONAL_SPACE,
    ControllerNames.TASK_IMPEDANCE,
    ControllerNames.TASK_QP,
    ControllerNames.VLA,
    ControllerNames.GRAVITY_COMPENSATION,
]


# ---------------------------------------------------------------------------
# Compatibility view of the canonical cho_robot_config registry.
# ---------------------------------------------------------------------------

def available_robot_types() -> List[str]:
    """Robot types discoverable from the canonical robot registry."""
    return registry_robot_types()


def load_robot_config(robot_type: str) -> dict:
    """
    Load the task-manager controller view for *robot_type* from cho_robot_config.

    Returns a flat dict, e.g.::

        {'robot_type': 'ur5e', 'joint_space': 'joint_space_position_controller',
         'task_space': 'task_space_ik_controller', 'gripper': None, 'vla': None}

    Raises ValueError for unknown robot types.
    """
    raw = load_registry_config(robot_type)
    controllers = raw['controllers']
    compatibility = raw.get('compatibility', {}).get('task_manager', {})
    return {
        'robot_type': raw['robot_type'],
        'joint_space': compatibility.get('joint_space', controllers['direct_joint']),
        'task_space': compatibility.get('task_space', controllers['direct_task']),
        'gripper': compatibility.get('gripper', controllers['gripper']),
        'vla': compatibility.get('vla', controllers['vla']),
    }


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
