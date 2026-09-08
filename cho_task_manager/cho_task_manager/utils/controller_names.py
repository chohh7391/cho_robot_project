from enum import Enum
from typing import List

from cho_robot_config import available_profiles as registry_profiles
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


# The historical Franka-only exclusive set. Every name here is a Franka
# controller: mutually exclusive on Franka, but meaningless for OpenArm or UR,
# which have their own (MIT / trajectory / left_ + right_ prefixed) controllers.
# Do NOT treat this as a general list -- call exclusive_arm_controllers() with a
# robot config instead. It stays as the no-config default so a call site that
# has no robot config keeps its exact previous behaviour.
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


def available_profiles(robot_type: str) -> List[str]:
    """Arm profiles selectable for *robot_type* (e.g. single, left, right)."""
    return registry_profiles(robot_type)


def load_robot_config(robot_type: str, profile: str = 'single') -> dict:
    """
    Load the task-manager controller view for *robot_type* from cho_robot_config.

    Returns a flat dict, e.g.::

        {'robot_type': 'ur5e', 'joint_space': 'joint_space_position_controller',
         'task_space': 'task_space_ik_controller', 'gripper': None, 'vla': None}

    Raises ValueError for unknown robot types.
    """
    raw = load_registry_config(robot_type, profile)
    controllers = raw['controllers']
    compatibility = raw.get('compatibility', {}).get('task_manager', {})
    return {
        'robot_type': raw['robot_type'],
        'profile': raw.get('profile', 'single'),
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


# Registry controller roles whose controllers claim the arm's own command
# interfaces. 'gripper' is deliberately absent: it claims the finger interfaces,
# so it must stay active across an arm-controller switch.
_EXCLUSIVE_CONTROLLER_ROLES = ('hold', 'direct_joint', 'direct_task',
                               'moveit_trajectory', 'vla')

# Prefix of a direct per-controller action server. The registry's action
# preferences also list robot-scoped MoveIt endpoints
# (/<robot>/controller_action_server/moveit_joint), which are not controllers;
# they do not carry this prefix, so matching on it selects exactly the
# controller-backed endpoints.
_DIRECT_ACTION_PREFIX = f'{ACTION_SERVER_NAMESPACE}/'


def exclusive_arm_controllers(robot_config=None) -> List[str]:
    """
    Arm controllers that must be deactivated when one of them is activated.

    They all claim the same arm command interfaces, so at most one may be
    active at a time. A switch that activates one must deactivate every other
    one, regardless of which was running before, so that re-runs after a
    failure (memory Sequence + OneShot re-tick) don't leave a conflicting
    controller active.

    ``robot_config`` is the dict returned by :func:`load_robot_config`. Without
    it the historical Franka-only names are returned, so a call site that has
    no robot config keeps its exact previous behaviour. With it the names come
    from the canonical cho_robot_config registry - the arm controller roles plus
    the direct controller action endpoints of the loaded profile, which is what
    makes a bimanual (``left_`` / ``right_`` prefixed) profile come out right.
    The gripper is never included: it claims a separate interface.
    """
    names = []

    def add(controller):
        if controller is None:
            return
        value = controller_name_value(controller)
        if value and value not in names:
            names.append(value)

    if robot_config is None:
        for controller in EXCLUSIVE_ARM_CONTROLLERS:
            add(controller)
        return names

    robot_type = robot_config.get('robot_type')
    profile = robot_config.get('profile', 'single')

    # The compatibility view a tree already holds: it carries the
    # compatibility.task_manager overrides, which the raw roles do not.
    for role in ('joint_space', 'task_space', 'vla'):
        add(robot_config.get(role))

    try:
        registry = load_registry_config(robot_type, profile)
    except (ValueError, KeyError):
        registry = None
    if registry is not None:
        controllers = registry.get('controllers', {})
        for role in _EXCLUSIVE_CONTROLLER_ROLES:
            add(controllers.get(role))
        preferences = registry.get('actions', {}).get('preferences', {})
        # Not 'gripper': those endpoints are backed by the gripper controller.
        for space in ('joint', 'task'):
            for action_name in preferences.get(space, []):
                if action_name.startswith(_DIRECT_ACTION_PREFIX):
                    add(action_name[len(_DIRECT_ACTION_PREFIX):])

    if robot_type == 'franka':
        # The Franka trees also drive controllers that are not registry roles
        # (joint/task impedance, operational space, gravity compensation).
        # Dropping them would regress the idempotency this set exists for.
        for controller in EXCLUSIVE_ARM_CONTROLLERS:
            add(controller)

    gripper = robot_config.get('gripper')
    if gripper is not None:
        gripper = controller_name_value(gripper)
        names = [name for name in names if name != gripper]
    return names


def controller_action_name(controller):
    return f'{ACTION_SERVER_NAMESPACE}/{controller_name_value(controller)}'


def _config_controller_names() -> List[str]:
    """All non-null controller names referenced by any robot config yaml."""
    names: List[str] = []
    for robot_type in available_robot_types():
        for profile in available_profiles(robot_type):
            try:
                config = load_robot_config(robot_type, profile)
            except ValueError:
                continue
            for role, controller in config.items():
                if role in ('robot_type', 'profile') or controller is None:
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
