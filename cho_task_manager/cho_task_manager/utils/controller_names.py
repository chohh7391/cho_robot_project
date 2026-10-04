from enum import Enum
from typing import List

from cho_robot_config import ACTION_KINDS, CONTROL_MODES, POUR_ACTION_KIND
from cho_robot_config import available_profiles as registry_profiles
from cho_robot_config import available_robot_types as registry_robot_types
from cho_robot_config import controller_action_name as registry_action_name
from cho_robot_config import hold_controllers_for_control_mode as registry_hold_controllers
from cho_robot_config import load_robot_config as load_registry_config
from cho_robot_config import moveit_bridge_node
from cho_robot_config import task_home_pose as registry_task_home


CONTROLLER_MANAGER_NAMESPACE = '/controller_manager'

SWITCH_CONTROLLER_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/switch_controller'
LIST_CONTROLLERS_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/list_controllers'


class ControllerNames(str, Enum):
    """Franka's controller names, as readable constants for the Franka trees.

    FRANKA ONLY, and not a source of truth: every one of these is declared by
    the Franka registry entry (cho_robot_config/config/franka.yaml -- a role,
    a hold_by_control_mode entry, controllers.additional_arm or the gripper),
    and test_controller_names pins that. Nothing robot-independent may read
    this enum: a leaf or subtree takes its controller from the robot config
    of the tree it is built for, and the exclusive-switch set and the valid
    action names are derived from the registry, not from here.
    """

    # Joint Space Controllers
    JOINT_IMPEDANCE = 'joint_space_impedance_controller'
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
         'task_space': 'task_space_ik_controller', 'gripper': None, 'vla': None,
         'arm_base_link': 'base_link'}

    ``arm_base_link`` is the frame an absolute task-space goal is interpreted
    in, and it is here so a task that needs a frame does not have to re-open
    the registry -- or worse, spell the frame out. Note it is the registry's
    ``model.arm_base_link`` and NOT its ``model.base_frame``: the latter is
    'world' for Franka, which MoveIt uses and which does not exist in the
    published TF tree.

    Raises ValueError for unknown robot types.
    """
    raw = load_registry_config(robot_type, profile)
    controllers = raw['controllers']
    compatibility = raw.get('compatibility', {}).get('task_manager', {})
    return {
        'robot_type': raw['robot_type'],
        'profile': raw.get('profile', 'single'),
        'arm_base_link': raw['model']['arm_base_link'],
        'joint_space': compatibility.get('joint_space', controllers['direct_joint']),
        'task_space': compatibility.get('task_space', controllers['direct_task']),
        'gripper': compatibility.get('gripper', controllers['gripper']),
        'vla': compatibility.get('vla', controllers['vla']),
        # Present only on a robot that has one (FR5). None elsewhere, which is
        # how every other optional role here reads.
        'pour': compatibility.get('pour', controllers.get('pour')),
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
                               'moveit_trajectory', 'vla', 'pour')


def _serving_node(action_name):
    """The node an action is served by: everything before its kind."""
    return action_name.strip('/').rsplit('/', 1)[0]


def _registry_entry(robot_config):
    return load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))


def _arm_controllers_of(registry):
    """Every arm controller one registry entry names, in a stable order.

    The roles, the per-control-mode holds, ``controllers.additional_arm`` (the
    controllers a bringup loads that hold no role) and the controllers behind
    the direct action preferences. Never the gripper, and never the MoveIt
    bridge, which serves actions but is not a controller.
    """
    names = []

    def add(controller):
        if controller and controller not in names:
            names.append(controller)

    controllers = registry.get('controllers', {})
    for role in _EXCLUSIVE_CONTROLLER_ROLES:
        add(controllers.get(role))
    # The per-control-mode holds claim the same arm interfaces as everything
    # else here. Leaving them out would let a switch to the torque hold run
    # without deactivating the velocity one on a robot that has both.
    for hold in (controllers.get('hold_by_control_mode') or {}).values():
        for name in (hold if isinstance(hold, list) else [hold]):
            add(name)
    for name in controllers.get('additional_arm') or []:
        add(name)
    preferences = registry.get('actions', {}).get('preferences', {})
    bridge = moveit_bridge_node(registry['robot_type'], registry.get('profile', 'single'))
    # Not 'gripper': those endpoints are backed by the gripper controller.
    for space in ('joint', 'task'):
        for action_name in preferences.get(space, []):
            node = _serving_node(action_name)
            if node != bridge:
                add(node)
    gripper = controllers.get('gripper')
    return [name for name in names if name != gripper]


def exclusive_arm_controllers(robot_config) -> List[str]:
    """
    Arm controllers that must be deactivated when one of them is activated.

    They all claim the same arm command interfaces, so at most one may be
    active at a time. A switch that activates one must deactivate every other
    one, regardless of which was running before, so that re-runs after a
    failure (memory Sequence + OneShot re-tick) don't leave a conflicting
    controller active.

    ``robot_config`` is the dict returned by :func:`load_robot_config`, and it
    is required: the set is this robot's, from the canonical cho_robot_config
    registry -- the compatibility view's roles (which carry the
    compatibility.task_manager overrides), the raw roles and per-mode holds,
    ``controllers.additional_arm`` and the direct action endpoints of the
    loaded profile, which is what makes a bimanual (``left_`` / ``right_``
    prefixed) profile come out right. There is no robot-independent default;
    one used to exist and it was Franka's list, which deactivates nothing that
    holds any other arm. The gripper is never included.
    """
    if not robot_config or not robot_config.get('robot_type'):
        raise ValueError(
            'exclusive_arm_controllers() needs the robot config of the tree it is '
            'built for (load_robot_config(robot_type, profile)); the set of '
            'controllers that hold an arm is a fact of that robot')
    names = []

    def add(controller):
        if controller is None:
            return
        value = controller_name_value(controller)
        if value and value not in names:
            names.append(value)

    for role in ('joint_space', 'task_space', 'vla', 'pour'):
        add(robot_config.get(role))
    for name in _arm_controllers_of(_registry_entry(robot_config)):
        add(name)

    gripper = robot_config.get('gripper')
    if gripper is not None:
        gripper = controller_name_value(gripper)
        names = [name for name in names if name != gripper]
    return names


def resolve_control_mode(robot_config, default=None) -> str:
    """The bringup control_mode a task should assume.

    The operator's ``control_mode`` parameter wins, because only the operator
    knows how the bringup was actually started. ``default`` is the mode the
    task itself is written for - every task here selects controllers from one
    mode's switchable set, so it has one.
    """
    mode = (robot_config or {}).get('control_mode') or default
    if not mode:
        raise ValueError(
            'No control_mode available: pass control_mode:=<mode> to the task '
            f'manager, or give the task a default. Valid modes: {list(CONTROL_MODES)}')
    mode = str(mode)
    if mode not in CONTROL_MODES:
        raise ValueError(
            f"Unknown control_mode '{mode}'. Valid modes: {list(CONTROL_MODES)}")
    return mode


def arm_model(robot_config) -> dict:
    """Return this profile's joint names and end-effector frame.

    A behaviour that reasons about the arm itself rather than its controllers -
    joint limits, a Jacobian - needs these, and reading them from the registry
    keeps a bimanual profile's per-arm joint names and TCP correct.
    """
    model = _registry_entry(robot_config)['model']
    return {
        'joints': list(model['joints']),
        'ee_link': model['ee_link'],
        'arm_base_link': model['arm_base_link'],
        'base_frame': model['base_frame'],
    }


def hold_controllers(robot_config, control_mode) -> List[str]:
    """Controllers that hold this robot's arm in *control_mode*.

    Delegates to the canonical registry so a bimanual profile's per-arm names
    come out right. Raises ValueError when the robot declares no hold for that
    mode - see cho_robot_config.hold_controllers_for_control_mode for why that
    is louder than falling back to ``controllers.hold``.
    """
    return registry_hold_controllers(_registry_entry(robot_config), control_mode)


def task_home_positions(robot_config) -> List[float]:
    """Where this robot's task trees start and finish a mission, as joint positions.

    The registry's ``poses.task_home`` (cho_robot_config.task_home_pose), so
    no tree spells a robot's home out for itself. Raises ValueError when the
    robot/profile declares none.
    """
    return registry_task_home(_registry_entry(robot_config))


def controller_action_name(controller, kind):
    """The action *kind* that *controller* serves -- the one way to name an action.

    Every controller serves its actions under its own node
    (cho_interfaces/CONTRACT.md), so this is ``/<controller>/<kind>``:
    ``controller_action_name('joint_space_qp_controller', 'joint_space')`` is
    ``/joint_space_qp_controller/joint_space``. *kind* is one of
    ``joint_space``, ``task_space``, ``gripper``, ``vla``,
    ``follow_joint_trajectory``, or ``pour`` -- the FR5 pour action, which the
    contract leaves at its own ``/controller_action_server/<controller>``.

    *controller* may be a :class:`ControllerNames` member, a registry role's
    value, or the MoveIt bridge's node name (``moveit_bridge_node()``), which
    serves ``joint_space`` / ``task_space`` by the same rule. The rule itself
    lives in cho_robot_config, which validates the registry against it.

    ``follow_joint_trajectory`` is the stock endpoint of a
    joint_trajectory_controller. MoveIt's execution config points at the same
    endpoint (cho_moveit_<robot>/config/moveit_controllers.yaml), which is
    exactly why only one consumer may own the arm at a time.
    """
    return registry_action_name(controller_name_value(controller), kind)


def _registry_entries():
    """Every (robot_type, profile) entry of the registry, loaded and validated."""
    for robot_type in available_robot_types():
        for profile in available_profiles(robot_type):
            yield load_registry_config(robot_type, profile)


def _command_controller_names() -> List[str]:
    """Every controller a tree may send a cho action to, from every registry entry.

    The compatibility view's roles (with its task_manager overrides), the raw
    command roles, the per-mode holds and ``controllers.additional_arm`` --
    but not ``moveit_trajectory``, which serves only FollowJointTrajectory and
    is listed by :func:`valid_controller_action_names` for that kind alone.
    """
    names: List[str] = []

    def add(controller):
        if controller and controller not in names:
            names.append(controller)

    for registry in _registry_entries():
        view = load_robot_config(registry['robot_type'], registry.get('profile', 'single'))
        for role in ('joint_space', 'task_space', 'gripper', 'vla', 'pour'):
            add(view.get(role))
        controllers = registry.get('controllers') or {}
        for role in ('hold', 'direct_joint', 'direct_task', 'gripper', 'vla', 'pour'):
            add(controllers.get(role))
        for hold in (controllers.get('hold_by_control_mode') or {}).values():
            for name in (hold if isinstance(hold, list) else [hold]):
                add(name)
        for name in controllers.get('additional_arm') or []:
            add(name)
    return names


def moveit_joint_action_name(robot_config) -> str:
    """The MoveIt plan-and-execute joint action for this robot.

    Read from ``actions.preferences.joint`` rather than assembled here: the
    registry VALIDATES that the first joint preference is exactly the bridge's
    ``/<robot>[_<profile>]_moveit_action_bridge/joint_space`` (cho_robot_config
    registry.py), and the bridge refuses to start under any other node name, so
    this cannot drift from what moveit_action_bridge.py actually serves.

    It is served by that bridge, not by a controller -- MoveIt executes through
    ``controllers.moveit_trajectory``, so that controller must be ACTIVE for a
    goal sent here to move anything.
    """
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    preferences = ((registry.get('actions') or {}).get('preferences') or {}).get('joint') or []
    if not preferences:
        raise ValueError(
            "robot_type '%s' declares no actions.preferences.joint, so it has "
            'no MoveIt joint action to home through' % robot_config['robot_type'])
    return preferences[0]


def _preference_action_names() -> List[str]:
    """Every absolute action name any robot config lists as a preference.

    These include the endpoints served by a node that is not a controller --
    the MoveIt bridge's ``joint_space`` / ``task_space`` -- so they do not come
    out of the controller roles, and the registry has already validated their
    shape.
    """
    names: List[str] = []
    for registry in _registry_entries():
        preferences = (registry.get('actions') or {}).get('preferences') or {}
        for space in ('joint', 'task', 'gripper', 'vla'):
            for name in preferences.get(space) or []:
                if name not in names:
                    names.append(name)
    return names


def _trajectory_controller_names() -> List[str]:
    """Every robot's ``controllers.moveit_trajectory``, from the raw registry.

    Read from the registry rather than from the compatibility view, which does
    not carry this role: the view exposes the roles a task tree commands
    directly (joint_space, task_space, gripper, vla, pour), and the trajectory
    controller is normally driven by MoveIt or by an external executor instead.
    """
    names: List[str] = []
    for registry in _registry_entries():
        controller = (registry.get('controllers') or {}).get('moveit_trajectory')
        if controller and controller not in names:
            names.append(controller)
    return names


def _pour_controller_names() -> List[str]:
    """Every robot's pour role, the one controller still named the old way."""
    names: List[str] = []
    for registry in _registry_entries():
        controller = (registry.get('controllers') or {}).get('pour')
        if controller and controller not in names:
            names.append(controller)
    return names


def valid_controller_action_names() -> List[str]:
    """Action names accepted by BaseActionBehavior, all from the registry.

    Every action kind of every controller a registry entry names, built by
    :func:`controller_action_name`. Includes each trajectory controller's own
    FollowJointTrajectory endpoint, so a behaviour that replays a recorded
    trajectory goes through the same name check as every other action
    behaviour instead of around it, and the pour role's legacy name. Nothing
    comes from :class:`ControllerNames`: a controller the registry does not
    know is not one a tree can address.
    """
    names: List[str] = []

    def add(action_name):
        if action_name not in names:
            names.append(action_name)

    for controller in _command_controller_names():
        for kind in ACTION_KINDS:
            add(controller_action_name(controller, kind))
    for controller in _trajectory_controller_names():
        add(controller_action_name(controller, 'follow_joint_trajectory'))
    for controller in _pour_controller_names():
        add(controller_action_name(controller, POUR_ACTION_KIND))
    for action_name in _preference_action_names():
        add(action_name)
    return names


def vla_completion_service_name(controller):
    """Completion service a VLA controller calls when its goal ends.

    The controller derives this from its OWN action name (it creates the client
    as ``~/vla/notify_completion``), so the two must agree:
    ``/<vla controller>/vla/notify_completion``. They differ per robot --
    Franka's is `vla_controller` and OpenArm MIT's is `vla_mit_controller` --
    so *controller* is required: pass ``load_robot_config(...)['vla']``.
    """
    if not controller:
        raise ValueError(
            'vla_completion_service_name() needs the VLA controller of the robot '
            "the tree is built for (load_robot_config(...)['vla']); this robot "
            'may declare none')
    return f"{controller_action_name(controller, 'vla')}/notify_completion"
