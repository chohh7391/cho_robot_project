# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Load and validate canonical per-robot metadata."""

from copy import deepcopy
import math
import os
from pathlib import Path

import yaml


_cache = {}
_required_controller_roles = {
    'hold', 'direct_joint', 'direct_task', 'moveit_trajectory', 'gripper', 'vla'
}

# Bringup control modes. A bringup exports exactly one command interface per
# joint, so the set of controllers that can even be loaded - and therefore the
# one that can hold the arm - depends on which mode it was started in.
CONTROL_MODES = ('position', 'velocity', 'torque')

# What a controller serves under its own node (cho_interfaces/CONTRACT.md):
# `/<controller>/<kind>`. The MoveIt bridge, which is not a controller, follows
# the same rule under its node. The names built here are ABSOLUTE, so a robot
# started under a ROS namespace needs its registry entry (and the bridge's node
# name check) to say so; an arm prefix is just part of the controller name.
ACTION_KINDS = ('joint_space', 'task_space', 'gripper', 'vla', 'follow_joint_trajectory')

# actions.preferences key -> the action kind every entry of that list must name.
PREFERENCE_ACTION_KINDS = {
    'joint': 'joint_space', 'task': 'task_space', 'gripper': 'gripper', 'vla': 'vla',
}


def controller_action_name(controller, kind):
    """The absolute name of action *kind* served by node *controller*.

    *controller* is a controller name (or the MoveIt bridge's node name, see
    :func:`moveit_bridge_node`); *kind* is one of :data:`ACTION_KINDS`. This is
    the one place the naming rule is written
    down: every client builds its action names through it, never from a string
    of its own.
    """
    node = str(controller).strip('/')
    if not node:
        raise ValueError('controller name must be non-empty')
    if kind not in ACTION_KINDS:
        raise ValueError(
            f"unknown action kind '{kind}'; expected one of {list(ACTION_KINDS)}")
    return f'/{node}/{kind}'


def moveit_bridge_node(robot_type, profile='single'):
    """The node name the MoveIt action bridge runs under for one robot profile.

    The bridge serves ``~/joint_space`` and ``~/task_space``, so its node name
    is what scopes those actions to one robot: a client for one robot can never
    bind to another robot's bridge. The launch files take the name from
    :func:`load_moveit_metadata` and the bridge refuses to start under any
    other, so the three cannot drift.
    """
    robot_type = str(robot_type).strip('/')
    if not robot_type:
        raise ValueError('robot_type must be non-empty')
    if profile in (None, '', 'single'):
        return f'{robot_type}_moveit_action_bridge'
    return f'{robot_type}_{profile}_moveit_action_bridge'


def static_scene_ready_service(robot_type, profile='single'):
    """The service the static planning-scene gate answers on for one robot profile.

    Each profile's MoveIt stack has its own gate, so a client waiting for one
    arm's scene never takes another's as ready.
    """
    robot_type = str(robot_type).strip('/')
    if not robot_type:
        raise ValueError('robot_type must be non-empty')
    if profile in (None, '', 'single'):
        return f'/cho_moveit/{robot_type}/static_scene_ready'
    return f'/cho_moveit/{robot_type}/{profile}/static_scene_ready'


def task_goal_frame(config, relative):
    """The ``frame_id`` a client stamps on a TaskSpace goal for this robot profile.

    Absolute goals: ``model.absolute_goal_frame``, a frame every task-space
    controller of the profile has among its root frames
    (cho_controller_base::root_frames()) on every bringup -- cho_task_manager's
    test_goal_frames proves it against the descriptions. Relative goals:
    ``model.relative_goal_frame``, the EE frame, declared only where it is the
    task-space controllers' fixed ``ee_name``. ``''`` when the registry
    declares none, which every controller reads as the frame it means.
    """
    model = _mapping(config, 'config').get('model', {})
    key = 'relative_goal_frame' if relative else 'absolute_goal_frame'
    return model.get(key) or ''


def _validate_goal_frames(model, robot_type):
    """Validate the optional frames clients stamp on TaskSpace goals."""
    absolute = model.get('absolute_goal_frame')
    _optional_name(absolute, f'{robot_type}: model.absolute_goal_frame')
    if absolute is not None and absolute not in (model['base_frame'], model['arm_base_link']):
        # Only a frame the registry already names: anything else would be a
        # third frame nobody else in the stack knows about.
        raise ValueError(
            f"{robot_type}: model.absolute_goal_frame '{absolute}' must be model.base_frame "
            f"('{model['base_frame']}') or model.arm_base_link ('{model['arm_base_link']}')")
    relative = model.get('relative_goal_frame')
    _optional_name(relative, f'{robot_type}: model.relative_goal_frame')
    if relative is not None and relative != model['ee_link']:
        # A profile that changes ee_link inherits this key unless it restates
        # it; failing here keeps a bimanual arm from stamping the other arm's
        # (or the single arm's) EE frame on its goals.
        raise ValueError(
            f"{robot_type}: model.relative_goal_frame '{relative}' must be model.ee_link "
            f"('{model['ee_link']}') or null")


def _config_dir() -> Path:
    override = os.environ.get('CHO_ROBOT_CONFIG_DIR')
    if override:
        return Path(override)
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('cho_robot_config')) / 'config'
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / 'config'


def available_robot_types():
    """Return robot types represented by registry YAML files."""
    paths = sorted(_config_dir().glob('*.yaml'))
    declared = []
    for path in paths:
        with path.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream) or {}
        if isinstance(document, dict):
            declared.append(document.get('robot_type'))
    nonempty = [name for name in declared if isinstance(name, str) and name]
    if len(nonempty) != len(set(nonempty)):
        raise ValueError('robot_type values must be unique across the registry')
    return [path.stem for path in paths]


def _vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{label} must contain exactly {length} values')
    if not all(not isinstance(item, bool) and isinstance(item, (int, float))
               and math.isfinite(item) for item in value):
        raise ValueError(f'{label} must contain only finite numbers')


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def _optional_name(value, label):
    if value is not None and (not isinstance(value, str) or not value):
        raise ValueError(f'{label} must be null or a non-empty string')


def _validate_hold_by_control_mode(mapping, robot_type):
    """Validate the optional per-control-mode hold declaration.

    Optional so a registry entry written before this key keeps validating; a
    consumer that needs it says so by calling
    :func:`hold_controllers_for_control_mode`, which raises when it is absent.
    """
    if mapping is None:
        return
    label = f'{robot_type}: controllers.hold_by_control_mode'
    mapping = _mapping(mapping, label)
    if not mapping:
        raise ValueError(f'{label} must not be empty when present')
    unknown = sorted(set(mapping) - set(CONTROL_MODES))
    if unknown:
        raise ValueError(f'{label} declares unknown control modes: {unknown}')
    for mode, value in mapping.items():
        # A single arm holds with one controller; a 14-axis bimanual profile
        # needs one per arm, so a list is accepted for the same role.
        names = value if isinstance(value, list) else [value]
        if not names:
            raise ValueError(f'{label}.{mode} must name at least one controller')
        if len(names) != len(set(names)):
            raise ValueError(f'{label}.{mode} must not repeat a controller')
        for name in names:
            if not isinstance(name, str) or not name:
                raise ValueError(
                    f'{label}.{mode} must contain only non-empty controller names')


def _validate_additional_arm(value, controllers, robot_type):
    """Validate the optional list of arm controllers that hold no named role.

    A bringup can load more arm controllers than the roles name - Franka's
    torque bringup alone loads six. They claim the same arm command interfaces
    as the role controllers, so an exclusive switch has to know them to take
    them down; listing them here is what keeps that knowledge out of code.
    """
    if value is None:
        return
    label = f'{robot_type}: controllers.additional_arm'
    if (not isinstance(value, list)
            or not all(isinstance(name, str) and name for name in value)
            or len(value) != len(set(value))):
        raise ValueError(f'{label} must be a unique list of non-empty controller names')
    # The gripper claims the finger interfaces, not the arm's: an exclusive arm
    # switch that took it down would drop whatever the jaws were holding.
    gripper = controllers.get('gripper')
    if gripper is not None and gripper in value:
        raise ValueError(f'{label} must not name the gripper controller ({gripper})')


def _validate_task_home(value, home, joint_count, home_safety, robot_type):
    """Validate the optional ``poses.task_home``.

    Either a ``poses.home`` selector, so a pose the operator tools also use is
    written once, or a joint vector of its own for a robot whose task trees
    start somewhere none of the operator presets is.
    """
    if value is None:
        return
    label = f'{robot_type}: poses.task_home'
    if isinstance(value, str):
        if value not in home:
            raise ValueError(
                f"{label} names home selector '{value}', which poses.home does not "
                f'declare; declared: {sorted(home)}')
        policy = _mapping(home_safety.get(value, {}), f'{label} policy')
        if policy.get('enabled', True) is False:
            raise ValueError(
                f"{label} names home selector '{value}', which poses.home_safety "
                f"disables: {policy.get('reason', '')}")
        return
    _vector(value, joint_count, label)


def validate_robot_config(config, expected_robot_type=None):
    """Validate a registry document and return it unchanged."""
    _mapping(config, 'config')
    if isinstance(config.get('schema_version'), bool) or config.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    robot_type = config.get('robot_type')
    if not isinstance(robot_type, str) or not robot_type:
        raise ValueError('robot_type must be a non-empty string')
    if expected_robot_type is not None and robot_type != expected_robot_type:
        raise ValueError(
            f"config filename '{expected_robot_type}' declares robot_type '{robot_type}'")
    profile = config.get('profile', 'single')
    if not isinstance(profile, str) or not profile or '/' in profile:
        raise ValueError(f'{robot_type}: profile must be a non-empty ROS-name segment')
    supports_task = config.get('supports_task', True)
    if not isinstance(supports_task, bool):
        raise ValueError(f'{robot_type}: supports_task must be boolean')

    model = _mapping(config.get('model'), f'{robot_type}: model')
    joints = model.get('joints')
    if (not isinstance(joints, list) or not joints
            or not all(isinstance(name, str) and name for name in joints)
            or len(joints) != len(set(joints))):
        raise ValueError(f'{robot_type}: model.joints must be a non-empty unique list')
    for field in ('base_frame', 'arm_base_link', 'ee_link'):
        if not isinstance(model.get(field), str) or not model[field]:
            raise ValueError(f'{robot_type}: model.{field} is required')
    _validate_goal_frames(model, robot_type)

    controllers = _mapping(config.get('controllers'), f'{robot_type}: controllers')
    missing = _required_controller_roles - set(controllers)
    if missing:
        raise ValueError(f'{robot_type}: missing controller roles: {sorted(missing)}')
    if controllers['hold'] is None or controllers['moveit_trajectory'] is None:
        raise ValueError(f'{robot_type}: hold and moveit_trajectory controllers are required')
    for role, controller in controllers.items():
        if role in ('hold_by_control_mode', 'additional_arm'):
            continue
        _optional_name(controller, f'{robot_type}: controllers.{role}')
    _validate_hold_by_control_mode(
        controllers.get('hold_by_control_mode'), robot_type)
    _validate_additional_arm(controllers.get('additional_arm'), controllers, robot_type)

    moveit = _mapping(config.get('moveit'), f'{robot_type}: moveit')
    for field in ('config_package', 'planning_group'):
        if not isinstance(moveit.get(field), str) or not moveit[field]:
            raise ValueError(f'{robot_type}: moveit.{field} is required')
    execution = _mapping(moveit.get('execution'), f'{robot_type}: moveit.execution')
    for field in ('max_velocity_scaling_factor', 'max_acceleration_scaling_factor'):
        value = execution.get(field)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0.0 < value <= 1.0):
            raise ValueError(
                f'{robot_type}: moveit.execution.{field} must be finite and in (0, 1]')

    poses = _mapping(config.get('poses'), f'{robot_type}: poses')
    home = _mapping(poses.get('home'), f'{robot_type}: poses.home')
    joint_reach = poses.get('reach')
    if joint_reach is not None:
        joint_reach = _mapping(joint_reach, f'{robot_type}: poses.reach')
    home_safety = poses.get('home_safety', {})
    home_safety = _mapping(home_safety, f'{robot_type}: poses.home_safety')
    motion_config = _mapping(config.get('motions'), f'{robot_type}: motions')
    motions = _mapping(motion_config.get('reach'), f'{robot_type}: motions.reach')
    for selector in ('0', '1', '2', '3'):
        if selector not in home:
            raise ValueError(f'{robot_type}: poses.home.{selector} is required')
        _vector(home[selector], len(joints), f'{robot_type}: poses.home.{selector}')
        if joint_reach is not None:
            if selector not in joint_reach:
                raise ValueError(f'{robot_type}: poses.reach.{selector} is required')
            _vector(
                joint_reach[selector], len(joints),
                f'{robot_type}: poses.reach.{selector}')
        if selector in home_safety:
            policy = _mapping(
                home_safety[selector], f'{robot_type}: poses.home_safety.{selector}')
            if not isinstance(policy.get('enabled'), bool):
                raise ValueError(
                    f'{robot_type}: poses.home_safety.{selector}.enabled must be boolean')
            reason = policy.get('reason')
            if not isinstance(reason, str) or not reason.strip():
                raise ValueError(
                    f'{robot_type}: poses.home_safety.{selector}.reason is required')
            max_joint_distance = policy.get('max_joint_distance')
            if (isinstance(max_joint_distance, bool)
                    or not isinstance(max_joint_distance, (int, float))
                    or not math.isfinite(max_joint_distance)
                    or max_joint_distance < 0.0):
                raise ValueError(
                    f'{robot_type}: poses.home_safety.{selector}.max_joint_distance '
                    'must be a finite non-negative number')
        if selector not in motions:
            raise ValueError(f'{robot_type}: motions.reach.{selector} is required')
        motion = _mapping(motions[selector], f'{robot_type}: motions.reach.{selector}')
        if not isinstance(motion.get('relative'), bool):
            raise ValueError(f'{robot_type}: motions.reach.{selector}.relative must be boolean')
        _vector(motion.get('position'), 3, f'{robot_type}: motions.reach.{selector}.position')
        quaternion = motion.get('orientation')
        _vector(quaternion, 4, f'{robot_type}: motions.reach.{selector}.orientation')
        norm = math.sqrt(sum(component * component for component in quaternion))
        if not math.isclose(norm, 1.0, rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError(
                f'{robot_type}: motions.reach.{selector}.orientation is not normalized')
    _validate_task_home(poses.get('task_home'), home, len(joints), home_safety, robot_type)

    action_config = _mapping(config.get('actions'), f'{robot_type}: actions')
    actions = _mapping(
        action_config.get('preferences'), f'{robot_type}: actions.preferences')
    unknown_spaces = sorted(set(actions) - set(PREFERENCE_ACTION_KINDS))
    if unknown_spaces:
        raise ValueError(
            f'{robot_type}: actions.preferences declares unknown spaces: {unknown_spaces}')
    for space, kind in PREFERENCE_ACTION_KINDS.items():
        names = actions.get(space)
        if names is None and space == 'vla':
            continue
        # Every entry must follow the contract's /<node>/<kind> rule for its
        # own space: a joint list naming a task_space server would bind a
        # JointSpace client to the wrong action type.
        if (not isinstance(names, list)
                or not all(isinstance(name, str) and name.startswith('/')
                           and name.endswith(f'/{kind}')
                           and name[1:-len(kind) - 1].strip('/') for name in names)
                or len(names) != len(set(names))):
            raise ValueError(
                f'{robot_type}: actions.preferences.{space} must be a unique list '
                f'of absolute /<node>/{kind} action names')
    bridge = moveit_bridge_node(robot_type, profile)
    expected_joint = controller_action_name(bridge, 'joint_space')
    expected_task = controller_action_name(bridge, 'task_space')
    if not actions['joint'] or actions['joint'][0] != expected_joint:
        raise ValueError(f'{robot_type}: first joint preference must be {expected_joint}')
    if supports_task and (not actions['task'] or actions['task'][0] != expected_task):
        raise ValueError(f'{robot_type}: first task preference must be {expected_task}')
    if not supports_task and actions['task']:
        raise ValueError(f'{robot_type}: task preferences must be empty when supports_task=false')
    for space, role in (('joint', 'direct_joint'), ('task', 'direct_task')):
        direct = controllers[role]
        if direct is not None and profile == 'single':
            expected_direct = controller_action_name(direct, PREFERENCE_ACTION_KINDS[space])
            if expected_direct not in actions[space]:
                raise ValueError(
                    f'{robot_type}: {space} preferences must contain {expected_direct}')

    gripper_command = action_config.get('gripper_command')
    if gripper_command is not None:
        gripper_command = _mapping(
            gripper_command, f'{robot_type}: actions.gripper_command')
        if (not isinstance(gripper_command.get('topic'), str)
                or not gripper_command['topic'].startswith('/')):
            raise ValueError(f'{robot_type}: gripper command topic must be absolute')
        for field in ('open', 'close'):
            value = gripper_command.get(field)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise ValueError(f'{robot_type}: gripper command {field} must be finite')

    compatibility = config.get('compatibility')
    if compatibility is not None:
        compatibility = _mapping(compatibility, f'{robot_type}: compatibility')
        task_manager = _mapping(
            compatibility.get('task_manager'),
            f'{robot_type}: compatibility.task_manager')
        allowed = {'joint_space', 'task_space', 'gripper', 'vla'}
        if set(task_manager) - allowed:
            raise ValueError(f'{robot_type}: unknown task_manager compatibility role')
        for role, controller in task_manager.items():
            _optional_name(
                controller, f'{robot_type}: compatibility.task_manager.{role}')
    return config


def home_pose_policy(config, selector):
    """Return the execution policy for a named home pose.

    Unannotated poses remain enabled for backwards-compatible robot registries.
    """
    selector = str(selector)
    policy = config.get('poses', {}).get('home_safety', {}).get(selector)
    if policy is None:
        return {'enabled': True, 'reason': ''}
    return deepcopy(policy)


def task_home_pose(config):
    """The joint positions a behaviour-tree task starts and finishes a mission at.

    ``poses.task_home`` is either a ``poses.home`` selector or a joint vector
    (see :func:`validate_robot_config`); this resolves it to the vector, so a
    task never has to know which. Raises ValueError when the robot/profile
    declares none: a task that homes has to go somewhere this registry chose,
    not somewhere it spelled out for itself.
    """
    config = _mapping(config, 'config')
    robot_type = config.get('robot_type', '<unknown>')
    profile = config.get('profile', 'single')
    poses = config.get('poses') or {}
    value = poses.get('task_home')
    if value is None:
        raise ValueError(
            f"Robot '{robot_type}' (profile '{profile}') declares no poses.task_home, "
            'so a task has no home pose to go to. Add one to its cho_robot_config entry.')
    if isinstance(value, str):
        value = poses['home'][value]
    return [float(position) for position in value]


def blocked_home_joint_goals(config):
    """Return disabled home targets and their operator-facing reasons."""
    blocked = []
    for selector, positions in config['poses']['home'].items():
        policy = home_pose_policy(config, selector)
        if not policy['enabled']:
            blocked.append({
                'selector': selector,
                'positions': list(positions),
                'reason': policy['reason'],
                'max_joint_distance': policy['max_joint_distance'],
            })
    return blocked


def hold_controllers_for_control_mode(config, control_mode):
    """Controllers that hold the arm when the bringup ran in *control_mode*.

    ``controllers.hold`` alone cannot answer this. A bringup exports exactly
    one command interface per joint, so the position-interface hold controller
    is not even loaded in a torque bringup: activating it there fails, and the
    exclusive switch path is deliberately BEST_EFFORT, so it fails *quietly*
    and leaves nothing holding the arm. On a torque robot that is worse than
    not trying at all. Raising here keeps the mistake at tree-build time,
    where it costs one message instead of a dropped arm.

    Returns a list because a 14-axis bimanual profile needs one hold
    controller per arm.
    """
    controllers = _mapping(config, 'config').get('controllers', {})
    robot_type = config.get('robot_type', '<unknown>')
    profile = config.get('profile', 'single')
    mapping = controllers.get('hold_by_control_mode')
    if not mapping:
        raise ValueError(
            f"Robot '{robot_type}' (profile '{profile}') declares no "
            'controllers.hold_by_control_mode, so no control-mode-specific hold '
            'controller can be resolved. Add one to its cho_robot_config entry.')
    if control_mode not in mapping:
        raise ValueError(
            f"Robot '{robot_type}' (profile '{profile}') has no hold controller for "
            f"control_mode '{control_mode}'. Declared modes: {sorted(mapping)}")
    value = mapping[control_mode]
    return list(value) if isinstance(value, list) else [value]


def declared_hold_control_modes(config):
    """Control modes this robot/profile declares a hold controller for."""
    mapping = config.get('controllers', {}).get('hold_by_control_mode') or {}
    return sorted(mapping)


def available_profiles(robot_type):
    """Profiles selectable for *robot_type*, always including 'single'."""
    path = _config_dir().resolve() / f'{robot_type}.yaml'
    if not path.is_file():
        raise ValueError(
            f"Unknown robot_type '{robot_type}'. Valid options: {available_robot_types()}")
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    profiles = document.get('profiles') or {}
    if not isinstance(profiles, dict):
        raise ValueError(f'{robot_type}: profiles must be a mapping')
    return ['single'] + sorted(profiles)


def load_robot_config(robot_type, profile=None):
    """Load a validated robot configuration by its stable robot_type key."""
    config_dir = _config_dir().resolve()
    profile = profile or 'single'
    cache_key = (str(config_dir), robot_type, profile)
    if cache_key in _cache:
        return deepcopy(_cache[cache_key])
    path = config_dir / f'{robot_type}.yaml'
    if not path.is_file():
        raise ValueError(
            f"Unknown robot_type '{robot_type}'. Valid options: {available_robot_types()}")
    with path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    validate_robot_config(config, robot_type)
    profiles = config.pop('profiles', {})
    if profile != 'single':
        if profile not in profiles:
            raise ValueError(
                f"Robot '{robot_type}' has no profile '{profile}'. Valid profiles: "
                f"{['single'] + sorted(profiles)}")
        overlay = _mapping(profiles[profile], f'{robot_type}: profiles.{profile}')
        forbidden = {'robot_type', 'schema_version', 'profile'} & set(overlay)
        if forbidden:
            raise ValueError(
                f'{robot_type}: profile overlay may not replace {sorted(forbidden)}')
        # 'actions' and 'compatibility' are REPLACED, not merged: both are
        # nested, so a shallow update would keep the top-level sub-mappings and
        # a profile could not fully own them. That matters for 'compatibility'
        # in particular -- its controller names are per-arm instances on a
        # bimanual build, and inheriting the single-arm name pointed the task
        # trees at a controller that does not exist there.
        for section in ('model', 'controllers', 'moveit', 'actions', 'poses',
                        'motions', 'compatibility'):
            if section in overlay:
                if section in ('actions', 'compatibility'):
                    config[section] = deepcopy(overlay[section])
                else:
                    config[section].update(deepcopy(overlay[section]))
        config['profile'] = profile
        config['supports_task'] = overlay.get('supports_task', True)
    else:
        config['profile'] = 'single'
        config['supports_task'] = True
    validate_robot_config(config, robot_type)
    _cache[cache_key] = config
    return deepcopy(config)


def load_moveit_metadata(robot_type, expected_config_package=None, profile=None):
    """Return validated, launch-friendly MoveIt metadata for one robot.

    Derived ROS names live here so launch files cannot drift independently from
    the canonical robot type. ``expected_config_package`` lets a robot-specific
    package fail early with an actionable message when it is wired to the wrong
    registry entry.
    """
    config = load_robot_config(robot_type, profile)
    package = config['moveit']['config_package']
    if expected_config_package is not None and package != expected_config_package:
        raise ValueError(
            f"Robot '{robot_type}' declares MoveIt package '{package}', expected "
            f"'{expected_config_package}'. Fix cho_robot_config before launching.")
    return {
        'robot_type': config['robot_type'],
        'config_package': package,
        'planning_group': config['moveit']['planning_group'],
        'base_frame': config['model']['base_frame'],
        'arm_base_link': config['model']['arm_base_link'],
        'ee_link': config['model']['ee_link'],
        'joint_names': list(config['model']['joints']),
        'hold_controller': config['controllers']['hold'],
        'trajectory_controller': config['controllers']['moveit_trajectory'],
        'trajectory_controllers': list(config['moveit'].get(
            'controllers', [config['controllers']['moveit_trajectory']])),
        'hold_controllers': list(config['moveit'].get(
            'hold_controllers', [config['controllers']['hold']])),
        'max_velocity_scaling_factor': config['moveit']['execution'][
            'max_velocity_scaling_factor'],
        'max_acceleration_scaling_factor': config['moveit']['execution'][
            'max_acceleration_scaling_factor'],
        'profile': config.get('profile', 'single'),
        'supports_task': config.get('supports_task', True),
        # The bridge's node name scopes its ~/joint_space and ~/task_space to
        # this robot profile, and the registry's first joint/task preferences
        # are validated against it.
        'action_bridge_node': moveit_bridge_node(
            config['robot_type'], config.get('profile', 'single')),
        'ready_service': static_scene_ready_service(
            config['robot_type'], config.get('profile', 'single')),
    }
