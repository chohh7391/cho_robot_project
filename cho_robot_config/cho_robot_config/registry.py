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

    controllers = _mapping(config.get('controllers'), f'{robot_type}: controllers')
    missing = _required_controller_roles - set(controllers)
    if missing:
        raise ValueError(f'{robot_type}: missing controller roles: {sorted(missing)}')
    if controllers['hold'] is None or controllers['moveit_trajectory'] is None:
        raise ValueError(f'{robot_type}: hold and moveit_trajectory controllers are required')
    for role, controller in controllers.items():
        if role == 'hold_by_control_mode':
            continue
        _optional_name(controller, f'{robot_type}: controllers.{role}')
    _validate_hold_by_control_mode(
        controllers.get('hold_by_control_mode'), robot_type)

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

    action_config = _mapping(config.get('actions'), f'{robot_type}: actions')
    actions = _mapping(
        action_config.get('preferences'), f'{robot_type}: actions.preferences')
    for space in ('joint', 'task', 'gripper'):
        names = actions.get(space)
        if (not isinstance(names, list)
                or not all(isinstance(name, str) and name.startswith('/') for name in names)
                or len(names) != len(set(names))):
            raise ValueError(
                f'{robot_type}: actions.preferences.{space} must be a unique list '
                'of absolute action names')
    action_root = f'/{robot_type}' if profile == 'single' else f'/{robot_type}/{profile}'
    expected_joint = f'{action_root}/controller_action_server/moveit_joint'
    expected_task = f'{action_root}/controller_action_server/moveit_task'
    if not actions['joint'] or actions['joint'][0] != expected_joint:
        raise ValueError(f'{robot_type}: first joint preference must be {expected_joint}')
    if supports_task and (not actions['task'] or actions['task'][0] != expected_task):
        raise ValueError(f'{robot_type}: first task preference must be {expected_task}')
    if not supports_task and actions['task']:
        raise ValueError(f'{robot_type}: task preferences must be empty when supports_task=false')
    for space, role in (('joint', 'direct_joint'), ('task', 'direct_task')):
        direct = controllers[role]
        if direct is not None and profile == 'single':
            expected_direct = f'/controller_action_server/{direct}'
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
        'ready_service': (
            f"/cho_moveit/{config['robot_type']}/static_scene_ready"
            if config.get('profile', 'single') == 'single'
            else f"/cho_moveit/{config['robot_type']}/"
                 f"{config['profile']}/static_scene_ready"),
    }
