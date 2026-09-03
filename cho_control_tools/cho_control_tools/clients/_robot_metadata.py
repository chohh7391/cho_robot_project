"""Lazy adapters for the metadata shipped with each operator executable.

The generic debug client deliberately uses ``cho_robot_config``. Operator
clients must also work in a robot-specific workspace that does not install that
central registry, so each loader imports only its own small metadata module.
"""


def _load_fr5(profile):
    from .fr5 import metadata
    return metadata.load(profile)


def _load_franka(profile):
    from .franka import metadata
    return metadata.load(profile)


def _load_openarm(profile):
    from .openarm import metadata
    return metadata.load(profile)


def _load_ur5e(profile):
    from .ur5e import metadata
    return metadata.load(profile)


_LOADERS = {
    'fr5': _load_fr5,
    'franka': _load_franka,
    'openarm': _load_openarm,
    'ur5e': _load_ur5e,
}


def config_loader_for(robot_type):
    """Return a loader fixed to one robot and free of registry dependencies."""
    try:
        loader = _LOADERS[robot_type]
    except KeyError as exc:
        raise ValueError(f'No bundled action-client metadata for {robot_type}') from exc

    def load_selected_robot(requested_robot_type, profile='single'):
        if requested_robot_type != robot_type:
            raise ValueError(
                f'Robot-specific client is fixed to {robot_type}, not {requested_robot_type}')
        return loader(profile)
    return load_selected_robot


def home_pose_policy(config, selector):
    """Return the local home-pose policy without consulting the registry."""
    policy = config.get('poses', {}).get('home_safety', {}).get(str(selector))
    if policy is None:
        return {'enabled': True, 'reason': ''}
    return dict(policy)
