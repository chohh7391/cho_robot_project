"""Action names under the controller action contract (cho_interfaces/CONTRACT.md).

Every controller serves its actions under its own node, ``/<controller>/<kind>``,
and the MoveIt bridge serves ``joint_space`` / ``task_space`` the same way under
``<robot>[_<profile>]_moveit_action_bridge``. This is the operator tools' copy
of the rule in ``cho_robot_config``: the robot-scoped clients must run in a
workspace that does not install the registry, so they cannot import it, and
``test_robot_scoped_entrypoints`` checks the two never disagree.

The FR5 pour action is application-specific and keeps its own name; see
``clients/fr5/pour_client.py``.
"""

# Relative names a controller serves, by the space the operator tools use.
ACTION_KINDS = {
    'joint': 'joint_space',
    'task': 'task_space',
    'gripper': 'gripper',
    'vla': 'vla',
}

MOVEIT_BRIDGE_SUFFIX = 'moveit_action_bridge'


def controller_action_name(controller, kind):
    """``/<controller>/<kind>``; *kind* is a space above or its action kind."""
    kind = ACTION_KINDS.get(kind, kind)
    if kind not in ACTION_KINDS.values():
        raise ValueError(
            f"unknown action kind '{kind}'; expected one of {sorted(ACTION_KINDS.values())}")
    node = str(controller).strip('/')
    if not node:
        raise ValueError('controller name must be non-empty')
    return f'/{node}/{kind}'


def moveit_bridge_node(robot_type, profile='single'):
    """The MoveIt bridge's node name for one robot profile."""
    if profile in (None, '', 'single'):
        return f'{robot_type}_{MOVEIT_BRIDGE_SUFFIX}'
    return f'{robot_type}_{profile}_{MOVEIT_BRIDGE_SUFFIX}'


def serving_node(action_name):
    """The node an action is served by: everything before its kind."""
    return action_name.strip('/').rsplit('/', 1)[0]


def action_kind(action_name):
    """The kind an action name ends in (``joint_space``, ``gripper``, ...)."""
    return action_name.rstrip('/').rsplit('/', 1)[-1]
