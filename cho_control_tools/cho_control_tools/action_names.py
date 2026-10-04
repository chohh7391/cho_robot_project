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


def static_scene_ready_service(robot_type, profile='single'):
    """The service one robot profile's static planning-scene gate answers on.

    Copy of ``cho_robot_config.static_scene_ready_service``: each profile has
    its own gate, so a client for one arm never waits on, or skips, another's.
    """
    if profile in (None, '', 'single'):
        return f'/cho_moveit/{robot_type}/static_scene_ready'
    return f'/cho_moveit/{robot_type}/{profile}/static_scene_ready'


def task_goal_frame(config, relative):
    """The ``frame_id`` a TaskSpace goal for *config*'s robot profile is stamped with.

    Copy of ``cho_robot_config.task_goal_frame``, reading the same ``model``
    keys of the registry entry or of the bundled metadata: ``''`` where none
    is declared, which the server reads as the frame it means.
    """
    model = (config or {}).get('model') or {}
    return model.get('relative_goal_frame' if relative else 'absolute_goal_frame') or ''


def goal_joint_names(config):
    """The joint names a JointSpace target for *config*'s profile carries ([] if unknown)."""
    model = (config or {}).get('model') or {}
    return list(model.get('joints') or [])


def serving_node(action_name):
    """The node an action is served by: everything before its kind."""
    return action_name.strip('/').rsplit('/', 1)[0]


def action_kind(action_name):
    """The kind an action name ends in (``joint_space``, ``gripper``, ...)."""
    return action_name.rstrip('/').rsplit('/', 1)[-1]
