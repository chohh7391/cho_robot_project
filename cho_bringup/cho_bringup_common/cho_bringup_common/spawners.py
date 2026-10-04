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

"""controller_manager spawners, and the order they run in."""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from .utils import unique_names


def make_spawner_node(controller_names, runtime_param_file=None, active=True, use_sim_time=None,
                      namespace=None, timeout=None, condition=None, controller_manager=None):
    """One spawner process for the whole of `controller_names`.

    A spawner loads, configures and activates its list in one deterministic
    sequence. One spawner per controller running concurrently races on
    switch_controller and intermittently leaves one of them inactive.

    runtime_param_file is forwarded with '-p' only for a controller_manager that
    cannot take the parameters as its own node parameters (one living inside a
    simulator plugin). Otherwise they are already loaded on the node, which
    avoids the spawner -> controller_manager file handoff entirely.

    use_sim_time is a parameter dict such as {'use_sim_time': ...}, or None for
    a spawner with no parameters. timeout is the --controller-manager-timeout.
    """
    arguments = list(controller_names)
    if runtime_param_file is not None:
        arguments += ['-p', runtime_param_file]
    if controller_manager is not None:
        arguments += ['--controller-manager', controller_manager]
    if not active:
        arguments.append('--inactive')
    if timeout:
        arguments += ['--controller-manager-timeout', str(timeout)]

    node_kwargs = {
        'package': 'controller_manager',
        'executable': 'spawner',
        'arguments': arguments,
        'parameters': [use_sim_time] if use_sim_time is not None else [],
        'output': 'screen',
    }
    if namespace is not None:
        node_kwargs['namespace'] = namespace
    if condition is not None:
        node_kwargs['condition'] = condition
    return Node(**node_kwargs)


def chain_spawners(first, followers):
    """Start `first` now and `followers` once it has exited, however it ended.

    Returns the actions to launch: the handler is listed before `first`, so it
    is registered before the process it waits for can exit.
    """
    followers = [action for action in followers if action is not None]
    if not followers:
        return [first]
    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=first, on_exit=followers)
        ),
        first,
    ]


def create_controller_spawners(always_active, switchable_controllers, initial_active_controllers,
                               runtime_param_file=None, use_sim_time=None, timeout=None,
                               optional_controllers=(), namespace=None, controller_manager=None):
    """Spawn the always-active set plus the requested controllers; the rest inactive.

    initial_active_controllers is a name or a list: a bimanual build brings up
    one arm controller per arm, and they claim disjoint interfaces, so all of
    them are active. Only names that are also switchable are activated.

    `optional_controllers` get a spawner of their own, started only after the
    active one has exited. A spawner loads its list in order and exits on the
    first failure, so anything that shares a list with the arm controller can
    prevent it from ever being spawned - which on real hardware means energised
    motors with nothing commanding them. Peripherals whose absence should
    degrade rather than disable the robot belong here.

    Returns the actions to launch. The active spawner is the only top-level
    Node among them (see top_level_spawner()); the followers are nested in the
    OnProcessExit handler that starts them.
    """
    if isinstance(initial_active_controllers, str):
        initial_active_controllers = [initial_active_controllers]
    switchable = unique_names(switchable_controllers)
    initial = [c for c in unique_names(initial_active_controllers) if c in switchable]

    # The broadcasters first, so they are up before any arm controller activates.
    active_controllers = unique_names(list(always_active) + initial)
    inactive_controllers = [c for c in switchable if c not in initial]
    optional = [c for c in unique_names(optional_controllers) if c not in active_controllers]

    spawner_kwargs = {
        'runtime_param_file': runtime_param_file,
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'timeout': timeout,
        'controller_manager': controller_manager,
    }
    active_spawner = make_spawner_node(active_controllers, active=True, **spawner_kwargs)

    followers = []
    if optional:
        followers.append(make_spawner_node(optional, active=True, **spawner_kwargs))
    if inactive_controllers:
        followers.append(make_spawner_node(inactive_controllers, active=False, **spawner_kwargs))
    return chain_spawners(active_spawner, followers)


def top_level_spawner(actions):
    """The one spawner Node in `actions` that is not nested in an event handler.

    That is the active spawner of create_controller_spawners(), the one whose
    exit means the requested controller is up (or failed to come up).
    """
    nodes = [action for action in actions if isinstance(action, Node)]
    if len(nodes) != 1:
        raise RuntimeError(
            f'expected exactly one top-level spawner Node, got {len(nodes)}')
    return nodes[0]
