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

"""Shared bringup helpers for every OpenArm launch file.

Adapted from cho_bringup_franka/utils/launch_utils.py. It lives in one module on
purpose: cho_bringup_ur copies the same runtime-param logic into each of its
three launch files, and the copies have already drifted apart.
"""

import os
import tempfile

from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

import yaml


# Controller names carry a per-arm prefix on a bimanual build - left_ and right_
# - because the same controller class is spawned once per arm. The instance name
# is what the action server and the /ee_state topics are derived from, so two
# instances never collide. A single arm keeps the bare names, which is what
# cho_task_manager's ControllerNames enum and the existing configs expect.
ARM_PREFIXES_BIMANUAL = ['left_', 'right_']

# Base names, before the per-arm prefix. Exactly one arm controller per arm may
# be active at a time; on a bimanual build the left and right instances claim
# disjoint interfaces and are both active.
TORQUE_CONTROLLERS = [
    'joint_space_impedance_controller',
]

# One list per control_mode. The description exports exactly one command
# interface per joint in simulation, so spawning a controller from the wrong list
# would fail on a missing interface - get_switchable_controllers() keeps them
# apart.
POSITION_CONTROLLERS = [
    'joint_space_position_controller',
]

VELOCITY_CONTROLLERS = [
    'joint_space_velocity_controller',
]


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'File not found: {file_path}')
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('true', '1', 'yes', 'on')


def unique_names(names):
    unique = []
    for name in names:
        if name not in unique:
            unique.append(name)
    return unique


def arm_prefixes(bimanual=False):
    return list(ARM_PREFIXES_BIMANUAL) if as_bool(bimanual) else ['']


def per_arm(base_name, bimanual=False):
    return [f'{prefix}{base_name}' for prefix in arm_prefixes(bimanual)]


def always_active_controllers(bimanual=False):
    """joint_state_broadcaster covers the whole model; the EE broadcaster is per arm."""
    return ['joint_state_broadcaster'] + per_arm('ee_state_broadcaster', bimanual)


def get_switchable_controllers(control_mode, requested_controller=None, bimanual=False):
    """Switchable controller instance names for this mode, one set per arm.

    `requested_controller` is a base name (no arm prefix): the caller asks for a
    controller class, and which instances that means follows from the build.
    """
    if control_mode == 'position':
        base_names = list(POSITION_CONTROLLERS)
    elif control_mode == 'velocity':
        base_names = list(VELOCITY_CONTROLLERS)
    else:
        base_names = list(TORQUE_CONTROLLERS)

    if requested_controller:
        base_names.append(requested_controller)

    base_names = unique_names(base_names)
    if not base_names:
        raise RuntimeError(
            f"No OpenArm controllers are implemented for control_mode:='{control_mode}'."
        )
    return unique_names(
        [name for base in base_names for name in per_arm(base, bimanual)])


def create_runtime_param_file(controller_names, bringup_type, control_mode, ee_name):
    """Write the per-bringup controller parameters and return the file path.

    bringup_type, control_mode and ee_name are the same for every controller in a
    given launch but differ between launches, so they are generated here rather
    than duplicated down every block of every controllers.yaml.
    """
    internal_control_mode = 'effort' if control_mode == 'torque' else control_mode
    wildcard_params = {}

    for controller_name in unique_names(controller_names):
        params = {
            'bringup_type': bringup_type,
            'control_mode': internal_control_mode,
        }
        # A bimanual build gives each arm its own ee_name in the controllers
        # file; overriding it from here would point both arms at one hand.
        if ee_name:
            params['ee_name'] = ee_name
        wildcard_params[controller_name] = {'ros__parameters': params}

    # Written under ROS_HOME (default ~/.ros) rather than /tmp. The file is read
    # by a controller_manager started from this same launch, and /tmp is not
    # reliably shared across processes under containers or systemd PrivateTmp.
    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml', prefix='cho_openarm_runtime_params_', dir=runtime_dir)
    with os.fdopen(fd, 'w') as runtime_file:
        yaml.dump({'/**': wildcard_params}, runtime_file)
    return runtime_path


def create_runtime_param_cleanup(runtime_param_file):
    def cleanup(context, *args, **kwargs):
        del context, args, kwargs
        if os.path.exists(runtime_param_file):
            os.unlink(runtime_param_file)
        return []

    return OpaqueFunction(function=cleanup)


def _make_spawner_node(controller_names, runtime_param_file=None, active=True,
                       use_sim_time=None, timeout=None):
    # One spawner process loads/configures/activates the whole list in a single
    # deterministic sequence. One spawner per controller running concurrently
    # races on switch_controller and intermittently leaves one un-activated.
    #
    # runtime_param_file is forwarded with '-p' only when the controller_manager
    # cannot take the parameters directly as node parameters, i.e. the Gazebo
    # in-plugin manager. Otherwise they are already loaded on the node, which
    # avoids the spawner -> controller_manager file handoff entirely.
    spawner_args = list(controller_names)
    if runtime_param_file is not None:
        spawner_args += ['-p', runtime_param_file]
    if not active:
        spawner_args.append('--inactive')
    if timeout:
        spawner_args.extend(['--controller-manager-timeout', str(timeout)])

    parameters = []
    if use_sim_time is not None:
        parameters.append(use_sim_time)

    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_args,
        parameters=parameters,
        output='screen',
    )


def create_controller_spawners(always_active, switchable_controllers,
                               initial_active_controllers, runtime_param_file=None,
                               use_sim_time=None, timeout=None):
    """Spawn the always-active set plus the requested controllers, rest inactive.

    initial_active_controllers is a list because a bimanual build brings up one
    arm controller per arm; they claim disjoint interfaces, so both are active.
    """
    switchable = unique_names(switchable_controllers)
    initial = [c for c in unique_names(initial_active_controllers) if c in switchable]

    # Active set: the broadcasters first so they are up before any arm
    # controller activates.
    active_controllers = unique_names(list(always_active) + initial)
    inactive_controllers = [c for c in switchable if c not in initial]

    active_spawner = _make_spawner_node(
        active_controllers, runtime_param_file, active=True,
        use_sim_time=use_sim_time, timeout=timeout)

    if not inactive_controllers:
        return [active_spawner]

    inactive_spawner = _make_spawner_node(
        inactive_controllers, runtime_param_file, active=False,
        use_sim_time=use_sim_time, timeout=timeout)

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=[inactive_spawner],
            )
        ),
        active_spawner,
    ]
