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
    'joint_trajectory_controller',
    'joint_space_position_controller',
]

VELOCITY_CONTROLLERS = [
    'joint_space_velocity_controller',
]

# Direct producers are always one seven-axis producer per arm.  On the
# bimanual model they may be brought up as two *independent* controller
# instances; they never use the 14-axis pair ownership token.  That token is
# reserved for the MoveIt FJT controller below.
MIT_DIRECT_CONTROLLERS = frozenset({
    'joint_position_mit_controller',
    'joint_impedance_mit_controller',
    'task_space_impedance_mit_controller',
})
MIT_SINGLE_FJT_CONTROLLER = 'single_arm_follow_joint_trajectory_mit_controller'
MIT_PAIRED_FJT_CONTROLLER = 'bimanual_follow_joint_trajectory_mit_controller'
REAL_MIT_DIRECT_CONTROLLERS = frozenset({
    'joint_impedance_mit_controller',
    'task_space_impedance_mit_controller',
})
# Only the action-producing MIT controllers implement the acknowledged,
# bounded return-to-zero phase.  Topic producers and paired trajectory
# controllers intentionally remain outside this launch contract.
RETURN_TO_ZERO_MIT_CONTROLLERS = frozenset({
    'joint_impedance_mit_controller',
    'task_space_impedance_mit_controller',
})


def enforce_mujoco_mit_description(enabled, requested_xacro, canonical_xacro):
    """Forbid replacing the hardware/plugin boundary of the MIT prototype."""
    if not as_bool(enabled):
        return
    if os.path.realpath(requested_xacro) != os.path.realpath(canonical_xacro):
        raise RuntimeError(
            "xacro_file overrides are forbidden with mujoco_mit_prototype:=true; "
            "the canonical OpenArm description selects the approved MIT wrapper.")


def resolve_mujoco_mit_selection(enabled, control_mode, bimanual,
                                 controller_name, arm, controllers_file=''):
    """Resolve the opt-in MIT controller without starting any launch actions.

    ``None`` is deliberately returned when disabled so the legacy selection
    path remains authoritative.  A direct controller is a one-arm producer:
    bimanual direct bringup selects one or two disjoint seven-axis instances.
    The 14-axis ownership token is a separate MoveIt-only boundary.
    """
    if not as_bool(enabled):
        return None
    if controllers_file:
        raise RuntimeError(
            "controllers_file overrides are forbidden with mujoco_mit_prototype:=true; "
            "the approved MIT controller-to-plugin mapping is fail-closed.")
    if control_mode != 'torque':
        raise RuntimeError(
            "'mujoco_mit_prototype' requires control_mode:=torque for its internal "
            "effort actuator. The approved MIT producer path owns that actuator.")
    paired_fjt = controller_name == MIT_PAIRED_FJT_CONTROLLER
    if paired_fjt:
        if not as_bool(bimanual) or arm != 'both':
            raise RuntimeError(
                "The paired MIT FJT requires bimanual:=true and mit_arm:=both")
        return {
            'controller_name': MIT_PAIRED_FJT_CONTROLLER,
            'controller_names': [MIT_PAIRED_FJT_CONTROLLER],
            'controllers_file': 'controllers_mit_moveit_bimanual.yaml',
            'controller_overrides': {},
        }
    mit_fjt = controller_name == MIT_SINGLE_FJT_CONTROLLER
    if controller_name not in MIT_DIRECT_CONTROLLERS | {MIT_SINGLE_FJT_CONTROLLER}:
        raise RuntimeError(f"Unknown MIT controller: {controller_name}")
    if mit_fjt and not as_bool(bimanual):
        raise RuntimeError(
            "The single-arm MIT FJT selects left or right names from the bimanual "
            "model; set bimanual:=true and mit_arm:=left|right.")
    if mit_fjt:
        if arm not in ('left', 'right'):
            raise RuntimeError("single-arm MIT FJT requires mit_arm:=left or mit_arm:=right")
        name = f'{arm}_follow_joint_trajectory_mit_controller'
        return {
            'controller_name': name,
            'controller_names': [name],
            'controllers_file': 'controllers_mit_bimanual.yaml',
            'controller_overrides': {name: {'arm': arm}},
        }
    if not as_bool(bimanual):
        # The single-arm description exports unprefixed interfaces. `mit_arm`
        # is intentionally ignored here so one operator command cannot remap
        # the physical resource contract.
        return {
            'controller_name': controller_name,
            'controller_names': [controller_name],
            'controllers_file': 'controllers_mit.yaml',
            'controller_overrides': {},
        }
    if arm not in ('left', 'right', 'both_independent'):
        raise RuntimeError(
            "bimanual direct MIT requires mit_arm:=left, right, or both_independent; "
            "use mit_arm:=both only with the paired MoveIt FJT")
    sides = ('left', 'right') if arm == 'both_independent' else (arm,)
    names = [f'{side}_{controller_name}' for side in sides]
    return {
        # Kept for compatibility with callers that display one selected name;
        # `controller_names` is authoritative for activation.
        'controller_name': names[0],
        'controller_names': names,
        'controllers_file': 'controllers_mit_direct_bimanual.yaml',
        'controller_overrides': {
            name: {'arm': side} for name, side in zip(names, sides)},
    }


def resolve_real_mit_selection(bimanual, controller_name, arm, controllers_file=''):
    """Resolve the fixed real-MIT ownership map without launching hardware.

    The real path deliberately accepts only direct seven-axis action
    controllers.  Collision-aware paired MoveIt remains a separate
    commissioning item because the real timing and pair-skew evidence has not
    yet been collected.  No user-supplied YAML may replace this boundary.
    """
    if controllers_file:
        raise RuntimeError(
            'controllers_file overrides are forbidden for real MIT bringup; '
            'the hardware/profile boundary is fixed.')
    if controller_name not in REAL_MIT_DIRECT_CONTROLLERS:
        raise RuntimeError(
            f"Unknown real MIT controller '{controller_name}'. Valid options: "
            f"{sorted(REAL_MIT_DIRECT_CONTROLLERS)}")
    if not as_bool(bimanual):
        return {
            'controller_names': [controller_name],
            'controllers_file': 'controllers_mit.yaml',
            'controller_overrides': {},
        }
    if arm not in ('left', 'right', 'both_independent'):
        raise RuntimeError(
            'real bimanual direct MIT requires mit_arm:=left, right, or '
            'both_independent')
    sides = ('left', 'right') if arm == 'both_independent' else (arm,)
    names = [f'{side}_{controller_name}' for side in sides]
    return {
        'controller_names': names,
        'controllers_file': 'controllers_mit_bimanual.yaml',
        'controller_overrides': {
            name: {'arm': side} for name, side in zip(names, sides)},
    }


def resolve_real_mit_hardware_scope(bimanual, arm, hand=False):
    """Limit real bimanual startup to the arm(s) selected for MIT control.

    A non-selected component is a state-only GenericSystem in the canonical
    description. It keeps the bimanual model visible without constructing a
    real transport, CAN socket, or motor command interface.

    The gripper controller follows the same selection. A non-selected arm's
    finger is state-only for the same reason its arm joints are, so spawning a
    gripper controller there would fail on a missing position command
    interface and take the whole spawner down with it.
    """
    if not as_bool(bimanual):
        return {
            'always_active_controllers': always_active_controllers(False),
            'optional_controllers': gripper_controllers(False) if as_bool(hand) else [],
        }
    if arm not in ('left', 'right', 'both_independent'):
        raise RuntimeError(
            'real bimanual hardware scope requires mit_arm:=left, right, or '
            'both_independent')
    sides = ('left', 'right') if arm == 'both_independent' else (arm,)
    return {
        'always_active_controllers': (
            ['joint_state_broadcaster'] +
            [f'{side}_ee_state_broadcaster' for side in sides]),
        'optional_controllers': (
            [f'{side}_gripper_controller' for side in sides] if as_bool(hand) else []),
    }


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


def gripper_controllers(bimanual=False):
    """One robot-independent gripper controller instance per hand."""
    return per_arm('gripper_controller', bimanual)


def always_active_controllers(bimanual=False):
    """joint_state_broadcaster covers the whole model; the EE broadcaster is per arm.

    The gripper is deliberately NOT here. One spawner loads its whole list in
    order and dies on the first failure, so a gripper in this list takes the arm
    controller down with it: measured on hardware 2026-09-05, a gripper
    controller that could not be loaded left the motors energised in their SAFE
    hold with no arm controller at all. Grippers go through
    `optional_controllers`, spawned separately and last.
    """
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


def create_runtime_param_file(controller_names, bringup_type, control_mode, ee_name,
                              controller_overrides=None):
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
        if controller_overrides and controller_name in controller_overrides:
            params.update(controller_overrides[controller_name])
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
                               use_sim_time=None, timeout=None,
                               optional_controllers=()):
    """Spawn the always-active set plus the requested controllers, rest inactive.

    initial_active_controllers is a list because a bimanual build brings up one
    arm controller per arm; they claim disjoint interfaces, so both are active.

    `optional_controllers` get a spawner of their own, started only after the
    arm is up. A spawner loads its list in order and exits on the first
    failure, so anything that shares a list with the arm controller can prevent
    it from ever being spawned - which on real hardware means energised motors
    with nothing commanding them. Peripherals whose absence should degrade
    rather than disable the robot belong here.
    """
    switchable = unique_names(switchable_controllers)
    initial = [c for c in unique_names(initial_active_controllers) if c in switchable]

    # Active set: the broadcasters first so they are up before any arm
    # controller activates.
    active_controllers = unique_names(list(always_active) + initial)
    inactive_controllers = [c for c in switchable if c not in initial]
    optional = [c for c in unique_names(optional_controllers) if c not in active_controllers]

    active_spawner = _make_spawner_node(
        active_controllers, runtime_param_file, active=True,
        use_sim_time=use_sim_time, timeout=timeout)

    followers = []
    if optional:
        followers.append(_make_spawner_node(
            optional, runtime_param_file, active=True,
            use_sim_time=use_sim_time, timeout=timeout))
    if inactive_controllers:
        followers.append(_make_spawner_node(
            inactive_controllers, runtime_param_file, active=False,
            use_sim_time=use_sim_time, timeout=timeout))

    if not followers:
        return [active_spawner]

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=followers,
            )
        ),
        active_spawner,
    ]
