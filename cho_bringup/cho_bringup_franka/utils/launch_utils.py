# Copyright (c) 2025 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import yaml

from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


ALWAYS_ACTIVE_CONTROLLERS = [
    'joint_state_broadcaster',
    'ee_state_broadcaster',
    'simulation_gripper_controller',
    'gripper_controller',
]

POSITION_CONTROLLERS = [
    'joint_space_position_controller',
    'task_space_ik_controller',
]

VELOCITY_CONTROLLERS = [
    'joint_space_velocity_controller',
    'task_space_velocity_controller',
]

TORQUE_CONTROLLERS = [
    'gravity_compensation_controller',
    'joint_space_impedance_controller',
    'task_space_impedance_controller',
    'operational_space_controller',
    'joint_space_qp_controller',
    'task_space_qp_controller',
]

VLA_CONTROLLER = 'vla_controller'


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
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


def get_initial_active_controller(controller_name, use_vla):
    if as_bool(use_vla):
        return VLA_CONTROLLER
    return controller_name


def get_switchable_controllers(
    control_mode,
    use_vla,
    requested_controller=None,
    extra_torque_controllers=None,
):
    if control_mode == 'position':
        controllers = list(POSITION_CONTROLLERS)
    elif control_mode == 'velocity':
        # Position/torque controllers claim interfaces a velocity-mode URDF
        # doesn't export, so only the velocity-interface controllers are spawned.
        controllers = list(VELOCITY_CONTROLLERS)
    else:
        controllers = list(TORQUE_CONTROLLERS)

    if extra_torque_controllers and control_mode not in ('position', 'velocity'):
        controllers.extend(extra_torque_controllers)

    if as_bool(use_vla):
        controllers.append(VLA_CONTROLLER)
    elif requested_controller:
        controllers.append(requested_controller)

    return unique_names(controllers)


def create_runtime_param_file(
    payload_config_path,
    controller_names,
    bringup_type,
    control_mode,
    ee_name,
):
    dynamic_params = load_yaml(payload_config_path) or {}
    wildcard_params = dynamic_params.setdefault('/**', {})
    internal_control_mode = 'effort' if control_mode == 'torque' else control_mode

    for controller_name in unique_names(controller_names):
        controller_params = wildcard_params.setdefault(
            controller_name,
            {'ros__parameters': {}},
        )
        ros_params = controller_params.setdefault('ros__parameters', {})
        ros_params['bringup_type'] = bringup_type
        ros_params['control_mode'] = internal_control_mode
        ros_params['ee_name'] = ee_name

    # Write under ROS_HOME (defaults to ~/.ros) instead of the system /tmp.
    # The file is read by the local controller_manager (ros2_control_node /
    # gazebo / mujoco plugin) started by this same launch, so a per-user,
    # non-/tmp location avoids the cross-process /tmp visibility problems
    # (containers, PrivateTmp systemd units, multi-PC setups) that the old
    # tempfile approach was vulnerable to.
    runtime_dir = os.environ.get('ROS_HOME') or os.path.join(
        os.path.expanduser('~'), '.ros'
    )
    os.makedirs(runtime_dir, exist_ok=True)
    fd, runtime_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='cho_runtime_params_',
        dir=runtime_dir,
    )
    with os.fdopen(fd, 'w') as runtime_file:
        yaml.dump(dynamic_params, runtime_file)
    return runtime_path


def create_runtime_param_cleanup(runtime_param_file):
    def cleanup(context, *args, **kwargs):
        if os.path.exists(runtime_param_file):
            os.unlink(runtime_param_file)
        return []

    return OpaqueFunction(function=cleanup)


def _make_spawner_node(
    controller_names,
    runtime_param_file=None,
    active=True,
    use_sim_time=None,
    namespace=None,
    timeout=None,
    condition=None,
):
    # The ros2_control spawner accepts multiple controller names and loads /
    # configures / activates them in a single, deterministic sequence inside one
    # process. Grouping controllers this way (instead of one spawner per
    # controller running in parallel) avoids the concurrent switch_controller
    # races that intermittently leave a controller un-activated.
    #
    # runtime_param_file is only forwarded via '-p' when the controller_manager
    # cannot receive the runtime parameters directly as node parameters (e.g.
    # the Gazebo-embedded controller_manager). When None, the parameters are
    # expected to already be loaded on the controller_manager node, which is the
    # robust path that avoids the spawner -> controller_manager file handoff.
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

    node_kwargs = {
        'package': 'controller_manager',
        'executable': 'spawner',
        'arguments': spawner_args,
        'parameters': parameters,
        'output': 'screen',
    }
    if namespace is not None:
        node_kwargs['namespace'] = namespace
    if condition is not None:
        node_kwargs['condition'] = condition

    return Node(**node_kwargs)


def create_controller_spawner(
    controller_name,
    runtime_param_file=None,
    active=True,
    use_sim_time=None,
    namespace=None,
    timeout=None,
    condition=None,
):
    return _make_spawner_node(
        [controller_name],
        runtime_param_file,
        active=active,
        use_sim_time=use_sim_time,
        namespace=namespace,
        timeout=timeout,
        condition=condition,
    )


def create_controller_spawners(
    always_active_controllers,
    switchable_controllers,
    initial_active_controller,
    runtime_param_file=None,
    use_sim_time=None,
    namespace=None,
    timeout=None,
):
    switchable = unique_names(switchable_controllers)

    # Everything that must come up active: the always-active controllers plus the
    # single requested/initial switchable controller (loaded last so the
    # broadcasters are available before the main controller activates).
    active_controllers = unique_names(
        list(always_active_controllers)
        + ([initial_active_controller] if initial_active_controller in switchable else [])
    )
    inactive_controllers = [c for c in switchable if c != initial_active_controller]

    active_spawner = _make_spawner_node(
        active_controllers,
        runtime_param_file,
        active=True,
        use_sim_time=use_sim_time,
        namespace=namespace,
        timeout=timeout,
    )

    if not inactive_controllers:
        return [active_spawner]

    inactive_spawner = _make_spawner_node(
        inactive_controllers,
        runtime_param_file,
        active=False,
        use_sim_time=use_sim_time,
        namespace=namespace,
        timeout=timeout,
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=[inactive_spawner],
            )
        ),
        active_spawner,
    ]
