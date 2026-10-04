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

"""
Franka-specific launch helpers: controller names and the payload-based params.

Everything robot-independent (the runtime parameter file, spawners, the Isaac
start-up order) is cho_bringup_common's.
"""

from cho_bringup_common import (
    as_bool,
    bringup_params,
    load_yaml,
    unique_names,
    write_runtime_param_file,
)
from cho_robot_config import motion_limit_parameters


ALWAYS_ACTIVE_CONTROLLERS = [
    'joint_state_broadcaster',
    'ee_state_broadcaster',
    'simulation_gripper_controller',
    'gripper_controller',
]

GRIPPER_CONTROLLERS = (
    'simulation_gripper_controller',
    'gripper_controller',
)

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

CONTROLLERS_BY_MODE = {
    'position': POSITION_CONTROLLERS,
    'velocity': VELOCITY_CONTROLLERS,
    'torque': TORQUE_CONTROLLERS,
}

VLA_CONTROLLER = 'vla_controller'


def always_active_controllers(load_gripper):
    """Return ALWAYS_ACTIVE_CONTROLLERS, without the gripper ones unless load_gripper is 'true'."""
    load_gripper_bool = str(load_gripper).lower() == 'true'
    return [
        controller for controller in ALWAYS_ACTIVE_CONTROLLERS
        if load_gripper_bool or controller not in GRIPPER_CONTROLLERS
    ]


def get_initial_active_controller(controller_name, use_vla):
    if as_bool(use_vla):
        return VLA_CONTROLLER
    return controller_name


def check_controller_matches_mode(controller_name, control_mode, use_vla):
    """
    Refuse a controller_name that cannot run in control_mode.

    The description exports exactly one command interface per joint, chosen by
    control_mode, while the runtime parameters tell every controller that same
    mode. A controller from another mode's list would therefore be told a mode
    it does not implement - control_mode:=position alone used to inject
    position mode into the default torque controller - or claim an interface
    the description does not export. vla_controller implements all three modes,
    and vla:=true replaces controller_name altogether.
    """
    if as_bool(use_vla) or controller_name == VLA_CONTROLLER:
        return
    allowed = CONTROLLERS_BY_MODE.get(control_mode)
    if allowed is None:
        raise RuntimeError(
            f"Unknown control_mode '{control_mode}'. Valid options: {sorted(CONTROLLERS_BY_MODE)}")
    if controller_name in allowed:
        return
    owner = next(
        (mode for mode, names in CONTROLLERS_BY_MODE.items() if controller_name in names), None)
    if owner is not None:
        raise RuntimeError(
            f"controller_name:={controller_name} is a {owner} controller, but "
            f"control_mode:={control_mode}. Pass control_mode:={owner}, or a {control_mode} "
            f"controller: {allowed} (or vla:=true).")
    raise RuntimeError(
        f"Unknown controller_name '{controller_name}' for control_mode:={control_mode}. "
        f"Valid options: {allowed + [VLA_CONTROLLER]} (or vla:=true).")


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
    """
    Write the runtime parameter file, on top of payload.yaml's end-effector payload.

    FR3's MoveIt joint/Cartesian limits bound the point-to-point goals.
    """
    params = bringup_params(bringup_type, control_mode, ee_name)
    return write_runtime_param_file(
        {name: params for name in unique_names(controller_names)},
        shared_params=motion_limit_parameters('franka'),
        base=load_yaml(payload_config_path) or {},
        prefix='cho_runtime_params_',
    )
