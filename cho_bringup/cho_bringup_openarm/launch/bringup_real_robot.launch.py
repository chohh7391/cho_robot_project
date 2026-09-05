# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Real OpenArm MIT bringup.

Invoking this launch starts the selected real ros2_control adapter.  The
adapter still validates the selected CAN interface and the explicit
commissioning safety profile before it constructs vendor transport.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


package_share = get_package_share_directory('cho_bringup_openarm')
launch_utils_path = os.path.abspath(os.path.join(
    package_share, '..', '..', 'lib', 'cho_bringup_openarm', 'utils', 'launch_utils.py'))
spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)


def generate_launch_description():
    description_path = get_package_share_directory('cho_description_openarm')
    bringup_path = get_package_share_directory('cho_bringup_openarm')
    canonical_xacro = os.path.join(
        description_path, 'robots', 'openarm_v10', 'openarm_v10.urdf.xacro')
    rviz_config = os.path.join(description_path, 'rviz', 'openarm.rviz')

    arguments = [
        DeclareLaunchArgument(
            'bimanual', default_value='false', choices=['true', 'false'],
            description='Use the two-arm torso; each arm has a separate CAN bus.'),
        DeclareLaunchArgument(
            'controller_name', default_value='joint_impedance_mit_controller',
            choices=['joint_impedance_mit_controller', 'task_space_impedance_mit_controller'],
            description='Direct seven-axis MIT action controller to activate.'),
        DeclareLaunchArgument(
            'mit_arm', default_value='both_independent',
            choices=['left', 'right', 'both_independent'],
            description='Bimanual direct controller selection; ignored for single arm.'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0', description='Single-arm SocketCAN interface.'),
        DeclareLaunchArgument(
            'left_can_interface', default_value='can1', description='Left-arm SocketCAN interface.'),
        DeclareLaunchArgument(
            'right_can_interface', default_value='can0', description='Right-arm SocketCAN interface.'),
        DeclareLaunchArgument(
            'can_fd', default_value='true', choices=['true', 'false'],
            description='Use CAN-FD framing on the selected SocketCAN interfaces.'),
        DeclareLaunchArgument(
            'mit_expected_update_rate_hz', default_value='750',
            description=(
                'Control rate declared to the hardware. Must match both the '
                'controller_manager update_rate and the safety profile '
                'update_rate_hz, or configure fails.')),
        DeclareLaunchArgument(
            'mit_state_from_command_reply', default_value='true',
            choices=['true', 'false'],
            description=(
                'Take joint state from the reply the MIT command frame already '
                'produces instead of sending a separate 0xCC query each cycle. '
                'Measured on the bus: every motor answers twice per cycle, once '
                'to the query and once to the command, so the query is pure '
                'duplication and half of all CAN traffic. Defaults ON because it '
                'is what makes the 750 Hz control rate fit: 7 tx + 7 rx per cycle '
                'is 50% of a 1 Mbps CAN FD bus, while 14 + 14 would be 101%. Set '
                'false only when also lowering the rate, and note the cost - one '
                'cycle of state age, 1.3 ms at 750 Hz.')),
        DeclareLaunchArgument(
            'hand', default_value='false', choices=['true', 'false'],
            description='Reserved for a future real gripper adapter; the current real MIT arm is seven-axis only.'),
        DeclareLaunchArgument(
            'return_to_zero', default_value='false', choices=['true', 'false'],
            description=(
                'Run the bounded nominal-zero phase before handing over to the '
                'selected controller. Defaults OFF because nominal zero is a '
                'kinematic singularity for this arm - joint 2 near zero makes the '
                'joint 1 and joint 3 axes parallel, and the Jacobian loses a '
                'Cartesian degree of freedom there (smallest singular value 0, '
                'cond 1e21). Entering Cartesian control at that posture makes the '
                'task computation fail, and the failure path stops writing MIT '
                'tuples, so the lease expires and the hardware drops to SAFE. '
                'gravity_compensation still requires it, because hand guiding '
                'needs the gravity-neutral posture and never forms a Jacobian.')),
        DeclareLaunchArgument(
            'gravity_compensation', default_value='false', choices=['true', 'false'],
            description='Hand-guiding mode for the task-space controller: zero Cartesian '
                        'stiffness and damping, no null-space posture, so the emitted MIT '
                        'tau_ff is the model term alone and the arm can be moved by hand. '
                        'The per-joint limit springs stay on as soft end stops and the MIT '
                        'kd stays on as actuator-side damping. Requires the task-space '
                        'controller and return_to_zero:=true.'),
        DeclareLaunchArgument(
            'base_rpy', default_value='0 0 0',
            description='Roll/pitch/yaw of the arm base in the world frame, forwarded to the '
                        "description's base_joint. This is what tells the model which way "
                        'gravity points, and the MIT motor has no gravity model of its own, so '
                        'a wrong value does not merely degrade compensation - it inverts it and '
                        'the model term pushes the arm down instead of holding it. The default '
                        '0 0 0 means the arm extends upward from its mount at q=0; an arm that '
                        'hangs downward from its mount needs a half turn, e.g. "3.14159 0 0".'),
        DeclareLaunchArgument(
            'gravity_scale', default_value='1.0',
            description='Multiplies the task-space controller model term before it is emitted '
                        'as MIT tau_ff. The MIT motor has no gravity model, so tau_ff is the '
                        "arm's entire gravity support and a URDF lighter than the real arm "
                        'shows up as sag no matter how correct the control law is. Sweep it '
                        'upward to find where a hand-placed posture holds; the value that '
                        'holds is the measured model deficit. Bounded to [0, 2].'),
        DeclareLaunchArgument(
            'controllers_file', default_value='',
            description='Forbidden on real MIT bringup; retained to fail closed on an override.'),
        DeclareLaunchArgument(
            'xacro_file', default_value=canonical_xacro,
            description='Canonical OpenArm description; custom hardware boundaries are forbidden.'),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
    ]

    def robot_description_for(profile):
        return {'robot_description': ParameterValue(Command([
            'xacro ', LaunchConfiguration('xacro_file'),
            ' hardware:=real',
            ' real_mit_hardware:=true',
            ' real_mit_arm:=', LaunchConfiguration('mit_arm'),
            ' real_mit_safety_profile:=', profile,
            ' mit_expected_update_rate_hz:=',
            LaunchConfiguration('mit_expected_update_rate_hz'),
            ' bimanual:=', LaunchConfiguration('bimanual'),
            ' can_interface:=', LaunchConfiguration('can_interface'),
            ' left_can_interface:=', LaunchConfiguration('left_can_interface'),
            ' right_can_interface:=', LaunchConfiguration('right_can_interface'),
            ' can_fd:=', LaunchConfiguration('can_fd'),
            ' mit_state_from_command_reply:=',
            LaunchConfiguration('mit_state_from_command_reply'),
            ' hand:=', LaunchConfiguration('hand'),
            # Quoted: the value carries spaces and Command() shlex-splits the
            # whole line, so an unquoted "0 0 0" reaches xacro as three tokens.
            ' rpy:="', LaunchConfiguration('base_rpy'), '"',
        ]), value_type=str)}
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': False}],
                condition=None)  # Added conditionally by the opaque setup.

    def setup(context, *args, **kwargs):
        del args, kwargs
        requested_xacro = LaunchConfiguration('xacro_file').perform(context)
        if os.path.realpath(requested_xacro) != os.path.realpath(canonical_xacro):
            raise RuntimeError(
                'xacro_file overrides are forbidden for real MIT bringup; the '
                'canonical description owns the plugin and safety boundary.')
        if launch_utils.as_bool(LaunchConfiguration('hand').perform(context)):
            raise RuntimeError(
                'hand:=true is unsupported by the current real MIT bringup: '
                'OpenArmMitRealSystem owns exactly the seven arm motors and '
                'does not export a gripper transport.')
        bimanual = launch_utils.as_bool(LaunchConfiguration('bimanual').perform(context))
        selection = launch_utils.resolve_real_mit_selection(
            bimanual,
            LaunchConfiguration('controller_name').perform(context),
            LaunchConfiguration('mit_arm').perform(context),
            LaunchConfiguration('controllers_file').perform(context))
        hardware_scope = launch_utils.resolve_real_mit_hardware_scope(
            bimanual, LaunchConfiguration('mit_arm').perform(context))
        return_to_zero = launch_utils.as_bool(
            LaunchConfiguration('return_to_zero').perform(context))
        gravity_compensation = launch_utils.as_bool(
            LaunchConfiguration('gravity_compensation').perform(context))
        gravity_scale = float(LaunchConfiguration('gravity_scale').perform(context))
        if not 0.0 <= gravity_scale <= 2.0:
            raise RuntimeError(
                f'gravity_scale={gravity_scale} is outside [0, 2]; above 2 the model term '
                'exceeds twice the arm\'s modelled weight and stops being compensation.')
        controller_name = LaunchConfiguration('controller_name').perform(context)
        if gravity_compensation:
            # Only the task-space controller carries the model term with zero
            # joint stiffness; the joint controller would still servo to a
            # position reference and could not be hand-guided.
            if controller_name != 'task_space_impedance_mit_controller':
                raise RuntimeError(
                    'gravity_compensation:=true requires '
                    'controller_name:=task_space_impedance_mit_controller.')
            # Nominal-zero is what makes this mode safe to enter. Activation
            # seeds a hardware SAFE hold whose tau_ff is zero until the first
            # controller tuple is accepted, so entering at an arbitrary bent
            # posture would let the arm sag before gravity compensation exists.
            # Nominal zero is gravity-neutral, and the controller only ramps
            # joint stiffness to zero once it has converged there.
            if not return_to_zero:
                raise RuntimeError(
                    'gravity_compensation:=true requires return_to_zero:=true: the '
                    'nominal-zero phase is what brings the arm to a gravity-neutral '
                    'posture before joint stiffness is ramped to zero.')
        # One profile for the whole session, whichever phases it contains. The
        # two real profiles differ in only three fields - kp_max, kd_max and
        # kp_slew_per_s - and the first two are incidental: they carry
        # upstream's canonical homing gains, which assume kp = 70. This project
        # homes at startup_kp = 3, so the conservative ceilings hold it
        # comfortably, and the phases already have separate gain parameters
        # (startup_kp/startup_kd against kp/kd).
        #
        # Selecting the return-to-zero profile whenever return_to_zero was true
        # therefore bought nothing and cost a great deal: it validated the TASK
        # gains against ceilings sized for homing, so a Cartesian kd of 5 was
        # rejected at configure with 'joint 1 gains/torque limit exceed safety
        # profile' - the same wall three separate times in commissioning. The
        # remaining difference, a 350/s rather than 10/s kp slew, only matters
        # for homing gains far larger than this project uses.
        profile = 'real_conservative_commissioning'
        robot_description = robot_description_for(profile)
        robot_state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': False}, robot_description])
        runtime_overrides = {}
        profile_path = os.path.join(
            description_path, 'config', 'mit_safety_profiles_v1.yaml')
        for name in selection['controller_names']:
            runtime_overrides[name] = {
                'safety_profile_file': profile_path,
                'safety_profile_name': profile,
                **selection['controller_overrides'].get(name, {}),
            }
            if (gravity_scale != 1.0 and
                    controller_name == 'task_space_impedance_mit_controller'):
                runtime_overrides[name]['gravity_scale'] = gravity_scale
            if gravity_compensation:
                # Zero Cartesian gains leave tau_ff = model term + limit
                # springs, so nothing pulls the arm back toward a reference and
                # the operator feels only the arm's own uncompensated residue.
                runtime_overrides[name].update({
                    # Zeroing kp_task/kd_task only disarms the LEGACY tau_ff
                    # law. Under drive-side impedance the Cartesian error still
                    # rides q_des and the drive's own kp acts on it, so the arm
                    # would push back against the operator with real stiffness.
                    # Hand guiding therefore has to select the tau_ff law and
                    # let it emit zero.
                    'drive_side_impedance': False,
                    # The joint gains sized for drive-side impedance are both
                    # unusable here (nothing should servo) and above the
                    # return-to-zero profile's ceilings, which this mode runs
                    # under because it requires return_to_zero:=true.
                    'kp': [0.0] * 7,
                    'kd': [1.0, 1.0, 0.8, 0.8, 0.3, 0.25, 0.2],
                    'kp_task': [0.0] * 6,
                    'kd_task': [0.0] * 6,
                    # Hand guiding is the one mode where dq_des is identically
                    # zero, so the friction term has to read measured velocity.
                    # That closes a feedback path: eps sits well above the
                    # one-count dither floor and scale starts at zero, to be
                    # walked up at runtime with a stationary hold checked at
                    # each step.
                    'friction_velocity_source': 'measured',
                    # The controllers YAML sets eps for the REFERENCE source,
                    # where 0.02 rad/s is right because dq_des carries no
                    # noise. Measured dq dithers by +/-0.033 rad/s, so that
                    # eps would flip the term hundreds of times a second at
                    # standstill; hand guiding keeps the dither-safe 0.1.
                    'friction_velocity_epsilon': 0.1,
                    'friction_scale': 0.0,
                    'use_nullspace_posture': False,
                    'kp_null': 0.0,
                    'kd_null': 0.0,
                    # Soft end stops matter more here than anywhere else: with
                    # zero Cartesian stiffness nothing else keeps a hand-guided
                    # joint off its mechanical limit. Joint 4 is exempt because
                    # its lower limit is the mechanical zero and the
                    # nominal-zero target sits 0.001 rad above it, so a nonzero
                    # entry would put a constant elbow lift on the very posture
                    # the operator starts from. The other six sit far outside
                    # their bands there and engage only if guided into one.
                    'joint_limit_stiffness': [5.0, 5.0, 5.0, 0.0, 2.0, 2.0, 2.0],
                })
            if return_to_zero:
                # The selected controller exclusively owns the initialization
                # phase after its MIT session handshake. Normal controller
                # gains in controllers_mit*.yaml are deliberately unchanged.
                runtime_overrides[name].update({
                    'return_to_zero': True,
                    # Match upstream openarm_ros2 return_to_zero():
                    # 200 interpolation steps at 10 ms per step.
                    'return_to_zero_duration': 2.0,
                    'return_to_zero_tolerance': 0.05,
                })
                if controller_name == 'task_space_impedance_mit_controller':
                    runtime_overrides[name].update({
                        'startup_kp': [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0],
                        'startup_kd': [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
                    })
                else:
                    runtime_overrides[name].update({
                        'return_to_zero_kp': [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0],
                        'return_to_zero_kd': [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
                    })
        runtime_file = launch_utils.create_runtime_param_file(
            controller_names=(hardware_scope['always_active_controllers'] +
                              selection['controller_names']),
            bringup_type='real', control_mode='torque', ee_name='',
            controller_overrides=runtime_overrides)
        controllers_file = os.path.join(
            bringup_path, 'config', 'real', selection['controllers_file'])
        control_node = Node(
            package='controller_manager', executable='ros2_control_node', output='screen',
            parameters=[{'use_sim_time': False}, robot_description,
                        controllers_file, runtime_file],
            remappings=[('~/robot_description', '/robot_description')], on_exit=Shutdown())
        spawners = launch_utils.create_controller_spawners(
            always_active=hardware_scope['always_active_controllers'],
            switchable_controllers=selection['controller_names'],
            initial_active_controllers=selection['controller_names'],
            use_sim_time={'use_sim_time': False}, timeout=60)
        result = [
            robot_state_publisher,
            control_node,
            RegisterEventHandler(event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[TimerAction(period=2.0, actions=spawners)])),
        ]
        if launch_utils.as_bool(LaunchConfiguration('use_rviz').perform(context)):
            result.append(rviz)
        return result

    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
