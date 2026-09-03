# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Fail-closed real OpenArm MIT bringup.

The default invocation only publishes the real robot description.  It does
not construct a ros2_control hardware component, open a CAN socket, or enable
a motor.  Starting the adapter requires all three independent acknowledgements
below; the adapter and its safety-profile loader repeat the gates before
transport construction.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
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
            'hand', default_value='false', choices=['true', 'false'],
            description='Reserved for a future real gripper adapter; the current real MIT arm is seven-axis only.'),
        DeclareLaunchArgument(
            'open_can', default_value='false', choices=['true', 'false'],
            description='First real-hardware acknowledgement. Defaults false: no vendor transport.'),
        DeclareLaunchArgument(
            'operator_approval', default_value='false', choices=['true', 'false'],
            description='Second acknowledgement: responsible operator has completed commissioning checks.'),
        DeclareLaunchArgument(
            'enable_motors', default_value='false', choices=['true', 'false'],
            description='Final acknowledgement. Required together with open_can and operator_approval.'),
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
            ' real_mit_safety_profile:=', profile,
            ' mit_expected_update_rate_hz:=200',
            ' bimanual:=', LaunchConfiguration('bimanual'),
            ' can_interface:=', LaunchConfiguration('can_interface'),
            ' left_can_interface:=', LaunchConfiguration('left_can_interface'),
            ' right_can_interface:=', LaunchConfiguration('right_can_interface'),
            ' can_fd:=', LaunchConfiguration('can_fd'),
            ' hand:=', LaunchConfiguration('hand'),
            ' open_can:=', LaunchConfiguration('open_can'),
            ' enable_motors:=', LaunchConfiguration('enable_motors'),
            ' operator_approval:=', LaunchConfiguration('operator_approval'),
        ]), value_type=str)}
    rviz = Node(package='rviz2', executable='rviz2', output='log',
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
        opt_in = all(launch_utils.as_bool(LaunchConfiguration(name).perform(context))
                     for name in ('open_can', 'operator_approval', 'enable_motors'))
        profile = ('real_conservative_commissioning' if opt_in else
                   'real_conservative_unapproved')
        robot_description = robot_description_for(profile)
        robot_state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': False}, robot_description])
        if not opt_in:
            message = (
                'OpenArm real MIT is description-only: ros2_control is not started and '
                'no CAN socket/motor can be opened. Set all of open_can:=true, '
                'operator_approval:=true, and enable_motors:=true only after the '
                'documented commissioning checks.')
            result = [robot_state_publisher, LogInfo(msg=message)]
            if launch_utils.as_bool(LaunchConfiguration('use_rviz').perform(context)):
                result.append(rviz)
            return result

        # The selected operational profile can only be reached through the
        # triple opt-in above. The adapter independently validates this exact
        # profile and the three flags before it creates vendor transport.
        runtime_overrides = {}
        profile_path = os.path.join(
            description_path, 'config', 'mit_safety_profiles_v1.yaml')
        for name in selection['controller_names']:
            runtime_overrides[name] = {
                'safety_profile_file': profile_path,
                'safety_profile_name': 'real_conservative_commissioning',
                **selection['controller_overrides'].get(name, {}),
            }
        runtime_file = launch_utils.create_runtime_param_file(
            controller_names=(launch_utils.always_active_controllers(bimanual) +
                              selection['controller_names']),
            bringup_type='real', control_mode='torque', ee_name='',
            controller_overrides=runtime_overrides)
        controllers_file = os.path.join(
            bringup_path, 'config', 'real', selection['controllers_file'])
        control_node = Node(
            package='controller_manager', executable='ros2_control_node', output='screen',
            parameters=[{'use_sim_time': False}, robot_description, controllers_file, runtime_file],
            remappings=[('~/robot_description', '/robot_description')], on_exit=Shutdown())
        spawners = launch_utils.create_controller_spawners(
            always_active=launch_utils.always_active_controllers(bimanual),
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
