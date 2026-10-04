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

"""The part of an Isaac Sim bringup that is the same for every robot.

Isaac Sim runs in its own process under its own Python interpreter, so an
Isaac bringup starts three things and orders them:

  1. cho_simulation_isaac's run_isaac_sim.py under Isaac's python.sh - physics,
     the OmniGraph ROS 2 bridge, /clock and /isaac_joint_states;
  2. a controller_manager on topic_based_ros2_control (the robot's own launch);
  3. the controller spawners once Isaac is stepping, then
     cho_simulation_isaac's isaac_command_gate.py once the requested controller
     is active - see that script for why the gate exists.

isaac_controller_startup() is the ordering of step 3.
"""

import os
import shlex

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .events import start_on_output
from .utils import strict_bool

DEFAULT_ISAAC_SIM_PATH = os.path.join(os.path.expanduser('~'), 'isaacsim')

# Printed by run_isaac_sim.py once physics is stepping and the ROS 2 bridge is
# publishing. The spawners key off it.
ISAAC_READY_MARKER = '[isaac_sim] running:'


def isaac_share():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('cho_simulation_isaac')


def check_isaac_install(isaac_sim_path, robot_usd, convert_args):
    """Return Isaac's python.sh, or fail with what to do about it.

    convert_args are the convert_urdf_to_usd.py arguments that build
    `robot_usd`; they are printed, shell-quoted, as the command to run when the
    asset is missing.
    """
    isaac_python = os.path.join(isaac_sim_path, 'python.sh')
    if not os.path.exists(isaac_python):
        raise RuntimeError(
            f"Isaac Sim interpreter not found at '{isaac_python}'. "
            'Pass isaac_sim_path:=<isaac sim install dir>.'
        )
    if not os.path.exists(robot_usd):
        convert = os.path.join(isaac_share(), 'isaac', 'convert_urdf_to_usd.py')
        raise RuntimeError(
            f"Isaac robot USD not found at '{robot_usd}'.\nBuild it once with:\n"
            f'  {shlex.join([isaac_python, convert, *[str(a) for a in convert_args]])}'
        )
    return isaac_python


def isaac_sim_command(isaac_python, robot_usd, robot_profile, control_mode, physics_rate, device,
                      headless=False, extra_args=()):
    """The run_isaac_sim.py command line; `extra_args` go before --headless."""
    cmd = [
        isaac_python,
        os.path.join(isaac_share(), 'isaac', 'run_isaac_sim.py'),
        '--robot-usd', robot_usd,
        '--robot-profile', robot_profile,
        '--control-mode', control_mode,
        '--physics-rate', physics_rate,
        '--device', device,
    ]
    cmd.extend(extra_args)
    if headless:
        cmd.append('--headless')
    return cmd


def isaac_sim_process(cmd, additional_env=None):
    """Isaac itself; the whole launch shuts down with it.

    The environment is inherited, which is what makes ROS_DISTRO (so Isaac
    binds the system Humble libraries rather than its bundled ones),
    ROS_DOMAIN_ID and FASTRTPS_DEFAULT_PROFILES_FILE reach the simulator.
    """
    kwargs = {'cmd': cmd, 'output': 'screen', 'on_exit': Shutdown()}
    if additional_env:
        kwargs['additional_env'] = additional_env
    return ExecuteProcess(**kwargs)


def isaac_command_gate(use_sim_time):
    """cho_simulation_isaac's isaac_command_gate.py; use_sim_time is a parameter dict."""
    return Node(
        package='cho_simulation_isaac',
        executable='isaac_command_gate.py',
        output='screen',
        parameters=[use_sim_time],
    )


def gate_failure_argument():
    """Declare shutdown_on_gate_failure, which every Isaac bringup passes to isaac_controller_startup()."""
    return DeclareLaunchArgument(
        'shutdown_on_gate_failure',
        default_value='false',
        description='Shut the launch down when the requested controller fails to activate, instead of '
                    'leaving Isaac holding the home pose behind a closed command gate. For headless runs.',
    )


def shutdown_on_gate_failure(context):
    """Read the shutdown_on_gate_failure launch argument; a typo is refused, not read as false."""
    return strict_bool(LaunchConfiguration('shutdown_on_gate_failure').perform(context))


def _fail_launch(_context, reason):
    raise RuntimeError(reason)


def isaac_controller_startup(isaac_sim, spawners, active_spawner, command_gate,
                             marker=ISAAC_READY_MARKER, shutdown_on_failure=False):
    """Order the spawners after Isaac, and the command gate after the controller.

    The spawners must not run until Isaac is actually stepping. The
    controller_manager's realtime loop blocks in wait_until_started() until the
    first /clock arrives, and Isaac needs tens of seconds to boot. A spawner
    started before that gets the controller_manager's services (they are up
    immediately) and then asks for a controller switch, which only completes
    from inside the realtime loop - so it dies on the switch's own 5 s timeout
    long before the simulator is ready:
        [controller_manager] Switch controller timed out after 5.000000 seconds!
        [spawner] Failed to activate controller : joint_state_broadcaster
    So `spawners` start when `isaac_sim` prints `marker`.

    `command_gate` starts when `active_spawner` (the spawner whose exit means
    the requested controller is active) exits with status 0, and only then.
    OnProcessExit fires however the spawner ended, and one that failed leaves
    the requested controller inactive: opening the gate then would hand Isaac
    exactly the zero commands the gate exists to keep from it.

    A failure is logged as an ERROR and the launch keeps running, with Isaac
    holding the home pose kinematically - easy to miss in a headless run, where
    Isaac's own output buries one line. `shutdown_on_failure` (the bringups'
    shutdown_on_gate_failure:=true, for unattended runs) fails the launch
    instead: it raises, so launch shuts down AND exits with code 1. A Shutdown
    action would exit 0, and a wrapper script would take the run as a success.
    """
    def on_active_spawner_exit(event, _context):
        if event.returncode == 0:
            return [command_gate]
        reason = (f'controller spawner exited with code {event.returncode}: the requested '
                  'controller is not active, so the Isaac command gate stays CLOSED and '
                  'Isaac holds the home pose and ignores every command. See the spawner '
                  'output above.')
        if shutdown_on_failure:
            get_logger('isaac_command_gate').error(f'{reason} Shutting down (shutdown_on_gate_failure).')
            return [OpaqueFunction(function=_fail_launch, args=[f'Isaac command gate: {reason}'])]
        get_logger('isaac_command_gate').error(
            f'{reason} The launch keeps running; pass shutdown_on_gate_failure:=true to stop it instead.')
        return None

    return [
        start_on_output(isaac_sim, marker, spawners),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_spawner,
                on_exit=on_active_spawner_exit,
            )
        ),
    ]
