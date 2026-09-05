"""Robot-scoped operator shell shared by the small executable modules.

This module deliberately has no import of ``action_client`` or
``cho_robot_config`` at module load time.  The individual robot entry points
select their metadata loader first, which keeps a client usable in a workspace
that contains only one robot vertical.
"""

from __future__ import annotations

import argparse
import sys
import time


# Direct MIT task control begins from nominal zero by default.  Selectors 0--2
# are bounded TCP-frame probes. Quaternion order is x, y, z, w.
_OPENARM_MIT_NOMINAL_ZERO_RELATIVE_REACH = {
    # +X translation, preserving tool orientation as the baseline probe.
    '0': ((0.045, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    # -X/+Y translation with a +0.20 rad roll.
    '1': ((-0.040, 0.015, 0.0), (0.0998334166, 0.0, 0.0, 0.9950041653)),
    # +X/+Y/-Z translation with a -0.25 rad pitch.
    '2': ((0.010, 0.040, -0.015), (0.0, -0.1246747334, 0.0, 0.9921976672)),
}

# Absolute world-frame TCP poses computed from the canonical OpenArm URDF at
# q=[0, 0, 0, pi/2, 0, 0, 0].  Unlike a large relative Cartesian displacement,
# this asks the controller for the actual forward-bent J4 posture.  The
# bimanual transforms are intentionally profile-specific.
_OPENARM_MIT_FORWARD_BEND_REACH = {
    'single': ((0.402000000000, 0.000000000000, 0.342500000000),
               (0.0, 0.707106781187, 0.0, 0.707106781187)),
    'left': ((0.402000000000, 0.153499191895, 0.477999550034),
             (0.707106781185, 0.000001298672,
              0.707106781185, 0.000001298672)),
    'right': ((0.402000000000, -0.153499191895, 0.477999550034),
              (0.707106781185, -0.000001298672,
               0.707106781185, -0.000001298672)),
}


def _control_suite_shell():
    """Load the ROS-dependent common shell only when an operator runs it."""
    from .action_client import ControlSuiteShell
    return ControlSuiteShell


class RobotActionShell:
    """Operator shell with a fixed robot identity and automatic endpoints."""

    def __init__(self, robot_type: str, arm: str = 'single', *,
                 robot_config_loader=None, home_pose_policy_loader=None,
                 shell_factory=None):
        # The MIT task controller's home-1 ramp is expected to finish within
        # this window. It bounds automatic handling to launch-time only.
        self._openarm_task_startup_deadline = time.monotonic() + 15.0
        control_space = 'joint' if arm == 'both' else 'task'
        base = shell_factory or _control_suite_shell()
        self._shell = base(
            robot_type=robot_type,
            arm=arm,
            control_space=control_space,
            operator_facing=True,
            robot_config_loader=robot_config_loader,
            home_pose_policy_loader=home_pose_policy_loader,
        )
        self._configure_user_facing_shell()

    def _configure_user_facing_shell(self):
        shell = self._shell
        shell.intro = (
            'Cho robot action client. Commands: home <0-3>, reach <0-3>, '
            'grasp <0|1>, status, quit.\n'
            'Action servers are selected automatically from the active robot controller.\n'
        )
        shell._print_selected_clients = self._print_availability
        shell.do_status = self._status
        shell.do_servers = self._servers
        shell.do_use_joint = self._automatic_selection_only
        shell.do_use_task = self._automatic_selection_only
        shell.do_use_gripper = self._automatic_selection_only

        def operator_command_names():
            hidden = {'do_use_joint', 'do_use_task', 'do_use_gripper'}
            return [name for name in dir(shell) if name not in hidden]

        shell.get_names = operator_command_names
        self._install_openarm_task_startup_retry()

    def _install_openarm_task_startup_retry(self):
        """Apply the direct-MIT zero-start contract and retry launch-time rejection."""
        shell = self._shell
        mit_task_endpoints = {
            '/controller_action_server/task_space_impedance_mit_controller',
            '/controller_action_server/left_task_space_impedance_mit_controller',
            '/controller_action_server/right_task_space_impedance_mit_controller',
        }
        if (getattr(shell, 'robot_type', None) != 'openarm' or
                getattr(shell, 'task_action_name', None) not in mit_task_endpoints):
            return

        # Keep the ordinary metadata untouched for MoveIt/non-MIT task
        # endpoints. Only the direct MIT endpoint starts from nominal zero, so
        # only it replaces its presets with these operator goals.
        motions = getattr(shell, 'robot_config', {}).get('motions')
        if isinstance(motions, dict):
            motions['reach'] = {
                selector: {
                    'relative': True,
                    'position': list(translation),
                    # geometry_msgs quaternion order is x, y, z, w.
                    'orientation': list(orientation),
                }
                for selector, (translation, orientation) in
                _OPENARM_MIT_NOMINAL_ZERO_RELATIVE_REACH.items()
            }
            forward_bend = _OPENARM_MIT_FORWARD_BEND_REACH.get(
                getattr(shell, 'arm', 'single'))
            if forward_bend is not None:
                position, orientation = forward_bend
                motions['reach']['3'] = {
                    'relative': False,
                    'position': list(position),
                    'orientation': list(orientation),
                }

        original_send = shell._send_goal_and_wait

        def send_with_startup_retry(client, goal):
            success = original_send(client, goal)
            if (success or client is not shell.task_space_action_client or
                    not getattr(shell, '_last_goal_rejected', False)):
                return success

            deadline = self._openarm_task_startup_deadline
            if time.monotonic() >= deadline:
                return False
            print('Initial posture preparing; waiting for task-space control …')
            while time.monotonic() < deadline:
                time.sleep(0.5)
                success = original_send(client, goal)
                if success or not getattr(shell, '_last_goal_rejected', False):
                    return success
            print('Task-space controller did not become ready within 15 seconds.')
            return False

        shell._send_goal_and_wait = send_with_startup_retry

    def _availability_lines(self):
        shell = self._shell
        return (
            ('joint-space', shell.joint_space_action_client is not None),
            ('task-space', shell.task_space_action_client is not None),
            ('gripper', shell.gripper_action_client is not None or
             shell.robotiq_command_publisher is not None),
        )

    def _print_availability(self):
        print('Action availability:')
        for name, available in self._availability_lines():
            print(f"  {name}: {'ready' if available else 'unavailable'}")

    def _status(self, arg):
        del arg
        self._print_availability()

    def _servers(self, arg):
        del arg
        self._print_availability()

    @staticmethod
    def _automatic_selection_only(arg):
        del arg
        print('Action servers are selected automatically. Use `status` to check availability.')

    def cmdloop(self):
        return self._shell.cmdloop()


def run_robot_client(robot_type: str, argv: list[str] | None = None, *,
                     allow_arm: bool = False, robot_config_loader=None,
                     home_pose_policy_loader=None, shell_factory=None):
    """Run a fixed-robot client without parsing generic robot selectors."""
    parser = argparse.ArgumentParser(
        description=f'Interactive action client for the {robot_type} robot.')
    if allow_arm:
        parser.add_argument(
            '--arm', choices=('single', 'left', 'right', 'both'), default='single',
            help='OpenArm profile (default: single).',
        )
    args = parser.parse_args(argv)
    arm = args.arm if allow_arm else 'single'
    try:
        RobotActionShell(
            robot_type,
            arm,
            robot_config_loader=robot_config_loader,
            home_pose_policy_loader=home_pose_policy_loader,
            shell_factory=shell_factory,
        ).cmdloop()
    except KeyboardInterrupt:
        print('\nInterrupt - shutting down …')
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(run_robot_client('franka'))
