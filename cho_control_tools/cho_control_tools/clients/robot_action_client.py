"""Robot-specific operator front ends for the packaged Cho action client.

The generic debug client remains fully configurable.  These operator entry
points fix robot identity in the executable name and discover active action
servers from ``cho_robot_config``.
"""

from __future__ import annotations

import argparse
import sys
import time

from .action_client import ControlSuiteShell


def _control_suite_shell():
    """Return the packaged generic action shell.

    Operator-facing clients use the same in-package implementation as the
    generic debug client.  Keeping this as a small factory preserves the
    lightweight test seam without dynamic source-file imports.
    """
    return ControlSuiteShell


class RobotActionShell:
    """A user-facing shell that fixes robot identity and hides endpoint names."""

    def __init__(self, robot_type: str, arm: str = 'single'):
        # The MIT task controller's home-1 ramp is expected to finish within
        # this window.  It bounds automatic handling to launch-time only,
        # rather than treating any future controller rejection as transient.
        self._openarm_task_startup_deadline = time.monotonic() + 15.0
        # ``control_space`` is only a legacy validation input.  Runtime command
        # routing remains automatic: reach uses task space when available and
        # the registered joint preset otherwise.  The paired OpenArm profile
        # has no single-pose TaskSpace representation, so it gets joint here.
        control_space = 'joint' if arm == 'both' else 'task'
        base = _control_suite_shell()
        self._shell = base(
            robot_type=robot_type,
            arm=arm,
            control_space=control_space,
            operator_facing=True,
        )
        self._configure_user_facing_shell()

    def _configure_user_facing_shell(self):
        shell = self._shell
        shell.intro = (
            'Cho robot action client. Commands: home <0-3>, reach <0-3>, '
            'grasp <0|1>, status, quit.\n'
            'Action servers are selected automatically from the active robot controller.\n'
        )

        # The generic shell retains manual endpoint selection for debugging.
        # Operator clients must not turn controller names into part of normal
        # operation, so show availability rather than endpoint identifiers.
        shell._print_selected_clients = self._print_availability
        shell.do_status = self._status
        shell.do_servers = self._servers
        shell.do_use_joint = self._automatic_selection_only
        shell.do_use_task = self._automatic_selection_only
        shell.do_use_gripper = self._automatic_selection_only

        def operator_command_names():
            hidden = {'do_use_joint', 'do_use_task', 'do_use_gripper'}
            return [name for name in dir(shell) if name not in hidden]

        # cmd.Cmd builds its help listing from get_names().  Keep the manual
        # commands callable only as a benign explanatory fallback, but do not
        # advertise controller selection in the normal operator UI.
        shell.get_names = operator_command_names
        self._install_openarm_task_startup_retry()

    def _install_openarm_task_startup_retry(self):
        """Handle only OpenArm MIT's brief initial home-pose rejection.

        The controller advertises its action before completing a safe home-1
        ramp.  It rejects, rather than aborts, goals during that window.  A
        goal accepted by the controller is never retried here, so ordinary
        workspace/controller failures retain their original meaning.
        """
        shell = self._shell
        # Single-arm and bimanual-direct MIT task controllers use the same
        # guarded startup ramp.  The arm selection remains in the executable
        # (`openarm_action_client --arm left|right`), never in a controller
        # name supplied by the operator.
        mit_task_endpoints = {
            '/controller_action_server/task_space_impedance_mit_controller',
            '/controller_action_server/left_task_space_impedance_mit_controller',
            '/controller_action_server/right_task_space_impedance_mit_controller',
        }
        if (getattr(shell, 'robot_type', None) != 'openarm' or
                getattr(shell, 'task_action_name', None) not in mit_task_endpoints):
            return

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


def _run(robot_type: str, argv: list[str] | None = None, allow_arm: bool = False):
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
        RobotActionShell(robot_type, arm).cmdloop()
    except KeyboardInterrupt:
        print('\nInterrupt - shutting down …')
        # The wrapped shell owns rclpy, but import it only on this path so the
        # argument/UX helpers remain lightweight and easy to test.
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
        return 130
    return 0


def openarm_main():
    return _run('openarm', allow_arm=True)


def fr5_main():
    return _run('fr5')


def franka_main():
    return _run('franka')


def ur5e_main():
    return _run('ur5e')


if __name__ == '__main__':
    sys.exit(openarm_main())
