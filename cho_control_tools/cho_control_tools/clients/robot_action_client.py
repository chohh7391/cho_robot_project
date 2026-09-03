"""Compatibility facade for pre-split robot action-client imports.

New console scripts point at one module per robot. Keep these names for
downstream Python callers that used the original combined module.
"""

from .operator_client import RobotActionShell, run_robot_client


def _run(robot_type, argv=None, allow_arm=False):
    return run_robot_client(robot_type, argv, allow_arm=allow_arm)


def openarm_main():
    return _run('openarm', allow_arm=True)


def fr5_main():
    return _run('fr5')


def franka_main():
    return _run('franka')


def ur5e_main():
    return _run('ur5e')


if __name__ == '__main__':
    raise SystemExit(openarm_main())
