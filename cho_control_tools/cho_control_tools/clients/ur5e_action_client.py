"""UR5e-only operator action-client executable."""

from ._robot_metadata import config_loader_for, home_pose_policy
from .operator_client import run_robot_client


def main(argv=None):
    return run_robot_client(
        'ur5e', argv, robot_config_loader=config_loader_for('ur5e'),
        home_pose_policy_loader=home_pose_policy)
