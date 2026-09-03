"""FR5-only operator action-client executable."""

from .._robot_metadata import config_loader_for, home_pose_policy
from ..operator_client import run_robot_client


def main(argv=None):
    return run_robot_client(
        'fr5', argv, robot_config_loader=config_loader_for('fr5'),
        home_pose_policy_loader=home_pose_policy)
