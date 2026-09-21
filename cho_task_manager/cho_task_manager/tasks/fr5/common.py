"""Fragments shared by the FR5 task trees.

The same split ``tasks/franka/forge/common.py`` makes: what more than one FR5
tree needs is here, so the second tree to need it does not get to pick a
different home pose than the first one did.
"""

from collections import namedtuple

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.utils.msg_utils import make_joint_state

# Every FR5 bringup hard-codes control_mode 'position' (cho_bringup_fr5), and
# that is also the only mode fr5.yaml declares a hold controller for.
CONTROL_MODE = 'position'

# The registry's canonical ready pose, keyed the same way the action client and
# the MoveIt SRDF key it. Not 'home 0': that puts the wrist at the floor and is
# recorded as diagnostic-only.
HOME_POSE_KEY = '1'


def home_joint_state(robot_config):
    """The registry's canonical ready pose as a JointState."""
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    poses = registry.get('poses', {}).get('home', {})
    if HOME_POSE_KEY not in poses:
        raise ValueError(
            f"robot_type '{robot_config['robot_type']}' has no home pose "
            f"'{HOME_POSE_KEY}' in its registry entry")
    return make_joint_state(poses[HOME_POSE_KEY])


#: One tagged vessel on the bench: what it is called, where its detected pose
#: comes out, and the blackboard entry a tree latches it into.
VesselSpec = namedtuple('VesselSpec', 'name topic key')

#: The vessels the cameras track.
#:
#: ``name`` is the joining key and has to be the same word in three places: the
#: object table (``config/perception/vessel_detect.yaml``), which decides the
#: topic; the layout a recording assumes
#: (``layout_the_trajectory_assumes`` in its meta) and the cell layout files in
#: ``config/replay/``, which is what makes a detected pose checkable against a
#: replay. Tests assert the first of those; the recordings are produced
#: elsewhere, so the third is checked at run time by the layout gate saying an
#: object is unverified rather than by anything failing here.
#:
#: Shared rather than repeated because two trees use it -- vessel_detect
#: latches them, perceived_replay checks a recording against them -- and a
#: second copy would be a topic string that can go stale in one of the two.
VESSELS = (
    VesselSpec('beaker', '/perception/object_pose/beaker', 'beaker_pose'),
    VesselSpec('flask', '/perception/object_pose/flask', 'flask_pose'),
)
