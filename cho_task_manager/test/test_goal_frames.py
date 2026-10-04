"""The frames and joint names the trees stamp on their goals are ones the servers accept.

A TaskSpace server accepts an absolute goal stamped '' or one of the frames at
the root of its robot model -- the URDF root link and every link fixed to it at
the identity (``cho_controller_base::root_frames()``) -- and every controller
builds that model from the full ``robot_description`` its bringup loads. So the
registry's ``model.absolute_goal_frame`` is proven here the only way it can be:
by expanding the description of EVERY bringup of the robot, with the mappings
that bringup passes, and applying the same rule to it in Pinocchio.

The variants are the ones the bringup launch files build (cho_bringup_<robot>/
launch/*.launch.py): what changes between them is what decides the root --
Franka's description is rooted at 'base' on the real robot, MuJoCo and Isaac,
and at 'world' in Gazebo, which is why its goals say fr3_link0.
"""

import os
import re

import py_trees
import pytest

from cho_robot_config import available_profiles
from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    OcclusionSweepBehavior,
    SinglePassSweepBehavior,
    TaskSpaceActionBehavior,
)
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.franka.tag_reach import create_franka_tag_reach_tree
from cho_task_manager.utils.controller_names import (
    arm_joint_names,
    goal_frame,
    load_robot_config,
)

FRANKA_XACRO = 'robots/fr3/fr3.urdf.xacro'
FRANKA_FLAT = 'urdf/fr3_with_ft_sensor/fr3_franka_hand.urdf'
FR5_XACRO = 'urdf/fr5.urdf.xacro'
UR_XACRO = 'urdf/ur.urdf.xacro'
UR_FLAT = 'urdf/ur5e.urdf'
OPENARM_XACRO = 'robots/openarm_v10/openarm_v10.urdf.xacro'


def _ur_real(config_dir):
    return dict(
        robot_ip='192.168.1.102', name='ur5e', tf_prefix='',
        joint_limit_params=f'{config_dir}/joint_limits.yaml',
        kinematics_params=f'{config_dir}/default_kinematics.yaml',
        physical_params=f'{config_dir}/physical_parameters.yaml',
        visual_params=f'{config_dir}/visual_parameters.yaml',
        safety_limits='true', safety_pos_margin='0.15', safety_k_position='20',
        use_fake_hardware='false', fake_sensor_commands='false',
        headless_mode='false', use_tool_communication='false')


def _openarm(environment, bimanual, rpy='0 0 0'):
    if environment == 'real':
        return dict(
            hardware='real', real_mit_hardware='true', real_mit_arm='both_independent',
            real_mit_safety_profile='real_conservative_commissioning',
            mit_expected_update_rate_hz='750', bimanual=bimanual, can_interface='can0',
            left_can_interface='can1', right_can_interface='can0', can_fd='true',
            mit_state_from_command_reply='true', hand='false', rpy=rpy)
    if environment == 'mujoco':
        return dict(
            hardware='mujoco', control_mode='torque', mujoco_position_profile='standard',
            mujoco_mit_prototype='false', mujoco_mit_headless='false',
            bimanual=bimanual, hand='true')
    return dict(hardware='isaac', control_mode='torque', bimanual=bimanual)


# (robot, profiles, bringup, description package, file, mappings). The
# mappings are the launch files' (cho_bringup_<robot>/launch/bringup_*.launch.py);
# the UR real one is ur_robot_driver's ur_control.launch.py, which that bringup
# includes.
VARIANTS = [
    ('franka', ('single',), 'real', 'cho_description_franka', FRANKA_XACRO, dict(
        ros2_control='true', robot_type='fr3', arm_prefix='', robot_ip='172.16.0.2',
        hand='true', use_fake_hardware='false', fake_sensor_commands='false',
        special_connection='ft_sensor', xyz_ee='0 0 0')),
    ('franka', ('single',), 'gz', 'cho_description_franka', FRANKA_XACRO, dict(
        robot_type='fr3', hand='true', ros2_control='true', gazebo='true',
        ee_id='franka_hand', gazebo_effort='true', special_connection='ft_sensor',
        xyz_ee='0 0 0', runtime_param_file='')),
    ('franka', ('single',), 'mujoco', 'cho_description_franka', FRANKA_FLAT,
     dict(control_mode='torque')),
    ('franka', ('single',), 'isaac', 'cho_description_franka', FRANKA_FLAT,
     dict(control_mode='torque', hardware='isaac')),
    ('fr5', ('single',), 'real', 'cho_description_fr5', FR5_XACRO,
     dict(hardware='fairino', robot_ip='192.168.58.2', gripper='ag95')),
    ('fr5', ('single',), 'gz', 'cho_description_fr5', FR5_XACRO,
     dict(hardware='gazebo', simulation_controllers='/dev/null')),
    ('fr5', ('single',), 'mujoco', 'cho_description_fr5', FR5_XACRO,
     dict(hardware='mujoco', gripper='none', mujoco_initial_keyframe='home1',
          mujoco_scene='')),
    ('fr5', ('single',), 'isaac', 'cho_description_fr5', FR5_XACRO, dict(hardware='isaac')),
    ('ur5e', ('single',), 'real', 'cho_description_ur', UR_XACRO, 'ur_real'),
    ('ur5e', ('single',), 'gz', 'cho_description_ur', UR_XACRO, dict(
        safety_limits='true', safety_pos_margin='0.15', safety_k_position='20', name='ur',
        ur_type='ur5e', tf_prefix='', sim_ignition='true',
        simulation_controllers='/dev/null', load_gripper='true')),
    ('ur5e', ('single',), 'mujoco', 'cho_description_ur', UR_FLAT, dict(hardware='mujoco')),
    ('ur5e', ('single',), 'isaac', 'cho_description_ur', UR_FLAT, dict(hardware='isaac')),
] + [
    ('openarm', profiles, environment, 'cho_description_openarm', OPENARM_XACRO,
     _openarm(environment, bimanual))
    for environment in ('real', 'mujoco', 'isaac')
    for profiles, bimanual in ((('single',), 'false'), (('left', 'right', 'both'), 'true'))
] + [
    # The real single arm takes a mount rotation (base_rpy). Turned, its link0
    # is no longer at the root -- the reason OpenArm's goals say 'world'.
    ('openarm', ('single',), 'real, base_rpy turned', 'cho_description_openarm',
     OPENARM_XACRO, _openarm('real', 'false', rpy='0 0 1.5708')),
]


def _model(package, relative_path, mappings):
    xacro = pytest.importorskip('xacro')
    pin = pytest.importorskip('pinocchio')
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package)
    except (ImportError, LookupError) as error:
        pytest.skip(f'{package} is not installed: {error}')
    if mappings == 'ur_real':
        mappings = _ur_real(os.path.join(share, 'config', 'ur5e'))
    urdf = xacro.process_file(os.path.join(share, relative_path), mappings=mappings).toxml()
    return pin, pin.buildModelFromXML(urdf)


def _root_frames(pin, model):
    """cho_controller_base::root_frames(), in Python: BODY frames on the universe at identity."""
    return [frame.name for frame in model.frames
            if frame.type == pin.FrameType.BODY and frame.parentJoint == 0
            and frame.placement.isIdentity(1e-9)]


@pytest.mark.parametrize(
    'robot_type,profiles,bringup,package,relative_path,mappings', VARIANTS,
    ids=[f'{variant[0]}-{variant[2]}-{"+".join(variant[1])}' for variant in VARIANTS])
def test_the_absolute_goal_frame_is_a_root_frame_of_every_bringup(
        robot_type, profiles, bringup, package, relative_path, mappings):
    pin, model = _model(package, relative_path, mappings)
    roots = _root_frames(pin, model)
    for profile in profiles:
        registry = load_registry_config(robot_type, profile)
        frame = registry['model'].get('absolute_goal_frame')
        assert frame, f'{robot_type}/{profile} declares no absolute_goal_frame'
        assert frame in roots, (
            f'{robot_type}/{profile} on {bringup}: {frame!r} is not among the root '
            f'frames {roots}, so its controllers would reject every absolute goal')
        # The other frames the profile names are in the model: the arm's joints
        # and the EE frame a relative goal may be stamped in.
        for joint in registry['model']['joints']:
            assert model.existJointName(joint), (robot_type, profile, joint)
        relative = registry['model'].get('relative_goal_frame')
        if relative:
            assert model.existFrame(relative), (robot_type, profile, relative)


@pytest.mark.parametrize('robot_type,profile,frame', [
    ('franka', 'single', 'fr3_link0'),
    ('fr5', 'single', 'base_link'),
    ('ur5e', 'single', 'base_link'),
    ('openarm', 'single', 'world'),
    ('openarm', 'left', 'world'),
    ('openarm', 'right', 'world'),
])
def test_the_frame_proven_is_the_one_the_registry_hands_out(robot_type, profile, frame):
    assert goal_frame(load_robot_config(robot_type, profile), relative=False) == frame


def test_openarm_arm_base_links_are_not_root_frames_on_the_torso():
    # Why the bimanual profiles cannot stamp their arm_base_link: it hangs off
    # openarm_body_link0 at an offset, and the controllers reject it.
    pin, model = _model('cho_description_openarm', OPENARM_XACRO, _openarm('mujoco', 'true'))
    roots = _root_frames(pin, model)
    for profile in ('left', 'right'):
        assert load_registry_config('openarm', profile)['model']['arm_base_link'] not in roots


# ------------------------------------------------------------------- trees

_TREES = {}
_BUILD_ERRORS = {}

#: What a tree that cannot be built from the registry alone says: it needs a
#: task parameter (a recording, a sweep table). Those trees are checked in their
#: own tests, with their fixtures (test_fr5_occlusion_recovery,
#: test_fr5_trajectory_replay). ANY other ValueError is a failure, not a skip.
_NEEDS_A_PARAMETER = re.compile(r"^\w+ needs '\w+'")

#: Trees that refuse a profile on purpose, with what they say.
REFUSED = {
    ('openarm', 'both', 'controller_check_torque'): 'run it once per arm',
    ('openarm', 'both', 'mit_task_tuning'): 'run it once per arm',
}


def _registry_only_tasks():
    """(robot, profile, task) for every tree the registry alone builds, on every profile."""
    cases = []
    for robot_type in ('franka', 'fr5', 'ur5e', 'openarm'):
        for profile in available_profiles(robot_type):
            for task in available_tasks(robot_type):
                key = (robot_type, profile, task)
                try:
                    tree = build_task_tree(task, load_robot_config(robot_type, profile))
                except ValueError as error:
                    if not _NEEDS_A_PARAMETER.match(str(error)):
                        _BUILD_ERRORS[key] = str(error)
                    continue
                _TREES[key] = tree
                cases.append(key)
    return cases


REGISTRY_ONLY_TASKS = _registry_only_tasks()


def test_the_registry_only_trees_are_a_real_sample():
    tasks = {(robot, task) for robot, _profile, task in REGISTRY_ONLY_TASKS}
    assert {('franka', 'controller_check_torque'), ('franka', 'tag_reach'),
            ('ur5e', 'pick_place'), ('openarm', 'mit_task_tuning')} <= tasks
    assert ('openarm', 'left', 'controller_check_torque') in REGISTRY_ONLY_TASKS


def test_no_tree_fails_to_build_for_any_other_reason():
    # A tree that raises while naming its goals -- a target of the wrong width
    # for the profile, say -- is a bug, not a tree that needs a parameter.
    unexpected = {key: error for key, error in _BUILD_ERRORS.items() if key not in REFUSED}
    assert unexpected == {}


@pytest.mark.parametrize('key', sorted(REFUSED))
def test_a_tree_refuses_a_profile_it_cannot_drive_and_says_why(key):
    assert REFUSED[key] in _BUILD_ERRORS.get(key, '<it built>')


def _goal_leaves(root):
    return [node for node in root.iterate()
            if isinstance(node, (TaskSpaceActionBehavior, JointSpaceActionBehavior,
                                 OcclusionSweepBehavior, SinglePassSweepBehavior))]


@pytest.mark.parametrize('robot_type,profile,task', REGISTRY_ONLY_TASKS)
def test_every_goal_a_tree_sends_carries_the_registry_frame_and_names(
        robot_type, profile, task):
    config = load_robot_config(robot_type, profile)
    tree = _TREES[(robot_type, profile, task)]
    joints = arm_joint_names(config)
    for leaf in _goal_leaves(tree):
        if isinstance(leaf, TaskSpaceActionBehavior):
            if leaf.target_key is not None and task == 'tag_reach':
                continue      # the latched pose's own frame; see the next test
            assert leaf.frame_id == goal_frame(config, leaf.relative), (task, leaf.name)
        elif isinstance(leaf, JointSpaceActionBehavior):
            assert leaf.joint_names == joints, (task, leaf.name)
            if leaf.target_joints is not None:
                assert list(leaf.target_joints.name) == joints, (task, leaf.name)
        else:
            assert leaf.joint_names == joints, (task, leaf.name)


def test_tag_reach_stamps_the_frame_its_pose_was_checked_in():
    config = load_robot_config('franka')
    root = create_franka_tag_reach_tree(config)
    detect = next(node for node in root.iterate() if node.name == 'Detect_Tag')
    move = next(node for node in root.iterate() if node.name == 'Move_To_Tag_Standoff')
    # Passed through, not dropped: the frame the latch refused anything else in.
    assert move.frame_id == detect.required_frame == config['arm_base_link']
    # ...and one the Franka controllers accept on every bringup.
    assert move.frame_id == goal_frame(config, relative=False)


def test_a_named_goal_goes_out_named():
    config = load_robot_config('openarm', 'left')
    tree = build_task_tree('controller_check_torque', config)
    leaf = next(node for node in tree.iterate()
                if isinstance(node, JointSpaceActionBehavior))
    sent = []
    leaf.node = _Node()
    leaf.client = _Client(sent)
    leaf.initialise()
    assert list(sent[0].target_joints.name) == [
        f'openarm_left_joint{index}' for index in range(1, 8)]


class _Node:
    def get_clock(self):
        from rclpy.clock import Clock
        return Clock()

    def get_logger(self):
        import logging
        return logging.getLogger('test_goal_frames')


class _Client:
    def __init__(self, sent):
        self.sent = sent

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return py_trees.common.Status.RUNNING


@pytest.mark.parametrize('profile,broadcaster', [
    ('single', 'ee_state_broadcaster'),
    ('left', 'left_ee_state_broadcaster'),
    ('right', 'right_ee_state_broadcaster'),
])
def test_the_openarm_check_waits_on_the_profiles_own_broadcaster(profile, broadcaster):
    # The bimanual build has no unprefixed ee_state_broadcaster, so a check
    # that required one failed before it moved either arm.
    tree = _TREES[('openarm', profile, 'controller_check_torque')]
    check = next(node for node in tree.iterate() if node.name == 'Broadcasters_Active')
    assert broadcaster in check.require_active
