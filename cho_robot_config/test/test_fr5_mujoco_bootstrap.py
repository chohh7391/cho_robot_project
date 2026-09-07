import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import math

import pytest
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = PROJECT_ROOT / 'cho_description' / 'cho_description_fr5'
BRINGUP = PROJECT_ROOT / 'cho_bringup' / 'cho_bringup_fr5'

# MJCF has no conditionals and the AG-95 is a body nested inside wrist3_link,
# which an including file cannot splice into, so the tool is made optional by
# having two robot files. fr5.ros2_control.xacro picks the matching scene from
# the `gripper` argument. Each entry is what the file carries past the arm.
MJCF_VARIANTS = (
    ('fr5.xml', []),
    ('fr5_ag95.xml', ['gripper_finger_joint']),
)


def _launch_argument_default(path, argument_name):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not (isinstance(function, ast.Name)
                and function.id == 'DeclareLaunchArgument'):
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        if node.args[0].value != argument_name:
            continue
        for keyword in node.keywords:
            if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
                return keyword.value.value
    raise AssertionError(f'{argument_name} is not declared in {path}')


def test_fr5_direct_mujoco_defaults_to_ready_keyframe(tmp_path):
    xacro_file = DESCRIPTION / 'urdf' / 'fr5.urdf.xacro'
    try:
        completed = subprocess.run(
            ['xacro', str(xacro_file), 'hardware:=mujoco'],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        pytest.skip('xacro executable is not installed')

    assert '<param name="initial_keyframe">' not in completed.stdout
    base_launch = BRINGUP / 'launch' / 'bringup_mujoco_robot.launch.py'
    assert _launch_argument_default(base_launch, 'mujoco_initial_keyframe') == 'home1'


def test_fr5_moveit_mujoco_defaults_to_home1_keyframe():
    wrapper = BRINGUP / 'launch' / 'bringup_mujoco_moveit.launch.py'
    assert _launch_argument_default(wrapper, 'mujoco_initial_keyframe') == 'home1'

    source = wrapper.read_text()
    assert "'mujoco_initial_keyframe': LaunchConfiguration('mujoco_initial_keyframe')" in source


def test_fr5_mujoco_jtc_continues_from_last_command_without_affecting_other_backends():
    mujoco = yaml.safe_load(
        (BRINGUP / 'config' / 'mujoco' / 'controllers.yaml').read_text())
    assert mujoco['joint_trajectory_controller']['ros__parameters'][
        'open_loop_control'] is True

    # The setting compensates MuJoCo position-actuator gravity deflection.  It
    # must not silently change feedback seeding on unvalidated backends.
    for backend in ('gz', 'isaac', 'real'):
        config = yaml.safe_load(
            (BRINGUP / 'config' / backend / 'controllers.yaml').read_text())
        jtc = config['joint_trajectory_controller']['ros__parameters']
        assert 'open_loop_control' not in jtc


def test_fr5_home1_keyframe_matches_registry_joint_order_and_limits():
    registry = yaml.safe_load(
        (PROJECT_ROOT / 'cho_robot_config' / 'config' / 'fr5.yaml').read_text()
    )
    expected_joints = registry['model']['joints']
    expected_home = registry['poses']['home']['1']

    arm = len(expected_joints)
    for filename, extra in MJCF_VARIANTS:
        model_root = ET.parse(DESCRIPTION / 'xml' / filename).getroot()
        joints = model_root.findall('.//worldbody//joint')
        names = [joint.attrib['name'] for joint in joints]
        # Arm joints first, in registry order, then whatever the variant adds.
        # Pinning the tail exactly still catches a reordering or a joint nobody
        # meant to add.
        assert names[:arm] == expected_joints, filename
        assert names[arm:] == extra, filename

        keyframe = model_root.find("./keyframe/key[@name='home1']")
        assert keyframe is not None, filename
        qpos = [float(value) for value in keyframe.attrib['qpos'].split()]
        ctrl = [float(value) for value in keyframe.attrib['ctrl'].split()]
        assert qpos[:arm] == pytest.approx(expected_home, abs=1e-9), filename
        assert ctrl[:arm] == pytest.approx(expected_home, abs=1e-9), filename
        # Keyframes are sized by nq and nu, so a tool joint rides along: closed.
        assert qpos[arm:] == [0.0] * len(extra), filename
        assert ctrl[arm:] == [0.0] * len(extra), filename

        # Inside the loop: every variant's home1 has to sit within its own
        # joint ranges, not just whichever file the loop happened to end on.
        assert len(qpos) == len(joints), filename
        for value, joint in zip(qpos, joints):
            lower, upper = (float(item) for item in joint.attrib['range'].split())
            assert lower <= value <= upper, (
                f"{filename}: home1 {joint.attrib['name']}={value} is outside "
                f'[{lower}, {upper}]'
            )


def test_fr5_ready_pose_is_consistent_across_registry_description_srdf_and_isaac():
    registry = yaml.safe_load(
        (PROJECT_ROOT / 'cho_robot_config' / 'config' / 'fr5.yaml').read_text()
    )
    expected = registry['poses']['home']['1']
    joints = registry['model']['joints']

    initial = yaml.safe_load((DESCRIPTION / 'config' / 'initial_positions.yaml').read_text())
    assert list(initial) == joints
    assert [initial[name] for name in joints] == pytest.approx(expected, abs=1e-9)

    srdf = ET.parse(PROJECT_ROOT / 'cho_moveit' / 'cho_moveit_fr5' / 'config' / 'fr5.srdf')
    home1 = srdf.getroot().find("./group_state[@name='home1']")
    assert home1 is not None
    srdf_values = {joint.attrib['name']: float(joint.attrib['value'])
                   for joint in home1.findall('joint')}
    assert list(srdf_values) == joints
    assert [srdf_values[name] for name in joints] == pytest.approx(expected, abs=1e-9)

    import json
    isaac = json.loads((PROJECT_ROOT / 'cho_bringup' / 'cho_bringup_fr5' /
                        'config' / 'isaac' / 'robot_profile.json').read_text())
    assert isaac['arm_joints'] == joints
    assert isaac['arm_home'] == pytest.approx(expected, abs=1e-9)

    assert registry['poses']['home']['0'] == [0.0] * 6

    # Every keyframe carries one value per MJCF joint, so compare the arm prefix
    # against the ready pose and pin whatever the variant adds rather than
    # loosening the comparison.
    for filename, extra in MJCF_VARIANTS:
        model_root = ET.parse(DESCRIPTION / 'xml' / filename).getroot()
        for key_name in ('home', 'home1'):
            key = model_root.find(f"./keyframe/key[@name='{key_name}']")
            assert key is not None, filename
            qpos = [float(value) for value in key.attrib['qpos'].split()]
            assert qpos[:len(joints)] == pytest.approx(expected, abs=1e-9), filename
            assert qpos[len(joints):] == [0.0] * len(extra), filename
        zero = model_root.find("./keyframe/key[@name='zero']")
        assert zero is not None, filename
        zero_qpos = [float(value) for value in zero.attrib['qpos'].split()]
        assert zero_qpos == [0.0] * (len(joints) + len(extra)), filename


def test_fr5_ready_pose_fk_floor_clearance_and_conditioning(tmp_path):
    pinocchio = pytest.importorskip('pinocchio')
    numpy = pytest.importorskip('numpy')
    urdf = tmp_path / 'fr5.urdf'
    completed = subprocess.run(
        ['xacro', str(DESCRIPTION / 'urdf' / 'fr5.urdf.xacro'), 'hardware:=mock'],
        check=True, capture_output=True, text=True)
    urdf.write_text(completed.stdout)

    registry = yaml.safe_load(
        (PROJECT_ROOT / 'cho_robot_config' / 'config' / 'fr5.yaml').read_text())
    model = pinocchio.buildModelFromUrdf(str(urdf))
    data = model.createData()
    q = numpy.zeros(model.nq)
    for name, value in zip(registry['model']['joints'], registry['poses']['home']['1']):
        joint = model.joints[model.getJointId(name)]
        assert model.lowerPositionLimit[joint.idx_q] <= value <= model.upperPositionLimit[joint.idx_q]
        q[joint.idx_q] = value

    frame_id = model.getFrameId(registry['model']['ee_link'])
    pinocchio.forwardKinematics(model, data, q)
    pinocchio.updateFramePlacements(model, data)
    assert data.oMf[frame_id].translation[2] >= 0.70

    jacobian = pinocchio.computeFrameJacobian(
        model, data, q, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    singular_values = numpy.linalg.svd(jacobian, compute_uv=False)
    assert math.isfinite(float(singular_values[0] / singular_values[-1]))
    assert singular_values[-1] >= 0.08


def test_fr5_mujoco_scene_variants_share_one_arm():
    """The two robot files must differ by the tool and nothing else."""
    mujoco = pytest.importorskip('mujoco')
    numpy = pytest.importorskip('numpy')
    registry = yaml.safe_load(
        (PROJECT_ROOT / 'cho_robot_config' / 'config' / 'fr5.yaml').read_text()
    )
    arm_joints = registry['model']['joints']

    def joints_of(model):
        return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                for i in range(model.njnt)]

    plain = mujoco.MjModel.from_xml_path(str(DESCRIPTION / 'xml' / 'scene.xml'))
    ag95 = mujoco.MjModel.from_xml_path(str(DESCRIPTION / 'xml' / 'scene_ag95.xml'))

    assert joints_of(plain) == arm_joints
    assert joints_of(ag95) == arm_joints + ['gripper_finger_joint']

    # Duplicating the arm chain across two files is the cost of MJCF having no
    # conditionals. This is what stops the copies drifting: the arm's limits and
    # its position-servo gains have to stay identical in both.
    arm = len(arm_joints)
    assert plain.jnt_range[:arm] == pytest.approx(ag95.jnt_range[:arm], abs=1e-12)
    assert plain.jnt_axis[:arm] == pytest.approx(ag95.jnt_axis[:arm], abs=1e-12)
    assert plain.actuator_gainprm[:arm] == pytest.approx(
        ag95.actuator_gainprm[:arm], abs=1e-12)
    assert plain.actuator_biasprm[:arm] == pytest.approx(
        ag95.actuator_biasprm[:arm], abs=1e-12)
    assert plain.body_pos[:plain.nbody] == pytest.approx(
        ag95.body_pos[:plain.nbody], abs=1e-12)

    # Only the gripper variant carries tool geometry, and it is drawn: an
    # invisible tool is what let the scene disagree with the robot model
    # unnoticed in the first place.
    def tool_geoms(model):
        return sum(
            1 for i in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  model.geom_bodyid[i]) or '').startswith('gripper'))

    assert tool_geoms(plain) == 0
    assert tool_geoms(ag95) > 0
    assert numpy.all(ag95.geom_rgba[:, 3] > 0.0)


def test_fr5_mujoco_scene_follows_the_gripper_in_the_description():
    """A gripper in the viewer means a gripper in the robot model, and vice versa."""
    urdf = DESCRIPTION / 'urdf' / 'fr5.urdf.xacro'
    for gripper, scene, tool_joints in (
        ('none', 'scene.xml', []),
        ('ag95', 'scene_ag95.xml', ['gripper_finger_joint']),
    ):
        completed = subprocess.run(
            ['xacro', str(urdf), 'hardware:=mujoco', f'gripper:={gripper}'],
            check=True, capture_output=True, text=True)
        control = ET.fromstring(completed.stdout).find('ros2_control')
        assert control is not None, gripper

        model = [param.text for param in control.findall('hardware/param')
                 if param.attrib['name'] == 'mujoco_model']
        assert len(model) == 1, gripper
        assert Path(model[0]).name == scene, gripper

        names = [joint.attrib['name'] for joint in control.findall('joint')]
        registry = yaml.safe_load(
            (PROJECT_ROOT / 'cho_robot_config' / 'config' / 'fr5.yaml').read_text())
        assert names == registry['model']['joints'] + tool_joints, gripper
