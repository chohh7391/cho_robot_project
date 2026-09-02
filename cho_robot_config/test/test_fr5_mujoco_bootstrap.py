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

    model_root = ET.parse(DESCRIPTION / 'xml' / 'fr5.xml').getroot()
    joints = model_root.findall('.//worldbody//joint')
    assert [joint.attrib['name'] for joint in joints] == expected_joints

    keyframe = model_root.find("./keyframe/key[@name='home1']")
    assert keyframe is not None
    qpos = [float(value) for value in keyframe.attrib['qpos'].split()]
    ctrl = [float(value) for value in keyframe.attrib['ctrl'].split()]
    assert qpos == pytest.approx(expected_home, abs=1e-9)
    assert ctrl == pytest.approx(expected_home, abs=1e-9)

    assert len(qpos) == len(joints)
    for value, joint in zip(qpos, joints):
        lower, upper = (float(item) for item in joint.attrib['range'].split())
        assert lower <= value <= upper, (
            f"home1 {joint.attrib['name']}={value} is outside [{lower}, {upper}]"
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
    isaac = json.loads((PROJECT_ROOT / 'cho_bringup' / 'cho_bringup_isaac' /
                        'isaac' / 'robots' / 'fr5.json').read_text())
    assert isaac['arm_joints'] == joints
    assert isaac['arm_home'] == pytest.approx(expected, abs=1e-9)

    assert registry['poses']['home']['0'] == [0.0] * 6

    model_root = ET.parse(DESCRIPTION / 'xml' / 'fr5.xml').getroot()
    for key_name in ('home', 'home1'):
        key = model_root.find(f"./keyframe/key[@name='{key_name}']")
        assert key is not None
        assert [float(value) for value in key.attrib['qpos'].split()] == pytest.approx(
            expected, abs=1e-9)
    zero = model_root.find("./keyframe/key[@name='zero']")
    assert zero is not None
    assert [float(value) for value in zero.attrib['qpos'].split()] == [0.0] * 6


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
