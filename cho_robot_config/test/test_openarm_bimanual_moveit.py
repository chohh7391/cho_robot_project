from pathlib import Path
import importlib.util
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from cho_robot_config import load_moveit_metadata, load_robot_config
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import perform_substitutions
import yaml
import pytest


ROOT = Path(__file__).resolve().parents[2]
MOVEIT = ROOT / 'cho_moveit' / 'cho_moveit_openarm' / 'config'
BRINGUP = ROOT / 'cho_bringup' / 'cho_bringup_openarm'
DESCRIPTION = ROOT / 'cho_description' / 'cho_description_openarm'


def _load_launch_module(path):
    """Load a launch file so its resolved launch contract can be exercised."""
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _moveit_wrapper_includes(**overrides):
    """Evaluate the wrapper's OpaqueFunction with its declared defaults."""
    module = _load_launch_module(
        BRINGUP / 'launch' / 'bringup_mujoco_moveit.launch.py')
    description = module.generate_launch_description()
    context = LaunchContext()
    declarations = [entity for entity in description.entities
                    if isinstance(entity, DeclareLaunchArgument)]
    for declaration in declarations:
        context.launch_configurations[declaration.name] = perform_substitutions(
            context, declaration.default_value)
    context.launch_configurations.update(overrides)
    setup = next(entity for entity in description.entities
                 if isinstance(entity, OpaqueFunction))
    includes = setup.execute(context)
    return {('robot' if 'control_mode' in dict(include.launch_arguments)
             else 'moveit'): dict(include.launch_arguments)
            for include in includes}, declarations


def test_bimanual_profiles_have_exact_ordered_arm_joints_and_joint_only_both():
    left = [f'openarm_left_joint{i}' for i in range(1, 8)]
    right = [f'openarm_right_joint{i}' for i in range(1, 8)]
    assert load_robot_config('openarm', 'left')['model']['joints'] == left
    assert load_robot_config('openarm', 'right')['model']['joints'] == right
    both = load_robot_config('openarm', 'both')
    assert both['model']['joints'] == left + right
    assert not both['supports_task']
    assert both['actions']['preferences']['task'] == []
    assert all('finger' not in name for name in both['model']['joints'])
    assert set(both['poses']['reach']) == {'0', '1', '2', '3'}
    assert all(len(target) == 14 for target in both['poses']['reach'].values())


def test_bimanual_srdf_keeps_inter_arm_collisions_enabled():
    root = ET.parse(MOVEIT / 'openarm_bimanual.srdf').getroot()
    groups = {group.attrib['name']: group for group in root.findall('group')}
    assert set(groups) == {'left_arm', 'right_arm', 'both_arms'}
    assert groups['left_arm'].find('chain').attrib == {
        'base_link': 'openarm_left_link0', 'tip_link': 'openarm_left_hand_tcp'}
    assert groups['right_arm'].find('chain').attrib == {
        'base_link': 'openarm_right_link0', 'tip_link': 'openarm_right_hand_tcp'}
    assert [item.attrib['name'] for item in groups['both_arms'].findall('group')] == [
        'left_arm', 'right_arm']
    for item in root.findall('disable_collisions'):
        first, second = item.attrib['link1'], item.attrib['link2']
        assert not (first.startswith('openarm_left_')
                    and second.startswith('openarm_right_'))
        assert not (first.startswith('openarm_right_')
                    and second.startswith('openarm_left_'))


def test_bimanual_controller_coverage_matches_registry_and_runtime_yaml():
    moveit = yaml.safe_load((MOVEIT / 'moveit_controllers_bimanual.yaml').read_text())
    manager = moveit['moveit_simple_controller_manager']
    expected = ['left_joint_trajectory_controller', 'right_joint_trajectory_controller']
    assert manager['controller_names'] == expected
    runtime = yaml.safe_load(
        (BRINGUP / 'config' / 'mujoco' / 'controllers_bimanual.yaml').read_text())['/**']
    for profile, controller in zip(('left', 'right'), expected):
        metadata = load_moveit_metadata('openarm', 'cho_moveit_openarm', profile)
        assert manager[controller]['joints'] == metadata['joint_names']
        assert runtime['controller_manager']['ros__parameters'][controller]['type'] == (
            'joint_trajectory_controller/JointTrajectoryController')
        assert runtime[controller]['ros__parameters']['command_interfaces'] == ['position']


def test_bimanual_wrapper_preserves_single_default_and_exposes_profile_switch():
    default_includes, declarations = _moveit_wrapper_includes()
    defaults = {
        declaration.name: perform_substitutions(
            LaunchContext(), declaration.default_value)
        for declaration in declarations
    }
    assert defaults['bimanual'] == 'false'
    assert defaults['arm'] == 'single'
    assert default_includes['robot'].items() >= {
        'bimanual': 'false',
        'control_mode': 'position',
        'mujoco_position_profile': 'moveit',
    }.items()
    assert default_includes['moveit'].items() >= {
        'bimanual': 'false', 'arm': 'single', 'mit_paired': 'false',
    }.items()

    bimanual_includes, _ = _moveit_wrapper_includes(
        bimanual='true', arm='both')
    assert bimanual_includes['robot'].items() >= {
        'bimanual': 'true',
        'control_mode': 'position',
        'mujoco_position_profile': 'moveit',
    }.items()
    assert bimanual_includes['moveit'].items() >= {
        'bimanual': 'true', 'arm': 'both', 'mit_paired': 'false',
    }.items()


def test_moveit_position_actuators_are_separate_and_force_limited():
    robot_launch = (BRINGUP / 'launch' / 'bringup_mujoco_robot.launch.py').read_text()
    control_xacro = (DESCRIPTION / 'robots' / 'openarm_v10' /
                     'openarm_v10.ros2_control.xacro').read_text()
    assert "'mujoco_position_profile', default_value='standard'" in robot_launch
    assert 'mujoco_position_profile:=standard' in control_xacro
    for model in ('openarm_v10', 'openarm_v10_bimanual'):
        folder = DESCRIPTION / 'xml' / model
        standard = ET.parse(folder / 'actuators_position.xml').getroot()
        moveit = ET.parse(folder / 'actuators_position_moveit.xml').getroot()
        standard_actuators = list(standard.find('actuator'))
        moveit_actuators = list(moveit.find('actuator'))
        assert [a.attrib['name'] for a in moveit_actuators] == [
            a.attrib['name'] for a in standard_actuators]
        for base, tuned in zip(standard_actuators, moveit_actuators):
            is_finger = 'finger' in base.attrib['name']
            if is_finger:
                assert tuned.tag == base.tag
                assert tuned.attrib == base.attrib
                continue
            assert tuned.tag == base.tag == 'position'
            for attribute in ('name', 'joint', 'class', 'forcerange'):
                assert tuned.attrib[attribute] == base.attrib[attribute]
            assert set(tuned.attrib) == set(base.attrib)
            assert float(tuned.attrib['kp']) == 4.0 * float(base.attrib['kp'])
            assert float(tuned.attrib['kv']) == 2.0 * float(base.attrib['kv'])


@pytest.mark.parametrize('bimanual,profile,expected_model', [
    ('false', 'standard', 'openarm_v10/scene_position.xml'),
    ('false', 'moveit', 'openarm_v10/scene_position_moveit.xml'),
    ('true', 'standard', 'openarm_v10_bimanual/scene_position.xml'),
    ('true', 'moveit', 'openarm_v10_bimanual/scene_position_moveit.xml'),
])
def test_position_profile_xacro_selects_expected_mujoco_model(
        bimanual, profile, expected_model):
    xacro = DESCRIPTION / 'robots' / 'openarm_v10' / 'openarm_v10.urdf.xacro'
    completed = subprocess.run(
        ['xacro', str(xacro), 'hardware:=mujoco', 'control_mode:=position',
         f'bimanual:={bimanual}', f'mujoco_position_profile:={profile}'],
        check=True, capture_output=True, text=True)
    root = ET.fromstring(completed.stdout)
    models = [param.text for param in root.findall('.//param')
              if param.attrib.get('name') == 'mujoco_model']
    assert len(models) == 1
    assert models[0].endswith(expected_model)


def test_moveit_position_models_are_installed_with_description_xml_tree():
    installed = Path(get_package_share_directory('cho_description_openarm'))
    for model in ('openarm_v10', 'openarm_v10_bimanual'):
        for filename in ('actuators_position_moveit.xml', 'scene_position_moveit.xml'):
            assert (installed / 'xml' / model / filename).is_file()


@pytest.mark.parametrize('mutate,match', [
    (lambda profile: profile.__setitem__('supports_task', 'no'), 'supports_task'),
    (lambda profile: profile.__setitem__('robot_type', 'other'), 'may not replace'),
    (lambda profile: profile['model'].__setitem__('joints', ['broken']), 'exactly'),
])
def test_malformed_profile_overlay_is_rejected_after_merge(
        tmp_path, monkeypatch, mutate, match):
    source_dir = ROOT / 'cho_robot_config' / 'config'
    for source in source_dir.glob('*.yaml'):
        shutil.copy(source, tmp_path / source.name)
    path = tmp_path / 'openarm.yaml'
    document = yaml.safe_load(path.read_text())
    mutate(document['profiles']['left'])
    path.write_text(yaml.safe_dump(document))
    monkeypatch.setenv('CHO_ROBOT_CONFIG_DIR', str(tmp_path))
    with pytest.raises(ValueError, match=match):
        load_robot_config('openarm', 'left')


def test_openarm_moveit_launch_forwards_rviz_and_timeout_arguments():
    source = (ROOT / 'cho_moveit' / 'cho_moveit_openarm' /
              'launch' / 'moveit.launch.py').read_text()
    assert "LaunchConfiguration('launch_rviz').perform" in source
    assert "LaunchConfiguration('scene_ready_timeout'), value_type=float" in source
    assert "LaunchConfiguration('controller_ready_timeout'), value_type=float" in source
    assert "'timeout': 210.0" not in source
    assert "'controller_ready_timeout': 90.0" not in source
    assert "'bimanual': str(bimanual).lower()" in source
