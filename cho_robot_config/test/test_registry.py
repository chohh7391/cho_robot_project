from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from cho_robot_config import (
    available_robot_types,
    blocked_home_joint_goals,
    home_pose_policy,
    load_moveit_metadata,
    load_robot_config,
    validate_robot_config,
)
import pytest
import yaml


EXPECTED_HOME = {
    'fr5': {
        '0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        '1': [0.0, -0.7853981634, -1.5707963268, 0.7853981634, -1.5707963268, 0.0],
        '2': [0.4, -0.7, -1.8, 1.2, 0.4, 1.0],
        '3': [-0.4, -0.7, -1.8, 1.2, -0.4, 1.0],
    },
    'ur5e': {
        '0': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        '1': [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
        '2': [0.2, -1.4, 1.4, -1.6, -1.5, 0.2],
        '3': [-0.2, -1.4, 1.4, -1.6, -1.5, -0.2],
    },
    'openarm': {
        '0': [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
        '1': [0.0, -0.5, 0.0, 1.2, 0.0, 0.4, 0.0],
        '2': [0.3, -0.4, 0.2, 1.0, 0.2, 0.2, 0.0],
        '3': [-0.3, -0.4, -0.2, 1.0, -0.2, 0.2, 0.0],
    },
    'franka': {
        '0': [0.0, -0.7853981633974483, 0.0, -2.356194490192345,
              0.0, 1.5707963267948966, 0.7853981633974483],
        '1': [0.0, 0.0, 0.0, -1.57, 0.0, 2.355, 0.0],
        '2': [-0.3202889859676361, 0.5399062633514404, 0.3390618860721588,
              -1.862808346748352, -0.24342849850654602,
              2.361226797103882, 0.30928418040275574],
        '3': [-0.46396875381469727, 0.6291446089744568, 0.4975337088108063,
              -1.9110225439071655, -0.4653533399105072,
              2.424884796142578, 0.85429847240448],
    },
}

ABSOLUTE_REACH = {
    '0': {'relative': False, 'position': [0.2, -0.2, 0.5],
          'orientation': [1.0, 0.0, 0.0, 0.0]},
    '1': {'relative': False, 'position': [0.2, 0.2, 0.6],
          'orientation': [1.0, 0.0, 0.0, 0.0]},
    '2': {'relative': True, 'position': [0.0, 0.0, -0.2],
          'orientation': [0.0, 0.0, 0.0, 1.0]},
    '3': {'relative': False, 'position': [0.6, 0.0, 0.1],
          'orientation': [1.0, 0.0, 0.0, 0.0]},
}

RELATIVE_REACH = {
    '0': {'relative': True, 'position': [0.0, 0.0, 0.1],
          'orientation': [0.0, 0.0, 0.0, 1.0]},
    '1': {'relative': True, 'position': [0.0, 0.0, -0.1],
          'orientation': [0.0, 0.0, 0.0, 1.0]},
    '2': {'relative': True, 'position': [0.1, 0.0, 0.0],
          'orientation': [0.0, 0.0, 0.0, 1.0]},
    '3': {'relative': True, 'position': [0.0, 0.1, 0.0],
          'orientation': [0.0, 0.0, 0.0, 1.0]},
}

# OpenArm direct task-space impedance starts at home 1 before accepting a
# goal.  These world-frame endpoints preserve the historical EE-frame +/-Z,
# +X and +Y 10 cm probes, but are deliberately absolute so repeated `reach N`
# commands cannot accumulate an offset.
OPENARM_ABSOLUTE_REACH = {
    '0': {'relative': False, 'position': [0.446841389, -0.286500255, 0.414628896],
          'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
    '1': {'relative': False, 'position': [0.275148419, -0.186149069, 0.393389049],
          'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
    '2': {'relative': False, 'position': [0.397230680, -0.191640328, 0.322214847],
          'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
    '3': {'relative': False, 'position': [0.397290216, -0.162259069, 0.460550447],
          'orientation': [0.358992547, 0.563936817, 0.028220362, 0.743171063]},
}


def test_registry_contains_each_supported_robot_once():
    assert available_robot_types() == ['fr5', 'franka', 'openarm', 'ur5e']
    declared = [load_robot_config(name)['robot_type'] for name in available_robot_types()]
    assert len(declared) == len(set(declared)) == 4


@pytest.mark.parametrize('robot_type', ['fr5', 'franka', 'openarm', 'ur5e'])
def test_all_documents_pass_schema(robot_type):
    config = load_robot_config(robot_type)
    assert validate_robot_config(config, robot_type) is config
    joint_count = len(config['model']['joints'])
    assert all(len(value) == joint_count for value in config['poses']['home'].values())


def test_home_commands_preserve_action_client_values():
    for robot_type, expected in EXPECTED_HOME.items():
        assert load_robot_config(robot_type)['poses']['home'] == expected


def test_fr5_zero_home_is_retained_but_disabled_for_normal_execution():
    config = load_robot_config('fr5')
    assert config['poses']['home']['0'] == [0.0] * 6
    policy = home_pose_policy(config, '0')
    assert policy['enabled'] is False
    assert 'floor' in policy['reason']
    assert blocked_home_joint_goals(config) == [{
        'selector': '0',
        'positions': [0.0] * 6,
        'reason': policy['reason'],
        'max_joint_distance': 0.01,
    }]


@pytest.mark.parametrize('robot_type', ['fr5', 'franka', 'openarm', 'ur5e'])
def test_unannotated_home_poses_default_to_enabled(robot_type):
    config = load_robot_config(robot_type)
    assert home_pose_policy(config, '1') == {'enabled': True, 'reason': ''}


@pytest.mark.parametrize('robot_type,expected', [
    ('fr5', RELATIVE_REACH), ('openarm', OPENARM_ABSOLUTE_REACH),
    ('franka', ABSOLUTE_REACH), ('ur5e', ABSOLUTE_REACH),
])
def test_all_reach_commands_preserve_action_client_values(robot_type, expected):
    assert load_robot_config(robot_type)['motions']['reach'] == expected


def test_openarm_reach_targets_are_fixed_absolute_world_poses():
    reach = load_robot_config('openarm')['motions']['reach']
    assert all(not target['relative'] for target in reach.values())
    # Targets have distinct fixed endpoints; none is a reusable +/-10 cm
    # delta vector that could be applied again from the last measured pose.
    assert all(any(abs(value) > 0.2 for value in target['position'])
               for target in reach.values())


@pytest.mark.parametrize('mutation, message', [
    (lambda c: c['poses']['home'].__setitem__('0', [0.0]), 'exactly'),
    (lambda c: c['poses']['home_safety']['0'].__setitem__('enabled', 'no'),
     'enabled must be boolean'),
    (lambda c: c['poses']['home_safety']['0'].__setitem__(
        'max_joint_distance', -0.1), 'finite non-negative'),
    (lambda c: c['motions']['reach']['0'].__setitem__(
        'orientation', [0.0, 0.0, 0.0, 0.5]), 'normalized'),
    (lambda c: c['controllers'].pop('hold'), 'controller roles'),
    (lambda c: c.__setitem__('schema_version', 2), 'schema_version'),
    (lambda c: c['model'].__setitem__('joints', ['j1', True]), 'unique list'),
    (lambda c: c['actions']['preferences'].__setitem__(
        'joint', ['/controller_action_server/not_the_direct_controller']),
     'first joint preference'),
])
def test_invalid_documents_are_rejected(mutation, message):
    config = deepcopy(load_robot_config('fr5'))
    mutation(config)
    with pytest.raises(ValueError, match=message):
        validate_robot_config(config, 'fr5')


def test_moveit_action_namespace_and_backend_are_consistent():
    for robot_type in available_robot_types():
        config = load_robot_config(robot_type)
        preferences = config['actions']['preferences']
        assert preferences['joint'][0] == (
            f'/{robot_type}/controller_action_server/moveit_joint')
        assert preferences['task'][0] == (
            f'/{robot_type}/controller_action_server/moveit_task')
        assert config['controllers']['moveit_trajectory']


def test_launch_metadata_is_derived_from_every_registry_document():
    for robot_type in available_robot_types():
        config = load_robot_config(robot_type)
        metadata = load_moveit_metadata(
            robot_type, config['moveit']['config_package'])
        assert metadata['robot_type'] == robot_type
        assert metadata['planning_group'] == config['moveit']['planning_group']
        assert metadata['ee_link'] == config['model']['ee_link']
        assert metadata['joint_names'] == config['model']['joints']
        assert metadata['hold_controller'] == config['controllers']['hold']
        assert metadata['trajectory_controller'] == config['controllers']['moveit_trajectory']
        assert metadata['max_velocity_scaling_factor'] == (
            config['moveit']['execution']['max_velocity_scaling_factor'])
        assert metadata['max_acceleration_scaling_factor'] == (
            config['moveit']['execution']['max_acceleration_scaling_factor'])
        assert metadata['ready_service'] == f'/cho_moveit/{robot_type}/static_scene_ready'


def test_openarm_moveit_execution_scaling_does_not_drift():
    metadata = load_moveit_metadata('openarm', 'cho_moveit_openarm')
    assert metadata['max_velocity_scaling_factor'] == 0.15
    assert metadata['max_acceleration_scaling_factor'] == 0.05


@pytest.mark.parametrize('value', [0.0, -0.1, 1.01, float('nan')])
@pytest.mark.parametrize('field', [
    'max_velocity_scaling_factor', 'max_acceleration_scaling_factor'])
def test_invalid_moveit_execution_scaling_is_rejected(field, value):
    config = deepcopy(load_robot_config('openarm'))
    config['moveit']['execution'][field] = value
    with pytest.raises(ValueError, match=r'finite and in \(0, 1\]'):
        validate_robot_config(config, 'openarm')


def test_launch_metadata_reports_package_drift():
    with pytest.raises(ValueError, match='Fix cho_robot_config'):
        load_moveit_metadata('fr5', 'cho_moveit_wrong')


@pytest.mark.parametrize('robot_type,package_name,srdf_name', [
    ('fr5', 'cho_moveit_fr5', 'fr5.srdf'),
    ('franka', 'cho_moveit_franka', 'fr3.srdf'),
    ('openarm', 'cho_moveit_openarm', 'openarm.srdf'),
    ('ur5e', 'cho_moveit_ur', 'ur5e.srdf'),
])
def test_moveit_controller_yaml_does_not_drift_from_registry(
        robot_type, package_name, srdf_name):
    """Catch silent divergence between the canonical registry and MoveIt."""
    metadata = load_moveit_metadata(robot_type, package_name)
    project_root = Path(__file__).parents[2]
    controller_path = (
        project_root / 'cho_moveit' / package_name / 'config' /
        'moveit_controllers.yaml')
    document = yaml.safe_load(controller_path.read_text())
    manager = document['moveit_simple_controller_manager']
    assert manager['controller_names'] == [metadata['trajectory_controller']]
    controller = manager[metadata['trajectory_controller']]
    assert controller['joints'] == metadata['joint_names']

    srdf_root = ET.parse(controller_path.parent / srdf_name).getroot()
    group = srdf_root.find(f"./group[@name='{metadata['planning_group']}']")
    assert group is not None
    chain = group.find('chain')
    assert chain is not None
    assert chain.attrib['base_link'] == metadata['arm_base_link']
    assert chain.attrib['tip_link'] == metadata['ee_link']


def test_environment_override_has_an_independent_cache_key(tmp_path, monkeypatch):
    source = Path(__file__).parents[1] / 'config' / 'fr5.yaml'
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    first.mkdir()
    second.mkdir()
    shutil.copy(source, first / 'fr5.yaml')
    shutil.copy(source, second / 'fr5.yaml')

    monkeypatch.setenv('CHO_ROBOT_CONFIG_DIR', str(first))
    assert load_robot_config('fr5')['poses']['home']['0'][0] == 0.0
    document = yaml.safe_load((second / 'fr5.yaml').read_text())
    document['poses']['home']['0'][0] = 0.25
    (second / 'fr5.yaml').write_text(yaml.safe_dump(document))
    monkeypatch.setenv('CHO_ROBOT_CONFIG_DIR', str(second))
    assert load_robot_config('fr5')['poses']['home']['0'][0] == 0.25
