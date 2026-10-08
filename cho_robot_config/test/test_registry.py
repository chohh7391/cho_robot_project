# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from cho_robot_config import (
    CONTROL_MODES,
    available_profiles,
    available_robot_types,
    blocked_home_joint_goals,
    controller_action_name,
    declared_hold_control_modes,
    hold_controllers_for_control_mode,
    home_pose_policy,
    load_moveit_metadata,
    load_robot_config,
    moveit_bridge_node,
    static_scene_ready_service,
    task_goal_frame,
    task_home_pose,
    validate_robot_config,
)
import pytest
import yaml


EXPECTED_HOME = {
    'fr5': {
        '0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        '1': [-0.0836, -1.1209, -2.0723, -1.7125, 1.6049, 0.0798],
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

# FR5 world-frame endpoints, 10 cm from the home1 EE along one world axis each,
# all at home1's tool-down orientation.  Absolute for the same reason openarm's
# are: a relative reach is composed in the tool frame (so [0, 0, +0.1] meant
# "down" at home1) and repeated commands accumulate an offset.  -x/-y rather
# than +x/+y keeps the wrist away from the j1-axis shoulder singularity.
FR5_ABSOLUTE_REACH = {
    '0': {'relative': False, 'position': [-0.123206132, -0.102101755, 0.831834257],
          'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
    '1': {'relative': False, 'position': [-0.123206132, -0.102101755, 0.631834257],
          'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
    '2': {'relative': False, 'position': [-0.223206132, -0.102101755, 0.731834257],
          'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
    '3': {'relative': False, 'position': [-0.123206132, -0.202101755, 0.731834257],
          'orientation': [0.707106781186548, 0.707106781186548, 0.0, 0.0]},
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


# The values the task trees hard-coded before the registry held them
# (franka/{pick_place,pick_place_position,tag_reach}.py, ur/{pick_place,
# multi_move}.py, fr5/common.py's home 1, openarm/controller_check.py). They
# must not change by moving.
EXPECTED_TASK_HOME = {
    'franka': [0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785],
    'ur5e': [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    'fr5': [-0.0836, -1.1209, -2.0723, -1.7125, 1.6049, 0.0798],
    'openarm': [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
}


@pytest.mark.parametrize('robot_type,expected', sorted(EXPECTED_TASK_HOME.items()))
def test_task_home_keeps_the_values_the_trees_hard_coded(robot_type, expected):
    assert task_home_pose(load_robot_config(robot_type)) == expected


@pytest.mark.parametrize('robot_type,selector', [('fr5', '1'), ('ur5e', '1'), ('openarm', '0')])
def test_a_selector_task_home_is_the_home_pose_it_names(robot_type, selector):
    # Written once: the operator's `home N` and the task trees cannot drift.
    config = load_robot_config(robot_type)
    assert config['poses']['task_home'] == selector
    assert task_home_pose(config) == config['poses']['home'][selector]


def test_every_profile_resolves_a_task_home_of_its_own_width():
    for robot_type in available_robot_types():
        for profile in available_profiles(robot_type):
            config = load_robot_config(robot_type, profile)
            assert len(task_home_pose(config)) == len(config['model']['joints'])


def test_a_missing_task_home_raises_rather_than_guessing():
    config = deepcopy(load_robot_config('ur5e'))
    del config['poses']['task_home']
    validate_robot_config(config, 'ur5e')    # optional in the schema...
    with pytest.raises(ValueError, match='declares no poses.task_home'):
        task_home_pose(config)                # ...but required by a task that homes


def test_additional_arm_controllers_are_franka_only_today():
    franka = load_robot_config('franka')['controllers']['additional_arm']
    assert franka == [
        'task_space_impedance_controller', 'operational_space_controller',
        'task_space_ik_controller', 'task_space_velocity_controller',
        'gravity_compensation_controller']
    for robot_type in ('fr5', 'openarm', 'ur5e'):
        assert 'additional_arm' not in load_robot_config(robot_type)['controllers']


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
    ('fr5', FR5_ABSOLUTE_REACH), ('openarm', OPENARM_ABSOLUTE_REACH),
    ('franka', ABSOLUTE_REACH), ('ur5e', ABSOLUTE_REACH),
])
def test_all_reach_commands_preserve_action_client_values(robot_type, expected):
    assert load_robot_config(robot_type)['motions']['reach'] == expected


def test_fr5_reach_targets_are_fixed_absolute_world_poses():
    reach = load_robot_config('fr5')['motions']['reach']
    assert all(not target['relative'] for target in reach.values())
    # Fixed endpoints, not reusable deltas: every one names a real workspace
    # point, so issuing the same reach twice cannot walk the arm across the cell.
    assert all(any(abs(value) > 0.2 for value in target['position'])
               for target in reach.values())
    # One shared tool-down orientation (tool z along -world z), so a reach only
    # ever translates.  Normalised, or the schema would have rejected it.
    orientations = {tuple(target['orientation']) for target in reach.values()}
    assert orientations == {(0.707106781186548, 0.707106781186548, 0.0, 0.0)}
    # Each target clears the task_space_ik_controller floor guard at 0.15 m.
    assert all(target['position'][2] > 0.15 for target in reach.values())


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
    (lambda c: c['controllers'].__setitem__('hold_by_control_mode', {}),
     'must not be empty'),
    (lambda c: c['controllers'].__setitem__(
        'hold_by_control_mode', {'effort': 'a_controller'}), 'unknown control modes'),
    (lambda c: c['controllers'].__setitem__(
        'hold_by_control_mode', {'position': []}), 'at least one controller'),
    (lambda c: c['controllers'].__setitem__(
        'hold_by_control_mode', {'position': ['a', 'a']}), 'must not repeat'),
    (lambda c: c['controllers'].__setitem__(
        'hold_by_control_mode', {'position': ['']}), 'non-empty controller names'),
    (lambda c: c.__setitem__('schema_version', 2), 'schema_version'),
    (lambda c: c['model'].__setitem__('joints', ['j1', True]), 'unique list'),
    (lambda c: c['actions']['preferences'].__setitem__(
        'joint', ['/not_the_direct_controller/joint_space']),
     'first joint preference'),
    # The pre-contract scheme is no longer an action name anything serves.
    (lambda c: c['actions']['preferences'].__setitem__(
        'joint', ['/fr5/controller_action_server/moveit_joint']),
     r'/<node>/joint_space'),
    # A joint list naming a task_space server would bind the wrong action type.
    (lambda c: c['actions']['preferences']['joint'].append(
        '/task_space_ik_controller/task_space'), r'/<node>/joint_space'),
    (lambda c: c['actions']['preferences'].__setitem__('gripper', ['/gripper']),
     r'/<node>/gripper'),
    (lambda c: c['actions']['preferences'].__setitem__(
        'vla', ['/vla_controller/joint_space']), r'/<node>/vla'),
    (lambda c: c['actions']['preferences'].__setitem__('scoop', []), 'unknown spaces'),
    (lambda c: c['actions']['preferences'].__setitem__(
        'task', ['/fr5_moveit_action_bridge/task_space']),
     'task preferences must contain /task_space_ik_controller/task_space'),
    (lambda c: c['poses'].__setitem__('task_home', '7'), 'does not declare'),
    # home 0 is disabled for the FR5: a task must never be sent there.
    (lambda c: c['poses'].__setitem__('task_home', '0'), 'disables'),
    (lambda c: c['poses'].__setitem__('task_home', [0.0] * 5), 'exactly 6'),
    (lambda c: c['poses'].__setitem__('task_home', [0.0] * 5 + [float('nan')]), 'finite'),
    (lambda c: c['controllers'].__setitem__('additional_arm', 'a_controller'), 'unique list'),
    (lambda c: c['controllers'].__setitem__('additional_arm', ['a', 'a']), 'unique list'),
    (lambda c: c['controllers'].__setitem__('additional_arm', ['']), 'unique list'),
    (lambda c: c['controllers'].__setitem__(
        'additional_arm', ['gripper_controller']), 'must not name the gripper'),
    # A third frame nobody else in the stack knows about.
    (lambda c: c['model'].__setitem__('absolute_goal_frame', 'camera_link'),
     'must be model.base_frame'),
    (lambda c: c['model'].__setitem__('absolute_goal_frame', ''), 'non-empty string'),
    (lambda c: c['model'].__setitem__('relative_goal_frame', 'tool_tcp'),
     'must be model.ee_link'),
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
            f'/{robot_type}_moveit_action_bridge/joint_space')
        assert preferences['task'][0] == (
            f'/{robot_type}_moveit_action_bridge/task_space')
        assert config['controllers']['moveit_trajectory']


def test_controller_action_names_follow_the_contract():
    assert controller_action_name('joint_space_qp_controller', 'joint_space') == (
        '/joint_space_qp_controller/joint_space')
    assert controller_action_name('left_vla_mit_controller', 'vla') == (
        '/left_vla_mit_controller/vla')
    assert controller_action_name('/gripper_controller/', 'gripper') == (
        '/gripper_controller/gripper')
    assert controller_action_name('joint_trajectory_controller', 'follow_joint_trajectory') == (
        '/joint_trajectory_controller/follow_joint_trajectory')
    with pytest.raises(ValueError, match='unknown action kind'):
        controller_action_name('joint_space_qp_controller', 'moveit_joint')
    with pytest.raises(ValueError, match='non-empty'):
        controller_action_name('/', 'joint_space')


@pytest.mark.parametrize('robot_type', ['fr5', 'franka', 'openarm', 'ur5e'])
def test_every_profile_preference_is_a_node_and_kind_pair(robot_type):
    kinds = {'joint': 'joint_space', 'task': 'task_space', 'gripper': 'gripper', 'vla': 'vla'}
    for profile in available_profiles(robot_type):
        config = load_robot_config(robot_type, profile)
        bridge = moveit_bridge_node(robot_type, profile)
        for space, names in config['actions']['preferences'].items():
            for name in names:
                node = name[1:].rsplit('/', 1)[0]
                assert name == controller_action_name(node, kinds[space])
                assert 'controller_action_server' not in name
        assert config['actions']['preferences']['joint'][0] == (
            controller_action_name(bridge, 'joint_space'))


def test_bridge_node_names_are_scoped_by_robot_and_profile():
    assert moveit_bridge_node('ur5e') == 'ur5e_moveit_action_bridge'
    assert moveit_bridge_node('openarm', 'single') == 'openarm_moveit_action_bridge'
    assert moveit_bridge_node('openarm', 'left') == 'openarm_left_moveit_action_bridge'
    assert moveit_bridge_node('fr5') != moveit_bridge_node('ur5e')


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
        assert metadata['action_bridge_node'] == f'{robot_type}_moveit_action_bridge'
        assert config['actions']['preferences']['joint'][0] == (
            f"/{metadata['action_bridge_node']}/joint_space")


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


@pytest.mark.parametrize('profile', ['left', 'right'])
def test_bimanual_profile_owns_its_compatibility_controller(profile):
    """A per-arm profile must not inherit the single-arm compatibility name.

    `compatibility.task_manager.joint_space` names the legacy effort controller
    the task trees still select. Every controller on a bimanual build carries a
    per-arm prefix (`launch_utils.per_arm()`), so inheriting the top-level
    single-arm name pointed the trees at a controller that does not exist there
    and their switch could never activate anything.
    """
    single = load_robot_config('openarm', 'single')
    scoped = load_robot_config('openarm', profile)

    inherited = single['compatibility']['task_manager']['joint_space']
    resolved = scoped['compatibility']['task_manager']['joint_space']

    assert resolved != inherited
    assert resolved == f'{profile}_{inherited}'


def test_profile_compatibility_is_replaced_not_merged():
    """The overlay replaces `compatibility` outright, so a profile fully owns it.

    A shallow update would keep the top-level `task_manager` sub-mapping and
    leave the single-arm names in place under it.
    """
    scoped = load_robot_config('openarm', 'left')
    task_manager = scoped['compatibility']['task_manager']

    assert set(task_manager) == {'joint_space'}
    assert all(name.startswith('left_') for name in task_manager.values())


# ---------------------------------------------------------------------------
# controllers.hold_by_control_mode
#
# `controllers.hold` alone cannot say which controller can hold the arm: a
# bringup exports exactly one command interface per joint, so the position
# hold is not even loaded in a torque bringup. The consumer that matters is
# cho_task_manager's safe-abort branch, and its switch path is BEST_EFFORT --
# naming a controller the bringup never loaded fails quietly and leaves
# nothing holding the arm.
# ---------------------------------------------------------------------------

def _all_profiles():
    for robot_type in available_robot_types():
        for profile in available_profiles(robot_type):
            yield robot_type, profile


def test_every_registry_profile_declares_a_hold_per_control_mode():
    for robot_type, profile in _all_profiles():
        config = load_robot_config(robot_type, profile)
        modes = declared_hold_control_modes(config)
        assert modes, f'{robot_type}/{profile} declares no hold_by_control_mode'
        for mode in modes:
            holds = hold_controllers_for_control_mode(config, mode)
            assert holds, f'{robot_type}/{profile}/{mode} resolved to nothing'
            assert len(holds) == len(set(holds))
            assert all(isinstance(name, str) and name for name in holds)


def test_position_hold_matches_the_mode_independent_hold():
    """The two keys must not disagree about the position-interface controller.

    `controllers.hold` predates this mapping and still feeds the MoveIt launch
    metadata, which is position-mode only.
    """
    for robot_type, profile in _all_profiles():
        config = load_robot_config(robot_type, profile)
        if 'position' not in declared_hold_control_modes(config):
            continue
        holds = hold_controllers_for_control_mode(config, 'position')
        assert config['controllers']['hold'] in holds, (
            f"{robot_type}/{profile}: controllers.hold is not among the position holds")


def test_bimanual_profiles_never_inherit_the_single_arm_hold_names():
    """The regression this key exists to avoid.

    `controllers` is merged per key across a profile overlay, so a profile that
    did not restate the mapping would inherit the bare single-arm names --
    which launch_utils.per_arm() never spawns on a bimanual build.
    """
    for profile in ('left', 'right'):
        config = load_robot_config('openarm', profile)
        for mode in declared_hold_control_modes(config):
            for name in hold_controllers_for_control_mode(config, mode):
                assert name.startswith(f'{profile}_'), (
                    f'openarm/{profile}/{mode} holds {name}, which that build has no instance of')

    both = load_robot_config('openarm', 'both')
    for mode in declared_hold_control_modes(both):
        holds = hold_controllers_for_control_mode(both, mode)
        # Two independent seven-axis arms: holding this profile means both.
        assert any(name.startswith('left_') for name in holds)
        assert any(name.startswith('right_') for name in holds)


def test_undeclared_control_mode_raises_and_names_the_declared_ones():
    # Every UR bringup hard-codes control_mode position, so torque is genuinely
    # absent rather than an oversight -- and must not silently fall back.
    config = load_robot_config('ur5e')
    assert declared_hold_control_modes(config) == ['position']
    with pytest.raises(ValueError, match="no hold controller for control_mode 'torque'"):
        hold_controllers_for_control_mode(config, 'torque')


def test_absent_mapping_raises_rather_than_falling_back_to_hold():
    config = deepcopy(load_robot_config('fr5'))
    config['controllers'].pop('hold_by_control_mode')
    # Still a valid document: the key is optional so older entries validate.
    validate_robot_config(config, 'fr5')
    with pytest.raises(ValueError, match='declares no'):
        hold_controllers_for_control_mode(config, 'position')


def test_declared_modes_are_a_subset_of_the_known_control_modes():
    for robot_type, profile in _all_profiles():
        config = load_robot_config(robot_type, profile)
        assert set(declared_hold_control_modes(config)) <= set(CONTROL_MODES)


# ------------------------------------------------------- TaskSpace goal frames

GOAL_FRAMES = {
    # (robot, profile): (absolute, relative). Absolute is proven against every
    # bringup's description by cho_task_manager test_goal_frames.
    ('franka', 'single'): ('fr3_link0', ''),
    # FR5, UR and the single OpenArm take ee_name at launch: relative goals unstamped.
    ('fr5', 'single'): ('base_link', ''),
    ('ur5e', 'single'): ('base_link', ''),
    ('openarm', 'single'): ('world', ''),
    ('openarm', 'left'): ('world', 'openarm_left_hand_tcp'),
    ('openarm', 'right'): ('world', 'openarm_right_hand_tcp'),
    ('openarm', 'both'): ('world', ''),
}


def test_every_profile_has_its_goal_frames_pinned():
    profiles = {(robot, profile) for robot in available_robot_types()
                for profile in available_profiles(robot)}
    assert profiles == set(GOAL_FRAMES)


@pytest.mark.parametrize('robot_type,profile', sorted(GOAL_FRAMES))
def test_goal_frames_per_profile(robot_type, profile):
    config = load_robot_config(robot_type, profile)
    assert (task_goal_frame(config, relative=False),
            task_goal_frame(config, relative=True)) == GOAL_FRAMES[(robot_type, profile)]


def test_a_profile_that_inherits_another_arms_ee_frame_is_rejected(tmp_path, monkeypatch):
    # A bimanual arm that changed ee_link but not relative_goal_frame would
    # stamp the single arm's EE frame on its goals; the registry refuses it.
    source = Path(__file__).resolve().parents[1] / 'config'
    for path in source.glob('*.yaml'):
        shutil.copy(path, tmp_path / path.name)
    document = yaml.safe_load((tmp_path / 'openarm.yaml').read_text())
    # The single arm declares none (its ee_name is a launch argument); give it
    # one here so there is something for the left arm to inherit.
    document['model']['relative_goal_frame'] = 'openarm_hand_tcp'
    del document['profiles']['left']['model']['relative_goal_frame']
    (tmp_path / 'openarm.yaml').write_text(yaml.safe_dump(document))
    monkeypatch.setenv('CHO_ROBOT_CONFIG_DIR', str(tmp_path))
    with pytest.raises(ValueError, match='relative_goal_frame'):
        load_robot_config('openarm', 'left')


def test_a_registry_without_goal_frames_stamps_nothing():
    config = deepcopy(load_robot_config('fr5'))
    config['model'].pop('absolute_goal_frame')
    config['model'].pop('relative_goal_frame', None)
    validate_robot_config(config, 'fr5')
    assert task_goal_frame(config, relative=False) == ''
    assert task_goal_frame(config, relative=True) == ''


@pytest.mark.parametrize('robot_type,profile', sorted(GOAL_FRAMES))
def test_static_scene_ready_service_is_per_profile(robot_type, profile):
    assert static_scene_ready_service(robot_type, profile) == (
        load_moveit_metadata(robot_type, profile=profile)['ready_service'])
    if profile != 'single':
        assert f'/{profile}/' in static_scene_ready_service(robot_type, profile)
