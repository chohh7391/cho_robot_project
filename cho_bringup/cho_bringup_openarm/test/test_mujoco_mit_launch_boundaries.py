import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
UTILS = PACKAGE / 'utils' / 'launch_utils.py'
CONFIG = PACKAGE / 'config' / 'mujoco' / 'controllers_mit_bimanual.yaml'
DIRECT_CONFIG = PACKAGE / 'config' / 'mujoco' / 'controllers_mit.yaml'
DIRECT_BIMANUAL_CONFIG = PACKAGE / 'config' / 'mujoco' / 'controllers_mit_direct_bimanual.yaml'
SAFETY_PROFILE = (PACKAGE.parents[1] / 'cho_description' / 'cho_description_openarm' /
                  'config' / 'mit_safety_profiles_v1.yaml')
PAIRED_CONFIG = PACKAGE / 'config' / 'mujoco' / 'controllers_mit_moveit_bimanual.yaml'
MOVEIT_PAIRED_CONFIG = (PACKAGE.parents[1] / 'cho_moveit' / 'cho_moveit_openarm' /
                        'config' / 'moveit_controllers_bimanual_mit.yaml')
MOVEIT_WRAPPER = PACKAGE / 'launch' / 'bringup_mujoco_moveit.launch.py'
MOVEIT_LAUNCH = (PACKAGE.parents[1] / 'cho_moveit' / 'cho_moveit_openarm' /
                 'launch' / 'moveit.launch.py')
spec = importlib.util.spec_from_file_location('openarm_launch_utils_under_test', UTILS)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)
moveit_spec = importlib.util.spec_from_file_location(
    'openarm_moveit_launch_under_test', MOVEIT_LAUNCH)
moveit_launch = importlib.util.module_from_spec(moveit_spec)
moveit_spec.loader.exec_module(moveit_launch)


def test_disabled_mit_leaves_legacy_controller_selection_authoritative():
    assert launch_utils.resolve_mujoco_mit_selection(
        False, 'position', False, 'anything', 'left', '/tmp/custom.yaml') is None


def test_single_arm_impedance_maps_to_the_canonical_direct_mit_config():
    assert launch_utils.resolve_mujoco_mit_selection(
        True, 'torque', False, 'joint_impedance_mit_controller', 'left') == {
            'controller_name': 'joint_impedance_mit_controller',
            'controller_names': ['joint_impedance_mit_controller'],
            'controllers_file': 'controllers_mit.yaml',
            'controller_overrides': {},
    }


def test_single_arm_task_impedance_maps_to_the_canonical_direct_mit_config():
    assert launch_utils.resolve_mujoco_mit_selection(
        True, 'torque', False, 'task_space_impedance_mit_controller', 'left') == {
            'controller_name': 'task_space_impedance_mit_controller',
            'controller_names': ['task_space_impedance_mit_controller'],
            'controllers_file': 'controllers_mit.yaml',
            'controller_overrides': {},
        }


def test_direct_impedance_yaml_has_an_explicit_single_arm_safe_profile():
    params = yaml.safe_load(DIRECT_CONFIG.read_text())['/**']
    manager = params['controller_manager']['ros__parameters']
    name = 'joint_impedance_mit_controller'
    assert manager[name]['type'] == 'cho_controller_openarm_mit/JointImpedanceMitActionController'
    config = params[name]['ros__parameters']
    assert config['arm'] == 'single'
    assert config['safety_profile_name'] == 'mujoco_sim_safe'
    assert config['safety_profile_file'] == ''
    assert all(len(config[key]) == 7 for key in ('kp', 'kd', 'torque_limit'))
    assert config['kp'] == [35.0, 35.0, 30.0, 25.0, 5.0, 5.0, 5.0]
    assert config['kd'] == [1.75, 1.5, 1.25, 1.25, 0.5, 0.45, 0.4]
    # This is the action-only tau_ff cap, not the MIT wrapper final-torque
    # cap.  It must cover the validated single-arm MuJoCo gravity envelope for
    # home 0 -> reach 0 while remaining bounded by the named safety profile.
    assert config['torque_limit'] == [2.0, 5.0, 2.5, 4.0, 1.0, 1.0, 1.0]
    profile = yaml.safe_load(SAFETY_PROFILE.read_text())['profiles']['mujoco_sim_safe']
    tau_ff_limit = profile['torque']['tau_ff_magnitude']
    assert all(0.0 < configured <= allowed
               for configured, allowed in zip(config['torque_limit'], tau_ff_limit))


def test_task_impedance_yaml_exposes_explicit_cartesian_wrench_contract():
    params = yaml.safe_load(DIRECT_CONFIG.read_text())['/**']
    manager = params['controller_manager']['ros__parameters']
    name = 'task_space_impedance_mit_controller'
    assert manager[name]['type'] == 'cho_controller_openarm_mit/TaskSpaceImpedanceMitController'
    config = params[name]['ros__parameters']
    assert config['arm'] == 'single'
    assert config['safety_profile_name'] == 'mujoco_sim_safe'
    assert config['ee_frame'] == 'openarm_hand_tcp'
    assert all(len(config[key]) == 7 for key in ('kp', 'kd', 'torque_limit'))
    assert all(len(config[key]) == 6 for key in ('kp_task', 'kd_task'))
    assert config['lambda'] > 0.0 and config['max_delta_q'] > 0.0


def test_mit_description_rejects_custom_xacro_but_legacy_keeps_override():
    canonical = '/opt/cho/openarm_v10.urdf.xacro'
    launch_utils.enforce_mujoco_mit_description(False, '/tmp/custom.xacro', canonical)
    launch_utils.enforce_mujoco_mit_description(True, canonical, canonical)
    with pytest.raises(RuntimeError, match='xacro_file overrides are forbidden'):
        launch_utils.enforce_mujoco_mit_description(True, '/tmp/custom.xacro', canonical)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_single_fjt_maps_only_the_selected_bimanual_arm(arm):
    result = launch_utils.resolve_mujoco_mit_selection(
        True, 'torque', True, launch_utils.MIT_SINGLE_FJT_CONTROLLER, arm)
    assert result == {
        'controller_name': f'{arm}_follow_joint_trajectory_mit_controller',
        'controller_names': [f'{arm}_follow_joint_trajectory_mit_controller'],
        'controllers_file': 'controllers_mit_bimanual.yaml',
        'controller_overrides': {
            f'{arm}_follow_joint_trajectory_mit_controller': {'arm': arm},
        },
    }


@pytest.mark.parametrize('mode,bimanual,controller', [
    ('position', True, 'single_arm_follow_joint_trajectory_mit_controller'),
    ('torque', False, 'single_arm_follow_joint_trajectory_mit_controller'),
])
def test_invalid_mit_launch_combinations_fail_closed(mode, bimanual, controller):
    with pytest.raises(RuntimeError):
        launch_utils.resolve_mujoco_mit_selection(
            True, mode, bimanual, controller, 'left')


def test_mit_rejects_custom_controller_file_that_could_remap_plugin_boundary():
    with pytest.raises(RuntimeError, match='controllers_file overrides are forbidden'):
        launch_utils.resolve_mujoco_mit_selection(
            True, 'torque', True,
            launch_utils.MIT_SINGLE_FJT_CONTROLLER, 'left', '/tmp/untrusted.yaml')


def test_paired_moveit_maps_to_exclusive_canonical_controller():
    assert launch_utils.resolve_mujoco_mit_selection(
        True, 'torque', True, launch_utils.MIT_PAIRED_FJT_CONTROLLER, 'both') == {
            'controller_name': launch_utils.MIT_PAIRED_FJT_CONTROLLER,
            'controller_names': [launch_utils.MIT_PAIRED_FJT_CONTROLLER],
            'controllers_file': 'controllers_mit_moveit_bimanual.yaml',
            'controller_overrides': {},
        }


@pytest.mark.parametrize('bimanual,arm', [(False, 'both'), (True, 'left'), (True, 'right')])
def test_paired_moveit_rejects_non_bimanual_or_single_arm_combinations(bimanual, arm):
    with pytest.raises(RuntimeError, match='bimanual:=true and mit_arm:=both'):
        launch_utils.resolve_mujoco_mit_selection(
            True, 'torque', bimanual, launch_utils.MIT_PAIRED_FJT_CONTROLLER, arm)


def test_bimanual_direct_defaults_to_two_independent_seven_axis_instances():
    result = launch_utils.resolve_mujoco_mit_selection(
        True, 'torque', True, 'task_space_impedance_mit_controller', 'both_independent')
    assert result == {
        'controller_name': 'left_task_space_impedance_mit_controller',
        'controller_names': [
            'left_task_space_impedance_mit_controller',
            'right_task_space_impedance_mit_controller',
        ],
        'controllers_file': 'controllers_mit_direct_bimanual.yaml',
        'controller_overrides': {
            'left_task_space_impedance_mit_controller': {'arm': 'left'},
            'right_task_space_impedance_mit_controller': {'arm': 'right'},
        },
    }


@pytest.mark.parametrize('arm,expected', [
    ('left', ['left_joint_impedance_mit_controller']),
    ('right', ['right_joint_impedance_mit_controller']),
])
def test_bimanual_direct_can_select_one_disjoint_arm(arm, expected):
    result = launch_utils.resolve_mujoco_mit_selection(
        True, 'torque', True, 'joint_impedance_mit_controller', arm)
    assert result['controller_names'] == expected
    assert result['controller_overrides'][expected[0]] == {'arm': arm}


def test_bimanual_direct_rejects_paired_moveit_selector():
    with pytest.raises(RuntimeError, match='both_independent'):
        launch_utils.resolve_mujoco_mit_selection(
            True, 'torque', True, 'task_space_impedance_mit_controller', 'both')


def test_direct_bimanual_yaml_has_two_disjoint_one_arm_task_and_joint_plugins():
    params = yaml.safe_load(DIRECT_BIMANUAL_CONFIG.read_text())['/**']
    manager = params['controller_manager']['ros__parameters']
    for side in ('left', 'right'):
        joint = f'{side}_joint_impedance_mit_controller'
        task = f'{side}_task_space_impedance_mit_controller'
        assert manager[joint]['type'] == 'cho_controller_openarm_mit/JointImpedanceMitActionController'
        assert manager[task]['type'] == 'cho_controller_openarm_mit/TaskSpaceImpedanceMitController'
        for name in (joint, task):
            assert params[name]['ros__parameters']['arm'] == side
        task_params = params[task]['ros__parameters']
        assert task_params['ee_frame'] == f'openarm_{side}_hand_tcp'
        assert len(task_params['startup_posture']) == 7
    serialized = DIRECT_BIMANUAL_CONFIG.read_text()
    assert 'BimanualFollowJointTrajectoryController' not in serialized
    assert 'mit_pair_ownership' not in serialized


def test_paired_yaml_contains_only_one_14_axis_producer():
    params = yaml.safe_load(PAIRED_CONFIG.read_text())['/**']
    manager = params['controller_manager']['ros__parameters']
    producers = {
        name: value['type'] for name, value in manager.items()
        if isinstance(value, dict) and 'type' in value and
        ('TrajectoryController' in value['type'] or 'mit_controller' in name)
    }
    assert producers == {
        'bimanual_follow_joint_trajectory_mit_controller':
        'cho_controller_openarm_mit/BimanualFollowJointTrajectoryController'
    }
    serialized = PAIRED_CONFIG.read_text()
    assert 'SingleArmFollowJointTrajectoryController' not in serialized
    assert 'left_follow_joint_trajectory_mit_controller' not in serialized
    assert 'right_follow_joint_trajectory_mit_controller' not in serialized
    assert 'joint_position_mit_controller' not in serialized


def test_moveit_mapping_targets_the_same_single_14_axis_action_server():
    config = yaml.safe_load(MOVEIT_PAIRED_CONFIG.read_text())
    simple = config['moveit_simple_controller_manager']
    name = 'bimanual_follow_joint_trajectory_mit_controller'
    assert simple['controller_names'] == [name]
    assert simple[name]['action_ns'] == 'follow_joint_trajectory'
    assert len(simple[name]['joints']) == 14
    assert len(set(simple[name]['joints'])) == 14
    assert all('finger' not in joint for joint in simple[name]['joints'])


def test_moveit_wrapper_has_fail_closed_paired_boundary():
    source = MOVEIT_WRAPPER.read_text()
    assert "not bimanual or arm != 'both'" in source
    assert 'controllers_file overrides are forbidden for paired MIT MoveIt' in source
    assert "'mit_controller_name': 'bimanual_follow_joint_trajectory_mit_controller'" in source
    assert "'mit_paired': str(enabled).lower()" in source
    assert "'mujoco_mit_headless': 'true'" in source
    # The assignment stays inside the MIT-only update block. Legacy launch
    # therefore retains bringup_mujoco_robot's existing false default.
    assert source.index("if enabled:\n            robot_args.update") < source.index(
        "'mujoco_mit_headless': 'true'")


def test_paired_gate_runtime_parameters_do_not_construct_empty_tuple_value():
    metadata = {
        'trajectory_controllers': ['left_jtc', 'right_jtc'],
        'hold_controllers': ['left_hold', 'right_hold'],
    }
    paired = moveit_launch.build_gate_controller_parameters(metadata, True)
    assert paired == {
        'activate_controllers': ['bimanual_follow_joint_trajectory_mit_controller']
    }
    assert all(value != [] and value != () for value in paired.values())
    legacy = moveit_launch.build_gate_controller_parameters(metadata, False)
    assert legacy['deactivate_controllers'] == ['left_hold', 'right_hold']


def test_bimanual_yaml_exposes_exact_single_arm_plugins_without_pair_controller():
    with CONFIG.open() as stream:
        params = yaml.safe_load(stream)['/**']
    manager = params['controller_manager']['ros__parameters']
    expected = {
        'left_follow_joint_trajectory_mit_controller',
        'right_follow_joint_trajectory_mit_controller',
    }
    declared = {
        name for name, value in manager.items()
        if isinstance(value, dict) and
        value.get('type') ==
        'cho_controller_openarm_mit/SingleArmFollowJointTrajectoryController'
    }
    assert declared == expected
    assert params['left_follow_joint_trajectory_mit_controller']['ros__parameters']['arm'] == 'left'
    assert params['right_follow_joint_trajectory_mit_controller']['ros__parameters']['arm'] == 'right'
    serialized = CONFIG.read_text()
    assert 'BimanualFollowJointTrajectoryController' not in serialized
    assert 'mit_pair_ownership' not in serialized
    # rcl_yaml_param_parser in ROS 2 Humble rejects otherwise-valid YAML
    # anchors, aliases, and merge keys ("Will not support aliasing").
    assert '&' not in serialized
    assert '*mit_' not in serialized
    assert '<<:' not in serialized
