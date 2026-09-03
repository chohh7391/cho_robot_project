import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
UTILS = PACKAGE / 'utils' / 'launch_utils.py'
REAL_CONFIG = PACKAGE / 'config' / 'real' / 'controllers_mit.yaml'
REAL_BIMANUAL_CONFIG = PACKAGE / 'config' / 'real' / 'controllers_mit_bimanual.yaml'
REAL_LAUNCH = PACKAGE / 'launch' / 'bringup_real_robot.launch.py'
DESCRIPTION = (PACKAGE.parents[1] / 'cho_description' / 'cho_description_openarm' /
               'robots' / 'openarm_v10' / 'openarm_v10.ros2_control.xacro')
SAFETY = (PACKAGE.parents[1] / 'cho_description' / 'cho_description_openarm' /
          'config' / 'mit_safety_profiles_v1.yaml')

spec = importlib.util.spec_from_file_location('openarm_real_launch_utils_under_test', UTILS)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)


def test_real_single_arm_selection_is_fixed_to_the_commissioning_map():
    assert launch_utils.resolve_real_mit_selection(
        False, 'joint_impedance_mit_controller', 'both_independent') == {
            'controller_names': ['joint_impedance_mit_controller'],
            'controllers_file': 'controllers_mit.yaml',
            'controller_overrides': {},
        }


def test_real_bimanual_selection_only_owns_disjoint_direct_arms():
    assert launch_utils.resolve_real_mit_selection(
        True, 'task_space_impedance_mit_controller', 'both_independent') == {
            'controller_names': [
                'left_task_space_impedance_mit_controller',
                'right_task_space_impedance_mit_controller',
            ],
            'controllers_file': 'controllers_mit_bimanual.yaml',
            'controller_overrides': {
                'left_task_space_impedance_mit_controller': {'arm': 'left'},
                'right_task_space_impedance_mit_controller': {'arm': 'right'},
            },
        }


@pytest.mark.parametrize('controller,arm', [
    ('joint_position_mit_controller', 'left'),
    ('bimanual_follow_joint_trajectory_mit_controller', 'both'),
    ('task_space_impedance_mit_controller', 'both'),
])
def test_real_rejects_unsupported_or_paired_controller_ownership(controller, arm):
    with pytest.raises(RuntimeError):
        launch_utils.resolve_real_mit_selection(True, controller, arm)


def test_real_rejects_user_controller_yaml_override():
    with pytest.raises(RuntimeError, match='controllers_file overrides are forbidden'):
        launch_utils.resolve_real_mit_selection(
            False, 'joint_impedance_mit_controller', 'left', '/tmp/untrusted.yaml')


def test_real_configs_are_explicit_commissioning_profile_without_yaml_aliases():
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        raw = config.read_text()
        assert '&' not in raw and '<<:' not in raw
        assert 'finger' not in raw
        params = yaml.safe_load(raw)['/**']
        manager = params['controller_manager']['ros__parameters']
        assert manager['update_rate'] == 200
        for name, entry in manager.items():
            if name.endswith('_mit_controller'):
                assert entry['type'].startswith('cho_controller_openarm_mit/')
                assert params[name]['ros__parameters']['safety_profile_name'] == (
                    'real_conservative_commissioning')
                assert params[name]['ros__parameters']['safety_backend'] == 'real'


def test_description_exposes_the_full_gated_real_mit_plugin_contract():
    source = DESCRIPTION.read_text()
    assert 'cho_hardware_openarm_mit_real/OpenArmMitRealSystem' in source
    for name in ('mit_safety_profile_file', 'mit_safety_profile',
                 'mit_expected_update_rate_hz', 'can_interface', 'can_fd',
                 'arm_side', 'open_can', 'enable_motors', 'operator_approval'):
        assert f'name="{name}"' in source
    assert 'side="left"' in source and 'side="right"' in source and 'side="single"' in source
    assert "not (hardware == 'real' and real_mit_hardware)" in source


def test_real_launch_requires_all_three_acknowledgements_before_control_node_exists():
    source = REAL_LAUNCH.read_text()
    assert "for name in ('open_can', 'operator_approval', 'enable_motors')" in source
    assert 'if not opt_in:' in source
    assert 'ros2_control is not started' in source
    for argument in ('open_can', 'operator_approval', 'enable_motors'):
        assert f"'{argument}', default_value='false'" in source
    assert "'hand', default_value='false'" in source
    assert 'hand:=true is unsupported by the current real MIT bringup' in source


def test_commissioning_profile_is_explicit_and_derated_but_default_stays_unapproved():
    profiles = yaml.safe_load(SAFETY.read_text())['profiles']
    default = profiles['real_conservative_unapproved']
    commissioning = profiles['real_conservative_commissioning']
    assert default['hardware_enable_allowed'] is False
    assert commissioning['status'] == 'commissioning_experiment_allowed'
    assert commissioning['hardware_enable_allowed'] is True
    assert commissioning['approval_gate'] == 'triple_runtime_opt_in_required'
    assert commissioning['update_rate_hz'] == 200
    assert all(c < u for c, u in zip(
        commissioning['torque']['final_magnitude'],
        default['torque']['final_magnitude']))
