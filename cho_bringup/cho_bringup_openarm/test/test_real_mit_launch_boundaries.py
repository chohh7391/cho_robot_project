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

import importlib.util
import math
import re
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
URDF_XACRO = DESCRIPTION.with_name('openarm_v10.urdf.xacro')
SAFETY = (PACKAGE.parents[1] / 'cho_description' / 'cho_description_openarm' /
          'config' / 'mit_safety_profiles_v1.yaml')
RVIZ_CONFIG = (PACKAGE.parents[1] / 'cho_description' / 'cho_description_openarm' /
               'rviz' / 'openarm.rviz')

spec = importlib.util.spec_from_file_location('openarm_real_launch_utils_under_test', UTILS)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

UPSTREAM_POSITION_LOWER = [-1.396263, -1.745329, -1.570796, 0.0,
                           -1.570796, -0.785398, -1.570796]
UPSTREAM_POSITION_UPPER = [3.490659, 1.745329, 1.570796, 2.443461,
                           1.570796, 0.785398, 1.570796]
UPSTREAM_VELOCITY = [16.754666, 16.754666, 5.445426, 5.445426,
                     20.943946, 20.943946, 20.943946]


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


def test_real_bimanual_right_scope_never_starts_left_hardware_or_broadcaster():
    scope = launch_utils.resolve_real_mit_hardware_scope(True, 'right')
    assert scope['always_active_controllers'] == [
        'joint_state_broadcaster', 'right_ee_state_broadcaster']
    assert scope['optional_controllers'] == []
    source = REAL_LAUNCH.read_text()
    assert "' real_mit_arm:=', LaunchConfiguration('mit_arm')" in source
    assert 'hardware_components_initial_state.unconfigured' not in source
    assert "always_active=hardware_scope['always_active_controllers']" in source


def test_real_bimanual_both_scope_keeps_both_hardware_components_available():
    scope = launch_utils.resolve_real_mit_hardware_scope(
        True, 'both_independent')
    assert scope['always_active_controllers'] == [
        'joint_state_broadcaster', 'left_ee_state_broadcaster', 'right_ee_state_broadcaster']
    assert scope['optional_controllers'] == []


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
        # The finger may appear, but only as the gripper controller's own
        # joint. It must never enter an arm controller's block: the gripper is
        # outside the MIT arm contract, so its value in an arm vector would be
        # a value inside a lease generation and a SAFE acknowledgement.
        params = yaml.safe_load(raw)['/**']
        for name, entry in params.items():
            if name == 'controller_manager' or not isinstance(entry, dict):
                continue
            block = entry.get('ros__parameters', {})
            if name.endswith('gripper_controller'):
                assert 'finger' in block['gripper_joint']
                continue
            joints = block.get('joints', [])
            assert not any('finger' in joint for joint in joints), name
            assert 'finger' not in str(block.get('gripper_joint', '')), name
        params = yaml.safe_load(raw)['/**']
        manager = params['controller_manager']['ros__parameters']
        # The three declarations of the control rate have to agree: the
        # hardware refuses to configure when the profile's update_rate_hz
        # and mit_expected_update_rate_hz disagree, and every cycle-count
        # timeout in the profile is denominated in this rate.
        assert manager['update_rate'] == 750
        for name, entry in manager.items():
            if name.endswith('_mit_controller'):
                assert entry['type'].startswith('cho_controller_openarm_mit/')
                assert params[name]['ros__parameters']['safety_profile_name'] == (
                    'real_conservative_commissioning')
                assert params[name]['ros__parameters']['safety_backend'] == 'real'


def test_real_bimanual_broadcasters_do_not_override_empty_string_arrays():
    raw = REAL_BIMANUAL_CONFIG.read_text()
    params = yaml.safe_load(raw)['/**']
    for name in ('left_ee_state_broadcaster', 'right_ee_state_broadcaster'):
        broadcaster = params[name]['ros__parameters']
        assert broadcaster['gripper_joint'] == ''
        # In Humble, an untyped YAML [] cannot be converted to the string-array
        # parameter declared by OpenArmBaseController. Its declared default is
        # harmless because an empty gripper_joint disables all gripper state.
        assert 'gripper_mimic_joints' not in broadcaster


def test_real_task_profiles_remove_joint_reference_generator_parameters():
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        params = yaml.safe_load(config.read_text())['/**']
        for name, entry in params['controller_manager']['ros__parameters'].items():
            if not name.endswith('task_space_impedance_mit_controller'):
                continue
            task = params[name]['ros__parameters']
            for retired in ('max_delta_q', 'max_goal_translation', 'max_goal_rotation',
                            'max_absolute_radius', 'max_cartesian_velocity',
                            'max_angular_velocity', 'use_cartesian_speed_limits',
                            'lambda', 'task_joint_velocity_limits',
                            'task_joint_position_lower', 'task_joint_position_upper'):
                assert retired not in task


def _real_task_profiles(config):
    params = yaml.safe_load(config.read_text())['/**']
    return {
        name: params[name]['ros__parameters']
        for name in params['controller_manager']['ros__parameters']
        if name.endswith('task_space_impedance_mit_controller')
    }


def drive_side_joint_torque_limits(block):
    """The most impedance torque kp * |q_des - q| a drive-side block can ask of each joint.

    max_reference_offset bounds |q_des - q|; where it is unset (0) the controller
    derives 0.5 * torque_limit / kp, so the limit is half the motor peak.
    """
    limits = []
    for kp, torque, bound in zip(
            block['kp'], block['torque_limit'],
            block.get('max_reference_offset', [0.0] * 7)):
        offset = bound if bound > 0.0 else 0.5 * abs(torque) / kp
        limits.append(kp * offset)
    return limits


def test_real_task_tuning_stays_inside_profile_boundaries_and_matches_its_law():
    profiles = yaml.safe_load(SAFETY.read_text())['profiles']
    commissioning = profiles['real_conservative_commissioning']
    expected_wrench = [10.0, 10.0, 10.0, 1.5, 1.5, 1.5]
    expected_torque = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0]
    # What the drive-side law can ask of each joint: half the motor peak.
    expected_drive_torque = [20.0, 20.0, 13.5, 13.5, 3.5, 3.5, 3.5]
    task_profiles = {}
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        task_profiles.update(_real_task_profiles(config))
    assert len(task_profiles) == 3
    for name, task in task_profiles.items():
        assert len(task['kp']) == 7, name
        assert len(task['kd']) == 7, name
        assert len(task['kp_task']) == 6 and len(task['kd_task']) == 6, name
        assert all(value >= 0.0 for value in task['kp_task'] + task['kd_task']), name
        assert task['torque_limit'] == expected_torque, name
        # The real adapter rejects the whole arm tuple above these ceilings.
        assert all(value <= ceiling for value, ceiling in zip(
            task['kp'], commissioning['gains']['kp_max'])), name
        assert all(value <= ceiling for value, ceiling in zip(
            task['kd'], commissioning['gains']['kd_max'])), name
        assert task['friction_velocity_epsilon'] > 0.0, name
        if task['drive_side_impedance']:
            # The drive evaluates kp*(q_des - q): a zero kp is a dead loop, the
            # Cartesian error would ride q_des and nothing would act on it. The
            # controller refuses to configure that combination.
            assert all(value > 0.0 for value in task['kp']), name
            # max_task_wrench is inert here (the controller only requires six
            # positive values), so it is not the force limit. The limit is the
            # joint torque kp * max|q_des - q|, which is what to pin.
            assert len(task['max_task_wrench']) == 6, name
            assert all(value > 0.0 for value in task['max_task_wrench']), name
            limits = drive_side_joint_torque_limits(task)
            assert limits == pytest.approx(expected_drive_torque), name
            assert all(limit <= 0.5 * final + 1e-9 for limit, final in zip(
                limits, commissioning['torque']['final_magnitude'])), name
        else:
            assert task['max_task_wrench'] == expected_wrench, name
            # The tau_ff law carries the stiffness itself, and with kd_task
            # zero the drive kd is the only damping in the loop. Both must be
            # present or the arm has either no spring or no dissipation.
            assert all(value > 0.0 for value in task['kp_task'][:3]), name
            assert all(value > 0.0 for value in task['kd']), name


def test_real_drive_side_blocks_weight_the_reference_offset_by_kp():
    # With reference_offset_weight proportional to kp the drive-side law's
    # Cartesian stiffness is (J K^-1 J^T)^-1, K = diag(kp), instead of whatever
    # the least-norm split happens to give. Geometric-mean normalisation (the
    # solve's damping term is not rescaled with the weight) keeps it from
    # attenuating the offset as a whole. MuJoCo: achieved motion within 1.3% of
    # unit weights, null-direction joint motion 31% lower. The legacy law (kp 0)
    # never reads the weight, so it must not carry one.
    drive_side = []
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        params = yaml.safe_load(config.read_text())['/**']
        for name in params['controller_manager']['ros__parameters']:
            if not name.endswith(('task_space_impedance_mit_controller', 'vla_mit_controller')):
                continue
            block = params[name]['ros__parameters']
            if not block['drive_side_impedance']:
                assert 'reference_offset_weight' not in block, name
                continue
            drive_side.append(name)
            kp = block['kp']
            geomean = math.exp(sum(math.log(value) for value in kp) / len(kp))
            assert block['reference_offset_weight'] == pytest.approx(
                [value / geomean for value in kp], abs=1e-3), name
    assert sorted(drive_side) == ['task_space_impedance_mit_controller', 'vla_mit_controller']


def test_real_bimanual_task_tuning_is_symmetric_between_the_arms():
    # Both arms are the same mechanism bolted to the same torso, so a gain that
    # is right for one is right for the other. The single-arm bringup is NOT
    # held to these values: it mounts the arm differently, gravity enters the
    # model along a different axis, and it is tuned on its own hardware runs.
    task_profiles = _real_task_profiles(REAL_BIMANUAL_CONFIG)
    left = task_profiles['left_task_space_impedance_mit_controller']
    right = task_profiles['right_task_space_impedance_mit_controller']
    for key in ('drive_side_impedance', 'kp', 'kd', 'kp_task', 'kd_task', 'max_task_wrench',
                'torque_limit', 'friction_level', 'friction_scale', 'friction_velocity_epsilon',
                'friction_kinetic_ratio', 'friction_stribeck_velocity',
                'friction_velocity_source', 'task_inertia_weighting', 'use_nullspace_posture',
                'kp_null', 'kd_null', 'joint_limit_stiffness', 'gravity_joint_scale'):
        assert left[key] == right[key], key
    # Measured on hardware 2026-09-05 (right arm, hanging posture, +x 50 mm /
    # 5 s probes): 40, 60, 86 and 120 N/m were all monotonic stick-slip with no
    # oscillation, and 120 is where the sweep stopped because drive-side kd 5
    # gives roughly zeta 0.7 there. Pin the value so it cannot drift upward
    # without another damping check.
    assert right['kp_task'] == [120.0, 120.0, 120.0, 7.0, 7.0, 7.0]
    assert right['kd_task'] == [0.0] * 6
    assert right['friction_velocity_epsilon'] == 0.02


def test_real_task_controllers_keep_nonzero_joint_damping_for_the_null_space():
    # J^T*Dx*J damps only the range space of the Jacobian, so a seven-axis arm
    # under a six-axis Cartesian task needs actuator-side kd for its null
    # space. A zero kd vector would leave that direction entirely undamped.
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        params = yaml.safe_load(config.read_text())['/**']
        for name in params['controller_manager']['ros__parameters']:
            if not name.endswith('task_space_impedance_mit_controller'):
                continue
            task = params[name]['ros__parameters']
            assert len(task['kd']) == 7
            assert all(value > 0.0 for value in task['kd'])
            # startup_kd is deliberately NOT task kd any more. Startup and
            # return-to-zero run a joint move under the RTZ profile's ceilings,
            # while task kd is the Cartesian damping and sits at the MIT
            # packet's kd cap of 5 on the proximal joints.
            assert task['startup_kd'] != task['kd']
            assert all(value > 0.0 for value in task['startup_kd'])


def test_the_control_rate_and_the_can_frame_budget_stay_consistent():
    # The rate and the per-cycle frame count are not independent. Measured on
    # hardware: 7 tx + 7 rx per cycle is 10.5k frames/s at 750 Hz, about 50% of
    # a 1 Mbps CAN FD bus. Leaving the redundant 0xCC state query in doubles
    # that to 101% - the bus cannot deliver the rate, and the cycle silently
    # slips rather than failing loudly. So a 750 Hz default requires the query
    # to be off by default.
    launch_text = (REAL_CONFIG.parents[2] / 'launch' / 'bringup_real_robot.launch.py').read_text()
    profile = yaml.safe_load(SAFETY.read_text())['profiles']['real_conservative_commissioning']
    rate = profile['update_rate_hz']
    if rate > 400:
        assert "'mit_state_from_command_reply', default_value='true'" in launch_text, (
            f'{rate} Hz needs the redundant state query off by default; with it the '
            f'frame budget is about {rate * 28 * 48e-6 * 100:.0f}% of the bus')


def test_launch_does_not_swap_the_safety_profile_for_the_return_to_zero_phase():
    # Selecting the return-to-zero profile when return_to_zero was true meant
    # the TASK gains were validated against ceilings sized for homing, and a
    # Cartesian kd of 5 was rejected at configure. The phases already have
    # separate gain parameters; the profile is the session-wide envelope and
    # must not depend on which phases the session contains.
    launch_text = (REAL_CONFIG.parents[2] / 'launch' / 'bringup_real_robot.launch.py').read_text()
    assert "profile = 'real_conservative_commissioning'" in launch_text
    assert "'real_return_to_zero_commissioning' if return_to_zero" not in launch_text
    # And the gains the config actually asks for have to fit that envelope, in
    # both phases, or the wall simply moves rather than disappearing.
    profile = yaml.safe_load(SAFETY.read_text())['profiles']['real_conservative_commissioning']
    gains = profile['gains']
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        params = yaml.safe_load(config.read_text())['/**']
        for name in params['controller_manager']['ros__parameters']:
            if not name.endswith('task_space_impedance_mit_controller'):
                continue
            task = params[name]['ros__parameters']
            for field, ceiling in (('kp', 'kp_max'), ('kd', 'kd_max'),
                                   ('startup_kp', 'kp_max'), ('startup_kd', 'kd_max')):
                for joint, (value, cap) in enumerate(zip(task[field], gains[ceiling]), start=1):
                    assert value <= cap, (
                        f'{config.name}:{name} {field}[{joint}]={value} exceeds '
                        f'{ceiling}={cap}')


def test_every_openarm_launch_file_actually_parses():
    # These tests read the launch files as TEXT and assert on substrings, which
    # says nothing about whether Python can load them. A missing parenthesis in
    # an argument description passed every string assertion here and only
    # surfaced as a launch-time SyntaxError in front of a live robot. Compiling
    # is cheap and catches exactly that class of edit.
    import py_compile
    import tempfile
    launch_dir = REAL_CONFIG.parents[2] / 'launch'
    files = sorted(launch_dir.glob('*.launch.py'))
    assert files, f'no launch files under {launch_dir}'
    for path in files:
        with tempfile.NamedTemporaryFile(suffix='.pyc') as out:
            try:
                py_compile.compile(str(path), cfile=out.name, doraise=True)
            except py_compile.PyCompileError as exc:
                raise AssertionError(f'{path.name} does not compile: {exc}') from exc


def test_control_rate_is_declared_consistently_across_every_place_that_states_it():
    # The rate is written down three times: controller_manager update_rate, the
    # safety profile update_rate_hz, and the mit_expected_update_rate_hz launch
    # argument. The hardware only cross-checks the last two and fails configure
    # on a mismatch, which is a late and cryptic way to learn about it - the
    # launch used to hardcode 200 while the YAML said otherwise. Every
    # cycle-count timeout in the profile is denominated in this rate too, so a
    # silent disagreement rescales lease, staleness and watchdog windows.
    profiles = yaml.safe_load(SAFETY.read_text())['profiles']
    real = [p for p in profiles.values()
            if p['backend'] == 'real' and p['update_rate_hz'] is not None]
    assert real, 'no real profile declares a rate'
    rates = {p['update_rate_hz'] for p in real}
    assert len(rates) == 1, f'real profiles disagree on the control rate: {rates}'
    profile_rate = rates.pop()
    for config in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        params = yaml.safe_load(config.read_text())['/**']
        manager = params['controller_manager']['ros__parameters']
        assert manager['update_rate'] == profile_rate, (
            f'{config.name} runs at {manager["update_rate"]} Hz while the safety '
            f'profile declares {profile_rate} Hz')
    launch_text = (REAL_CONFIG.parents[2] / 'launch' / 'bringup_real_robot.launch.py').read_text()
    assert "'mit_expected_update_rate_hz', default_value='" f"{profile_rate}'" in launch_text, (
        'the launch default for mit_expected_update_rate_hz must match the profile rate')
    assert 'mit_expected_update_rate_hz:=200' not in launch_text, (
        'the expected rate must not be hardcoded again')


def test_cycle_count_timeouts_still_mean_the_same_wall_clock_windows():
    # lease/staleness/producer windows are cycle counts, so they only mean what
    # the profile intends at the declared rate. These are the durations the
    # 200 Hz profile committed to; a rate change has to carry them over.
    expected_ms = {'lease_default_cycles': 50.0, 'lease_cap_cycles': 100.0,
                   'producer_refresh_cycles': 25.0, 'state_stale_cycles': 100.0}
    for name, profile in yaml.safe_load(SAFETY.read_text())['profiles'].items():
        if profile['backend'] != 'real' or profile['update_rate_hz'] is None:
            continue
        rate = profile['update_rate_hz']
        for field, want_ms in expected_ms.items():
            got_ms = profile['timing'][field] / rate * 1000.0
            assert abs(got_ms - want_ms) <= 0.1 * want_ms, (
                f'{name}.{field} is {got_ms:.1f} ms at {rate} Hz, expected ~{want_ms} ms')
        timing = profile['timing']
        assert 0 < timing['producer_refresh_cycles'] < timing['lease_default_cycles']
        assert timing['lease_default_cycles'] <= timing['lease_cap_cycles']


def test_all_real_direct_mit_controller_configs_use_the_upstream_motor_torque_tuple():
    expected = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0]
    for path in (REAL_CONFIG, REAL_BIMANUAL_CONFIG):
        params = yaml.safe_load(path.read_text())['/**']
        manager = params['controller_manager']['ros__parameters']
        for name in manager:
            if not name.endswith('_mit_controller'):
                continue
            assert params[name]['ros__parameters']['torque_limit'] == expected
    for profile in yaml.safe_load(SAFETY.read_text())['profiles'].values():
        assert profile['joint_limits']['physical_torque'] == expected
        assert profile['torque']['tau_ff_magnitude'] == expected
        assert profile['torque']['final_magnitude'] == expected


def test_description_exposes_the_real_mit_plugin_contract_without_runtime_gates():
    source = DESCRIPTION.read_text()
    assert 'cho_hardware_openarm_mit_real/OpenArmMitRealSystem' in source
    for name in ('mit_safety_profile_file', 'mit_safety_profile',
                 'mit_expected_update_rate_hz', 'can_interface', 'can_fd',
                 'arm_side'):
        assert f'name="{name}"' in source
    for retired in ('open_can', 'enable_motors', 'operator_approval'):
        assert f'name="{retired}"' not in source
    assert 'side="left"' in source and 'side="right"' in source and 'side="single"' in source
    # The real adapter drives the hand now, so the finger is emitted for it -
    # but as an ordinary joint. A command interface of any MIT tuple field, or
    # the arm's hold_torque, on the finger would let the gripper controller
    # claim part of the arm contract, which the contract forbids.
    assert 'name="hand"' in source
    assert 'name="gripper_motor_open"' in source
    assert '<command_interface name="max_effort"/>' in source
    finger_block = source[source.index('<xacro:macro name="one_hand_joints"'):]
    finger_block = finger_block[:finger_block.index('</xacro:macro>')]
    for tuple_field in ('stiffness', 'damping', 'hold_torque'):
        assert f'command_interface name="{tuple_field}"' not in finger_block


def test_real_launch_always_starts_control_node_without_runtime_gates():
    source = REAL_LAUNCH.read_text()
    assert 'ros2_control_node' in source
    for retired in ('open_can', 'operator_approval', 'enable_motors', 'opt_in'):
        assert retired not in source
    # Off by default: the motor id, the closed motor angle and the force scale
    # are per-hand measurements nobody has taken on this robot yet.
    assert "'hand', default_value='false'" in source
    assert 'hand:=true is unsupported by the current real MIT bringup' not in source
    assert "LaunchConfiguration('hand').perform(context)" in source


def test_real_gripper_controllers_follow_the_arm_selection():
    # A non-selected bimanual arm is a state-only GenericSystem, finger
    # included, so a gripper controller there would fail on a missing position
    # command interface.
    scope = launch_utils.resolve_real_mit_hardware_scope(True, 'right', hand=True)
    assert scope['always_active_controllers'] == [
        'joint_state_broadcaster', 'right_ee_state_broadcaster']
    assert scope['optional_controllers'] == ['right_gripper_controller']
    assert launch_utils.resolve_real_mit_hardware_scope(
        True, 'right', hand=False)['optional_controllers'] == []
    assert launch_utils.resolve_real_mit_hardware_scope(
        True, 'both_independent', hand=True)['optional_controllers'] == [
        'left_gripper_controller', 'right_gripper_controller']
    assert launch_utils.resolve_real_mit_hardware_scope(
        False, 'left', hand=True)['optional_controllers'] == ['gripper_controller']


def test_a_gripper_that_cannot_load_never_blocks_the_arm_controller():
    # Measured on hardware 2026-09-05: a gripper controller in the same spawner
    # list as the arm exited the spawner on its own failure, and the arm
    # controller was never spawned at all - motors energised in their SAFE hold
    # with nothing commanding them. The gripper gets its own spawner, started
    # only after the arm one has finished.
    spawners = launch_utils.create_controller_spawners(
        always_active=['joint_state_broadcaster', 'right_ee_state_broadcaster'],
        switchable_controllers=['right_task_space_impedance_mit_controller'],
        initial_active_controllers=['right_task_space_impedance_mit_controller'],
        optional_controllers=['right_gripper_controller'])

    # launch.Node keeps its arguments as substitutions; render them to text.
    def controller_arguments(node):
        rendered = []
        for argument in getattr(node, '_Node__arguments', []) or []:
            for piece in (argument if isinstance(argument, (list, tuple)) else [argument]):
                rendered.append(str(getattr(piece, 'text', piece)))
        return rendered

    arm_spawner = spawners[-1]
    arm_arguments = controller_arguments(arm_spawner)
    assert 'right_task_space_impedance_mit_controller' in arm_arguments
    # The gripper is not in the arm's list, so its failure cannot stop the arm.
    assert 'right_gripper_controller' not in arm_arguments
    # The follower spawners live inside the OnProcessExit handler; whichever
    # private attribute holds them, the gripper must be in exactly one spawner
    # and that spawner must not be the arm's.
    handler = spawners[0]._RegisterEventHandler__event_handler
    followers = next(
        value for value in vars(handler).values()
        if isinstance(value, list) and any(hasattr(item, '_Node__arguments') for item in value))
    assert any('right_gripper_controller' in controller_arguments(node) for node in followers)


def test_real_launch_loads_repo_owned_robot_model_rviz_config():
    source = REAL_LAUNCH.read_text()
    assert "rviz_config = os.path.join(description_path, 'rviz', 'openarm.rviz')" in source
    assert "arguments=['-d', rviz_config]" in source

    config = yaml.safe_load(RVIZ_CONFIG.read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'world'
    robot_models = [
        display for display in manager['Displays']
        if display.get('Class') == 'rviz_default_plugins/RobotModel'
    ]
    assert len(robot_models) == 1
    robot_model = robot_models[0]
    assert robot_model['Enabled'] is True
    assert robot_model['Description Source'] == 'Topic'
    assert robot_model['Description Topic']['Value'] == '/robot_description'
    assert robot_model['Description Topic']['Durability Policy'] == 'Transient Local'


def test_real_return_to_zero_defaults_off_but_keeps_the_canonical_upstream_gains():
    source = REAL_LAUNCH.read_text()
    # Defaults OFF: nominal zero is a singularity for this arm and Cartesian
    # entry there faults the controller. The argument still exists, and
    # gravity_compensation still requires it.
    assert "'return_to_zero', default_value='false'" in source
    assert "'return_to_zero_duration': 2.0" in source
    assert '200 interpolation steps at 10 ms per step' in source
    assert "'return_to_zero_kp': [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0]" in source
    assert "'startup_kp': [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0]" in source
    assert source.index("profile = 'real_conservative_commissioning'") < source.index('runtime_overrides = {}')


def test_commissioning_profiles_are_explicit_and_derated():
    profiles = yaml.safe_load(SAFETY.read_text())['profiles']
    default = profiles['real_conservative_unapproved']
    commissioning = profiles['real_conservative_commissioning']
    assert default['hardware_enable_allowed'] is False
    assert commissioning['status'] == 'commissioning_experiment_allowed'
    assert commissioning['hardware_enable_allowed'] is True
    assert commissioning['approval_gate'] == 'real_bringup_invocation_required'
    assert commissioning['update_rate_hz'] == 750
    upstream_torque = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0]
    for profile in (default, commissioning):
        assert profile['joint_limits']['physical_torque'] == upstream_torque
        assert profile['torque']['tau_ff_magnitude'] == upstream_torque
        assert profile['torque']['final_magnitude'] == upstream_torque
    return_zero = profiles['real_return_to_zero_commissioning']
    assert return_zero['hardware_enable_allowed'] is True
    assert return_zero['approval_gate'] == 'real_bringup_invocation_required'
    assert return_zero['joint_limits']['command_velocity'] == commissioning['joint_limits']['command_velocity']
    assert return_zero['torque'] == commissioning['torque']
    assert return_zero['joint_limits']['physical_torque'] == upstream_torque
    assert return_zero['gains']['kp_max'] == [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0]


def test_real_vla_controller_gets_the_task_space_startup_gains_for_return_to_zero():
    # VlaController derives from TaskSpaceImpedanceController, which reads
    # startup_kp/kd; return_to_zero_kp/kd is read only by a joint-space action
    # controller. Overriding the latter on a VLA left the YAML startup gains in
    # force. The launch golden case vla_return_to_zero evaluates the result.
    source = REAL_LAUNCH.read_text()
    guarded = source[:source.index("'startup_kp': [70.0")]
    guard = guarded[guarded.rindex('if controller_name'):]
    assert "'task_space_impedance_mit_controller'" in guard
    assert "'vla_mit_controller'" in guard
    assert "'joint_impedance_mit_controller'" not in guard


def test_no_openarm_controller_config_carries_the_unread_startup_posture_key():
    # The return-to-zero phase targets the nominal zero posture and the measured
    # pose is latched otherwise; no controller reads a startup_posture parameter.
    def keys(node):
        if isinstance(node, dict):
            for key, value in node.items():
                yield key
                yield from keys(value)
        elif isinstance(node, list):
            for value in node:
                yield from keys(value)

    files = sorted((PACKAGE / 'config').rglob('*.yaml'))
    assert files
    for path in files:
        assert 'startup_posture' not in set(keys(yaml.safe_load(path.read_text()))), path.name


def test_xacro_default_expected_update_rate_is_the_real_profile_rate():
    # A direct xacro run (without the launch, which passes 750) used to default
    # to 200, which the adapter refuses at configure against the 750 Hz profile.
    rate = yaml.safe_load(SAFETY.read_text())['profiles'][
        'real_conservative_commissioning']['update_rate_hz']
    urdf = re.search(r'name="mit_expected_update_rate_hz" default="(\d+)"', URDF_XACRO.read_text())
    assert urdf and int(urdf.group(1)) == rate
    macro = re.search(r'mit_expected_update_rate_hz:=(\d+)', DESCRIPTION.read_text())
    assert macro and int(macro.group(1)) == rate


def _values_of_key(node, key):
    """Every value stored under `key`, anywhere in a parsed YAML tree."""
    found = []
    if isinstance(node, dict):
        for name, value in node.items():
            if name == key:
                found.append(value)
            found.extend(_values_of_key(value, key))
    elif isinstance(node, list):
        for value in node:
            found.extend(_values_of_key(value, key))
    return found


def test_real_safe_hold_torque_band_equals_every_friction_level():
    # The SAFE hold's band in motion is the identified static friction of the real
    # controllers' joints, kept in two places: the profile's
    # torque.safe_hold_torque_band and the controllers' friction_level. This is
    # the only thing that keeps them equal.
    configs = sorted((PACKAGE / 'config' / 'real').glob('controllers_mit*.yaml'))
    assert configs, 'no real controllers_mit*.yaml'
    levels = []
    for config in configs:
        for level in _values_of_key(yaml.safe_load(config.read_text()), 'friction_level'):
            levels.append((config.name, level))
    assert levels, 'no friction_level in any real controllers_mit*.yaml'

    profiles = yaml.safe_load(SAFETY.read_text())['profiles']
    real_bands = {
        name: profile['torque']['safe_hold_torque_band']
        for name, profile in profiles.items()
        if name.startswith('real_') and profile['torque']['safe_hold_torque_band'] is not None
    }
    assert real_bands, 'no real profile carries a safe_hold_torque_band'
    for profile_name, band in real_bands.items():
        assert len(band) == 7, profile_name
        for config_name, level in levels:
            assert level == band, (
                f'{config_name}: friction_level {level} differs from '
                f'{profile_name}.torque.safe_hold_torque_band {band}')
    # The profiles agree with each other as well.
    assert len({tuple(band) for band in real_bands.values()}) == 1


def test_friction_level_walk_finds_nested_and_listed_keys():
    # The walk that the band comparison relies on must see a key at any depth,
    # inside lists included, and nothing that only contains the name.
    tree = {'a': {'friction_level': [1.0]}, 'b': [{'c': {'friction_level': [2.0]}}],
            'friction_level_scale': [3.0], 'd': 'friction_level'}
    assert _values_of_key(tree, 'friction_level') == [[1.0], [2.0]]
