from math import isfinite
from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = yaml.safe_load((ROOT / "config" / "mit_command_v1.yaml").read_text())
SAFETY = yaml.safe_load((ROOT / "config" / "mit_safety_profiles_v1.yaml").read_text())


def test_v1_schema_and_exact_interface_order():
    assert CONTRACT["schema"] == "cho.openarm.mit_command"
    assert CONTRACT["schema_version"] == "1.0.0-draft"
    assert CONTRACT["approval"] == "math_and_shape_only"
    assert [entry["name"] for entry in CONTRACT["interfaces"]] == [
        "position", "velocity", "stiffness", "damping", "effort"
    ]
    assert [entry["symbol"] for entry in CONTRACT["interfaces"]] == [
        "q_des", "dq_des", "kp", "kd", "tau_ff"
    ]


def test_joint_names_are_seven_and_prefix_consistent():
    expected = {
        "single": "openarm_joint",
        "left": "openarm_left_joint",
        "right": "openarm_right_joint",
    }
    for profile, prefix in expected.items():
        assert CONTRACT["joint_order"][profile] == [f"{prefix}{i}" for i in range(1, 8)]


def test_runtime_gains_and_safe_seed_are_fail_closed():
    by_symbol = {entry["symbol"]: entry for entry in CONTRACT["interfaces"]}
    assert by_symbol["kp"]["minimum"] == 0.0
    assert by_symbol["kd"]["minimum"] == 0.0
    assert CONTRACT["non_finite_allowed"] is False
    assert CONTRACT["invalid_policy"] == "reject_whole_arm_command"
    assert CONTRACT["claim_policy"] == "all_five_per_joint"
    assert CONTRACT["safe_seed"] == {
        "q_des": "measured_position",
        "dq_des": 0.0,
        "kp": "safe_hold_stiffness",
        "kd": "safe_hold_damping",
        "tau_ff": 0.0,
    }


def test_gripper_is_not_part_of_mit_joint_order():
    assert CONTRACT["scope"] == "arm_only"
    assert CONTRACT["gripper_contract"] == "separate_position_control"
    assert all("finger" not in joint for joints in CONTRACT["joint_order"].values() for joint in joints)


def test_commit_ack_protocol_is_explicit_and_numeric_profiles_are_explicitly_selected():
    assert [item["name"] for item in CONTRACT["protocol_interfaces"]["command"]] == [
        "mit_session_echo", "mit_lease_cycles", "mit_commit_generation",
        "mit_safe_request_generation"
    ]
    assert [item["name"] for item in CONTRACT["protocol_interfaces"]["state"]] == [
        "mit_session_id", "mit_ack_generation", "mit_safe_generation",
        "mit_safe_ack_generation", "mit_status"
    ]
    assert CONTRACT["protocol_interfaces"]["resources"] == {
        "single": "openarm_arm",
        "left": "openarm_left_arm",
        "right": "openarm_right_arm",
    }
    assert CONTRACT["generation_max"] == 2**53 - 1
    assert CONTRACT["topology"] == "single_synchronous_controller_manager"
    assert CONTRACT["numeric_policy"]["selection_required"] is True


def _profiles():
    return CONTRACT["numeric_policy"]["profiles"]


def _numeric_vectors(profile):
    for section in ("joint_limits", "gains", "torque"):
        for name, value in profile[section].items():
            if isinstance(value, list):
                yield section, name, value


def test_numeric_profiles_are_seven_axis_finite_or_explicitly_tbd():
    for profile in _profiles().values():
        for _, _, values in _numeric_vectors(profile):
            assert len(values) == 7
            assert all(isfinite(value) for value in values)
        for section in ("gains", "torque", "timing"):
            for value in profile[section].values():
                if value is None or isinstance(value, (str, list)):
                    continue
                assert isfinite(value)


def test_authoritative_safety_source_and_contract_projection_do_not_drift():
    assert CONTRACT["numeric_policy"]["authoritative_source"] == "mit_safety_profiles_v1.yaml"
    assert CONTRACT["numeric_policy"]["default_profile"] is None
    assert SAFETY["default_profile"] is None
    for name, projected in _profiles().items():
        authoritative = SAFETY["profiles"][name]
        for key in ("status", "backend", "hardware_enable_allowed", "joint_limits", "gains"):
            assert projected[key] == authoritative[key]
        for key in ("tau_ff_magnitude", "tau_ff_slew_per_s", "final_magnitude", "final_slew_per_s", "ordering"):
            assert projected["torque"][key] == authoritative["torque"][key]


def test_authoritative_profile_does_not_drift_from_urdf_gains_limits_or_pinned_packet_ranges():
    sim = SAFETY["profiles"]["mujoco_sim_safe"]
    arm = ROOT / "assets" / "robot" / "openarm_v1.0" / "config" / "arm"
    limits = yaml.safe_load((arm / "joint_limits.yaml").read_text())
    gains = yaml.safe_load((arm / "control_gains.yaml").read_text())
    assert sim["joint_limits"]["position_lower"] == [limits[f"joint{i}"]["limit"]["lower"] for i in range(1, 8)]
    assert sim["joint_limits"]["position_upper"] == [limits[f"joint{i}"]["limit"]["upper"] for i in range(1, 8)]
    assert sim["joint_limits"]["physical_velocity"] == [limits[f"joint{i}"]["limit"]["velocity"] for i in range(1, 8)]
    assert sim["joint_limits"]["physical_torque"] == [float(limits[f"joint{i}"]["limit"]["effort"]) for i in range(1, 8)]
    assert sim["gains"]["kp_max"] == [gains[f"joint{i}"]["kp"] for i in range(1, 8)]
    assert sim["gains"]["kd_max"] == [gains[f"joint{i}"]["kd"] for i in range(1, 8)]
    constants = (ROOT.parents[1] / "extern" / "openarm_can" / "include" / "openarm" / "damiao_motor" / "dm_motor_constants.hpp").read_text()
    packet_torque = {}
    for motor in ("DM4310", "DM4340", "DM8009"):
        match = re.search(r"\{12\.5,\s*[^,]+,\s*([0-9.]+)\},\s*//\s*" + motor + r"\b", constants)
        assert match, motor
        packet_torque[motor] = float(match.group(1))
    assert all(urdf <= packet for urdf, packet in zip(
        sim["joint_limits"]["physical_torque"],
        [packet_torque["DM8009"]] * 2 + [packet_torque["DM4340"]] * 2 + [packet_torque["DM4310"]] * 3,
    ))


def test_limits_are_ordered_and_derated_commands_do_not_exceed_physical_limits():
    for profile in _profiles().values():
        limits = profile["joint_limits"]
        assert all(lo < hi for lo, hi in zip(limits["position_lower"], limits["position_upper"]))
        assert all(0 < cmd <= physical for cmd, physical in zip(
            limits["command_velocity"], limits["physical_velocity"]
        ))
        torque = profile["torque"]
        assert all(0 < ff <= final <= physical for ff, final, physical in zip(
            torque["tau_ff_magnitude"], torque["final_magnitude"], limits["physical_torque"]
        ))
        assert all(value >= 0 for name in ("kp_max", "kd_max") for value in profile["gains"][name])


def test_sim_slew_lease_watchdog_and_safe_hold_are_bounded():
    sim = _profiles()["mujoco_sim_safe"]
    assert sim["status"] == "prototype_experiment_allowed"
    assert sim["hardware_enable_allowed"] is False
    assert all(value > 0 for name in ("kp_slew_per_s", "kd_slew_per_s") for value in sim["gains"][name])
    assert all(value > 0 for name in ("tau_ff_slew_per_s", "final_slew_per_s") for value in sim["torque"][name])
    assert all(0 <= hold <= maximum for hold, maximum in zip(
        sim["gains"]["safe_hold_damping"], sim["gains"]["kd_max"]
    ))
    assert all(0 <= hold <= maximum for hold, maximum in zip(
        sim["gains"]["safe_hold_stiffness"], sim["gains"]["kp_max"]
    ))
    timing = sim["timing"]
    assert 0 < timing["producer_refresh_cycles"] < timing["lease_default_cycles"] <= timing["lease_cap_cycles"]
    assert timing["controller_write_watchdog_ms"] > 0
    assert timing["state_stale_cycles"] > 0
    assert timing["bimanual_send_skew_us"] is None


def test_real_profile_is_unapproved_and_unknown_hardware_timing_remains_tbd():
    real = _profiles()["real_conservative_unapproved"]
    assert real["status"] == "unapproved"
    assert real["hardware_enable_allowed"] is False
    assert real["approval_gate"] == "manual_low_output_commissioning_required"
    assert real["gains"]["kp_slew_per_s"] is None
    assert real["gains"]["kd_slew_per_s"] is None
    assert real["gains"]["safe_hold_damping"] is None
    assert real["gains"]["safe_hold_stiffness"] is None
    assert real["torque"]["tau_ff_slew_per_s"] is None
    assert real["torque"]["final_slew_per_s"] is None
    assert all(value is None for name, value in real["timing"].items() if name != "bimanual_send_skew_gate")


def _clamp(value, magnitude):
    return max(-magnitude, min(magnitude, value))


def _slew(target, previous, rate, dt):
    return previous + _clamp(target - previous, rate * dt)


def test_saturation_and_slew_order_matches_contract():
    sim = _profiles()["mujoco_sim_safe"]
    j = 0
    ff = _clamp(100.0, sim["torque"]["tau_ff_magnitude"][j])
    ff = _slew(ff, 0.0, sim["torque"]["tau_ff_slew_per_s"][j], 0.001)
    raw = 70.0 * 10.0 + ff
    final = _clamp(raw, sim["torque"]["final_magnitude"][j])
    final = _slew(final, 0.0, sim["torque"]["final_slew_per_s"][j], 0.001)
    assert ff == 0.1
    assert final == 0.2
    assert sim["torque"]["ordering"] == (
        "tau_ff_saturation_then_slew_then_mit_equation_then_final_saturation_then_final_slew"
    )


def test_pure_torque_reduces_exactly_to_tau_ff_before_final_guards():
    q_des, q, dq_des, dq, kp, kd, tau_ff = 100.0, -100.0, 50.0, -50.0, 0.0, 0.0, 1.25
    tau_raw = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
    assert tau_raw == tau_ff


def test_moveit_real_and_bimanual_draft_architecture_is_named():
    assert CONTRACT["moveit_producer"] == "cho_follow_joint_trajectory_mit_controller"
    assert CONTRACT["real_consumer"] == "cho_system_interface_composing_openarm_can"
    assert CONTRACT["bimanual_consumer"] == "one_system_interface_two_can_sockets"
    assert CONTRACT["bimanual_fault_policy"]["faulty_arm"] == "mandatory_safe"
    assert CONTRACT["bimanual_fault_policy"]["both_arms_moveit_session"] == "abort_and_safe_both"
    assert CONTRACT["both_arms_producer"] == "one_14_joint_follow_joint_trajectory_controller"
    assert CONTRACT["moveit_map_scope"] == "both_arms_only"
    assert CONTRACT["pair_commit"] == "preflight_shadow_submit_and_ack_both"
    assert CONTRACT["fault_latch_reset"] == "lifecycle_cleanup_configure_new_session"
    assert CONTRACT["lease_cap_owner"] == "hardware_configuration"
    assert CONTRACT["status_enum"]["SAFE_TRANSITION"] == 2
    assert CONTRACT["status_enum"]["DISABLED"] == 6
