#include "cho_controller_openarm_mit/task_space_impedance_mit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace cho_controller_openarm_mit
{
namespace
{
bool finite_array(const std::vector<double> & values, std::size_t count, bool nonnegative = false)
{
  return values.size() == count && std::all_of(values.begin(), values.end(), [nonnegative](double v) {
    return std::isfinite(v) && (!nonnegative || v >= 0.0);
  });
}

// Above 2.0 the model term would exceed twice the arm's modelled weight,
// which stops being compensation and starts being a commanded lift.
bool valid_gravity_scale(const double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 2.0;
}

// Cartesian gains stay non-negative and finite. The ceilings are sanity bounds
// against a mistyped sweep value; the real force bound is max_task_wrench,
// which is not runtime-settable.
bool valid_task_gain(const std::vector<double> & values, const double ceiling)
{
  return values.size() == 6 && std::all_of(values.begin(), values.end(), [ceiling](double v) {
    return std::isfinite(v) && v >= 0.0 && v <= ceiling;
  });
}

// Per-joint factors may be negative: inverting one joint is the whole point of
// the diagnostic. The magnitude bound is the same.
bool valid_gravity_joint_scale(const std::vector<double> & values)
{
  return values.size() == 7 && std::all_of(values.begin(), values.end(), [](double v) {
    return std::isfinite(v) && v >= -2.0 && v <= 2.0;
  });
}

// Slew and blend steps must never see a zero, negative or non-finite period.
double bounded_dt(const double dt)
{
  return (std::isfinite(dt) && dt > 0.0) ? std::max(dt, 1e-4) : 1e-4;
}
}  // namespace

controller_interface::CallbackReturn TaskSpaceImpedanceMitController::on_init()
{
  if (DirectMitControllerBase::on_init() != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  auto_declare<std::vector<double>>("kp_task", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("kd_task", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("max_task_wrench", std::vector<double>(6, 0.0));
  auto_declare<std::string>("ee_frame", "openarm_hand_tcp");
  auto_declare<std::vector<double>>("startup_posture", std::vector<double>(7, 0.0));
  auto_declare<std::vector<double>>("startup_kp", std::vector<double>(7, 0.0));
  auto_declare<std::vector<double>>("startup_kd", std::vector<double>(7, 0.0));
  auto_declare<double>("startup_duration", 0.0);
  auto_declare<double>("startup_tolerance", 0.05);
  auto_declare<bool>("task_inertia_weighting", false);
  auto_declare<double>("task_inertia_regularization", 1e-4);
  auto_declare<double>("task_velocity_reference_damping", 1e-2);
  auto_declare<bool>("use_nullspace_posture", false);
  auto_declare<double>("kp_null", 10.0);
  auto_declare<double>("kd_null", 1.0);
  auto_declare<std::vector<double>>("nullspace_posture", std::vector<double>{});
  auto_declare<std::vector<double>>("joint_limit_stiffness", std::vector<double>(7, 0.0));
  auto_declare<double>("joint_limit_margin", 0.05);
  auto_declare<double>("release_duration", 1.0);
  auto_declare<std::vector<double>>("friction_level", std::vector<double>(7, 0.0));
  auto_declare<double>("friction_scale", 0.0);
  auto_declare<double>("friction_velocity_epsilon", 0.1);
  auto_declare<double>("friction_kinetic_ratio", 1.0);
  auto_declare<double>("friction_stribeck_velocity", 0.15);
  auto_declare<std::string>("friction_velocity_source", "reference");
  auto_declare<bool>("drive_side_impedance", true);
  auto_declare<std::vector<double>>("max_reference_offset", std::vector<double>(7, 0.0));
  auto_declare<double>("gravity_scale", 1.0);
  auto_declare<std::vector<double>>("gravity_joint_scale", std::vector<double>(7, 1.0));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceImpedanceMitController::on_configure(
  const rclcpp_lifecycle::State & state)
{
  if (DirectMitControllerBase::on_configure(state) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  const auto kp = get_node()->get_parameter("kp_task").as_double_array();
  const auto kd = get_node()->get_parameter("kd_task").as_double_array();
  const auto wrench_limit = get_node()->get_parameter("max_task_wrench").as_double_array();
  if (!finite_array(kp, 6, true) || !finite_array(kd, 6, true) || !finite_array(wrench_limit, 6)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Cartesian parameters require exactly 6 finite values (kp_task/kd_task nonnegative): kp_task=%zu, kd_task=%zu, max_task_wrench=%zu",
      kp.size(), kd.size(), wrench_limit.size());
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (wrench_limit[i] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "max_task_wrench[%zu]=%g must be positive", i, wrench_limit[i]);
      return CallbackReturn::ERROR;
    }
    kp_task_[i].store(kp[i], std::memory_order_release);
    kd_task_[i].store(kd[i], std::memory_order_release);
    wrench_limit_[i] = wrench_limit[i];
  }
  task_inertia_weighting_ = get_node()->get_parameter("task_inertia_weighting").as_bool();
  task_inertia_regularization_ =
    get_node()->get_parameter("task_inertia_regularization").as_double();
  if (!std::isfinite(task_inertia_regularization_) || task_inertia_regularization_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "task_inertia_regularization=%g must be finite and positive; it is what keeps "
      "Lambda bounded at a kinematic singularity", task_inertia_regularization_);
    return CallbackReturn::ERROR;
  }
  task_velocity_reference_damping_ =
    get_node()->get_parameter("task_velocity_reference_damping").as_double();
  if (!std::isfinite(task_velocity_reference_damping_) || task_velocity_reference_damping_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "task_velocity_reference_damping=%g must be finite and positive; it bounds the "
      "damped least-squares velocity reference at a singularity", task_velocity_reference_damping_);
    return CallbackReturn::ERROR;
  }
  use_nullspace_posture_ = get_node()->get_parameter("use_nullspace_posture").as_bool();
  kp_null_ = get_node()->get_parameter("kp_null").as_double();
  kd_null_ = get_node()->get_parameter("kd_null").as_double();
  if (!std::isfinite(kp_null_) || kp_null_ < 0.0 || !std::isfinite(kd_null_) || kd_null_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "kp_null=%g and kd_null=%g must be finite and nonnegative", kp_null_, kd_null_);
    return CallbackReturn::ERROR;
  }
  const auto posture = get_node()->get_parameter("nullspace_posture").as_double_array();
  nullspace_posture_explicit_ = !posture.empty();
  if (nullspace_posture_explicit_) {
    if (!finite_array(posture, 7)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "nullspace_posture must be empty or exactly 7 finite values (got %zu)", posture.size());
      return CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < 7; ++i) {
      if (posture[i] < position_lower_[i] || posture[i] > position_upper_[i]) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "nullspace_posture for joint %zu (%g) lies outside the profile window [%g, %g]",
          i + 1, posture[i], position_lower_[i], position_upper_[i]);
        return CallbackReturn::ERROR;
      }
      nullspace_posture_[i] = posture[i];
    }
  }
  const auto friction_level = get_node()->get_parameter("friction_level").as_double_array();
  const auto friction_scale = get_node()->get_parameter("friction_scale").as_double();
  friction_velocity_epsilon_ =
    get_node()->get_parameter("friction_velocity_epsilon").as_double();
  const auto friction_source =
    get_node()->get_parameter("friction_velocity_source").as_string();
  const auto kinetic_ratio = get_node()->get_parameter("friction_kinetic_ratio").as_double();
  const auto stribeck_velocity =
    get_node()->get_parameter("friction_stribeck_velocity").as_double();
  if (!std::isfinite(kinetic_ratio) || kinetic_ratio < 0.0 || kinetic_ratio > 1.0 ||
    !std::isfinite(stribeck_velocity) || stribeck_velocity <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "friction_kinetic_ratio=%g must be in [0, 1] (sliding friction cannot exceed breakaway) "
      "and friction_stribeck_velocity=%g must be finite and positive",
      kinetic_ratio, stribeck_velocity);
    return CallbackReturn::ERROR;
  }
  friction_kinetic_ratio_.store(kinetic_ratio, std::memory_order_release);
  friction_stribeck_velocity_.store(stribeck_velocity, std::memory_order_release);
  if (!finite_array(friction_level, 7, true) || !std::isfinite(friction_scale) ||
    friction_scale < 0.0 || friction_scale > 1.0 ||
    !std::isfinite(friction_velocity_epsilon_) || friction_velocity_epsilon_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "friction_level needs 7 finite nonnegative values (got %zu), friction_scale=%g must be "
      "in [0, 1] and friction_velocity_epsilon=%g must be finite and positive",
      friction_level.size(), friction_scale, friction_velocity_epsilon_);
    return CallbackReturn::ERROR;
  }
  if (friction_source == "reference") {
    friction_velocity_source_ = FrictionVelocity::REFERENCE;
  } else if (friction_source == "measured") {
    friction_velocity_source_ = FrictionVelocity::MEASURED;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
      "friction_velocity_source must be 'reference' or 'measured' (got '%s')",
      friction_source.c_str());
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    // A friction term may never exceed the torque budget on its own: it is a
    // feed-forward that acts whenever the arm moves, so a mis-identified level
    // would be a constant unexplained push rather than a transient.
    if (friction_level[i] > std::abs(torque_limit_[i])) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "friction_level for joint %zu (%g) exceeds its torque_limit (%g)",
        i + 1, friction_level[i], torque_limit_[i]);
      return CallbackReturn::ERROR;
    }
    friction_level_[i].store(friction_level[i], std::memory_order_release);
  }
  friction_scale_.store(friction_scale, std::memory_order_release);
  const bool friction_active =
    std::any_of(friction_level.begin(), friction_level.end(), [](double v) {return v > 0.0;});
  if (friction_active) {
    RCLCPP_INFO(get_node()->get_logger(),
      "friction feed-forward: level=[%g %g %g %g %g %g %g] Nm scale=%g source=%s eps=%g rad/s"
      "%s",
      friction_level[0], friction_level[1], friction_level[2], friction_level[3],
      friction_level[4], friction_level[5], friction_level[6], friction_scale,
      friction_source.c_str(), friction_velocity_epsilon_,
      friction_velocity_source_ == FrictionVelocity::MEASURED ?
      " (MEASURED closes a feedback path: keep scale below 1)" : "");
    RCLCPP_INFO(get_node()->get_logger(),
      "friction Stribeck shaping: sliding/breakaway ratio %g, decay velocity %g rad/s%s",
      kinetic_ratio, stribeck_velocity,
      kinetic_ratio >= 1.0 ?
      " (ratio 1 = flat law, no shaping: over-compensates once sliding)" : "");
  }
  drive_side_impedance_ = get_node()->get_parameter("drive_side_impedance").as_bool();
  const auto offset_limit = get_node()->get_parameter("max_reference_offset").as_double_array();
  if (!finite_array(offset_limit, 7, true)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_reference_offset requires exactly 7 finite nonnegative values (got %zu)",
      offset_limit.size());
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    // An unset (all-zero) vector derives the bound from the gains instead of
    // silently pinning q_des to the measured position, which would leave the
    // drive with nothing to push against and look exactly like a dead loop.
    // Half of the saturating offset: at kp_[i] * offset the joint would be
    // asking for its entire torque budget on tracking error alone, leaving
    // none for gravity.
    reference_offset_limit_[i] = offset_limit[i] > 0.0 ? offset_limit[i]
      : (kp_[i] > 0.0 ? 0.5 * std::abs(torque_limit_[i]) / kp_[i] : 0.0);
    if (drive_side_impedance_ && reference_offset_limit_[i] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "drive_side_impedance needs a positive reference offset bound on joint %zu: "
        "set max_reference_offset, or a nonzero kp to derive it from", i + 1);
      return CallbackReturn::ERROR;
    }
  }
  if (drive_side_impedance_) {
    RCLCPP_INFO(get_node()->get_logger(),
      "drive-side Cartesian impedance: kp=[%g %g %g %g %g %g %g] "
      "max|q_des - q|=[%g %g %g %g %g %g %g] rad "
      "(worst-case impedance torque [%g %g %g %g %g %g %g] Nm)",
      kp_[0], kp_[1], kp_[2], kp_[3], kp_[4], kp_[5], kp_[6],
      reference_offset_limit_[0], reference_offset_limit_[1], reference_offset_limit_[2],
      reference_offset_limit_[3], reference_offset_limit_[4], reference_offset_limit_[5],
      reference_offset_limit_[6],
      kp_[0] * reference_offset_limit_[0], kp_[1] * reference_offset_limit_[1],
      kp_[2] * reference_offset_limit_[2], kp_[3] * reference_offset_limit_[3],
      kp_[4] * reference_offset_limit_[4], kp_[5] * reference_offset_limit_[5],
      kp_[6] * reference_offset_limit_[6]);
  }
  const auto limit_stiffness = get_node()->get_parameter("joint_limit_stiffness").as_double_array();
  joint_limit_margin_ = get_node()->get_parameter("joint_limit_margin").as_double();
  if (!finite_array(limit_stiffness, 7, true) || !std::isfinite(joint_limit_margin_) ||
    joint_limit_margin_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "joint_limit_stiffness requires 7 finite nonnegative values (got %zu) and "
      "joint_limit_margin=%g must be finite and nonnegative",
      limit_stiffness.size(), joint_limit_margin_);
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    joint_limit_stiffness_[i] = limit_stiffness[i];
    if (joint_limit_stiffness_[i] > 0.0 &&
      position_upper_[i] - position_lower_[i] <= 2.0 * joint_limit_margin_)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "joint_limit_margin=%g leaves no free range inside joint %zu's window [%g, %g]",
        joint_limit_margin_, i + 1, position_lower_[i], position_upper_[i]);
      return CallbackReturn::ERROR;
    }
  }
  release_duration_ = get_node()->get_parameter("release_duration").as_double();
  if (!std::isfinite(release_duration_) || release_duration_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "release_duration=%g must be finite and nonnegative", release_duration_);
    return CallbackReturn::ERROR;
  }
  const double gravity_scale = get_node()->get_parameter("gravity_scale").as_double();
  if (!valid_gravity_scale(gravity_scale)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "gravity_scale=%g must be finite and within [0, 2]", gravity_scale);
    return CallbackReturn::ERROR;
  }
  gravity_scale_.store(gravity_scale, std::memory_order_release);
  const auto joint_scale = get_node()->get_parameter("gravity_joint_scale").as_double_array();
  if (!valid_gravity_joint_scale(joint_scale)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "gravity_joint_scale requires exactly 7 finite values within [-2, 2] (got %zu)",
      joint_scale.size());
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    gravity_joint_scale_[i].store(joint_scale[i], std::memory_order_release);
  }
  gravity_scale_callback_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "friction_level") {
          const auto values = parameter.as_double_array();
          bool ok = values.size() == 7;
          for (std::size_t i = 0; ok && i < 7; ++i) {
            // Same bound as configure: a friction feed-forward acts whenever
            // the arm moves, so a level beyond the torque limit would be a
            // permanent unexplained push rather than a transient.
            ok = std::isfinite(values[i]) && values[i] >= 0.0 &&
              values[i] <= std::abs(torque_limit_[i]);
          }
          if (!ok) {
            result.successful = false;
            result.reason =
              "friction_level needs 7 finite values, each within [0, torque_limit]";
            break;
          }
          for (std::size_t i = 0; i < 7; ++i) {
            friction_level_[i].store(values[i], std::memory_order_release);
          }
          continue;
        }
        if (parameter.get_name() == "friction_kinetic_ratio") {
          const double value = parameter.as_double();
          if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
            result.successful = false;
            result.reason = "friction_kinetic_ratio must be finite and within [0, 1]";
            break;
          }
          friction_kinetic_ratio_.store(value, std::memory_order_release);
          continue;
        }
        if (parameter.get_name() == "friction_stribeck_velocity") {
          const double value = parameter.as_double();
          if (!std::isfinite(value) || value <= 0.0) {
            result.successful = false;
            result.reason = "friction_stribeck_velocity must be finite and positive";
            break;
          }
          friction_stribeck_velocity_.store(value, std::memory_order_release);
          continue;
        }
        if (parameter.get_name() == "friction_scale") {
          const double value = parameter.as_double();
          // Capped at 1 rather than validated loosely: above 1 the term
          // cancels more than the friction present, which is negative damping
          // and makes the joint accelerate on its own. The commissioning ramp
          // walks this up from 0 and re-checks a stationary hold at each step.
          if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
            result.successful = false;
            result.reason = "friction_scale must be finite and within [0, 1]";
            break;
          }
          friction_scale_.store(value, std::memory_order_release);
          continue;
        }
        if (parameter.get_name() == "gravity_scale") {
          const double value = parameter.as_double();
          if (!valid_gravity_scale(value)) {
            result.successful = false;
            result.reason = "gravity_scale must be finite and within [0, 2]";
            break;
          }
          // Only the store is on the RT path; the validation above runs here.
          gravity_scale_.store(value, std::memory_order_release);
          RCLCPP_INFO(get_node()->get_logger(), "gravity_scale set to %g", value);
        } else if (parameter.get_name() == "kp_task" || parameter.get_name() == "kd_task") {
          const bool is_kp = parameter.get_name() == "kp_task";
          const auto values = parameter.as_double_array();
          if (!valid_task_gain(values, is_kp ? 500.0 : 200.0)) {
            result.successful = false;
            result.reason = "task gain needs 6 finite values within the sanity bound";
            break;
          }
          auto & target = is_kp ? kp_task_ : kd_task_;
          for (std::size_t i = 0; i < 6; ++i) {
            target[i].store(values[i], std::memory_order_release);
          }
          RCLCPP_INFO(get_node()->get_logger(), "%s set to [%g %g %g %g %g %g]",
            parameter.get_name().c_str(), values[0], values[1], values[2],
            values[3], values[4], values[5]);
        } else if (parameter.get_name() == "gravity_joint_scale") {
          const auto values = parameter.as_double_array();
          if (!valid_gravity_joint_scale(values)) {
            result.successful = false;
            result.reason = "gravity_joint_scale needs 7 finite values within [-2, 2]";
            break;
          }
          for (std::size_t i = 0; i < 7; ++i) {
            gravity_joint_scale_[i].store(values[i], std::memory_order_release);
          }
          RCLCPP_INFO(get_node()->get_logger(),
            "gravity_joint_scale set to [%g %g %g %g %g %g %g]",
            values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
        }
      }
      return result;
    });
  RCLCPP_INFO(get_node()->get_logger(),
    "Cartesian law: %s (kp_task/kd_task are %s); velocity reference dq_des = J^+ v_des (damping %g); "
    "null-space posture %s (kp_null=%g, kd_null=%g, reference %s); joint-limit spring margin %g rad; "
    "release blend %g s; gravity_scale %g; only the model feed-forward is rate-limited",
    task_inertia_weighting_ ? "Lambda-weighted operational space" : "plain Jacobian transpose",
    task_inertia_weighting_ ? "acceleration gains 1/s^2 and 1/s" : "N/m and N*s/m (runtime-settable)",
    task_velocity_reference_damping_,
    use_nullspace_posture_ ? "enabled" : "disabled", kp_null_, kd_null_,
    nullspace_posture_explicit_ ? "explicit" : "latched at Cartesian entry",
    joint_limit_margin_, release_duration_, gravity_scale_.load());
  ee_frame_ = get_node()->get_parameter("ee_frame").as_string();
  const auto startup_kp = get_node()->get_parameter("startup_kp").as_double_array();
  const auto startup_kd = get_node()->get_parameter("startup_kd").as_double_array();
  startup_duration_ = get_node()->get_parameter("startup_duration").as_double();
  startup_tolerance_ = get_node()->get_parameter("startup_tolerance").as_double();
  if (ee_frame_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter 'ee_frame' must not be empty");
    return CallbackReturn::ERROR;
  }
  // This controller has exactly one joint-space initialization path:
  // return_to_zero. Opting out means holding the measured Cartesian pose in
  // direct-torque mode, not running a second YAML startup posture.
  if (return_to_zero_) {
    startup_duration_ = return_to_zero_duration_;
    startup_tolerance_ = return_to_zero_tolerance_;
    if (!finite_array(startup_kp, 7, true) || !finite_array(startup_kd, 7, true)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "return_to_zero requires exactly 7 finite/nonnegative startup_kp and startup_kd values: sizes=%zu/%zu",
        startup_kp.size(), startup_kd.size());
      return CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < 7; ++i) {
      const double target = return_to_zero_target_[i];
      if (target <= position_lower_[i] + 1e-4 || target >= position_upper_[i] - 1e-4) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "return_to_zero target for joint %zu (%g) must lie strictly inside safety limits [%g, %g]",
          i + 1, target, position_lower_[i], position_upper_[i]);
        return CallbackReturn::ERROR;
      }
      // Homing ceilings, not task ceilings: startup_kp/startup_kd drive the
      // return-to-zero move, which is a position servo against gravity rather
      // than an impedance the drive has to damp.
      if (startup_kp[i] > profile_rtz_kp_max_[i] || startup_kd[i] > profile_rtz_kd_max_[i]) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "return_to_zero gains for joint %zu exceed selected safety limits: kp=%g (max %g), kd=%g (max %g)",
          i + 1, startup_kp[i], profile_rtz_kp_max_[i], startup_kd[i], profile_rtz_kd_max_[i]);
        return CallbackReturn::ERROR;
      }
      startup_posture_[i] = target;
      startup_kp_[i] = startup_kp[i];
      startup_kd_[i] = startup_kd[i];
      // The real adapter rejects any tuple above the per-joint TASK ceiling,
      // whatever the return_to_zero ceiling says. The ramp clamps there
      // (ramped_return_to_zero_gains), so this is a weaker homing servo on the
      // named joint rather than the SAFE drop it used to be.
      if (startup_kp[i] > profile_kp_max_[i] || startup_kd[i] > profile_kd_max_[i]) {
        RCLCPP_WARN(get_node()->get_logger(),
          "return_to_zero gains for joint %zu (kp=%g, kd=%g) exceed the hardware per-joint "
          "ceiling (kp_max=%g, kd_max=%g); the ramp is clamped there",
          i + 1, startup_kp[i], startup_kd[i], profile_kp_max_[i], profile_kd_max_[i]);
      }
    }
  }
  if (!configure_task_model()) return CallbackReturn::ERROR;
  const auto action_name = std::string("/controller_action_server/") + get_node()->get_name();
  task_server_ = rclcpp_action::create_server<Action>(get_node(), action_name,
    std::bind(&TaskSpaceImpedanceMitController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskSpaceImpedanceMitController::cancel_callback, this, std::placeholders::_1),
    std::bind(&TaskSpaceImpedanceMitController::accepted_callback, this, std::placeholders::_1));
  task_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(5),
    std::bind(&TaskSpaceImpedanceMitController::non_rt_tick, this));
  task_diagnostics_service_ = get_node()->create_service<std_srvs::srv::Trigger>("~/task_diagnostics",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      std::ostringstream out; out << "last_pose_error=[";
      for (std::size_t i = 0; i < 6; ++i) {if (i) out << ','; out << task_last_error_[i].load();}
      out << "] peak_wrench=[";
      for (std::size_t i = 0; i < 6; ++i) {if (i) out << ','; out << task_peak_wrench_[i].load();}
      out << "] peak_tau_ff=[";
      for (std::size_t i = 0; i < 7; ++i) {if (i) out << ','; out << task_peak_tau_ff_[i].load();}
      out << "] q_ref=[";
      for (std::size_t i = 0; i < 7; ++i) {if (i) out << ','; out << task_q_reference_observed_[i].load();}
      out << "]"; response->success = true; response->message = out.str();
    });
  return CallbackReturn::SUCCESS;
}

bool TaskSpaceImpedanceMitController::configure_task_model()
{
  if (configure_action_mujoco_dynamics() != CallbackReturn::SUCCESS) return false;
  if (!action_model_ || !action_model_data_) return false;
  full_jacobian_ = Eigen::MatrixXd::Zero(6, action_model_->nv);
  try {
    ee_frame_id_ = action_model_->getFrameId(ee_frame_);
    if (ee_frame_id_ >= action_model_->frames.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "EE frame '%s' is not in robot_description", ee_frame_.c_str());
      return false;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "EE frame rejected: %s", error.what());
    return false;
  }
  return true;
}

controller_interface::CallbackReturn TaskSpaceImpedanceMitController::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (DirectMitControllerBase::on_activate(state) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  task_ready_.store(false, std::memory_order_release);
  task_id_ = 0; task_public_id_.store(0); task_percent_.store(0.0);
  task_compute_failed_ = false; task_capacity_rejected_ = false;
  for (auto & value : task_last_error_) value.store(0.0);
  for (auto & value : task_peak_wrench_) value.store(0.0);
  for (auto & value : task_peak_tau_ff_) value.store(0.0);
  task_last_started_id_ = task_goal_buffer_.readFromNonRT()->id;
  task_q_ref_ = measured();
  for (std::size_t i = 0; i < 7; ++i) task_q_reference_observed_[i].store(task_q_ref_[i]);
  task_last_model_feedforward_.fill(0.0);
  task_dynamics_valid_ = false;
  startup_start_ = task_q_ref_; startup_elapsed_ = 0.0; startup_active_ = false;
  idle_pose_valid_ = false;
  idle_release_active_ = false;
  idle_release_elapsed_ = 0.0;
  return_to_zero_handoff_active_ = false;
  return_to_zero_handoff_elapsed_ = 0.0;
  return CallbackReturn::SUCCESS;
}

bool TaskSpaceImpedanceMitController::finite_pose(const Action::Goal & goal)
{
  const auto & p = goal.target_pose.position;
  const auto & q = goal.target_pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(goal.duration) && goal.duration > 0.0F && std::isfinite(p.x) && std::isfinite(p.y) &&
    std::isfinite(p.z) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) && norm > 1e-6;
}

rclcpp_action::GoalResponse TaskSpaceImpedanceMitController::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  if (!goal || !task_ready_.load(std::memory_order_acquire) || !finite_pose(*goal)) return rclcpp_action::GoalResponse::REJECT;
  // Cartesian goals have no independent displacement, orientation, workspace,
  // or path-speed admission cap. The direct torque path remains bounded by
  // max_task_wrench, the final motor torque tuple, and the hardware contract.
  if (goal->duration < 0.25F) return rclcpp_action::GoalResponse::REJECT;
  std::lock_guard<std::mutex> lock(task_handles_mutex_);
  return task_handles_.size() < 2U ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse TaskSpaceImpedanceMitController::cancel_callback(const std::shared_ptr<GoalHandle> & handle)
{
  std::lock_guard<std::mutex> lock(task_handles_mutex_);
  for (const auto & entry : task_handles_) if (entry.second == handle) {
    task_cancel_id_.store(entry.first, std::memory_order_release); return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void TaskSpaceImpedanceMitController::accepted_callback(const std::shared_ptr<GoalHandle> & handle)
{
  const auto & in = *handle->get_goal();
  Goal staged; staged.id = task_next_id_.fetch_add(1, std::memory_order_relaxed); staged.duration = in.duration; staged.relative = in.relative;
  staged.translation = {in.target_pose.position.x, in.target_pose.position.y, in.target_pose.position.z};
  staged.rotation = Eigen::Quaterniond(in.target_pose.orientation.w, in.target_pose.orientation.x,
    in.target_pose.orientation.y, in.target_pose.orientation.z).normalized();
  {std::lock_guard<std::mutex> lock(task_handles_mutex_); task_handles_.emplace(staged.id, handle);}
  task_goal_buffer_.writeFromNonRT(staged);
}

bool TaskSpaceImpedanceMitController::task_pose_and_jacobian(
  const std::array<double, 7> & q, pinocchio::SE3 & pose, Jacobian & jacobian)
{
  if (!action_model_ || !action_model_data_) return false;
  for (std::size_t i = 0; i < 7; ++i) action_model_q_[action_q_indices_[i]] = q[i];
  try {
    pinocchio::forwardKinematics(*action_model_, *action_model_data_, action_model_q_);
    pinocchio::updateFramePlacements(*action_model_, *action_model_data_);
    pose = action_model_data_->oMf[ee_frame_id_];
    full_jacobian_.setZero();
    pinocchio::computeFrameJacobian(*action_model_, *action_model_data_, action_model_q_, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, full_jacobian_);
    for (std::size_t i = 0; i < 7; ++i) jacobian.col(i) = full_jacobian_.col(action_v_indices_[i]);
    return pose.translation().allFinite() && jacobian.allFinite();
  } catch (...) {return false;}
}

bool TaskSpaceImpedanceMitController::model_nle(const std::array<double, 7> & q,
  const std::array<double, 7> & dq, std::array<double, 7> & nle)
{
  if (!action_model_ || !action_model_data_) return false;
  for (std::size_t i = 0; i < 7; ++i) {action_model_q_[action_q_indices_[i]] = q[i]; action_model_v_[action_v_indices_[i]] = dq[i];}
  try {
    pinocchio::nonLinearEffects(*action_model_, *action_model_data_, action_model_q_, action_model_v_);
    for (std::size_t i = 0; i < 7; ++i) {nle[i] = action_model_data_->nle[action_v_indices_[i]]; if (!std::isfinite(nle[i])) return false;}
    return true;
  } catch (...) {return false;}
}

void TaskSpaceImpedanceMitController::finish(std::uint64_t id, Terminal terminal)
{
  if (id && !task_terminal_queue_.push(TerminalEvent{id, terminal})) {state_ = State::FAULT; stop_failed_.store(true);}
}
void TaskSpaceImpedanceMitController::abort_active()
{
  if (task_id_) {finish(task_id_, Terminal::ABORTED); task_id_ = 0; task_public_id_.store(0); task_percent_.store(0.0);}
}

bool TaskSpaceImpedanceMitController::latch_idle_pose()
{
  Jacobian jacobian;
  const auto q = measured();
  if (!task_pose_and_jacobian(q, idle_pose_, jacobian)) return false;
  idle_pose_valid_ = true;
  idle_release_active_ = false;
  // Without an explicit posture, hold the null space where Cartesian control
  // began rather than pulling it toward an unrelated configuration.
  if (!nullspace_posture_explicit_) nullspace_posture_ = q;
  return true;
}

bool TaskSpaceImpedanceMitController::begin_idle_release()
{
  pinocchio::SE3 measured_pose;
  Jacobian jacobian;
  if (!task_pose_and_jacobian(measured(), measured_pose, jacobian)) return false;
  begin_idle_release_from(measured_pose);
  return true;
}

void TaskSpaceImpedanceMitController::begin_idle_release_from(const pinocchio::SE3 & measured_pose)
{
  if (!idle_pose_valid_ || release_duration_ <= 0.0) {
    idle_pose_ = measured_pose;
    idle_pose_valid_ = true;
    idle_release_active_ = false;
    return;
  }
  idle_release_start_ = idle_pose_;
  idle_release_goal_ = measured_pose;
  idle_release_elapsed_ = 0.0;
  idle_release_active_ = true;
}

void TaskSpaceImpedanceMitController::sample_pose_trajectory(
  const pinocchio::SE3 & start, const pinocchio::SE3 & goal, const double u, const double duration,
  pinocchio::SE3 & desired, Vector6 & twist)
{
  // Cubic smoothstep: zero endpoint velocity, matching the dq contract of an
  // MIT packet when the twist is mapped to dq_des.
  const double s = u * u * (3.0 - 2.0 * u);
  const double ds = duration > 0.0 ? 6.0 * u * (1.0 - u) / duration : 0.0;
  const Eigen::Quaterniond qa(start.rotation()), qb(goal.rotation());
  desired = pinocchio::SE3(
    qa.slerp(s, qb).toRotationMatrix(),
    (1.0 - s) * start.translation() + s * goal.translation());
  twist.head<3>() = ds * (goal.translation() - start.translation());
  twist.tail<3>() = start.rotation() *
    (ds * pinocchio::log3(start.rotation().transpose() * goal.rotation()));
}

bool TaskSpaceImpedanceMitController::task_dynamics(const Jacobian & jacobian)
{
  task_dynamics_valid_ = false;
  if (!action_model_ || !action_model_data_) return false;
  try {
    // action_model_q_ already holds the measured configuration: the caller runs
    // task_pose_and_jacobian() first, and crba writes only data.M.
    pinocchio::crba(*action_model_, *action_model_data_, action_model_q_);
    // Pinocchio fills the upper triangle only.
    action_model_data_->M.triangularView<Eigen::StrictlyLower>() =
      action_model_data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    // The description carries the two prismatic finger joints, which this
    // controller never commands. Holding them fixed makes the seven-axis block
    // the correct constrained mass matrix rather than an approximation.
    for (std::size_t r = 0; r < 7; ++r) {
      for (std::size_t c = 0; c < 7; ++c) {
        task_arm_mass_(r, c) = action_model_data_->M(action_v_indices_[r], action_v_indices_[c]);
      }
    }
    if (!task_arm_mass_.allFinite()) return false;
    const Eigen::LLT<Eigen::Matrix<double, 7, 7>> mass_llt(task_arm_mass_);
    if (mass_llt.info() != Eigen::Success) return false;
    task_mass_inverse_jt_ = mass_llt.solve(jacobian.transpose());
    Eigen::Matrix<double, 6, 6> lambda_inverse = jacobian * task_mass_inverse_jt_;
    // Near a singularity Lambda_inverse loses rank and Lambda would blow up.
    // The regularization bounds it, and max_task_wrench bounds the result.
    lambda_inverse.diagonal().array() += task_inertia_regularization_;
    const Eigen::LLT<Eigen::Matrix<double, 6, 6>> lambda_llt(lambda_inverse);
    if (lambda_llt.info() != Eigen::Success) return false;
    task_lambda_ = lambda_llt.solve(Eigen::Matrix<double, 6, 6>::Identity());
    task_dynamics_valid_ = task_lambda_.allFinite() && task_mass_inverse_jt_.allFinite();
    return task_dynamics_valid_;
  } catch (...) {
    return false;
  }
}

bool TaskSpaceImpedanceMitController::nullspace_posture_torque(
  const Jacobian & jacobian, const std::array<double, 7> & q,
  const std::array<double, 7> & dq, Vector7 & torque) const
{
  if (!task_dynamics_valid_) return false;
  // Jbar^T = Lambda J M^-1 = Lambda (M^-1 J^T)^T because M is symmetric.
  const Eigen::Matrix<double, 6, 7> jbar_t = task_lambda_ * task_mass_inverse_jt_.transpose();
  const Eigen::Matrix<double, 7, 7> projector =
    Eigen::Matrix<double, 7, 7>::Identity() - jacobian.transpose() * jbar_t;
  Vector7 acceleration;
  for (std::size_t i = 0; i < 7; ++i) {
    acceleration[i] = kp_null_ * (nullspace_posture_[i] - q[i]) - kd_null_ * dq[i];
  }
  torque = projector * (task_arm_mass_ * acceleration);
  return torque.allFinite();
}

namespace
{
// Saturate a vector by scaling it, not by clipping each component.
//
// Per-component clipping changes the DIRECTION, and the more unequal the
// components the worse: measured on hardware, a Lambda-weighted law asked for
// [-19.9, 6.2, 118.8] N, the per-axis clamp at 10 N returned [-10, 6.2, 10],
// and the arm drove 100 mm the wrong way. Scaling by the single tightest ratio
// keeps the direction exactly and only gives up magnitude.
//
// Translation and rotation are scaled as separate 3-vectors: they carry
// different units, so one common factor would let a saturating torque throttle
// an unsaturated force for no physical reason.
template<typename Vector>
void scale_into_limits(Vector & v, const double * limit, std::size_t offset)
{
  double scale = 1.0;
  for (std::size_t k = 0; k < 3; ++k) {
    const std::size_t i = offset + k;
    const double magnitude = std::abs(v[i]);
    if (limit[i] <= 0.0) {
      scale = 0.0;
      break;
    }
    if (magnitude > limit[i]) scale = std::min(scale, limit[i] / magnitude);
  }
  if (scale < 1.0) {
    for (std::size_t k = 0; k < 3; ++k) v[offset + k] *= scale;
  }
}
}  // namespace

void TaskSpaceImpedanceMitController::friction_torque(
  const std::array<double, 7> & dq, const std::array<double, 7> & dq_des,
  Vector7 & torque) const
{
  torque.setZero();
  const double scale = friction_scale_.load(std::memory_order_acquire);
  if (scale <= 0.0) return;
  const double ratio = friction_kinetic_ratio_.load(std::memory_order_acquire);
  const double stribeck = friction_stribeck_velocity_.load(std::memory_order_acquire);
  const auto & velocity =
    friction_velocity_source_ == FrictionVelocity::MEASURED ? dq : dq_des;
  for (std::size_t i = 0; i < 7; ++i) {
    const double breakaway = friction_level_[i].load(std::memory_order_acquire);
    if (breakaway <= 0.0) continue;
    // tanh saturates to +/-1 a few epsilon out and passes smoothly through
    // zero, so a joint at rest gets no push and a joint that is barely moving
    // gets a proportionally small one. The scale stays below 1 by parameter
    // validation, so this can only ever cancel part of the real friction:
    // cancelling more than all of it would make the joint self-accelerating.
    // Stribeck bracket: breakaway level near zero, sliding level once moving.
    const double v = velocity[i];
    const double decay = v / stribeck;
    const double level = breakaway * (ratio + (1.0 - ratio) * std::exp(-decay * decay));
    torque[i] = scale * level * std::tanh(v / friction_velocity_epsilon_);
  }
}

void TaskSpaceImpedanceMitController::joint_limit_torque(
  const std::array<double, 7> & q, Vector7 & torque) const
{
  for (std::size_t i = 0; i < 7; ++i) {
    torque[i] = 0.0;
    if (joint_limit_stiffness_[i] <= 0.0) continue;
    const double lower_band = position_lower_[i] + joint_limit_margin_;
    const double upper_band = position_upper_[i] - joint_limit_margin_;
    // Continuous at the band edge: zero force there, then a linear spring.
    if (q[i] < lower_band) {
      torque[i] = joint_limit_stiffness_[i] * (lower_band - q[i]);
    } else if (q[i] > upper_band) {
      torque[i] = -joint_limit_stiffness_[i] * (q[i] - upper_band);
    }
  }
}

void TaskSpaceImpedanceMitController::joint_velocity_reference(
  const Jacobian & jacobian, const Vector6 & twist, std::array<double, 7> & dq_des) const
{
  dq_des.fill(0.0);
  if (twist.squaredNorm() == 0.0) return;
  Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
  jjt.diagonal().array() += task_velocity_reference_damping_;
  const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(jjt);
  // A failed solve degrades to the pure-damping behaviour rather than aborting.
  if (llt.info() != Eigen::Success) return;
  const Vector7 dq = jacobian.transpose() * llt.solve(twist);
  if (!dq.allFinite()) return;
  for (std::size_t i = 0; i < 7; ++i) {
    // The consumer rejects the whole arm tuple when one dq_des leaves the
    // profile command-velocity window, so this clamp is mandatory.
    dq_des[i] = std::clamp(dq[i], -command_velocity_[i], command_velocity_[i]);
  }
}

void TaskSpaceImpedanceMitController::joint_reference_offset(
  const Jacobian & jacobian, const Vector6 & pose_error, Vector7 & offset) const
{
  offset.setZero();
  if (pose_error.squaredNorm() == 0.0) return;
  Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
  jjt.diagonal().array() += task_velocity_reference_damping_;
  const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(jjt);
  // Degrade to "no offset" rather than aborting: q_des then equals measured q
  // and the arm holds on gravity alone, which is the same failure the velocity
  // reference already chooses near a singularity.
  if (llt.info() != Eigen::Success) return;
  const Vector7 dq = jacobian.transpose() * llt.solve(pose_error);
  if (!dq.allFinite()) return;
  // One scale for all seven, for the same reason the wrench is scaled rather
  // than clipped: this offset IS a Cartesian direction expressed in joint
  // coordinates, and clipping one joint against its own bound tilts it.
  double scale = 1.0;
  for (std::size_t i = 0; i < 7; ++i) {
    if (reference_offset_limit_[i] <= 0.0) {
      scale = 0.0;
      break;
    }
    const double magnitude = std::abs(dq[i]);
    if (magnitude > reference_offset_limit_[i]) {
      scale = std::min(scale, reference_offset_limit_[i] / magnitude);
    }
  }
  offset = scale * dq;
}

void TaskSpaceImpedanceMitController::clamp_command_positions(ArmCommand & command) const
{
  // The Cartesian modes emit measured q as q_des with zero stiffness, so this
  // cannot change the applied torque.  It exists because the consumer rejects
  // the whole seven-axis tuple when one q_des leaves the profile window, and
  // joint 4's window starts exactly at its mechanical zero: a measured
  // position a few counts below 0 would otherwise drop every arm command to
  // the hardware-owned SAFE hold.  In the joint modes the clamp is the
  // ordinary meaning of a position limit - it pulls the reference back inside.
  for (std::size_t i = 0; i < 7; ++i) {
    command.joints[i].position = std::clamp(
      command.joints[i].position, position_lower_[i], position_upper_[i]);
  }
}

double TaskSpaceImpedanceMitController::slew_model_feedforward(
  const std::size_t joint, const double desired, const double dt)
{
  // The controller torque_limit may be narrower than the profile's tau_ff
  // magnitude. Bounding the tracker by both keeps it equal to what the final
  // clamp can emit, so a saturated model term cannot wind up past it.
  const double limit = std::min(feedforward_limit_[joint], std::abs(torque_limit_[joint]));
  const double bounded = std::clamp(desired, -limit, limit);
  const double max_step = feedforward_slew_[joint] * bounded_dt(dt);
  const double next = std::clamp(
    bounded,
    task_last_model_feedforward_[joint] - max_step,
    task_last_model_feedforward_[joint] + max_step);
  task_last_model_feedforward_[joint] = next;
  return next;
}

bool TaskSpaceImpedanceMitController::write_cartesian_torque_target(
  const pinocchio::SE3 & desired,
  const Vector6 & desired_twist,
  const double dt,
  DirectMitTarget & target,
  Vector6 * pose_error_out,
  pinocchio::SE3 * measured_pose_out)
{
  target = {};
  const auto q = measured();
  std::array<double, 7> dq{}, nle{};
  for (std::size_t i = 0; i < 7; ++i) dq[i] = state_interfaces_[2 * i + 1].get_value();
  pinocchio::SE3 measured_pose;
  Jacobian jacobian;
  if (!task_pose_and_jacobian(q, measured_pose, jacobian) || !model_nle(q, dq, nle)) return false;

  Vector6 pose_error;
  pose_error.head<3>() = desired.translation() - measured_pose.translation();
  pose_error.tail<3>() = measured_pose.rotation() *
    pinocchio::log3(measured_pose.rotation().transpose() * desired.rotation());
  Vector7 joint_velocity;
  for (std::size_t i = 0; i < 7; ++i) joint_velocity[i] = dq[i];
  const Vector6 ee_velocity = jacobian * joint_velocity;
  for (std::size_t i = 0; i < 6; ++i) {
    task_last_error_[i].store(pose_error[i], std::memory_order_release);
  }
  task_dynamics_valid_ = false;
  if ((task_inertia_weighting_ || use_nullspace_posture_) && !task_dynamics(jacobian)) return false;

  Vector7 feedback;
  if (drive_side_impedance_) {
    // The drive owns the impedance. q_des carries the Cartesian error as a
    // joint offset and the fixed kp/kd close the loop inside the current loop,
    // so neither the 200 Hz controller period nor CAN transport sits inside
    // it. tau_ff is left to the terms the drive cannot know about: the model
    // term, the null-space posture and the joint-limit spring.
    //
    // Only diag(J^T Kx J) is representable as a per-joint scalar. The
    // off-diagonal coupling is dropped rather than added to tau_ff, which
    // would put a fraction of the stiffness back on the delayed path and
    // reintroduce the phase lag in a form that is harder to reason about than
    // simply not having it.
    Vector7 offset;
    joint_reference_offset(jacobian, pose_error, offset);
    Vector7 impedance_torque;
    for (std::size_t i = 0; i < 7; ++i) {
      target.position[i] = q[i] + offset[i];
      impedance_torque[i] = kp_[i] * offset[i];
    }
    // Diagnostic only. peak_wrench has to keep meaning "Cartesian force the
    // arm is producing" across both laws or the tuning tool silently compares
    // unlike quantities between runs.
    Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
    jjt.diagonal().array() += task_velocity_reference_damping_;
    const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(jjt);
    if (llt.info() == Eigen::Success) {
      const Vector6 equivalent = llt.solve(jacobian * impedance_torque);
      if (equivalent.allFinite()) {
        for (std::size_t i = 0; i < 6; ++i) {
          task_peak_wrench_[i].store(
            std::max(
              task_peak_wrench_[i].load(std::memory_order_relaxed), std::abs(equivalent[i])),
            std::memory_order_release);
        }
      }
    }
    feedback.setZero();
  } else {
    // Historical law: the same world-aligned task-space law as the Franka
    // controller, synthesised into tau_ff. TRACKING_DAMPED_TORQUE maps it to
    // measured q_des, zero MIT stiffness, the configured MIT damping and
    // dq_des = J^+ v_des: J^T*Dx*J damps only the range space of the Jacobian,
    // so the one-dimensional null space of this 7-DoF arm under a 6-DoF task
    // would otherwise carry no damping at all.
    Vector6 law;
    for (std::size_t i = 0; i < 6; ++i) {
      law[i] = kp_task_[i].load(std::memory_order_acquire) * pose_error[i] +
        kd_task_[i].load(std::memory_order_acquire) * (desired_twist[i] - ee_velocity[i]);
    }
    if (task_inertia_weighting_) {
      // Lambda turns the PD law from a force into a task-space acceleration
      // request, so a direction that can only be served by the elbow (high
      // apparent inertia) is commanded proportionally harder than one the
      // wrist can serve. The clamp below keeps max_task_wrench a hard bound.
      law = task_lambda_ * law;
    }
    Vector6 wrench = law;
    scale_into_limits(wrench, wrench_limit_.data(), 0);
    scale_into_limits(wrench, wrench_limit_.data(), 3);
    for (std::size_t i = 0; i < 6; ++i) {
      task_peak_wrench_[i].store(
        std::max(task_peak_wrench_[i].load(std::memory_order_relaxed), std::abs(wrench[i])),
        std::memory_order_release);
    }
    feedback = jacobian.transpose() * wrench;
  }
  if (use_nullspace_posture_) {
    Vector7 posture_torque;
    if (!nullspace_posture_torque(jacobian, q, dq, posture_torque)) return false;
    feedback += posture_torque;
  }
  Vector7 limit_torque;
  joint_limit_torque(q, limit_torque);
  feedback += limit_torque;
  joint_velocity_reference(jacobian, desired_twist, target.velocity);
  // After the velocity reference, because the REFERENCE source reads it. Not
  // rate-limited, like the other feedback terms: it is bounded in magnitude by
  // friction_level and a rate limiter inside the loop would add phase lag.
  Vector7 friction;
  friction_torque(dq, target.velocity, friction);
  feedback += friction;

  for (std::size_t i = 0; i < 7; ++i) {
    // Only the model term is rate-limited. The feedback terms are bounded in
    // magnitude by max_task_wrench above and by the profile/controller torque
    // limits below; rate-limiting them would put a limit-cycle source inside
    // the closed loop.
    const double model = slew_model_feedforward(i, gravity_scale_.load(std::memory_order_acquire) * gravity_joint_scale_[i].load(std::memory_order_acquire) * nle[i], dt);
    const double bounded_feedback = std::clamp(
      feedback[i], -feedforward_limit_[i], feedforward_limit_[i]);
    target.feedforward[i] = std::clamp(
      model + bounded_feedback, -feedforward_limit_[i], feedforward_limit_[i]);
    task_peak_tau_ff_[i].store(
      std::max(
        task_peak_tau_ff_[i].load(std::memory_order_relaxed),
        std::abs(target.feedforward[i])),
      std::memory_order_release);
  }
  if (pose_error_out) *pose_error_out = pose_error;
  if (measured_pose_out) *measured_pose_out = measured_pose;
  return true;
}

bool TaskSpaceImpedanceMitController::write_task_target(
  double control_time, double dt, DirectMitTarget & target)
{
  task_compute_failed_ = false; task_capacity_rejected_ = false;
  const Goal incoming = *task_goal_buffer_.readFromRT();
  const auto canceled = task_cancel_id_.exchange(0, std::memory_order_acq_rel);
  const auto cancel_active = [this]() {
    finish(task_id_, Terminal::CANCELED); task_id_ = 0; task_public_id_.store(0); task_percent_.store(0.0);
    // A canceled trajectory must neither keep its loaded reference nor hand
    // the arm to the hardware SAFE hold, which has no gravity model of its
    // own. Release the reference toward the measured pose and stay ready.
    if (!begin_idle_release()) {task_compute_failed_ = true; return false;}
    return true;
  };
  if (canceled && canceled == task_id_ && !cancel_active()) return false;
  if (incoming.id && incoming.id != task_last_started_id_) {
    abort_active(); task_id_ = incoming.id; task_last_started_id_ = incoming.id; task_public_id_.store(task_id_);
    task_start_time_ = control_time;
    // Continue from the exact Cartesian reference emitted on the preceding
    // idle/action cycle, including a partially released one. Rebasing to
    // measured pose would drop a non-zero Kp wrench at u=0 and introduce a
    // torque step.
    if (idle_pose_valid_) {
      task_start_pose_ = idle_pose_;
      idle_release_active_ = false;
    } else {
      Jacobian start_jacobian;
      if (!task_pose_and_jacobian(measured(), task_start_pose_, start_jacobian)) {
        task_compute_failed_ = true;
        return false;
      }
    }
    task_goal_pose_ = incoming.relative ? task_start_pose_ * pinocchio::SE3(incoming.rotation.toRotationMatrix(), incoming.translation) :
      pinocchio::SE3(incoming.rotation.toRotationMatrix(), incoming.translation);
    if (!task_goal_pose_.translation().allFinite() || !task_goal_pose_.rotation().allFinite()) {
      finish(task_id_, Terminal::ABORTED); task_id_ = 0; task_public_id_.store(0);
      task_capacity_rejected_ = true; return false;
    }
  }
  // A replacement canceled before its first cycle releases from the reference
  // it would have started from.
  if (canceled && canceled == task_id_ && !cancel_active()) return false;
  if (!task_id_) {
    if (!idle_pose_valid_) return false;
    Vector6 twist = Vector6::Zero();
    if (idle_release_active_) {
      idle_release_elapsed_ += dt;
      const double u = std::clamp(idle_release_elapsed_ / release_duration_, 0.0, 1.0);
      sample_pose_trajectory(
        idle_release_start_, idle_release_goal_, u, release_duration_, idle_pose_, twist);
      if (u >= 1.0) {
        idle_pose_ = idle_release_goal_;
        twist.setZero();
        idle_release_active_ = false;
      }
    }
    if (!write_cartesian_torque_target(idle_pose_, twist, dt, target)) {
      task_compute_failed_ = true;
      return false;
    }
    return true;
  }
  const double elapsed = std::max(0.0, control_time - task_start_time_);
  const double u = std::clamp(elapsed / incoming.duration, 0.0, 1.0);
  pinocchio::SE3 desired;
  Vector6 v_des;
  sample_pose_trajectory(task_start_pose_, task_goal_pose_, u, incoming.duration, desired, v_des);
  Vector6 pose_error;
  pinocchio::SE3 measured_pose;
  if (!write_cartesian_torque_target(
      desired, v_des, dt, target, &pose_error, &measured_pose)) {
    task_compute_failed_ = true;
    return false;
  }
  // Publish the exact commanded reference for a possible preempting goal.
  // The replacement trajectory therefore begins without resetting the Kp
  // error or reference velocity to a measured-pose trajectory.
  idle_pose_ = desired;
  idle_pose_valid_ = true;
  task_percent_.store(100.0 * u, std::memory_order_release);
  if (u >= 1.0) {
    const double pos_error = pose_error.head<3>().norm(), rot_error = pose_error.tail<3>().norm();
    if (pos_error < 0.02 && rot_error < 0.10) {
      // Keep the terminal commanded reference so residual Kp wrench remains
      // continuous while the Cartesian idle controller finishes settling.
      idle_pose_ = task_goal_pose_;
      idle_pose_valid_ = true;
      finish(task_id_, Terminal::SUCCEEDED); task_id_ = 0; task_public_id_.store(0);
    } else if (elapsed > incoming.duration + 2.0) {
      // An unreachable goal must not remain loaded indefinitely. Release the
      // reference toward the measured pose over release_duration; the blend,
      // not a torque rate limiter, keeps the saturated wrench from dropping in
      // one control period.
      finish(task_id_, Terminal::ABORTED); task_id_ = 0; task_public_id_.store(0);
      begin_idle_release_from(measured_pose);
    }
  }
  return true;
}

controller_interface::return_type TaskSpaceImpedanceMitController::update(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (state_ == State::INACTIVE || state_ == State::SAFE_STOPPED) return controller_interface::return_type::OK;
  const double dt = period.seconds(); if (std::isfinite(dt) && dt > 0.0 && dt < 0.1) action_control_time_ += dt;
  const double slew_dt = bounded_dt(dt);
  if (!protocol_ok()) {state_ = State::FAULT; stop_failed_.store(true); return controller_interface::return_type::ERROR;}
  if (stop_requested_.exchange(false) && state_ != State::STOPPING) {abort_active(); if (!request_safe()) return controller_interface::return_type::ERROR;}
  if (state_ == State::STOPPING) {
    if (exact_safe_stop_ack(static_cast<double>(requested_safe_generation_), state_interfaces_[16].get_value(), state_interfaces_[17].get_value(), state_interfaces_[18].get_value())) {state_ = State::SAFE_STOPPED; safe_stopped_.store(true); task_ready_.store(false);}
    else if (++wait_cycles_ > max_wait_cycles_) {state_ = State::FAULT; stop_failed_.store(true); return controller_interface::return_type::ERROR;}
    return controller_interface::return_type::OK;
  }
  if (generation_ && state_interfaces_[15].get_value() != static_cast<double>(generation_)) {if (++wait_cycles_ > max_wait_cycles_ && !request_safe()) return controller_interface::return_type::ERROR; return controller_interface::return_type::OK;}
  wait_cycles_ = 0;
  if (state_ == State::SEEDING && generation_) {
    state_ = State::ACTIVE;
    startup_start_ = measured(); task_q_ref_ = startup_start_; startup_elapsed_ = 0.0;
    if (!return_to_zero_) {
      if (!latch_idle_pose()) {
        if (!request_safe()) return controller_interface::return_type::ERROR;
        return controller_interface::return_type::OK;
      }
      startup_active_ = false;
      task_ready_.store(true, std::memory_order_release);
      return controller_interface::return_type::OK;
    }
    for (std::size_t i = 0; i < 7; ++i) {
      const double peak = 1.5 * std::abs(startup_posture_[i] - startup_start_[i]) / startup_duration_;
      if (!std::isfinite(startup_start_[i]) || !std::isfinite(peak) || peak > command_velocity_[i]) {
        if (!request_safe()) return controller_interface::return_type::ERROR;
        return controller_interface::return_type::OK;
      }
    }
    startup_active_ = true; task_ready_.store(false, std::memory_order_release);
    return controller_interface::return_type::OK;
  }
  DirectMitTarget target = seed_;
  bool startup_command = false;
  if (state_ == State::ACTIVE && startup_active_) {
    startup_command = true;
    if (std::isfinite(dt) && dt > 0.0 && dt < 0.1) startup_elapsed_ += dt;
    const double u = std::clamp(startup_elapsed_ / startup_duration_, 0.0, 1.0);
    const double s = u * u * (3.0 - 2.0 * u);
    const double ds = 6.0 * u * (1.0 - u) / startup_duration_;
    for (std::size_t i = 0; i < 7; ++i) {
      const double delta = startup_posture_[i] - startup_start_[i];
      target.position[i] = startup_start_[i] + s * delta;
      target.velocity[i] = std::clamp(ds * delta, -command_velocity_[i], command_velocity_[i]);
      task_q_ref_[i] = target.position[i];
      task_q_reference_observed_[i].store(task_q_ref_[i], std::memory_order_release);
    }
    const auto q = measured(); std::array<double, 7> dq{}, nle{};
    for (std::size_t i = 0; i < 7; ++i) dq[i] = state_interfaces_[2 * i + 1].get_value();
    if (!model_nle(q, dq, nle)) {if (!request_safe()) return controller_interface::return_type::ERROR; return controller_interface::return_type::OK;}
    // The same slewed model tracker as the Cartesian path, so the gravity
    // term neither steps in at the first ACTIVE cycle nor at the mode
    // boundary into Cartesian control.
    for (std::size_t i = 0; i < 7; ++i) target.feedforward[i] = slew_model_feedforward(i, gravity_scale_.load(std::memory_order_acquire) * gravity_joint_scale_[i].load(std::memory_order_acquire) * nle[i], slew_dt);
    if (u >= 1.0) {
      bool converged = true;
      for (std::size_t i = 0; i < 7; ++i) converged = converged && std::abs(q[i] - startup_posture_[i]) <= startup_tolerance_;
      if (converged) {
        if (return_to_zero_) {
          ramped_return_to_zero_gains(startup_kp_, startup_kd_, startup_elapsed_,
            return_to_zero_handoff_source_kp_, return_to_zero_handoff_source_kd_);
          // The Cartesian controller's post-RTZ mode carries the task law in
          // tau_ff with zero MIT stiffness, so ramp kp continuously to zero.
          // kd instead ramps to the controller's own `kd`, which the damped
          // Cartesian mode keeps applying: dropping it to zero here and back
          // to `kd` at the mode boundary would be a discontinuity, and would
          // leave the arm's null space undamped in between.
          return_to_zero_handoff_duration_ = 0.0;
          const auto handoff_kd = current_kd();
          for (std::size_t i = 0; i < 7; ++i) {
            // Distance to the gains the task law will actually hold at, not to
            // zero. Zero was the old target, when Cartesian control ran on
            // tau_ff with kp = 0; under drive-side impedance the target is kp_
            // itself, and sizing the window for the shorter trip would end the
            // handoff with kp part-way there and then step it the rest of the
            // way - defeating kp_slew_per_s, which is the whole point of the
            // ramp.
            const double kp_target = drive_side_impedance_ ? kp_[i] : 0.0;
            return_to_zero_handoff_duration_ = std::max(
              return_to_zero_handoff_duration_,
              std::abs(kp_target - return_to_zero_handoff_source_kp_[i]) / profile_kp_slew_[i]);
            return_to_zero_handoff_duration_ = std::max(
              return_to_zero_handoff_duration_,
              std::abs(return_to_zero_handoff_source_kd_[i] - handoff_kd[i]) / profile_kd_slew_[i]);
          }
          return_to_zero_handoff_active_ = true;
          return_to_zero_handoff_elapsed_ = 0.0;
        }
        startup_active_ = false;
        task_q_ref_ = q;
        if (!return_to_zero_) {
          if (!latch_idle_pose()) {
            if (!request_safe()) return controller_interface::return_type::ERROR;
            return controller_interface::return_type::OK;
          }
          task_ready_.store(true, std::memory_order_release);
        }
      }
      else if (startup_elapsed_ > startup_duration_ + 2.0) {if (!request_safe()) return controller_interface::return_type::ERROR; return controller_interface::return_type::OK;}
    }
  }
  if (state_ == State::ACTIVE && !startup_command && return_to_zero_handoff_active_) {
    if (std::isfinite(dt) && dt > 0.0 && dt < 0.1) return_to_zero_handoff_elapsed_ += dt;
    if (return_to_zero_handoff_elapsed_ >= return_to_zero_handoff_duration_) {
      if (!latch_idle_pose()) {
        if (!request_safe()) return controller_interface::return_type::ERROR;
        return controller_interface::return_type::OK;
      }
      return_to_zero_handoff_active_ = false;
      task_ready_.store(true, std::memory_order_release);
    }
  }
  // A return-to-zero gain handoff must retain the converged zero reference.
  // In particular, do not leave `target` at `seed_` (the activation posture)
  // while the handoff flag is true: that would command a discontinuous move
  // back to the pre-initialization posture before the first task goal.
  bool task_command = false;
  if (state_ == State::ACTIVE && !startup_command) {
    task_command = write_task_target(action_control_time_, slew_dt, target);
  }
  if (state_ == State::ACTIVE && !startup_command && !task_command) {
    if (task_compute_failed_) {
      abort_active();
      if (!request_safe()) return controller_interface::return_type::ERROR;
      return controller_interface::return_type::OK;
    }
    if (task_capacity_rejected_) {
      if (!request_safe()) return controller_interface::return_type::ERROR;
      return controller_interface::return_type::OK;
    }
    // Before the Cartesian pose is latched (startup/gain handoff), retain the
    // converged joint reference with model feed-forward. Normal Cartesian idle
    // takes the TRACKING_DAMPED_TORQUE branch above.
    target.position = task_q_ref_;
    const auto q = measured(); std::array<double, 7> dq{}, nle{};
    for (std::size_t i = 0; i < 7; ++i) dq[i] = state_interfaces_[2 * i + 1].get_value();
    if (!model_nle(q, dq, nle)) {state_ = State::FAULT; stop_failed_.store(true); return controller_interface::return_type::ERROR;}
    for (std::size_t i = 0; i < 7; ++i) target.feedforward[i] = slew_model_feedforward(i, gravity_scale_.load(std::memory_order_acquire) * gravity_joint_scale_[i].load(std::memory_order_acquire) * nle[i], slew_dt);
  }
  auto command_kp = startup_command ? startup_kp_ : kp_;
  auto command_kd = startup_command ? startup_kd_ : current_kd();
  if (startup_command && return_to_zero_) {
    ramped_return_to_zero_gains(
      startup_kp_, startup_kd_, startup_elapsed_, command_kp, command_kd);
  } else if (return_to_zero_handoff_active_) {
    const auto live_kd = current_kd();
    for (std::size_t i = 0; i < 7; ++i) {
      // Ramp from the return-to-zero gains to whatever the task law will then
      // hold at, honouring the profile slew. Under drive-side impedance that
      // target is kp_ itself. Ramping to zero and letting `command_kp = kp_`
      // take over on the next cycle would step kp from 0 to 32 in a single
      // cycle and bypass kp_slew_per_s entirely; that is harmless only because
      // the idle pose is latched at that same instant, so the error the step
      // multiplies happens to be zero. Relying on that coincidence is not a
      // safety argument.
      const double kp_target = drive_side_impedance_ ? kp_[i] : 0.0;
      const double kp_step = profile_kp_slew_[i] * return_to_zero_handoff_elapsed_;
      const double kd_step = profile_kd_slew_[i] * return_to_zero_handoff_elapsed_;
      command_kp[i] = return_to_zero_handoff_source_kp_[i] + std::clamp(
        kp_target - return_to_zero_handoff_source_kp_[i], -kp_step, kp_step);
      command_kd[i] = return_to_zero_handoff_source_kd_[i] + std::clamp(
        live_kd[i] - return_to_zero_handoff_source_kd_[i], -kd_step, kd_step);
    }
  }
  // IMPEDANCE is the mode that actually carries kp and q_des to the drive.
  // TRACKING_DAMPED_TORQUE pins q_des to the measured position and sends kp=0,
  // which is right only while the controller is synthesising the whole
  // Cartesian law into tau_ff itself.
  const auto command_mode = state_ == State::SEEDING ? DirectMitMode::POSITION :
    ((task_command && !drive_side_impedance_) ? DirectMitMode::TRACKING_DAMPED_TORQUE
                                              : DirectMitMode::IMPEDANCE);
  auto command = map_direct_mit_command(
    command_mode, target, measured(), command_kp, command_kd, torque_limit_);
  clamp_command_positions(command);
  ++generation_; for (std::size_t i = 0; i < 7; ++i) {const auto o = 5 * i; command_interfaces_[o].set_value(command.joints[i].position); command_interfaces_[o + 1].set_value(command.joints[i].velocity); command_interfaces_[o + 2].set_value(command.joints[i].stiffness); command_interfaces_[o + 3].set_value(command.joints[i].damping); command_interfaces_[o + 4].set_value(command.joints[i].effort);}
  command_interfaces_[35].set_value(session_); command_interfaces_[36].set_value(lease_); command_interfaces_[37].set_value(generation_);
  return controller_interface::return_type::OK;
}

void TaskSpaceImpedanceMitController::non_rt_tick()
{
  TerminalEvent event; while (task_terminal_queue_.pop(event)) {
    std::shared_ptr<GoalHandle> handle; {std::lock_guard<std::mutex> lock(task_handles_mutex_); const auto it = task_handles_.find(event.id); if (it == task_handles_.end()) continue; handle = it->second; task_handles_.erase(it);}
    auto result = std::make_shared<Action::Result>(); result->is_completed = event.terminal == Terminal::SUCCEEDED;
    if (event.terminal == Terminal::SUCCEEDED) handle->succeed(result); else if (event.terminal == Terminal::CANCELED) handle->canceled(result); else handle->abort(result);
  }
  const auto id = task_public_id_.load(std::memory_order_acquire); if (!id) return;
  std::shared_ptr<GoalHandle> active; {std::lock_guard<std::mutex> lock(task_handles_mutex_); const auto it = task_handles_.find(id); if (it != task_handles_.end()) active = it->second;}
  if (active) {auto feedback = std::make_shared<Action::Feedback>(); feedback->percent_complete = static_cast<float>(task_percent_.load()); active->publish_feedback(feedback);}
}
}  // namespace cho_controller_openarm_mit

PLUGINLIB_EXPORT_CLASS(cho_controller_openarm_mit::TaskSpaceImpedanceMitController, controller_interface::ControllerInterface)
