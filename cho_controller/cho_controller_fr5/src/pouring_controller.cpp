#include "cho_controller_fr5/pouring_controller.hpp"

#include <algorithm>
#include <cmath>

namespace cho_controller {
namespace fr5 {

controller_interface::InterfaceConfiguration
PouringController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
    }
    return config;
}

CallbackReturn PouringController::on_init()
{
    if (FR5BaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<std::string>("scale_topic", scale_topic_);
        auto_declare<std::string>("pour_joint", "");
        auto_declare<double>("scale_timeout", scale_timeout_);
        auto_declare<double>("scale_startup_grace", scale_startup_grace_);
        auto_declare<double>("control_period", control_period_);
        auto_declare<double>("kp", kp_);
        auto_declare<double>("kd", kd_);
        auto_declare<double>("kernel_horizon", kernel_horizon_);
        auto_declare<double>("shaping_freq", shaping_freq_);
        auto_declare<double>("shaping_decay", shaping_decay_);
        auto_declare<double>("kernel_alpha", kernel_alpha_);
        auto_declare<double>("max_tilt_rate", max_tilt_rate_);
        auto_declare<double>("max_tilt", max_tilt_);
        auto_declare<double>("weight_tolerance", weight_tolerance_);
        auto_declare<double>("max_delta_q", max_delta_q_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

bool PouringController::assign_parameters()
{
    auto node = get_node();
    scale_topic_ = node->get_parameter("scale_topic").as_string();
    pour_joint_ = node->get_parameter("pour_joint").as_string();
    scale_timeout_ = node->get_parameter("scale_timeout").as_double();
    scale_startup_grace_ = node->get_parameter("scale_startup_grace").as_double();
    control_period_ = node->get_parameter("control_period").as_double();
    kp_ = node->get_parameter("kp").as_double();
    kd_ = node->get_parameter("kd").as_double();
    kernel_horizon_ = node->get_parameter("kernel_horizon").as_double();
    shaping_freq_ = node->get_parameter("shaping_freq").as_double();
    shaping_decay_ = node->get_parameter("shaping_decay").as_double();
    kernel_alpha_ = node->get_parameter("kernel_alpha").as_double();
    max_tilt_rate_ = node->get_parameter("max_tilt_rate").as_double();
    max_tilt_ = node->get_parameter("max_tilt").as_double();
    weight_tolerance_ = node->get_parameter("weight_tolerance").as_double();
    max_delta_q_ = node->get_parameter("max_delta_q").as_double();

    // Every one of these bounds a real motion, so a nonsense value is refused at
    // configure rather than discovered mid-pour with a vessel in the gripper.
    const std::vector<std::pair<const char *, double>> positives = {
        {"scale_timeout", scale_timeout_}, {"control_period", control_period_},
        {"scale_startup_grace", scale_startup_grace_},
        {"kernel_horizon", kernel_horizon_}, {"max_tilt_rate", max_tilt_rate_},
        {"max_tilt", max_tilt_}, {"weight_tolerance", weight_tolerance_},
        {"max_delta_q", max_delta_q_},
    };
    for (const auto & [name, value] : positives) {
        if (!std::isfinite(value) || value <= 0.0) {
            RCLCPP_ERROR(node->get_logger(), "%s must be positive (got %f)", name, value);
            return false;
        }
    }
    if (scale_topic_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "scale_topic must name the topic the weight arrives on");
        return false;
    }
    if (kernel_horizon_ < control_period_) {
        RCLCPP_ERROR(node->get_logger(),
            "kernel_horizon (%f) is shorter than one control_period (%f): the shaping "
            "filter would have no taps", kernel_horizon_, control_period_);
        return false;
    }
    return true;
}

void PouringController::build_shaping_kernel()
{
    // Same kernel as the simulated pour: a decaying sinusoid, normalised by its
    // absolute sum and blended with its peak normalisation. The sum normalisation
    // fixes the total response to a step of error; the alpha of peak normalisation
    // keeps a single large sample from being smoothed into nothing.
    const std::size_t taps = std::max<std::size_t>(
        1, static_cast<std::size_t>(kernel_horizon_ / control_period_));
    kernel_.assign(taps, 0.0);

    double abs_sum = 0.0;
    double abs_max = 0.0;
    for (std::size_t i = 0; i < taps; ++i) {
        const double t = static_cast<double>(i) * control_period_;
        kernel_[i] = std::exp(-shaping_decay_ * t) * std::sin(2.0 * M_PI * shaping_freq_ * t);
        abs_sum += std::abs(kernel_[i]);
        abs_max = std::max(abs_max, std::abs(kernel_[i]));
    }
    if (abs_sum > 1e-6) {
        for (std::size_t i = 0; i < taps; ++i) {
            kernel_[i] = (1.0 - kernel_alpha_) * (kernel_[i] / abs_sum)
                       + kernel_alpha_ * (kernel_[i] / (abs_max + 1e-6));
        }
    }
    pd_history_.assign(taps, 0.0);
    pd_head_ = 0;
    pd_count_ = 0;
}

CallbackReturn PouringController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    if (!assign_parameters()) {
        return CallbackReturn::FAILURE;
    }
    if (FR5BaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    // The pour joint defaults to the last one: under the AG-95's side grasp its
    // axis runs through the grasp, so the vessel's tilt tracks it 1:1.
    pour_index_ = num_dof_ - 1;
    if (!pour_joint_.empty()) {
        const auto it = std::find(joint_names_.begin(), joint_names_.end(), pour_joint_);
        if (it == joint_names_.end()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "pour_joint '%s' is not one of this arm's joints", pour_joint_.c_str());
            return CallbackReturn::FAILURE;
        }
        pour_index_ = static_cast<int>(std::distance(joint_names_.begin(), it));
    }

    build_shaping_kernel();

    scale_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        scale_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            // A non-finite weight is dropped rather than stored: the scale driver
            // publishes NaN for a unit it cannot convert, and letting that reach
            // the law would make the error, the PD output and the command NaN.
            if (!std::isfinite(msg->data)) {
                return;
            }
            ScaleSample sample;
            sample.grams = msg->data;
            sample.stamp = get_node()->now().seconds();
            sample.valid = true;
            scale_buffer_.writeFromNonRT(sample);
        });

    action_server_ = std::make_shared<FR5PourActionServer>(
        get_node(), "/controller_action_server/pouring_controller", num_dof_);
    action_server_->init();
    PourBounds defaults;
    defaults.max_tilt_rate = max_tilt_rate_;
    defaults.max_tilt = max_tilt_;
    defaults.tolerance = weight_tolerance_;
    defaults.timeout = 120.0;
    action_server_->set_defaults(defaults);

    RCLCPP_INFO(get_node()->get_logger(),
        "PouringController configured: pour joint '%s', weight from %s, law at %.0f Hz "
        "(%zu shaping taps)",
        joint_names_[pour_index_].c_str(), scale_topic_.c_str(), 1.0 / control_period_,
        kernel_.size());
    return CallbackReturn::SUCCESS;
}

CallbackReturn PouringController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    if (FR5BaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    // From the held command, not the measurement: the controller this one takes
    // over from left a position on the interface, and starting from the measured
    // (drooped) one would step the command by the droop on the first cycle -- with
    // a full vessel in the gripper.
    q_ref_ = FR5BaseController::held_command_position();
    phase_ = Phase::Idle;
    commanded_rate_ = 0.0;
    law_accumulator_ = 0.0;
    feedback_accumulator_ = 0.0;
    scale_buffer_.writeFromNonRT(ScaleSample{});
    return CallbackReturn::SUCCESS;
}

CallbackReturn PouringController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    // A goal that outlives its controller can never finish: nothing is left to
    // run the law, and the caller would wait for a result that cannot come.
    if (action_server_) {
        action_server_->abort_active_goal(
            "the pouring controller was deactivated mid-pour", last_grams_, peak_tilt_, elapsed_);
    }
    phase_ = Phase::Idle;
    return FR5BaseController::on_deactivate(previous_state);
}

double PouringController::shaped_velocity(double error)
{
    const double d_error = (error - prev_error_) / control_period_;
    prev_error_ = error;
    const double pd = kp_ * error + kd_ * d_error;

    pd_history_[pd_head_] = pd;
    pd_head_ = (pd_head_ + 1) % pd_history_.size();
    pd_count_ = std::min(pd_count_ + 1, pd_history_.size());

    // Causal convolution: tap 0 multiplies the newest sample.
    double v_cmd = 0.0;
    for (std::size_t i = 0; i < pd_count_; ++i) {
        const std::size_t index = (pd_head_ + pd_history_.size() - 1 - i) % pd_history_.size();
        v_cmd += pd_history_[index] * kernel_[i];
    }
    return std::clamp(v_cmd, -action_server_->bounds().max_tilt_rate,
                      action_server_->bounds().max_tilt_rate);
}

void PouringController::write_command(const Eigen::VectorXd & q_cmd)
{
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_cmd(i));
    }
    state_.q_des = q_cmd;
    q_ref_ = q_cmd;
}

void PouringController::hold_reference()
{
    state_.q_des = q_ref_;
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_ref_(i));
    }
}

void PouringController::begin_untilt(bool succeeded, const std::string & reason)
{
    if (phase_ == Phase::Untilt) {
        return;
    }
    phase_ = Phase::Untilt;
    pending_success_ = succeeded;
    pending_reason_ = reason;
    commanded_rate_ = 0.0;
    if (!succeeded) {
        RCLCPP_WARN(get_node()->get_logger(),
            "Pour ending without reaching the target (%s); returning the vessel upright first",
            reason.c_str());
    }
}

void PouringController::finish(double grams)
{
    if (pending_success_) {
        action_server_->succeed(grams, peak_tilt_, elapsed_);
    } else {
        action_server_->abort_active_goal(pending_reason_, grams, peak_tilt_, elapsed_);
    }
    phase_ = Phase::Idle;
    commanded_rate_ = 0.0;
}

controller_interface::return_type PouringController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (FR5BaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    if (q_ref_.size() != num_dof_) {
        q_ref_ = state_.q.head(num_dof_);
    }

    const bool running = action_server_ && action_server_->is_running();
    if (!running) {
        // Idle: freeze the reference. Re-deriving a hold from the measured
        // position would feed servo droop back into the command.
        if (phase_ != Phase::Idle) {
            phase_ = Phase::Idle;
            commanded_rate_ = 0.0;
        }
        hold_reference();
        return controller_interface::return_type::OK;
    }

    const double dt = nominal_period(period);

    if (phase_ == Phase::Idle) {
        // First cycle of a goal. The pour is measured from the attitude the
        // vessel is being carried in, which is the reference the previous steps
        // left behind -- not the measured angle, which lags it.
        phase_ = Phase::Pouring;
        theta_start_ = q_ref_(pour_index_);
        theta_ = theta_start_;
        peak_tilt_ = 0.0;
        elapsed_ = 0.0;
        prev_error_ = 0.0;
        law_accumulator_ = 0.0;
        feedback_accumulator_ = 0.0;
        commanded_rate_ = 0.0;
        scale_seen_ = false;
        pd_head_ = 0;
        pd_count_ = 0;
        std::fill(pd_history_.begin(), pd_history_.end(), 0.0);
    }

    elapsed_ += dt;
    const auto & bounds = action_server_->bounds();
    const ScaleSample sample = *scale_buffer_.readFromRT();
    last_grams_ = sample.grams;
    const double scale_age = sample.valid ? (time.seconds() - sample.stamp)
                                          : std::numeric_limits<double>::infinity();

    if (phase_ == Phase::Pouring) {
        const bool fresh = sample.valid && scale_age <= scale_timeout_;
        if (fresh) {
            scale_seen_ = true;
        }
        if (!fresh && !scale_seen_) {
            // No reading for THIS pour yet. Hold still and wait: a goal that
            // arrives before the scale node does is a late scale, not a failed
            // one, and aborting on the first cycle would make the order the two
            // are started in decide whether a pour can run at all. A sample left
            // over from an earlier pour does not count -- it is not a
            // measurement of what is in the vessel now.
            commanded_rate_ = 0.0;
            if (elapsed_ > scale_startup_grace_) {
                begin_untilt(false,
                    "no scale reading arrived within scale_startup_grace of the goal");
            }
        } else if (!fresh) {
            // Stop tilting the moment the weight goes quiet. Continuing on the
            // last number it heard is how a vessel empties onto the bench.
            commanded_rate_ = 0.0;
            begin_untilt(false,
                "the scale went quiet mid-pour (no reading for longer than scale_timeout)");
        } else if (action_server_->is_canceling()) {
            commanded_rate_ = 0.0;
            begin_untilt(false, "cancelled");
        } else if (elapsed_ > bounds.timeout) {
            commanded_rate_ = 0.0;
            begin_untilt(false, "timed out before reaching the target weight");
        } else {
            const double error = bounds.target_grams - sample.grams;
            if (std::abs(error) < bounds.tolerance) {
                commanded_rate_ = 0.0;
                begin_untilt(true, "");
            } else {
                // Sample the law on ITS OWN period, not the controller's, so the
                // hardware pour and the simulated one are the same law at the same
                // rate. Between samples the commanded velocity is held.
                law_accumulator_ += dt;
                if (law_accumulator_ >= control_period_) {
                    law_accumulator_ -= control_period_;
                    commanded_rate_ = shaped_velocity(error);
                }
            }
        }
    }

    if (phase_ == Phase::Untilt) {
        const double remaining = theta_start_ - theta_;
        const double step = bounds.max_tilt_rate * dt;
        if (std::abs(remaining) <= step) {
            theta_ = theta_start_;
            Eigen::VectorXd q_cmd = q_ref_;
            q_cmd(pour_index_) = theta_;
            clamp_to_joint_limits(q_cmd);
            write_command(q_cmd);
            finish(sample.grams);
            return controller_interface::return_type::OK;
        }
        commanded_rate_ = std::copysign(bounds.max_tilt_rate, remaining);
    }

    theta_ += commanded_rate_ * dt;

    // Absolute tilt bound, measured from the attitude the pour started in. This
    // is what stops a pour the scale never satisfies -- an empty vessel, a
    // blocked spout, a scale reading a constant.
    const double tilt = theta_ - theta_start_;
    if (phase_ == Phase::Pouring && std::abs(tilt) >= bounds.max_tilt) {
        theta_ = theta_start_ + std::copysign(bounds.max_tilt, tilt);
        commanded_rate_ = 0.0;
        begin_untilt(false, "reached the tilt bound without reaching the target weight");
    }
    peak_tilt_ = std::max(peak_tilt_, std::abs(theta_ - theta_start_));

    Eigen::VectorXd q_cmd = q_ref_;
    q_cmd(pour_index_) = theta_;
    clamp_to_joint_limits(q_cmd);

    // Per-cycle command bound. The trajectory the law produces is already rate
    // limited, but this also covers the joint-limit clamp above and anything a
    // future edit puts between the two.
    Eigen::VectorXd delta = q_cmd - q_ref_;
    delta = delta.array().max(-max_delta_q_).min(max_delta_q_);
    q_cmd = q_ref_ + delta;

    if (!q_cmd.allFinite()) {
        action_server_->abort_active_goal(
            "the pour command went non-finite; holding the last finite command",
            sample.grams, peak_tilt_, elapsed_);
        phase_ = Phase::Idle;
        commanded_rate_ = 0.0;
        hold_reference();
        return controller_interface::return_type::OK;
    }

    write_command(q_cmd);
    // The commanded angle follows the clamped command, so a joint limit or the
    // per-cycle bound cannot leave the law integrating an angle the arm was
    // never asked to reach.
    theta_ = q_cmd(pour_index_);

    feedback_accumulator_ += dt;
    if (feedback_accumulator_ >= control_period_) {
        feedback_accumulator_ -= control_period_;
        action_server_->publish_feedback(sample.grams, theta_ - theta_start_, elapsed_);
    }
    return controller_interface::return_type::OK;
}

} // namespace fr5
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::fr5::PouringController,
                       controller_interface::ControllerInterface)
