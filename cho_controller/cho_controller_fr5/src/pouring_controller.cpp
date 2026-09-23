#include "cho_controller_fr5/pouring_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cho_controller {
namespace fr5 {

using pour::MaterialClass;
using pour::PourLimits;
using pour::PourPhase;

namespace {
// Seeds, not measurements, except where noted. The liquid/free endpoint is the
// one with hardware behind it: water on an HS-AA, 2026-09-16, which ran 6-11 g/s
// continuous, settled 1.0 s after the flow stopped, left <= 0.21 g of tail, and
// broke into ~1.2 g drops once the stream thinned. Everything else is a
// commissioning starting point and belongs in controllers.yaml, not here.
PourLimits liquid_free_defaults()
{
    PourLimits l;
    l.max_flow_rate = 10.0;
    l.trim_flow_rate = 2.0;
    l.trim_pulse_sec = 0.15;
    l.transport_delay = 0.4;
    l.afterflow_grams = 0.3;
    l.settle_hold_sec = 1.2;
    l.tilt_rate = 0.5;
    l.seek_tilt_rate = 0.15;
    l.dose_quantum = 0.3;
    return l;
}

PourLimits liquid_resistant_defaults()
{
    PourLimits l;
    l.max_flow_rate = 2.0;
    l.trim_flow_rate = 0.5;
    l.trim_pulse_sec = 0.4;
    l.transport_delay = 1.2;
    l.afterflow_grams = 4.0;
    l.settle_hold_sec = 3.0;
    l.tilt_rate = 0.2;
    l.seek_tilt_rate = 0.06;
    // Not 0.5 g. Every stop of a material this thick leaves a string behind, and
    // that string is the smallest amount it can be metered in: asking for finer
    // than the tail spends the whole pulse budget oscillating around a target
    // the material cannot hit.
    l.dose_quantum = 3.0;
    return l;
}

PourLimits granular_free_defaults()
{
    PourLimits l;
    l.max_flow_rate = 8.0;
    l.trim_flow_rate = 1.5;
    l.trim_pulse_sec = 0.2;
    l.transport_delay = 0.3;
    l.afterflow_grams = 0.5;
    l.settle_hold_sec = 1.5;
    l.tilt_rate = 0.35;
    l.seek_tilt_rate = 0.10;
    l.dose_quantum = 0.5;
    return l;
}

PourLimits granular_resistant_defaults()
{
    PourLimits l;
    l.max_flow_rate = 3.0;
    l.trim_flow_rate = 0.8;
    l.trim_pulse_sec = 0.5;
    l.transport_delay = 0.6;
    l.afterflow_grams = 3.0;
    l.settle_hold_sec = 3.0;
    l.tilt_rate = 0.18;
    l.seek_tilt_rate = 0.05;
    l.dose_quantum = 2.0;
    return l;
}
}  // namespace

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

void PouringController::declare_limits(const std::string & prefix, const PourLimits & d)
{
    auto_declare<double>(prefix + ".max_flow_rate", d.max_flow_rate);
    auto_declare<double>(prefix + ".trim_flow_rate", d.trim_flow_rate);
    auto_declare<double>(prefix + ".trim_pulse_sec", d.trim_pulse_sec);
    auto_declare<double>(prefix + ".transport_delay", d.transport_delay);
    auto_declare<double>(prefix + ".afterflow_grams", d.afterflow_grams);
    auto_declare<double>(prefix + ".settle_hold_sec", d.settle_hold_sec);
    auto_declare<double>(prefix + ".tilt_rate", d.tilt_rate);
    auto_declare<double>(prefix + ".seek_tilt_rate", d.seek_tilt_rate);
    auto_declare<double>(prefix + ".dose_quantum", d.dose_quantum);
}

PourLimits PouringController::read_limits(const std::string & prefix) const
{
    auto node = get_node();
    PourLimits l;
    l.max_flow_rate = node->get_parameter(prefix + ".max_flow_rate").as_double();
    l.trim_flow_rate = node->get_parameter(prefix + ".trim_flow_rate").as_double();
    l.trim_pulse_sec = node->get_parameter(prefix + ".trim_pulse_sec").as_double();
    l.transport_delay = node->get_parameter(prefix + ".transport_delay").as_double();
    l.afterflow_grams = node->get_parameter(prefix + ".afterflow_grams").as_double();
    l.settle_hold_sec = node->get_parameter(prefix + ".settle_hold_sec").as_double();
    l.tilt_rate = node->get_parameter(prefix + ".tilt_rate").as_double();
    l.seek_tilt_rate = node->get_parameter(prefix + ".seek_tilt_rate").as_double();
    l.dose_quantum = node->get_parameter(prefix + ".dose_quantum").as_double();
    return l;
}

bool PouringController::read_profile(const std::string & prefix, pour::MaterialProfile & out,
                                     std::string & why)
{
    out.free = read_limits(prefix + ".free");
    out.resistant = read_limits(prefix + ".resistant");
    return out.validate(prefix, why);
}

CallbackReturn PouringController::on_init()
{
    if (FR5BaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<std::string>("scale_topic", scale_topic_);
        auto_declare<std::string>("pour_joint", "");
        auto_declare<double>("pour_direction", pour_direction_);
        auto_declare<double>("scale_timeout", scale_timeout_);
        auto_declare<double>("scale_sample_period", scale_sample_period_);
        auto_declare<double>("outlier_factor", outlier_factor_);
        auto_declare<double>("max_tilt", max_tilt_);
        auto_declare<double>("max_back_tilt", max_back_tilt_);
        auto_declare<double>("weight_tolerance", weight_tolerance_);
        auto_declare<double>("max_delta_q", max_delta_q_);
        auto_declare<double>("feedback_period", feedback_period_);

        const pour::PlannerConfig d;
        auto_declare<double>("container_tolerance", d.container_tolerance);
        auto_declare<double>("verify_timeout", d.verify_timeout);
        auto_declare<double>("onset_grams", d.onset_grams);
        auto_declare<double>("approach_sec", d.approach_sec);
        auto_declare<double>("kp_tilt", d.kp_tilt);
        auto_declare<double>("no_flow_epsilon", d.no_flow_epsilon);
        auto_declare<double>("park_check_sec", d.park_check_sec);
        auto_declare<double>("park_drip_grams", d.park_drip_grams);
        auto_declare<double>("stop_margin_factor", d.stop_margin_factor);
        auto_declare<double>("settle_timeout", d.settle_timeout);
        auto_declare<double>("stall_timeout", d.stall_timeout);
        auto_declare<double>("trim_undershoot", d.trim_undershoot);
        auto_declare<int>("max_trim_pulses", d.max_trim_pulses);
        auto_declare<double>("retract_margin", d.retract_margin);
        auto_declare<double>("trim_tilt_margin", d.trim_tilt_margin);
        auto_declare<double>("tilt_epsilon", d.tilt_epsilon);
        auto_declare<int>("max_park_attempts", d.max_park_attempts);
        auto_declare<double>("retract_slack_sec", d.retract_slack_sec);
        auto_declare<double>("max_pulse_sec", d.max_pulse_sec);

        auto_declare<std::string>("law", law_name_);
        const pour::ShapingConfig sd;
        auto_declare<double>("shaping.control_period", sd.control_period);
        auto_declare<double>("shaping.kp", sd.kp);
        auto_declare<double>("shaping.kd", sd.kd);
        auto_declare<double>("shaping.kernel_horizon", sd.kernel_horizon);
        auto_declare<double>("shaping.shaping_freq", sd.shaping_freq);
        auto_declare<double>("shaping.shaping_decay", sd.shaping_decay);
        auto_declare<double>("shaping.kernel_alpha", sd.kernel_alpha);
        auto_declare<double>("shaping.max_tilt_rate", sd.max_tilt_rate);
        auto_declare<double>("shaping.settle_hold_sec", sd.settle_hold_sec);

        declare_limits("liquid.free", liquid_free_defaults());
        declare_limits("liquid.resistant", liquid_resistant_defaults());
        declare_limits("granular.free", granular_free_defaults());
        declare_limits("granular.resistant", granular_resistant_defaults());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

bool PouringController::assign_parameters()
{
    auto node = get_node();
    auto logger = node->get_logger();

    scale_topic_ = node->get_parameter("scale_topic").as_string();
    pour_joint_ = node->get_parameter("pour_joint").as_string();
    pour_direction_ = node->get_parameter("pour_direction").as_double();
    scale_timeout_ = node->get_parameter("scale_timeout").as_double();
    scale_sample_period_ = node->get_parameter("scale_sample_period").as_double();
    outlier_factor_ = node->get_parameter("outlier_factor").as_double();
    max_tilt_ = node->get_parameter("max_tilt").as_double();
    max_back_tilt_ = node->get_parameter("max_back_tilt").as_double();
    weight_tolerance_ = node->get_parameter("weight_tolerance").as_double();
    max_delta_q_ = node->get_parameter("max_delta_q").as_double();
    feedback_period_ = node->get_parameter("feedback_period").as_double();

    planner_config_.container_tolerance = node->get_parameter("container_tolerance").as_double();
    planner_config_.verify_timeout = node->get_parameter("verify_timeout").as_double();
    planner_config_.onset_grams = node->get_parameter("onset_grams").as_double();
    planner_config_.approach_sec = node->get_parameter("approach_sec").as_double();
    planner_config_.kp_tilt = node->get_parameter("kp_tilt").as_double();
    planner_config_.no_flow_epsilon = node->get_parameter("no_flow_epsilon").as_double();
    planner_config_.park_check_sec = node->get_parameter("park_check_sec").as_double();
    planner_config_.park_drip_grams = node->get_parameter("park_drip_grams").as_double();
    planner_config_.stop_margin_factor = node->get_parameter("stop_margin_factor").as_double();
    planner_config_.settle_timeout = node->get_parameter("settle_timeout").as_double();
    planner_config_.stall_timeout = node->get_parameter("stall_timeout").as_double();
    planner_config_.trim_undershoot = node->get_parameter("trim_undershoot").as_double();
    planner_config_.max_trim_pulses = static_cast<int>(node->get_parameter("max_trim_pulses").as_int());
    planner_config_.retract_margin = node->get_parameter("retract_margin").as_double();
    planner_config_.trim_tilt_margin = node->get_parameter("trim_tilt_margin").as_double();
    planner_config_.tilt_epsilon = node->get_parameter("tilt_epsilon").as_double();
    planner_config_.max_park_attempts = static_cast<int>(node->get_parameter("max_park_attempts").as_int());
    planner_config_.retract_slack_sec = node->get_parameter("retract_slack_sec").as_double();
    planner_config_.max_pulse_sec = node->get_parameter("max_pulse_sec").as_double();

    // Every one of these bounds a real motion, so a nonsense value is refused at
    // configure rather than discovered mid-pour with a vessel in the gripper.
    const std::pair<const char *, double> positives[] = {
        {"scale_timeout", scale_timeout_},
        {"scale_sample_period", scale_sample_period_},
        {"outlier_factor", outlier_factor_},
        {"max_tilt", max_tilt_},
        {"weight_tolerance", weight_tolerance_},
        {"max_delta_q", max_delta_q_},
        {"feedback_period", feedback_period_},
    };
    for (const auto & [name, value] : positives) {
        if (!std::isfinite(value) || value <= 0.0) {
            RCLCPP_ERROR(logger, "%s must be positive (got %f)", name, value);
            return false;
        }
    }
    // Zero is legal -- the vessel may never be turned behind the attitude it
    // was carried in -- but a negative or non-finite bound is not a bound.
    if (!std::isfinite(max_back_tilt_) || max_back_tilt_ < 0.0) {
        RCLCPP_ERROR(logger, "max_back_tilt must be zero or positive (got %f)", max_back_tilt_);
        return false;
    }
    if (scale_topic_.empty()) {
        RCLCPP_ERROR(logger, "scale_topic must name the topic the weight arrives on");
        return false;
    }
    // Not "anything nonzero": the sign is applied to a joint command, and a
    // pour_direction of 0.7 would silently scale every tilt the planner asked
    // for by 0.7 and make every rate and bound in pour/ a lie.
    if (pour_direction_ != 1.0 && pour_direction_ != -1.0) {
        RCLCPP_ERROR(logger,
            "pour_direction must be exactly 1.0 or -1.0 (got %f): it says which way the pour "
            "joint turns to bring the lip down, and guessing it wrong tips the vessel away "
            "from the scale", pour_direction_);
        return false;
    }
    // The scale's own period has to be shorter than the staleness bound, or a
    // perfectly healthy stream reads as a dead one on its very first sample.
    if (scale_timeout_ <= scale_sample_period_) {
        RCLCPP_ERROR(logger,
            "scale_timeout (%f s) must exceed scale_sample_period (%f s); a healthy stream "
            "would otherwise be declared stale between samples",
            scale_timeout_, scale_sample_period_);
        return false;
    }

    std::string why;
    if (!read_profile("liquid", liquid_, why) || !read_profile("granular", granular_, why)) {
        RCLCPP_ERROR(logger, "Material profile rejected: %s", why.c_str());
        return false;
    }
    if (!planner_.configure(planner_config_, liquid_, granular_, why)) {
        RCLCPP_ERROR(logger, "Pour planner rejected its configuration: %s", why.c_str());
        return false;
    }

    // The shaping law's own gains, plus the obligations it shares with the
    // phase machine taken from the SAME parameters -- two laws being compared
    // must be judged by one container check, one settle and one stall rule.
    shaping_config_.control_period = node->get_parameter("shaping.control_period").as_double();
    shaping_config_.kp = node->get_parameter("shaping.kp").as_double();
    shaping_config_.kd = node->get_parameter("shaping.kd").as_double();
    shaping_config_.kernel_horizon = node->get_parameter("shaping.kernel_horizon").as_double();
    shaping_config_.shaping_freq = node->get_parameter("shaping.shaping_freq").as_double();
    shaping_config_.shaping_decay = node->get_parameter("shaping.shaping_decay").as_double();
    shaping_config_.kernel_alpha = node->get_parameter("shaping.kernel_alpha").as_double();
    shaping_config_.max_tilt_rate = node->get_parameter("shaping.max_tilt_rate").as_double();
    shaping_config_.settle_hold_sec = node->get_parameter("shaping.settle_hold_sec").as_double();
    shaping_config_.container_tolerance = planner_config_.container_tolerance;
    shaping_config_.verify_timeout = planner_config_.verify_timeout;
    shaping_config_.settle_timeout = planner_config_.settle_timeout;
    shaping_config_.stall_timeout = planner_config_.stall_timeout;
    shaping_config_.no_flow_epsilon = planner_config_.no_flow_epsilon;
    shaping_config_.tilt_epsilon = planner_config_.tilt_epsilon;
    if (!shaping_.configure(shaping_config_, why)) {
        RCLCPP_ERROR(logger, "Shaping pour law rejected its configuration: %s", why.c_str());
        return false;
    }

    law_name_ = node->get_parameter("law").as_string();
    if (law_name_ == "phase_machine") {
        law_ = &planner_;
    } else if (law_name_ == "shaping") {
        law_ = &shaping_;
    } else {
        RCLCPP_ERROR(logger,
            "law must be 'phase_machine' or 'shaping' (got '%s'). They are two different "
            "answers to one question -- how to pour against a delayed scale -- and which is "
            "better on this cell is what running both is for", law_name_.c_str());
        return false;
    }
    return true;
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

    // An outlier bound in grams, derived from the fastest flow any configured
    // profile allows. Expressing it in grams rather than as a rate is what makes
    // it survive a dropped sample: a 3x margin covers two samples' worth.
    const double fastest = std::max({liquid_.free.max_flow_rate, liquid_.resistant.max_flow_rate,
                                     granular_.free.max_flow_rate, granular_.resistant.max_flow_rate});
    pour::ScaleFilter::Config filter_config;
    filter_config.max_step_grams = outlier_factor_ * fastest * scale_sample_period_;
    filter_config.rate_window_sec = 3.0 * scale_sample_period_;
    filter_config.history = 8;
    filter_.configure(filter_config);

    scale_sub_ = get_node()->create_subscription<cho_interfaces::msg::ScaleReading>(
        scale_topic_, rclcpp::SensorDataQoS(),
        [this](const cho_interfaces::msg::ScaleReading::SharedPtr msg) {
            pour::ScaleFilter::Sample sample;
            sample.grams = msg->grams;
            // Arrival time on THIS node's clock, not header.stamp. Every age and
            // settle window is measured against the controller_manager's update
            // time, and header.stamp is on whatever clock the driver ran --
            // wall time under a sim-time bringup, another PC's clock on a
            // multi-PC one -- so using it put two clocks in one subtraction.
            // It bought nothing, either: the HS-AA driver stamps when it has
            // PARSED a frame, after the 75 ms serial shift, and delivery is
            // 0.6 ms (measured), so the two differ by less than a control cycle.
            // The delay that matters is calibrated end to end as the profile's
            // transport_delay, on this clock.
            sample.stamp = get_node()->now().seconds();
            sample.stable = msg->stable;
            scale_buffer_.writeFromNonRT(sample);
        });

    action_server_ = std::make_shared<FR5PourActionServer>(
        get_node(), "/controller_action_server/pouring_controller", num_dof_);
    action_server_->init();
    PourBounds defaults;
    defaults.max_tilt_rate = 0.0;  // 0 means "whatever the material profile says"
    defaults.max_tilt = max_tilt_;
    defaults.tolerance = weight_tolerance_;
    defaults.timeout = 120.0;
    action_server_->set_defaults(defaults);

    RCLCPP_INFO(get_node()->get_logger(),
        "PouringController configured: law '%s', pour joint '%s' (direction %+.0f), weight "
        "from %s, outlier bound %.1f g/sample",
        law_->name(), joint_names_[pour_index_].c_str(), pour_direction_, scale_topic_.c_str(),
        filter_config.max_step_grams);
    return CallbackReturn::SUCCESS;
}

CallbackReturn PouringController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    if (FR5BaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    // From the held command, not the measurement: the controller this one takes
    // over from left a position on the interface, and starting from the measured
    // (drooped) one would step the command by the droop on the first cycle --
    // with a full vessel in the gripper.
    q_ref_ = FR5BaseController::held_command_position();
    phase_ = Phase::Idle;
    tilt_ = 0.0;
    filter_.reset();
    last_pushed_stamp_ = 0.0;
    scale_buffer_.writeFromNonRT(pour::ScaleFilter::Sample{});
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
    if (!succeeded) {
        RCLCPP_WARN(get_node()->get_logger(),
            "Pour ending without reaching the target (%s); returning the vessel to the "
            "attitude it was carried in first", reason.c_str());
    }
}

void PouringController::finish()
{
    const auto & report = law_->report();
    if (pending_success_) {
        action_server_->succeed(report.poured_grams, peak_tilt_, elapsed_, report.trim_pulses,
                                report.measured_afterflow);
    } else {
        action_server_->abort_active_goal(pending_reason_, report.poured_grams, peak_tilt_,
                                          elapsed_, report.trim_pulses,
                                          report.measured_afterflow);
    }
    phase_ = Phase::Idle;
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
    if (!running && phase_ != Phase::Untilt) {
        // Idle: freeze the reference. Re-deriving a hold from the measured
        // position would feed servo droop back into the command.
        if (phase_ != Phase::Idle) {
            phase_ = Phase::Idle;
        }
        hold_reference();
        return controller_interface::return_type::OK;
    }

    const double dt = nominal_period(period);
    const double now = time.seconds();

    // Take whatever the subscription left, but only once. Re-pushing the same
    // sample every cycle would make the least-squares fit see 125 identical
    // points per reading and report a flow rate of zero throughout a pour.
    const pour::ScaleFilter::Sample sample = *scale_buffer_.readFromRT();
    if (sample.stamp > last_pushed_stamp_) {
        last_pushed_stamp_ = sample.stamp;
        filter_.push(sample);
    }

    if (phase_ == Phase::Idle) {
        phase_ = Phase::Pouring;
        theta_start_ = q_ref_(pour_index_);
        tilt_ = 0.0;
        peak_tilt_ = 0.0;
        elapsed_ = 0.0;
        feedback_accumulator_ = 0.0;
        last_phase_ = 0;
        filter_.reset();
        last_pushed_stamp_ = 0.0;

        const auto & bounds = action_server_->bounds();
        pour::PourRequest request;
        request.target_grams = bounds.target_grams;
        request.container_grams = bounds.container_grams;
        request.tolerance = bounds.tolerance;
        request.timeout = bounds.timeout;
        request.max_tilt = bounds.max_tilt;
        request.max_tilt_rate = bounds.max_tilt_rate;
        request.material = bounds.material == 1 ? MaterialClass::Granular : MaterialClass::Liquid;
        request.flow_index = bounds.flow_index;
        request.max_back_tilt = max_back_tilt_;
        law_->begin(request, now);
    }

    elapsed_ += dt;
    last_grams_ = filter_.grams();
    last_flow_ = filter_.flow_rate();

    double commanded_rate = 0.0;

    if (phase_ == Phase::Pouring) {
        if (action_server_->is_canceling()) {
            law_->cancel();
        }
        pour::PourObservation obs;
        obs.now = now;
        obs.has_reading = filter_.has_sample();
        obs.scale_fresh = obs.has_reading && filter_.age(now) <= scale_timeout_;
        obs.grams = filter_.grams();
        obs.flow_rate = filter_.flow_rate();
        obs.settled = filter_.settled(now, law_->settle_hold());
        obs.tilt = tilt_;
        obs.consecutive_rejects = filter_.consecutive_rejects();
        obs.last_rejected_step = filter_.last_rejected_step();

        const pour::PourCommand cmd = law_->update(obs);
        last_phase_ = static_cast<std::uint8_t>(cmd.phase);
        if (cmd.finished) {
            begin_untilt(cmd.success, cmd.message);
        } else {
            commanded_rate = cmd.tilt_rate;
        }
    }

    if (phase_ == Phase::Untilt) {
        last_phase_ = static_cast<std::uint8_t>(PourPhase::Done);
        const double step = std::abs(pour_direction_) * max_delta_q_ / std::max(dt, 1e-9);
        const double rate = std::min(law_->return_tilt_rate(), step);
        if (std::abs(tilt_) <= rate * dt) {
            tilt_ = 0.0;
            Eigen::VectorXd q_cmd = q_ref_;
            q_cmd(pour_index_) = theta_start_;
            clamp_to_joint_limits(q_cmd);
            write_command(q_cmd);
            finish();
            return controller_interface::return_type::OK;
        }
        commanded_rate = -std::copysign(rate, tilt_);
    }

    tilt_ += commanded_rate * dt;

    // Tilt range, measured from the attitude the pour started in, and BOTH ends
    // of it. The laws have their own bounds and normally stay inside; this is
    // the one that holds if a law is ever wrong -- a park lowered too far, or a
    // law driving a negative error backwards through upright. Bounding only the
    // pour side (what this did after the rewrite) left the other direction
    // limited by nothing short of the joint limit.
    const double bound = action_server_->bounds().max_tilt;
    if (phase_ == Phase::Pouring) {
        tilt_ = std::clamp(tilt_, -max_back_tilt_, bound);
    }
    peak_tilt_ = std::max(peak_tilt_, std::abs(tilt_));

    Eigen::VectorXd q_cmd = q_ref_;
    q_cmd(pour_index_) = theta_start_ + pour_direction_ * tilt_;
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
            last_grams_, peak_tilt_, elapsed_, law_->report().trim_pulses,
            law_->report().measured_afterflow);
        phase_ = Phase::Idle;
        hold_reference();
        return controller_interface::return_type::OK;
    }

    write_command(q_cmd);
    // The tilt follows the CLAMPED command, so a joint limit or the per-cycle
    // bound cannot leave the planner reasoning about an angle the arm was never
    // asked to reach.
    tilt_ = pour_direction_ * (q_cmd(pour_index_) - theta_start_);

    feedback_accumulator_ += dt;
    if (feedback_accumulator_ >= feedback_period_) {
        feedback_accumulator_ -= feedback_period_;
        action_server_->publish_feedback(last_grams_ - law_->baseline_grams(), tilt_, elapsed_,
                                         last_flow_, last_phase_);
    }
    return controller_interface::return_type::OK;
}

} // namespace fr5
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::fr5::PouringController,
                       controller_interface::ControllerInterface)
