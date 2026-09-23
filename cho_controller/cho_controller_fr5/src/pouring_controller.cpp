#include "cho_controller_fr5/pouring_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

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

Eigen::Isometry3d to_isometry(const pinocchio::SE3 & H)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = H.rotation();
    T.translation() = H.translation();
    return T;
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
        auto_declare<double>("tilt_accel", tilt_accel_);
        auto_declare<double>("feedback_period", feedback_period_);

        const pour::PlannerConfig d;
        auto_declare<double>("container_tolerance", d.container_tolerance);
        auto_declare<double>("verify_timeout", d.verify_timeout);
        auto_declare<double>("onset_grams", d.onset_grams);
        auto_declare<double>("approach_sec", d.approach_sec);
        auto_declare<double>("kp_tilt", d.kp_tilt);
        auto_declare<double>("max_tilt_lead", d.max_tilt_lead);
        auto_declare<double>("flow_deadband", d.flow_deadband);
        auto_declare<double>("no_flow_epsilon", d.no_flow_epsilon);
        auto_declare<double>("park_check_sec", d.park_check_sec);
        auto_declare<double>("park_drip_grams", d.park_drip_grams);
        auto_declare<double>("stop_margin_factor", d.stop_margin_factor);
        auto_declare<double>("settle_timeout", d.settle_timeout);
        auto_declare<double>("stall_timeout", d.stall_timeout);
        auto_declare<double>("trim_undershoot", d.trim_undershoot);
        auto_declare<int>("max_trim_pulses", d.max_trim_pulses);
        auto_declare<double>("retract_margin", d.retract_margin);
        auto_declare<double>("trim_detect_grams", d.trim_detect_grams);
        auto_declare<double>("trim_creep_fraction", d.trim_creep_fraction);
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

        // ---- where the lip goes: `joint` or `measured` ----
        auto_declare<std::string>("pour_geometry", "joint");
        auto_declare<std::string>("geometry.vessel_pose_topic", "");
        auto_declare<std::string>("geometry.vessel_pose_frame", vessel_pose_frame_);
        auto_declare<double>("geometry.vessel_settle_sec", vessel_settle_sec_);
        auto_declare<double>("geometry.vessel_pose_max_age", vessel_pose_max_age_);
        auto_declare<double>("geometry.vessel_pose_timeout", vessel_pose_timeout_);
        auto_declare<double>("geometry.vessel_radius", vessel_.radius);
        auto_declare<double>("geometry.vessel_height", vessel_.height);
        // No defaults that would pass for measurements: where the marker is
        // stuck and where the receiver stands are facts about this bench. NaN,
        // not zero, because zero is a real answer (see configure).
        auto_declare<double>("geometry.tag_radius", std::numeric_limits<double>::quiet_NaN());
        auto_declare<double>("geometry.tag_height", std::numeric_limits<double>::quiet_NaN());
        auto_declare<std::vector<double>>("geometry.receiver_rim_center", {});
        auto_declare<double>("geometry.receiver_radius", receiver_.radius);
        const pour::LipPathConfig ld;
        auto_declare<double>("geometry.lip_clearance", ld.clearance);
        auto_declare<double>("geometry.lip_max_height", ld.max_height);
        auto_declare<double>("geometry.lip_height", ld.lip_height);
        auto_declare<double>("geometry.lip_inset", ld.inset);
        auto_declare<double>("geometry.lip_gap", ld.gap);
        auto_declare<double>("geometry.landing_margin", ld.landing_margin);
        auto_declare<double>("geometry.landing_by_tilt", ld.landing_by_tilt);
        auto_declare<double>("geometry.max_align_distance", ld.max_align_distance);
        auto_declare<double>("geometry.max_axis_elevation", ld.max_axis_elevation);
        auto_declare<double>("geometry.max_centering_error", ld.max_centering_error);
        auto_declare<double>("geometry.max_tag_distance", ld.max_tag_distance);
        auto_declare<std::vector<double>>("geometry.closing_axis_in_ee", {1.0, 0.0, 0.0});
        auto_declare<std::vector<double>>("geometry.approach_axis_in_ee", {0.0, 0.0, 1.0});
        auto_declare<double>("geometry.grasp_depth", ld.grasp_depth);
        auto_declare<double>("geometry.max_depth_error", ld.max_depth_error);
        auto_declare<double>("geometry.max_carry_lean", max_carry_lean_);
        auto_declare<std::vector<double>>("geometry.hand_boxes", {});
        auto_declare<double>("geometry.align_speed", align_speed_);
        auto_declare<double>("geometry.align_accel", align_accel_);
        auto_declare<double>("geometry.max_lip_speed", max_lip_speed_);
        auto_declare<double>("geometry.ik_lambda", ik_lambda_);
        auto_declare<double>("geometry.ik_tolerance", ik_tolerance_);
        auto_declare<double>("geometry.ik_rot_tolerance", ik_rot_tolerance_);
        auto_declare<double>("geometry.ik_fail_sec", ik_fail_sec_);

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
    tilt_accel_ = node->get_parameter("tilt_accel").as_double();
    feedback_period_ = node->get_parameter("feedback_period").as_double();

    planner_config_.container_tolerance = node->get_parameter("container_tolerance").as_double();
    planner_config_.verify_timeout = node->get_parameter("verify_timeout").as_double();
    planner_config_.onset_grams = node->get_parameter("onset_grams").as_double();
    planner_config_.approach_sec = node->get_parameter("approach_sec").as_double();
    planner_config_.kp_tilt = node->get_parameter("kp_tilt").as_double();
    planner_config_.max_tilt_lead = node->get_parameter("max_tilt_lead").as_double();
    planner_config_.flow_deadband = node->get_parameter("flow_deadband").as_double();
    planner_config_.no_flow_epsilon = node->get_parameter("no_flow_epsilon").as_double();
    planner_config_.park_check_sec = node->get_parameter("park_check_sec").as_double();
    planner_config_.park_drip_grams = node->get_parameter("park_drip_grams").as_double();
    planner_config_.stop_margin_factor = node->get_parameter("stop_margin_factor").as_double();
    planner_config_.settle_timeout = node->get_parameter("settle_timeout").as_double();
    planner_config_.stall_timeout = node->get_parameter("stall_timeout").as_double();
    planner_config_.trim_undershoot = node->get_parameter("trim_undershoot").as_double();
    planner_config_.max_trim_pulses = static_cast<int>(node->get_parameter("max_trim_pulses").as_int());
    planner_config_.retract_margin = node->get_parameter("retract_margin").as_double();
    planner_config_.trim_detect_grams = node->get_parameter("trim_detect_grams").as_double();
    planner_config_.trim_creep_fraction = node->get_parameter("trim_creep_fraction").as_double();
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
        {"tilt_accel", tilt_accel_},
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

    const std::string geometry = node->get_parameter("pour_geometry").as_string();
    if (geometry == "joint") {
        measured_geometry_ = false;
    } else if (geometry == "measured") {
        measured_geometry_ = true;
        if (!assign_geometry_parameters()) {
            return false;
        }
    } else {
        RCLCPP_ERROR(logger,
            "pour_geometry must be 'joint' or 'measured' (got '%s'): turn the pour joint alone, "
            "or measure where the vessel sits in the gripper and tip it about its lip",
            geometry.c_str());
        return false;
    }
    return true;
}

bool PouringController::assign_geometry_parameters()
{
    auto node = get_node();
    auto logger = node->get_logger();

    const auto vec3 = [&](const char * name, const char * meaning, Eigen::Vector3d & out) {
        const auto v = node->get_parameter(name).as_double_array();
        if (v.size() != 3 || !std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); })) {
            RCLCPP_ERROR(logger, "%s must be three finite numbers: %s", name, meaning);
            return false;
        }
        out = Eigen::Vector3d(v[0], v[1], v[2]);
        return true;
    };

    vessel_pose_topic_ = node->get_parameter("geometry.vessel_pose_topic").as_string();
    vessel_pose_frame_ = node->get_parameter("geometry.vessel_pose_frame").as_string();
    vessel_settle_sec_ = node->get_parameter("geometry.vessel_settle_sec").as_double();
    vessel_pose_max_age_ = node->get_parameter("geometry.vessel_pose_max_age").as_double();
    vessel_pose_timeout_ = node->get_parameter("geometry.vessel_pose_timeout").as_double();
    vessel_.radius = node->get_parameter("geometry.vessel_radius").as_double();
    vessel_.height = node->get_parameter("geometry.vessel_height").as_double();
    receiver_.radius = node->get_parameter("geometry.receiver_radius").as_double();
    lip_config_.clearance = node->get_parameter("geometry.lip_clearance").as_double();
    lip_config_.max_height = node->get_parameter("geometry.lip_max_height").as_double();
    lip_config_.lip_height = node->get_parameter("geometry.lip_height").as_double();
    lip_config_.inset = node->get_parameter("geometry.lip_inset").as_double();
    lip_config_.gap = node->get_parameter("geometry.lip_gap").as_double();
    lip_config_.landing_margin = node->get_parameter("geometry.landing_margin").as_double();
    lip_config_.landing_by_tilt = node->get_parameter("geometry.landing_by_tilt").as_double();
    lip_config_.max_align_distance = node->get_parameter("geometry.max_align_distance").as_double();
    lip_config_.max_axis_elevation = node->get_parameter("geometry.max_axis_elevation").as_double();
    lip_config_.max_centering_error = node->get_parameter("geometry.max_centering_error").as_double();
    lip_config_.max_tag_distance = node->get_parameter("geometry.max_tag_distance").as_double();
    align_speed_ = node->get_parameter("geometry.align_speed").as_double();
    align_accel_ = node->get_parameter("geometry.align_accel").as_double();
    max_lip_speed_ = node->get_parameter("geometry.max_lip_speed").as_double();
    ik_lambda_ = node->get_parameter("geometry.ik_lambda").as_double();
    ik_tolerance_ = node->get_parameter("geometry.ik_tolerance").as_double();
    ik_rot_tolerance_ = node->get_parameter("geometry.ik_rot_tolerance").as_double();
    ik_fail_sec_ = node->get_parameter("geometry.ik_fail_sec").as_double();

    if (vessel_pose_topic_.empty() || vessel_pose_frame_.empty()) {
        RCLCPP_ERROR(logger,
            "geometry.vessel_pose_topic and geometry.vessel_pose_frame must be set under "
            "pour_geometry: measured -- the PoseStamped of the held vessel's marker, and the "
            "frame it has to be in (the arm's base; nothing here transforms it)");
        return false;
    }
    vessel_.tag_radius = node->get_parameter("geometry.tag_radius").as_double();
    vessel_.tag_height = node->get_parameter("geometry.tag_height").as_double();
    lip_config_.grasp_depth = node->get_parameter("geometry.grasp_depth").as_double();
    lip_config_.max_depth_error = node->get_parameter("geometry.max_depth_error").as_double();
    max_carry_lean_ = node->get_parameter("geometry.max_carry_lean").as_double();
    // Unset is NaN. Zero is an answer: the pose already IS a point on the
    // vessel's axis, because the object table applied a known offset in the
    // marker's own yaw -- which, unlike a bare radius, fixes the depth along
    // the jaws even for a marker standing off to one side of them.
    if (!std::isfinite(vessel_.tag_radius) || vessel_.tag_radius < 0.0 ||
        !std::isfinite(vessel_.tag_height)) {
        RCLCPP_ERROR(logger,
            "geometry.tag_radius and geometry.tag_height must be measured: how far the "
            "pose's point sits from the held vessel's axis (0 when the object table puts it "
            "on the axis), and how high above its bottom (got %f, %f). Which side of the "
            "vessel it is on is not needed",
            vessel_.tag_radius, vessel_.tag_height);
        return false;
    }
    if (!std::isfinite(max_carry_lean_) || max_carry_lean_ <= 0.0) {
        RCLCPP_ERROR(logger, "geometry.max_carry_lean must be positive (got %f)", max_carry_lean_);
        return false;
    }
    if (!vec3("geometry.approach_axis_in_ee",
              "the direction the jaws reach along, in the EE frame", lip_config_.approach_axis_in_ee) ||
        !vec3("geometry.receiver_rim_center",
              "the centre of the receiver's rim in the arm's base frame -- the scale's centre, "
              "at the rim's height", receiver_.rim_center) ||
        !vec3("geometry.closing_axis_in_ee", "the direction the jaws close along, in the EE frame",
              lip_config_.closing_axis_in_ee)) {
        return false;
    }
    const auto boxes = node->get_parameter("geometry.hand_boxes").as_double_array();
    if (boxes.size() % 6 != 0) {
        RCLCPP_ERROR(logger,
            "geometry.hand_boxes must be a flat list of six numbers per box (min x y z, max x y "
            "z, EE frame); got %zu numbers", boxes.size());
        return false;
    }
    lip_config_.hand_boxes.clear();
    for (std::size_t i = 0; i < boxes.size(); i += 6) {
        pour::HandBox box;
        box.min = Eigen::Vector3d(boxes[i], boxes[i + 1], boxes[i + 2]);
        box.max = Eigen::Vector3d(boxes[i + 3], boxes[i + 4], boxes[i + 5]);
        lip_config_.hand_boxes.push_back(box);
    }

    const std::pair<const char *, double> positives[] = {
        {"geometry.vessel_settle_sec", vessel_settle_sec_},
        {"geometry.vessel_pose_max_age", vessel_pose_max_age_},
        {"geometry.vessel_pose_timeout", vessel_pose_timeout_},
        {"geometry.vessel_radius", vessel_.radius},
        {"geometry.vessel_height", vessel_.height},
        {"geometry.receiver_radius", receiver_.radius},
        {"geometry.align_speed", align_speed_},
        {"geometry.align_accel", align_accel_},
        {"geometry.max_lip_speed", max_lip_speed_},
        {"geometry.ik_lambda", ik_lambda_},
        {"geometry.ik_tolerance", ik_tolerance_},
        {"geometry.ik_rot_tolerance", ik_rot_tolerance_},
        {"geometry.ik_fail_sec", ik_fail_sec_},
    };
    for (const auto & [name, value] : positives) {
        if (!std::isfinite(value) || value <= 0.0) {
            RCLCPP_ERROR(logger, "%s must be positive (got %f)", name, value);
            return false;
        }
    }
    // The tolerance is how far off the path the arm may be and still count as
    // on it. The lip path keeps `gap` from the receiver, so a tolerance that is
    // not well inside it spends the gap on tracking error.
    if (ik_tolerance_ >= lip_config_.gap) {
        RCLCPP_ERROR(logger,
            "geometry.ik_tolerance (%f m) must be under geometry.lip_gap (%f m): the arm "
            "would be allowed off the path by more than the clearance the path keeps",
            ik_tolerance_, lip_config_.gap);
        return false;
    }
    std::string why;
    if (!lip_config_.validate(why)) {
        RCLCPP_ERROR(logger, "geometry rejected: %s", why.c_str());
        return false;
    }
    if (lip_config_.hand_boxes.empty()) {
        RCLCPP_WARN(logger,
            "geometry.hand_boxes is empty, so the lip path keeps only the vessel itself out of "
            "the receiver. Under the side grasp the pour tips the vessel toward a jaw, which "
            "hangs just below the lip and past it: declare the jaws");
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

    vessel_sub_.reset();
    if (measured_geometry_) {
        vessel_buffer_.writeFromNonRT(VesselSample{});
        vessel_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
            vessel_pose_topic_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                VesselSample sample;
                // Arrival, on this node's clock, for the same reason as the
                // scale -- and because what invalidates a pose is the arm
                // moving, which is timed on this clock too.
                sample.stamp = get_node()->now().seconds();
                sample.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                                                  msg->pose.position.z);
                sample.frame_ok = msg->header.frame_id == vessel_pose_frame_;
                sample.finite = sample.position.allFinite();
                vessel_buffer_.writeFromNonRT(sample);
            });
    }

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
        "from %s, outlier bound %.1f g/sample, geometry '%s'%s%s",
        law_->name(), joint_names_[pour_index_].c_str(), pour_direction_, scale_topic_.c_str(),
        filter_config.max_step_grams, measured_geometry_ ? "measured" : "joint",
        measured_geometry_ ? ", vessel from " : "",
        measured_geometry_ ? vessel_pose_topic_.c_str() : "");
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
    // Whatever arrived before now was measured under another controller, which
    // may have been moving the arm.
    vessel_buffer_.writeFromNonRT(VesselSample{});
    quiet_since_ = get_node()->now().seconds();
    law_started_ = false;
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
    if (q_ref_.size() == q_cmd.size() && (q_cmd - q_ref_).cwiseAbs().maxCoeff() > 1e-9) {
        quiet_since_ = now_;
    }
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
    const pour::PourReport none;
    const auto & report = law_started_ ? law_->report() : none;
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

pour::PourRequest PouringController::make_request() const
{
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
    return request;
}

bool PouringController::start_goal(double now)
{
    const int asked = action_server_->bounds().pour_direction;
    goal_direction_ = asked != 0 ? static_cast<double>(asked) : pour_direction_;
    if (asked != 0 && goal_direction_ != pour_direction_) {
        RCLCPP_INFO(get_node()->get_logger(),
            "This goal pours with direction %+.0f, not the configured %+.0f",
            goal_direction_, pour_direction_);
    }

    // How the goal says to tip: the axis the EE turns about from here to the
    // reference configuration, in this controller's own kinematics.
    reference_axis_valid_ = false;
    const auto & reference = action_server_->bounds().pour_reference_joints;
    if (!reference.empty()) {
        Eigen::VectorXd q_now = state_.q;
        q_now.head(num_dof_) = q_ref_;
        Eigen::VectorXd q_deep = state_.q;
        for (int i = 0; i < num_dof_; ++i) {
            q_deep(i) = reference[i];
        }
        pinocchio::SE3 H_now, H_deep;
        Eigen::MatrixXd J_now, J_unused;
        compute_arm_kinematics(q_now, H_now, J_now);
        compute_arm_kinematics(q_deep, H_deep, J_unused);
        const Eigen::Vector3d turn =
            pinocchio::log3(H_deep.rotation() * H_now.rotation().transpose());
        const double angle = turn.norm();
        // Less than this is not a pour; its axis would be noise.
        constexpr double kMinReferenceTilt = 0.2;
        if (!std::isfinite(angle) || angle < kMinReferenceTilt) {
            std::ostringstream os;
            os << "the pour was not started: its reference configuration turns the EE only "
               << std::fixed << std::setprecision(3) << angle << " rad from where the goal "
                  "starts, which does not show a pour";
            abort_before_motion(os.str());
            return false;
        }
        const Eigen::Vector3d axis_world = turn / angle;
        if (measured_geometry_) {
            reference_axis_ee_ = H_now.rotation().transpose() * axis_world;
            reference_axis_valid_ = true;
        } else {
            // The pour joint alone can only tip about its own axis.
            const Eigen::Vector3d joint_axis =
                (H_now.rotation() * J_now.col(pour_index_).tail<3>()).normalized();
            const double c = joint_axis.dot(axis_world);
            if (std::abs(c) < 0.8) {
                std::ostringstream os;
                os << "the pour was not started: it tips the vessel about an axis "
                   << std::fixed << std::setprecision(2)
                   << std::acos(std::min(1.0, std::abs(c))) << " rad from the pour joint's, "
                      "which turning that joint alone cannot do. pour_geometry: measured can";
                abort_before_motion(os.str());
                return false;
            }
            goal_direction_ = c > 0.0 ? 1.0 : -1.0;
        }
    }
    theta_start_ = q_ref_(pour_index_);
    tilt_ = 0.0;
    tilt_ramp_.reset();
    law_target_tilt_ = std::numeric_limits<double>::quiet_NaN();
    peak_tilt_ = 0.0;
    elapsed_ = 0.0;
    feedback_accumulator_ = 0.0;
    last_phase_ = 0;
    filter_.reset();
    last_pushed_stamp_ = 0.0;
    law_started_ = false;
    pending_success_ = false;
    pending_reason_.clear();

    if (measured_geometry_) {
        phase_ = Phase::Measure;
        measure_started_ = now;
        q_start_ = q_ref_;
        align_s_ = 0.0;
        align_ramp_.reset();
        inset_cmd_ = 0.0;
        ik_lagging_ = false;
        ik_bad_since_ = -1.0;
        return true;
    }
    start_pour(now);
    return true;
}

void PouringController::start_pour(double now)
{
    pour::PourRequest request = make_request();
    if (measured_geometry_) {
        // Past this the lip would have to leave the mouth to keep the jaws off
        // the rim, and the stream would leave with it.
        request.max_tilt = std::min(request.max_tilt, lip_path_.last_landing_tilt());
    }
    tilt_bound_ = request.max_tilt;
    law_->begin(request, now);
    law_started_ = true;
    phase_ = Phase::Pouring;
}

double PouringController::step_law(double now)
{
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
    law_target_tilt_ = cmd.target_tilt;
    if (cmd.finished) {
        begin_untilt(cmd.success, cmd.message);
        return 0.0;
    }
    return cmd.tilt_rate;
}

double PouringController::shaped_law_step(double rate, double dt)
{
    if (std::isfinite(law_target_tilt_)) {
        return tilt_ramp_.reach(tilt_, law_target_tilt_, std::abs(rate), tilt_accel_, dt);
    }
    return tilt_ramp_.follow(rate, tilt_accel_, dt);
}

void PouringController::publish_feedback_if_due(double dt)
{
    feedback_accumulator_ += dt;
    if (feedback_accumulator_ >= feedback_period_) {
        feedback_accumulator_ -= feedback_period_;
        const double poured = law_started_ ? last_grams_ - law_->baseline_grams() : 0.0;
        action_server_->publish_feedback(poured, tilt_, elapsed_, last_flow_, last_phase_);
    }
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
    now_ = time.seconds();

    const bool running = action_server_ && action_server_->is_running();
    const bool returning =
        phase_ == Phase::Untilt || phase_ == Phase::Unalign || phase_ == Phase::Return;
    if (!running && !returning) {
        // Idle: freeze the reference. Re-deriving a hold from the measured
        // position would feed servo droop back into the command.
        phase_ = Phase::Idle;
        hold_reference();
        return controller_interface::return_type::OK;
    }

    const double dt = nominal_period(period);
    const double now = now_;

    // Take whatever the subscription left, but only once. Re-pushing the same
    // sample every cycle would make the least-squares fit see 125 identical
    // points per reading and report a flow rate of zero throughout a pour.
    const pour::ScaleFilter::Sample sample = *scale_buffer_.readFromRT();
    if (sample.stamp > last_pushed_stamp_) {
        last_pushed_stamp_ = sample.stamp;
        filter_.push(sample);
    }

    if (phase_ == Phase::Idle && !start_goal(now)) {
        return controller_interface::return_type::OK;
    }

    elapsed_ += dt;
    last_grams_ = filter_.grams();
    last_flow_ = filter_.flow_rate();

    if (measured_geometry_) {
        return update_measured(now, dt);
    }

    const double tilt_before = tilt_;
    double step = 0.0;

    if (phase_ == Phase::Pouring) {
        const double rate = step_law(now);
        if (phase_ == Phase::Pouring) {
            step = shaped_law_step(rate, dt);
        }
    }

    if (phase_ == Phase::Untilt) {
        last_phase_ = static_cast<std::uint8_t>(PourPhase::Done);
        const double bound = std::abs(goal_direction_) * max_delta_q_ / std::max(dt, 1e-9);
        const double rate = std::min(law_->return_tilt_rate(), bound);
        step = tilt_ramp_.reach(tilt_, 0.0, rate, tilt_accel_, dt);
        if (tilt_ + step == 0.0) {
            tilt_ = 0.0;
            Eigen::VectorXd q_cmd = q_ref_;
            q_cmd(pour_index_) = theta_start_;
            clamp_to_joint_limits(q_cmd);
            write_command(q_cmd);
            finish();
            return controller_interface::return_type::OK;
        }
    }

    tilt_ += step;

    // Tilt range, measured from the attitude the pour started in, and BOTH ends
    // of it. The laws have their own bounds and normally stay inside; this is
    // the one that holds if a law is ever wrong -- a park lowered too far, or a
    // law driving a negative error backwards through upright. Bounding only the
    // pour side (what this did after the rewrite) left the other direction
    // limited by nothing short of the joint limit.
    if (phase_ == Phase::Pouring) {
        tilt_ = std::clamp(tilt_, -max_back_tilt_, tilt_bound_);
    }
    peak_tilt_ = std::max(peak_tilt_, std::abs(tilt_));

    Eigen::VectorXd q_cmd = q_ref_;
    q_cmd(pour_index_) = theta_start_ + goal_direction_ * tilt_;
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
    // asked to reach -- nor the ramp accelerating from a speed it never had.
    tilt_ = goal_direction_ * (q_cmd(pour_index_) - theta_start_);
    tilt_ramp_.observed(tilt_ - tilt_before, dt);

    publish_feedback_if_due(dt);
    return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------- measured

bool PouringController::vessel_sample_usable(const VesselSample & sample, double now) const
{
    return sample.stamp > 0.0 && sample.frame_ok && sample.finite &&
           now - sample.stamp <= vessel_pose_max_age_ &&
           sample.stamp >= quiet_since_ + vessel_settle_sec_;
}

std::string PouringController::vessel_sample_problem(const VesselSample & sample, double now) const
{
    std::ostringstream os;
    if (sample.stamp <= 0.0) {
        os << "nothing has arrived on " << vessel_pose_topic_ << " since the controller was "
           << "activated. Is cho_object_pose running with the held vessel's marker in its "
              "table, and can a side camera see the marker from where the vessel is held?";
    } else if (!sample.frame_ok) {
        os << "the poses on " << vessel_pose_topic_ << " are not in '" << vessel_pose_frame_
           << "', and nothing here transforms them";
    } else if (!sample.finite) {
        os << "the latest pose on " << vessel_pose_topic_ << " is not finite";
    } else if (now - sample.stamp > vessel_pose_max_age_) {
        os << "the latest pose on " << vessel_pose_topic_ << " is " << std::fixed
           << std::setprecision(1) << now - sample.stamp << " s old: the marker has gone out "
              "of view, or the pose node has stopped trusting it";
    } else if (sample.stamp < quiet_since_ + vessel_settle_sec_) {
        os << "every pose so far may have been averaged over the arm's last move; none has "
              "arrived vessel_settle_sec after it stopped";
    }
    return os.str();
}

void PouringController::abort_before_motion(const std::string & reason)
{
    RCLCPP_WARN(get_node()->get_logger(), "Pour refused before anything moved: %s",
                reason.c_str());
    action_server_->abort_active_goal(reason, 0.0, 0.0, elapsed_);
    phase_ = Phase::Idle;
    hold_reference();
}

void PouringController::fail_geometric(const std::string & reason, Phase via)
{
    if (phase_ == Phase::Unalign || phase_ == Phase::Return) {
        // Already on the way back with a reason of its own. Only a failure to
        // follow the way back changes anything: it goes to joint space.
        if (via == Phase::Return) {
            phase_ = Phase::Return;
        }
        return;
    }
    const bool poured = phase_ == Phase::Untilt && pending_success_;
    pending_reason_ = poured ? "the pour reached its target, but then " + reason : reason;
    pending_success_ = false;
    RCLCPP_WARN(get_node()->get_logger(),
        "Pour ending without reaching the target (%s); returning the arm to where the goal "
        "started first", pending_reason_.c_str());
    ik_lagging_ = false;
    ik_bad_since_ = -1.0;
    phase_ = via;
}

bool PouringController::track(const Eigen::Isometry3d & target, double now, std::string & why)
{
    Eigen::VectorXd q_full = state_.q;
    q_full.head(num_dof_) = q_ref_;
    pinocchio::SE3 H;
    Eigen::MatrixXd J;
    compute_arm_kinematics(q_full, H, J);

    // The task point is the LIP, not the EE origin. The origin sits ~0.3 m
    // from it, so a rotation lag the origin barely shows moves the lip by
    // millimetres -- 6 mm at the 0.02 rad the rotation tolerance allows,
    // measured against the riser recordings, which pour by turning j4 1.4x as
    // fast as the vessel tips and so run into max_delta_q.
    const Eigen::Vector3d r = lip_path_.lip_in_ee();
    const auto point_of = [&r](const Eigen::Matrix3d & R, const Eigen::Vector3d & p) {
        return Eigen::Vector3d(p + R * r);
    };
    // Local-frame task error against the COMMAND, like the task-space IK: the
    // measured joints never enter it, so servo noise is not fed back.
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = H.rotation().transpose() *
        (point_of(target.linear(), target.translation()) - point_of(H.rotation(), H.translation()));
    error.tail<3>() = pinocchio::log3(H.rotation().transpose() * target.linear());
    // A point r off the frame moves at v + w x r = v - [r]x w.
    Eigen::Matrix3d skew_r;
    skew_r << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
    Eigen::MatrixXd J_task = J;
    J_task.topRows<3>() -= skew_r * J.bottomRows<3>();
    Eigen::Matrix<double, 6, 6> JJt = J_task * J_task.transpose();
    JJt.diagonal().array() += ik_lambda_ * ik_lambda_;
    Eigen::VectorXd dq = J_task.transpose() * JJt.ldlt().solve(error);
    if (!dq.allFinite()) {
        why = "the lip path's IK solve went non-finite";
        return false;
    }
    // Scaled as a whole, not clamped joint by joint: clamping one joint and
    // not the others bends the step off the path it was solved for.
    const double biggest = dq.cwiseAbs().maxCoeff();
    if (biggest > max_delta_q_) {
        dq *= max_delta_q_ / biggest;
    }
    Eigen::VectorXd q_cmd = q_ref_ + dq;
    clamp_to_joint_limits(q_cmd);

    q_full.head(num_dof_) = q_cmd;
    compute_arm_kinematics(q_full, H, J);
    const double pos_err = (point_of(target.linear(), target.translation()) -
                            point_of(H.rotation(), H.translation())).norm();
    const double rot_err =
        pinocchio::log3(H.rotation().transpose() * target.linear()).norm();
    if (!q_cmd.allFinite() || !std::isfinite(pos_err) || !std::isfinite(rot_err)) {
        why = "the lip path's IK produced a non-finite command";
        return false;
    }
    write_command(q_cmd);

    ik_lagging_ = pos_err > ik_tolerance_ || rot_err > ik_rot_tolerance_;
    if (!ik_lagging_) {
        ik_bad_since_ = -1.0;
        return true;
    }
    if (ik_bad_since_ < 0.0) {
        ik_bad_since_ = now;
    }
    if (now - ik_bad_since_ > ik_fail_sec_) {
        std::ostringstream os;
        os << "the arm could not follow the lip path: " << std::fixed << std::setprecision(1)
           << pos_err * 1000.0 << " mm and " << std::setprecision(3) << rot_err
           << " rad off it for longer than ik_fail_sec. A joint limit or a singularity is in "
              "the way of tipping the vessel about its lip from this pose";
        why = os.str();
        return false;
    }
    return true;
}

void PouringController::advance_tilt(double proposed, double dt)
{
    // Nothing advances while the arm is behind the path: the next target would
    // only be further from where it is.
    if (ik_lagging_) {
        return;
    }
    const double goal = lip_path_.inset_limit(proposed);
    const double step = max_lip_speed_ * dt;
    if (goal < inset_cmd_ - step) {
        // At the proposed tilt the lip has to be further out than it is. Back
        // it out first and tip once it is there: tipping first would carry the
        // jaw or the wall into the receiver while the lip caught up.
        inset_cmd_ -= step;
    } else {
        inset_cmd_ = std::min(goal, inset_cmd_ + step);
        tilt_ = proposed;
    }
    peak_tilt_ = std::max(peak_tilt_, std::abs(tilt_));
}

controller_interface::return_type PouringController::update_measured(double now, double dt)
{
    std::string why;

    if (phase_ == Phase::Measure) {
        hold_reference();
        last_phase_ = static_cast<std::uint8_t>(PourPhase::Verify);
        if (action_server_->is_canceling()) {
            abort_before_motion("the goal was cancelled before the pour began");
            return controller_interface::return_type::OK;
        }
        const auto & bounds = action_server_->bounds();
        const bool measured_earlier = !bounds.grasp_joints.empty();
        VesselSample sample = *vessel_buffer_.readFromRT();
        if (measured_earlier) {
            // The goal carries a grasp measured before it: the marker's
            // base-frame centre and the joints the arm was in. The grasp is a
            // rigid offset, so the marker's place in the EE frame then is its
            // place now -- carried to the present by this controller's own
            // kinematics, the same model the lip path is followed with.
            Eigen::VectorXd q_then = state_.q;
            for (int i = 0; i < num_dof_; ++i) {
                q_then(i) = bounds.grasp_joints[i];
            }
            pinocchio::SE3 H_then, H_now;
            Eigen::MatrixXd J_unused;
            compute_arm_kinematics(q_then, H_then, J_unused);
            Eigen::VectorXd q_now = state_.q;
            q_now.head(num_dof_) = q_ref_;
            compute_arm_kinematics(q_now, H_now, J_unused);
            const Eigen::Vector3d marker_then(bounds.grasp_marker[0], bounds.grasp_marker[1],
                                              bounds.grasp_marker[2]);
            // The vessel was upright when it was measured -- standing on the
            // bench in the jaws. Whatever the EE has turned through since, it
            // has turned the vessel through too.
            const Eigen::Vector3d axis_now =
                H_now.rotation() * H_then.rotation().transpose() * Eigen::Vector3d::UnitZ();
            const double lean = std::acos(std::clamp(axis_now.z(), -1.0, 1.0));
            if (lean > max_carry_lean_) {
                std::ostringstream os;
                os << "the pour was not started: the vessel was measured upright at its grasp, "
                   << "and the carry since has tipped it " << std::fixed << std::setprecision(3)
                   << lean << " rad (limit " << max_carry_lean_ << " rad). The lip path "
                   << "assumes it hangs upright when the pour starts";
                abort_before_motion(os.str());
                return controller_interface::return_type::OK;
            }
            sample.position = to_isometry(H_now) * (to_isometry(H_then).inverse() * marker_then);
            sample.finite = sample.position.allFinite();
            sample.frame_ok = true;
            sample.stamp = now;
            if (!sample.finite) {
                abort_before_motion("the pour was not started: the grasp measured before the "
                                    "goal does not give a finite marker position");
                return controller_interface::return_type::OK;
            }
        } else if (!vessel_sample_usable(sample, now)) {
            if (now - measure_started_ > vessel_pose_timeout_) {
                std::ostringstream os;
                os << "no usable pose of the held vessel within vessel_pose_timeout ("
                   << vessel_pose_timeout_ << " s): " << vessel_sample_problem(sample, now);
                abort_before_motion(os.str());
                return controller_interface::return_type::OK;
            }
            publish_feedback_if_due(dt);
            return controller_interface::return_type::OK;
        }

        // The grasp, from the command the arm is holding and the marker.
        Eigen::VectorXd q_full = state_.q;
        q_full.head(num_dof_) = q_ref_;
        pinocchio::SE3 H;
        Eigen::MatrixXd J;
        compute_arm_kinematics(q_full, H, J);
        // The local Jacobian's angular column for a revolute joint is its axis
        // in the EE frame.
        const Eigen::Vector3d axis = reference_axis_valid_
            ? Eigen::Vector3d(H.rotation() * reference_axis_ee_)
            : Eigen::Vector3d(goal_direction_ * (H.rotation() * J.col(pour_index_).tail<3>()));
        if (!lip_path_.plan(to_isometry(H), axis, sample.position,
                            action_server_->bounds().max_tilt, vessel_, receiver_, lip_config_,
                            why)) {
            abort_before_motion("the pour was not started: " + why);
            return controller_interface::return_type::OK;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Pour geometry (%s): %s",
                    measured_earlier ? "grasp measured before the goal" : "marker measured now",
                    lip_path_.summary().c_str());
        align_s_ = 0.0;
        phase_ = Phase::Align;
    }

    if (phase_ == Phase::Align) {
        last_phase_ = static_cast<std::uint8_t>(PourPhase::Verify);
        if (action_server_->is_canceling()) {
            fail_geometric("the goal was cancelled before the pour began", Phase::Unalign);
        } else if (align_s_ >= lip_path_.align_length() && !ik_lagging_) {
            inset_cmd_ = lip_path_.inset_limit(0.0);
            start_pour(now);
        } else {
            if (!ik_lagging_) {
                align_s_ += align_ramp_.reach(align_s_, lip_path_.align_length(), align_speed_,
                                              align_accel_, dt);
            } else {
                align_ramp_.reset();
            }
            if (!track(lip_path_.ee_pose_aligning(align_s_), now, why)) {
                fail_geometric(why, Phase::Return);
            }
            publish_feedback_if_due(dt);
            return controller_interface::return_type::OK;
        }
    }

    if (phase_ == Phase::Pouring || phase_ == Phase::Untilt) {
        const double tilt_before = tilt_;
        double proposed = tilt_;
        if (phase_ == Phase::Pouring) {
            const double rate = step_law(now);
            if (phase_ == Phase::Pouring) {
                proposed = std::clamp(tilt_ + shaped_law_step(rate, dt), -max_back_tilt_,
                                      tilt_bound_);
            }
        }
        if (phase_ == Phase::Untilt) {
            last_phase_ = static_cast<std::uint8_t>(PourPhase::Done);
            const double rate = std::min(law_->return_tilt_rate(),
                                         max_delta_q_ / std::max(dt, 1e-9));
            proposed = tilt_ + tilt_ramp_.reach(tilt_, 0.0, rate, tilt_accel_, dt);
        }
        advance_tilt(proposed, dt);
        // advance_tilt holds the tilt back while the IK lags or the lip backs
        // out; the ramp carries on from what it actually did.
        tilt_ramp_.observed(tilt_ - tilt_before, dt);

        const bool home = phase_ == Phase::Untilt && tilt_ == 0.0 && !ik_lagging_ &&
                          inset_cmd_ >= lip_path_.inset_limit(0.0) - 1e-9;
        if (home) {
            align_s_ = lip_path_.align_length();
            align_ramp_.reset();
            phase_ = Phase::Unalign;
        } else {
            if (!track(lip_path_.ee_pose(tilt_, inset_cmd_), now, why)) {
                fail_geometric(why, Phase::Return);
            }
            publish_feedback_if_due(dt);
            return controller_interface::return_type::OK;
        }
    }

    if (phase_ == Phase::Unalign) {
        last_phase_ = static_cast<std::uint8_t>(PourPhase::Done);
        if (align_s_ <= 0.0 && !ik_lagging_) {
            phase_ = Phase::Return;
        } else {
            if (!ik_lagging_) {
                align_s_ += align_ramp_.reach(align_s_, 0.0, align_speed_, align_accel_, dt);
            } else {
                align_ramp_.reset();
            }
            if (!track(lip_path_.ee_pose_aligning(align_s_), now, why)) {
                fail_geometric(why, Phase::Return);
            }
            publish_feedback_if_due(dt);
            return controller_interface::return_type::OK;
        }
    }

    if (phase_ == Phase::Return) {
        // Joint space, to the configuration the goal started in. After a clean
        // unalign this is the IK's residual, a fraction of a milliradian; after
        // a failure to follow the path it is the whole way back.
        last_phase_ = static_cast<std::uint8_t>(PourPhase::Done);
        Eigen::VectorXd delta = q_start_ - q_ref_;
        if (delta.cwiseAbs().maxCoeff() <= 1e-9) {
            write_command(q_start_);
            finish();
            return controller_interface::return_type::OK;
        }
        delta = delta.array().max(-max_delta_q_).min(max_delta_q_);
        write_command(q_ref_ + delta);
        publish_feedback_if_due(dt);
        return controller_interface::return_type::OK;
    }

    hold_reference();
    return controller_interface::return_type::OK;
}

} // namespace fr5
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::fr5::PouringController,
                       controller_interface::ControllerInterface)
