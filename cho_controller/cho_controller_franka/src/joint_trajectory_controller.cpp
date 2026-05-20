#include "cho_controller_franka/joint_trajectory_controller.hpp"

#include <cmath>
#include <map>
#include <vector>


namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
JointTrajectoryController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (int i = 1; i <= num_dof_; ++i) {
        config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
}

CallbackReturn JointTrajectoryController::on_init()
{
    // Base class handles common parameter declarations
    if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    try {
        auto_declare<std::vector<double>>("kp_joint", {});
        auto_declare<std::vector<double>>("kd_joint", {});
        auto_declare<double>("traj_duration", 0.0);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
    auto kd_joint = get_node()->get_parameter("kd_joint").as_double_array();
    auto traj_duration = get_node()->get_parameter("traj_duration").as_double();

    if (kp_joint.empty() || kp_joint.size() != static_cast<uint>(num_dof_)) {
        RCLCPP_FATAL(get_node()->get_logger(), "Invalid kp_joint parameter");
        return CallbackReturn::FAILURE;
    }
    if (kd_joint.empty() || kd_joint.size() != static_cast<uint>(num_dof_)) {
        RCLCPP_FATAL(get_node()->get_logger(), "Invalid kd_joint parameter");
        return CallbackReturn::FAILURE;
    }

    for (int i = 0; i < num_dof_; ++i) {
        kp_joint_(i) = kp_joint.at(i);
        kd_joint_(i) = kd_joint.at(i);
    }
    traj_duration_ = traj_duration;
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    // Build trajectory parameters around the current joint config
    setup_trajectory_params();

    // Mark start time
    traj_start_time_ = get_node()->now();

    state_.q_arm_des = state_.q_arm_init;
    state_.v_arm_des = state_.v_arm_init;

    RCLCPP_INFO(get_node()->get_logger(),
                "JointTrajectoryController activated. Duration: %.1f s (0 = infinite)",
                traj_duration_);

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointTrajectoryController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // Elapsed time
    double tau = (time - traj_start_time_).seconds();
    if (tau < 0.0) tau = 0.0;
    
    bool is_finished = (traj_duration_ > 0.0 && tau >= traj_duration_);
    if (is_finished) {
        tau = traj_duration_;
    }

    // 2. 위치 (q_des) 계산
    const Vector7d q_des = compute_desired_q(tau);

    // 3. 속도 (dq_des) 계산
    Vector7d dq_des = Vector7d::Zero(); // 기본값 0
    for (int i = 0; i < num_dof_; ++i) {
        const double w   = 2.0 * M_PI * traj_f_hz_(i);
        const double sw  = std::sin(w * tau);
        const double cw  = std::cos(w * tau);
        dq_des(i) = traj_rho_(i) * w * cw - traj_delta_(i) * w * sw;
    }

    // PD + NLE compensation
    const Vector7d q_err  = q_des  - state_.q_arm;
    const Vector7d dq_err = dq_des - state_.v_arm;

    Vector7d tau_cmd = kp_joint_.cwiseProduct(q_err)
                     + kd_joint_.cwiseProduct(dq_err)
                     + state_.nle;

    clip_torque(tau_cmd);

    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(tau_cmd(i));
    }

    return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// setup_trajectory_params  (same table as original JointDataTrajectory)
// ---------------------------------------------------------------------------
void JointTrajectoryController::setup_trajectory_params()
{
    traj_f_hz_.setZero();
    traj_rho_.setZero();
    traj_delta_.setZero();

    struct Term { int k; double A; double phi; };
    const double f0 = 0.2; // base frequency [Hz]

    std::map<int, std::vector<Term>> table;
    table[1] = { {1, 0.10, 0.0},        {2, 0.05, M_PI / 2} };
    table[2] = { {2, 0.25, 0.0},        {1, 0.05, M_PI / 2} };
    table[3] = { {1, 0.20, 0.0},        {4, 0.05, M_PI / 2} };
    table[4] = { {2, 0.18, 0.0},        {4, 0.08, 0.0}       };
    table[5] = { {1, 0.30, 0.0}                               };
    table[6] = { {2, 0.10, 0.0},        {4, 0.07, M_PI / 2} };
    table[7] = { {1, 0.25, 0.0}, {2, 0.20, 0.0},
                 {2, 0.05, M_PI / 2}, {4, 0.08, M_PI / 2}   };

    for (int j = 1; j <= num_dof_; ++j) {
        auto it = table.find(j);
        if (it == table.end() || it->second.empty()) continue;

        // Pick the term with the largest amplitude
        const Term * best = &it->second[0];
        for (const auto & t : it->second) {
            if (t.A > best->A) best = &t;
        }

        const int idx     = j - 1;
        traj_f_hz_(idx)  = f0 * static_cast<double>(best->k);
        traj_rho_(idx)   = best->A * std::cos(best->phi);
        traj_delta_(idx) = best->A * std::sin(best->phi);
    }
}

// ---------------------------------------------------------------------------
// compute_desired_q
// ---------------------------------------------------------------------------
Vector7d JointTrajectoryController::compute_desired_q(double tau) const
{
    Vector7d q = state_.q_arm_init;
    for (int i = 0; i < num_dof_; ++i) {
        const double w  = 2.0 * M_PI * traj_f_hz_(i);
        const double sw = std::sin(w * tau);
        const double cw = std::cos(w * tau);
        q(i) += traj_rho_(i) * sw + traj_delta_(i) * cw;
    }
    return q;
}



} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointTrajectoryController,
                       controller_interface::ControllerInterface)
