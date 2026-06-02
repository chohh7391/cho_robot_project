#include "cho_controller_ur/task_space_ik_controller.hpp"

namespace cho_controller {
namespace ur {

controller_interface::InterfaceConfiguration
TaskSpaceIKController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
    }
    return config;
}

CallbackReturn TaskSpaceIKController::on_init()
{
    if (URBaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<double>("lambda", 0.01);
        auto_declare<double>("max_delta_q", 0.02);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (!assign_parameters()) {
        return CallbackReturn::FAILURE;
    }
    if (URBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    action_server_ = std::make_shared<URTaskSpaceActionServer>(
        get_node(), "/controller_action_server/task_space_ik_controller", num_dof_);
    action_server_->init();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceIKController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)period;

    if (URBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    if (action_server_ && action_server_->is_running()) {
        action_server_->compute(time, state_);
        auto sample = action_server_->trajectory_->computeNext();
        state_.H_ee_des.translation() = sample.pos.head<3>();
        state_.H_ee_des.rotation() =
            Eigen::Map<const Eigen::Matrix3d>(sample.pos.segment<9>(3).data());
    } else {
        state_.H_ee_des = state_.H_ee_init;
    }

    // Isaac Lab style differential IK:
    // delta_q = J^T (J J^T + lambda^2 I)^-1 delta_x
    // q_cmd = q_current + delta_q
    const Eigen::MatrixXd J = state_.J_world.leftCols(num_dof_);

    Eigen::Matrix<double, 6, 1> delta_pose;
    delta_pose.head<3>() = state_.H_ee_des.translation() - state_.H_ee.translation();
    delta_pose.tail<3>() = pinocchio::log3(
        state_.H_ee_des.rotation() * state_.H_ee.rotation().transpose());

    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += lambda_ * lambda_;
    Eigen::VectorXd delta_q = J.transpose() * JJt.inverse() * delta_pose;

    Eigen::VectorXd q_cmd = state_.q.head(num_dof_) + delta_q;
    clip_position(q_cmd, max_delta_q_);

    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_cmd(i));
    }
    return controller_interface::return_type::OK;
}

bool TaskSpaceIKController::assign_parameters()
{
    lambda_ = get_node()->get_parameter("lambda").as_double();
    max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
    if (lambda_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "lambda must be positive");
        return false;
    }
    if (max_delta_q_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be positive");
        return false;
    }
    return true;
}

} // namespace ur
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::ur::TaskSpaceIKController,
                       controller_interface::ControllerInterface)
