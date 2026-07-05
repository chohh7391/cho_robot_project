#include "cho_controller_franka/robot_utils.hpp"
#include "cho_controller_franka/base_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace cho_controller::common::robot;
using namespace pinocchio;
using namespace std;

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
FrankaBaseController::command_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaBaseController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // arm
    for (int i = 1; i <= num_dof_; ++i) {
        config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
        config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
    }

    if (bringup_type_ != "real") {
        // gripper
        if (nq_ > num_dof_) {
            config.names.push_back(robot_type_ + "_finger_joint1" + "/position");
            config.names.push_back(robot_type_ + "_finger_joint1" + "/velocity");
        }
    }
    
    return config;
}

CallbackReturn FrankaBaseController::on_init()
{
    try {
        auto_declare<std::string>("bringup_type", "");
        auto_declare<std::string>("robot_type", "");
        auto_declare<std::string>("ee_name", "");
        auto_declare<double>("mass", 0.0);
        auto_declare<std::vector<double>>("center_of_mass", {0.0, 0.0, 0.0});
        auto_declare<std::vector<double>>("load_inertia", {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {

    bool has_local_urdf = false;

    if (!get_node()->has_parameter("robot_description")) {
        get_node()->declare_parameter<std::string>("robot_description", "");
    }
    
    robot_description_ = get_node()->get_parameter("robot_description").as_string();

    if (!robot_description_.empty()) {
        has_local_urdf = true;
        RCLCPP_INFO(get_node()->get_logger(), "Got robot_description from local parameter.");
    }

    if (!has_local_urdf) {
        RCLCPP_INFO(get_node()->get_logger(), "Local robot_description is empty. Requesting from robot_state_publisher...");
        
        auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
            get_node(), "robot_state_publisher");
        
        if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_node()->get_logger(), "Service robot_state_publisher not available");
            return CallbackReturn::FAILURE;
        }

        auto future = parameters_client->get_parameters({"robot_description"});
        auto result = future.get(); // Gazebo 환경에서는 데드락이 걸리지 않음
        
        if (result.empty() || result[0].value_to_string().empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description from robot_state_publisher");
            return CallbackReturn::FAILURE;
        }
        
        robot_description_ = result[0].value_to_string();
        RCLCPP_INFO(get_node()->get_logger(), "Got robot_description from robot_state_publisher.");
    }

    try {
        bringup_type_ = get_node()->get_parameter("bringup_type").as_string();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get bringup_type: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    robot_type_ = cho_controller::franka::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

    robot_ = std::make_shared<RobotWrapper>(robot_description_, true, false);
    model_ = robot_->model();
    data_ = pinocchio::Data(model_);
    kin_data_ = pinocchio::Data(model_);
    kin_v_zero_ = Eigen::VectorXd::Zero(model_.nv);
    kin_J_.setZero(6, model_.nv);

    nq_ = robot_->nq();
    nv_ = robot_->nv();
    na_ = robot_->na();

    // find End-Effector Frame ID
    ee_name_ = get_node()->get_parameter("ee_name").as_string();
    ee_id_ = model_.getFrameId(ee_name_);
    if (ee_id_ == model_.frames.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "End-effector frame '%s' does not exist in URDF!", ee_name_.c_str());
        return CallbackReturn::FAILURE;
    }

    state_.q.setZero(nq_);
    state_.v.setZero(nv_);
    state_.q_init.setZero(nq_);
    state_.v_init.setZero(nv_);
    state_.J.setZero(6, nv_);
    state_.J_world.setZero(6, nv_);

    pose_log_pub_ = get_node()->create_publisher<cho_interfaces::msg::PoseLog>("/log/ee_pose", 10);
    joint_log_pub_ = get_node()->create_publisher<cho_interfaces::msg::JointLog>("/log/joint_pos", 10);
    pose_log_rt_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>>(pose_log_pub_);
    joint_log_rt_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<cho_interfaces::msg::JointLog>>(joint_log_pub_);
    // Preallocate the joint-log vectors once so the per-cycle resize() is a no-op
    // (no realtime heap traffic).
    joint_log_rt_pub_->msg_.desired_state.position.resize(num_dof_);
    joint_log_rt_pub_->msg_.current_state.position.resize(num_dof_);
    joint_log_rt_pub_->msg_.current_state.velocity.resize(num_dof_);

    // gravity compensation parameter
    mass_ = get_node()->get_parameter("mass").as_double();
    auto com_vec = get_node()->get_parameter("center_of_mass").as_double_array();
    auto inertia_vec = get_node()->get_parameter("load_inertia").as_double_array();

    std::copy_n(com_vec.begin(), std::min(com_vec.size(), size_t(3)), center_of_mass_.begin());
    std::copy_n(inertia_vec.begin(), std::min(inertia_vec.size(), size_t(9)), load_inertia_.begin());

    RCLCPP_INFO(get_node()->get_logger(), "FrankaBaseController Configured successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  update_joint_states();
  compute_all_terms();
  state_.q_init = state_.q;
  state_.v_init = state_.v;
  state_.q_arm_init = state_.q_arm;
  state_.v_arm_init = state_.v_arm;
  if (nq_ > num_dof_) state_.q_gripper_init = state_.q_gripper;
  state_.H_ee_init = state_.H_ee;
  // 명령 기준점을 현재 측정값으로 초기화 (clip_position의 rate-limit 기준).
  state_.q_arm_ref = state_.q_arm;

  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaBaseController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {

    update_joint_states();
    compute_all_terms();

    // logging
    log_ee_pose();
    log_joint_pos();

    return controller_interface::return_type::OK;
}

void FrankaBaseController::update_joint_states()
{
    int num_interface;
    if (bringup_type_ == "real") {
        num_interface = num_dof_; // real franka hardware does not have finger interface
    } else {
        if (nq_ > num_dof_) {
            num_interface = num_dof_ + 1;
        } else {
            num_interface = num_dof_;
        }
    }
    // arm
    for (auto i = 0; i < num_interface; ++i) {
        const auto& position_interface = state_interfaces_.at(2 * i);
        const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

        assert(position_interface.get_interface_name() == "position");
        assert(velocity_interface.get_interface_name() == "velocity");

        state_.q(i) = position_interface.get_value();
        state_.v(i) = velocity_interface.get_value();
    }
    state_.q_arm = state_.q.head(num_dof_);
    state_.v_arm = state_.v.head(num_dof_);

    // mimic
    if (nq_ > num_dof_) {
        state_.q(num_dof_+1) = state_.q(num_dof_);
        state_.v(num_dof_+1) = state_.v(num_dof_);
        state_.q_gripper = state_.q.tail(2);
        state_.v_gripper = state_.v.tail(2);
    }
}
    

void FrankaBaseController::compute_all_terms()
{
    robot_->computeAllTerms(data_, state_.q, state_.v);
    state_.M = robot_->mass(data_);

    // coriolis()/nonLinearEffects() are sized to the model's nv, which is > 7
    // when the URDF carries movable gripper-finger joints. state_.nle is the
    // arm-only Vector7d, so always take the leading num_dof_ entries (matches
    // the M_arm/J_arm truncation) to stay robust regardless of model DOF.
    // Coriolis/centrifugal joint torque is (nle - g) = C(q,v)*v, NOT a column of
    // the C matrix. coriolis() returns the full nv x nv C matrix, so extracting a
    // block gave a wrong feedforward as soon as the arm moved. Compute it from the
    // non-linear effects minus gravity, which is also robust to Pinocchio API changes.
    if (bringup_type_ == "real") {
        // libfranka compensates arm + hand gravity (set at Desk); add Coriolis only.
        state_.nle = (robot_->nonLinearEffects(data_) - robot_->GeneralizedGravity(data_)).head(num_dof_);
    } else if (bringup_type_ == "gazebo") {
        // franka_ros2 compensates arm gravity; add Coriolis + hand-payload gravity.
        state_.nle = (robot_->nonLinearEffects(data_) - robot_->GeneralizedGravity(data_)).head(num_dof_)
                     + compute_hand_gravity();
    }
    else {
        // mujoco_ros2 does not compensate; add full non-linear effects (Coriolis + gravity).
        state_.nle = robot_->nonLinearEffects(data_).head(num_dof_);
    }

    state_.H_ee = robot_->framePosition(data_, ee_id_);
    robot_->frameJacobianLocal(data_, ee_id_, state_.J);
    robot_->frameJacobianWorldAligned(data_, ee_id_, state_.J_world);

    state_.M_arm = state_.M.topLeftCorner(num_dof_, num_dof_);
    state_.J_arm = state_.J.leftCols(num_dof_);
    state_.J_arm_world = state_.J_world.leftCols(num_dof_);
}

void FrankaBaseController::compute_arm_kinematics(
    const Eigen::VectorXd & q_full, pinocchio::SE3 & H_ee,
    Eigen::Matrix<double, 6, 7> & J_arm)
{
    // Same wrapper calls as compute_all_terms, but evaluated at q_full on a private
    // scratch Data (v = 0: kinematics only). Leaves state_ and data_ untouched.
    if (kin_v_zero_.size() != nv_) kin_v_zero_ = Eigen::VectorXd::Zero(nv_);
    if (kin_J_.cols() != nv_) kin_J_.setZero(6, nv_);
    robot_->computeAllTerms(kin_data_, q_full, kin_v_zero_);
    H_ee = robot_->framePosition(kin_data_, ee_id_);
    robot_->frameJacobianLocal(kin_data_, ee_id_, kin_J_);
    J_arm = kin_J_.leftCols(num_dof_);
}

Vector7d FrankaBaseController::compute_hand_gravity()
{
    Eigen::Vector3d gravity_world(0, 0, -9.81);

    if (bringup_type_ == "gazebo") {
        gravity_world = Eigen::Vector3d(0, 0, -1.0);
    }
    
    Eigen::Matrix<double, 6, 1> wrench_local;
    wrench_local.setZero();

    Eigen::Matrix3d R_ee = state_.H_ee.rotation();
    
    Eigen::Vector3d gravity_local = R_ee.transpose() * gravity_world;
    
    Eigen::Vector3d force_local = mass_ * gravity_local;
    wrench_local.head<3>() = force_local;
    
    Eigen::Vector3d com_local(center_of_mass_[0], center_of_mass_[1], center_of_mass_[2]);
    wrench_local.tail<3>() = com_local.cross(force_local);

    Vector7d tau_hand = - (state_.J_arm.transpose() * wrench_local);
    
    return tau_hand;
}

void FrankaBaseController::clip_position(Vector7d & position, const double eps)
{
    // 직전 "명령값"(q_arm_ref) 기준으로 per-step 변화량을 eps 이내로 제한하고 기준점을 갱신한다.
    // 측정값이 아니라 명령값을 기준으로 해야 position actuator가 충분한 추종 오차(=구동력)를
    // 확보할 수 있고(특히 mujoco), 목표 점프/특이점 튐도 함께 방지된다.
    position = position.array()
                   .max(state_.q_arm_ref.array() - eps)
                   .min(state_.q_arm_ref.array() + eps);
    state_.q_arm_ref = position;
}

void FrankaBaseController::clip_torque(Vector7d & torque)
{
    // Never let a non-finite value reach the actuators: Eigen's array max/min
    // propagate NaN unchanged, so saturation alone does not sanitize it. A NaN
    // torque in torque mode trips a Franka reflex/fault. Fall back to zero
    // (gravity-comp hold on real/gazebo, where the robot compensates gravity).
    if (!torque.allFinite()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Non-finite torque command detected; commanding zero torque.");
        torque.setZero();
    }
    torque = torque.array().max(-torque_limits_.array()).min(torque_limits_.array());
}

void FrankaBaseController::log_ee_pose()
{
    auto fill_pose = [](geometry_msgs::msg::Pose & msg, const pinocchio::SE3 & pose) {
        msg.position.x = pose.translation()(0);
        msg.position.y = pose.translation()(1);
        msg.position.z = pose.translation()(2);
        Eigen::Quaterniond q(pose.rotation());
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
    };

    if (!pose_log_rt_pub_ || !pose_log_rt_pub_->trylock()) {
        return;  // publisher busy: drop this sample rather than block the RT loop
    }
    auto & m = pose_log_rt_pub_->msg_;
    fill_pose(m.pose_ref, state_.H_ee_ref);
    fill_pose(m.pose_des, state_.H_ee_des);
    fill_pose(m.pose_curr, state_.H_ee);
    pose_log_rt_pub_->unlockAndPublish();
}

void FrankaBaseController::log_joint_pos()
{
    if (!joint_log_rt_pub_ || !joint_log_rt_pub_->trylock()) {
        return;  // publisher busy: drop this sample rather than block the RT loop
    }
    auto & m = joint_log_rt_pub_->msg_;

    // resize() is a no-op after the on_configure preallocation (no RT heap traffic).
    // 1. Desired joint positions
    m.desired_state.position.resize(state_.q_arm_des.size());
    Eigen::VectorXd::Map(&m.desired_state.position[0], state_.q_arm_des.size()) = state_.q_arm_des;

    // 2. Current joint positions
    m.current_state.position.resize(state_.q_arm.size());
    Eigen::VectorXd::Map(&m.current_state.position[0], state_.q_arm.size()) = state_.q_arm;

    // 3. Current joint velocities
    m.current_state.velocity.resize(state_.v_arm.size());
    Eigen::VectorXd::Map(&m.current_state.velocity[0], state_.v_arm.size()) = state_.v_arm;

    joint_log_rt_pub_->unlockAndPublish();
}

} // namespace franka
} // namespace cho_controller
