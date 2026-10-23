// Copyright (c) 2023 Franka Robotics GmbH
// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Derived from franka_example_controllers (https://github.com/frankaemika/franka_ros2);
// modified for cho_robot_project, see NOTICE.

#include "cho_controller_franka/robot_utils.hpp"
#include "cho_controller_franka/base_controller.hpp"
#include "cho_controller_base/held_command.hpp"
#include "cho_controller_common/trajectory/motion_limits_params.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <future>
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
    } else {
        // The FCI's own clock, which nominal_period() steps by (read_robot_time()).
        config.names.push_back(robot_type_ + "/robot_time");
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
        // Reflected rotor inertia of each arm joint, kg m^2, one entry per arm
        // joint in URDF chain order. on_configure writes it into the model's
        // Model::armature, which crba() adds to the mass-matrix diagonal, so
        // every quantity derived from M(q) - the QP's inertia, Lambda, the
        // null-space projector - sees the same correction.
        //
        // It affects the MASS MATRIX ONLY. nonLinearEffects() (gravity +
        // Coriolis) does not depend on armature, so gravity compensation and the
        // state_.nle feed-forward are bit-identical with or without this
        // parameter, however reasonable the opposite assumption looks.
        //
        // Known values for the FR3: 0.195 on joints 1-4 and 0.074 on joints 5-7.
        // The URDF cannot express rotor inertia, so the model built here carries
        // link inertia only; the MuJoCo model this project tunes against declares
        // those numbers on its joints (cho_description_franka/xml/fr3/fr3.xml -
        // 0.195 comes from the "fr3" default class, joints 5-7 override it), and
        // the Isaac bringup authors the identical vector onto its drives
        // (cho_bringup_franka/config/isaac/robot_profile.json, "arm_armature",
        // whose comment records that the wrist links are ~15x lighter than their
        // own rotor).
        //
        // It is a parameter and not a constant because it belongs to the plant,
        // not to the controller: MuJoCo, Isaac and the real FR3 do not have to
        // agree, and a bringup that models no rotor wants none added. Empty is
        // therefore the default and means "none", which is what every current
        // config asks for - so this stays inert until some config opts in.
        auto_declare<std::vector<double>>("rotor_inertia", {});
        // Gravity [m/s^2] at which the "gazebo" bringup feeds forward the payload
        // above (mass, center_of_mass) on top of what the Gazebo plugin already
        // compensates. Finite and >= 0; 0 drops the term. Read nowhere else. What
        // the plugin covers and why the default is 1.0, not 9.81:
        // compute_hand_gravity().
        auto_declare<double>("gazebo_payload_gravity", 1.0);
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

        // Bounded: the answer is delivered by the executor this callback may be
        // running on (it does not deadlock under Gazebo's, but an unbounded get()
        // hung configure forever wherever it did).
        auto future = parameters_client->get_parameters({"robot_description"});
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "robot_state_publisher did not answer for robot_description within 5 s");
            return CallbackReturn::FAILURE;
        }
        auto result = future.get();
        
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
    // It says which backend compensates gravity, and so which torque terms update()
    // adds. Any other value took the simulation branch, which adds full gravity: on
    // the real robot that is gravity on top of libfranka's own compensation, an
    // upward push the size of the arm's weight.
    if (bringup_type_ != "real" && bringup_type_ != "gazebo" && bringup_type_ != "mujoco" &&
        bringup_type_ != "isaac") {
        RCLCPP_ERROR(get_node()->get_logger(),
            "bringup_type '%s' is none of real, gazebo, mujoco, isaac", bringup_type_.c_str());
        return CallbackReturn::FAILURE;
    }

    robot_type_ = cho_controller::franka::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
    arm_joints_ = arm_joint_names();

    robot_ = std::make_shared<RobotWrapper>(robot_description_, true, false);
    model_ = robot_->model();
    data_ = pinocchio::Data(model_);
    kin_data_ = pinocchio::Data(model_);
    kin_v_zero_ = Eigen::VectorXd::Zero(model_.nv);
    kin_J_.setZero(6, model_.nv);
    q_scratch_ = Eigen::VectorXd::Zero(model_.nq);

    // Optional rotor inertia (see the parameter's comment in on_init). Absent or
    // empty leaves model_.armature untouched, so nothing below this block changes.
    const auto rotor_inertia = get_node()->get_parameter("rotor_inertia").as_double_array();
    if (!rotor_inertia.empty()) {
        if (static_cast<int>(rotor_inertia.size()) != num_dof_) {
            RCLCPP_FATAL(get_node()->get_logger(),
                "rotor_inertia has %zu entries, expected %d (one per arm joint)",
                rotor_inertia.size(), num_dof_);
            return CallbackReturn::FAILURE;
        }
        for (int i = 0; i < num_dof_; ++i) {
            if (!std::isfinite(rotor_inertia[i]) || rotor_inertia[i] < 0.0) {
                RCLCPP_FATAL(get_node()->get_logger(),
                    "rotor_inertia[%d] = %f is not a finite, non-negative inertia", i, rotor_inertia[i]);
                return CallbackReturn::FAILURE;
            }
        }
        rotor_inertia_ = Eigen::Map<const Vector7d>(rotor_inertia.data());
        // Unlike the OpenArm base, which looks each arm joint up by name because a
        // bimanual build interleaves two arms, this base assumes the arm occupies
        // the LEADING num_dof_ entries of the configuration - the same assumption
        // the q_lower_limits_ computation below documents - so index the armature
        // vector by that leading block.
        model_.armature.setZero(model_.nv);
        model_.armature.head(num_dof_) = rotor_inertia_;
        // model_ is this controller's own copy of the wrapper's model, so push it
        // back before rebuilding anything that caches the model's dimensions.
        robot_->model() = model_;
        data_ = pinocchio::Data(model_);
        // kin_data_ too: armature does not change kinematics, so compute_arm_kinematics()
        // would still be correct - but two Data objects built from different models is a
        // trap for whatever gets added to that helper next.
        kin_data_ = pinocchio::Data(model_);
        RCLCPP_INFO(get_node()->get_logger(),
            "rotor inertia applied to the model: [%.4f %.4f %.4f %.4f %.4f %.4f %.4f] kg m^2",
            rotor_inertia_(0), rotor_inertia_(1), rotor_inertia_(2), rotor_inertia_(3),
            rotor_inertia_(4), rotor_inertia_(5), rotor_inertia_(6));
    }

    nq_ = robot_->nq();
    nv_ = robot_->nv();
    na_ = robot_->na();

    // Arm joints occupy the first num_dof_ configuration entries (URDF chain order).
    // Margin covers the physical tracking overshoot when a fast reference ramp is
    // clamped dead at the boundary (measured ~0.035 rad at 2.5 rad/s in MuJoCo).
    constexpr double kJointLimitMargin = 0.05;  // [rad]
    q_lower_limits_ = model_.lowerPositionLimit.head(num_dof_).array() + kJointLimitMargin;
    q_upper_limits_ = model_.upperPositionLimit.head(num_dof_).array() - kJointLimitMargin;

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

    // Per-controller namespaced logs. Relative names give each controller node its
    // own topic (e.g. /<controller_name>/controller_state, /<controller_name>/ee_state).
    ctrl_state_pub_ = get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "~/controller_state", 10);
    ee_state_pub_ = get_node()->create_publisher<cho_interfaces::msg::PoseLog>("~/ee_state", 10);
    ctrl_state_rt_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>(
            ctrl_state_pub_);
    ee_state_rt_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>>(ee_state_pub_);
    // The poses are oMf, in the model's root frame; set once, not per cycle.
    const auto root_frames = cho_controller_base::root_frames(model_);
    ee_state_rt_pub_->msg_.header.frame_id = root_frames.empty() ? std::string() : root_frames.front();
    // Preallocate the controller_state vectors (and fill joint_names once) so the
    // per-cycle fill is heap-free, matching the joint-log publisher above.
    ctrl_state_rt_pub_->msg_.joint_names = arm_joint_names();
    ctrl_state_rt_pub_->msg_.reference.positions.resize(num_dof_);
    ctrl_state_rt_pub_->msg_.reference.velocities.resize(num_dof_);
    ctrl_state_rt_pub_->msg_.feedback.positions.resize(num_dof_);
    ctrl_state_rt_pub_->msg_.feedback.velocities.resize(num_dof_);

    // gravity compensation parameter
    mass_ = get_node()->get_parameter("mass").as_double();
    auto com_vec = get_node()->get_parameter("center_of_mass").as_double_array();
    auto inertia_vec = get_node()->get_parameter("load_inertia").as_double_array();

    std::copy_n(com_vec.begin(), std::min(com_vec.size(), size_t(3)), center_of_mass_.begin());
    std::copy_n(inertia_vec.begin(), std::min(inertia_vec.size(), size_t(9)), load_inertia_.begin());

    const double payload_gravity = get_node()->get_parameter("gazebo_payload_gravity").as_double();
    if (!std::isfinite(payload_gravity) || payload_gravity < 0.0) {
        RCLCPP_FATAL(get_node()->get_logger(),
            "gazebo_payload_gravity = %f is not a finite, non-negative acceleration [m/s^2]", payload_gravity);
        return CallbackReturn::FAILURE;
    }
    gazebo_payload_gravity_ = payload_gravity;

    RCLCPP_INFO(get_node()->get_logger(), "FrankaBaseController Configured successfully.");
    return CallbackReturn::SUCCESS;
}

bool FrankaBaseController::state_interfaces_in_order() const
{
  // update_joint_states() reads position/velocity pairs by index: num_dof_ arm
  // joints, plus the finger joint outside the real bringup when the model has
  // one. Checked once at activation; the assert this replaces vanished from
  // Release builds.
  const std::size_t joints = (bringup_type_ != "real" && nq_ > num_dof_) ? num_dof_ + 1 : num_dof_;
  if (state_interfaces_.size() < 2 * joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "expected %zu state interfaces, got %zu",
        2 * joints, state_interfaces_.size());
    return false;
  }
  for (std::size_t i = 0; i < joints; ++i) {
    if (state_interfaces_[2 * i].get_interface_name() != "position" ||
        state_interfaces_[2 * i + 1].get_interface_name() != "velocity")
    {
      RCLCPP_ERROR(get_node()->get_logger(),
          "state interfaces out of order at joint %zu: expected position then velocity", i);
      return false;
    }
  }
  return true;
}

CallbackReturn FrankaBaseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!state_interfaces_in_order()) {
    return CallbackReturn::ERROR;
  }
  // The robot's clock starts unknown, and the rate limiter without a history:
  // nominal_period() and the first commands are this activation's own.
  robot_time_index_ = -1;
  for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (state_interfaces_[i].get_prefix_name() == robot_type_ &&
        state_interfaces_[i].get_interface_name() == "robot_time")
    {
      robot_time_index_ = static_cast<int>(i);
    }
  }
  last_robot_time_ = std::numeric_limits<double>::quiet_NaN();
  robot_period_ = 0.0;
  command_limiter_.reset();
  update_joint_states();
  compute_all_terms();
  state_.q_init = state_.q;
  state_.v_init = state_.v;
  state_.q_arm_init = state_.q_arm;
  state_.v_arm_init = state_.v_arm;
  if (nq_ > num_dof_) state_.q_gripper_init = state_.q_gripper;
  state_.H_ee_init = state_.H_ee;
  // Seed the command reference from the current measurement (rate-limit baseline for clip_position).
  state_.q_arm_ref = state_.q_arm;
  // Seed the logged "desired" from the current measurement too, so a controller
  // that has not written a desired yet (gravity_compensation, or an arm controller
  // between goals) never publishes an uninitialized q_arm_des / H_ee_des / H_ee_ref.
  state_.q_arm_des = state_.q_arm;
  // Seed the desired velocity too, so the new ~/controller_state reference never
  // publishes an uninitialized v_arm_des before a controller has written one.
  state_.v_arm_des.setZero();
  state_.H_ee_des = state_.H_ee;
  state_.H_ee_ref = state_.H_ee;
  // No torque of this controller's own to carry on from (fade_torque_switch()),
  // but the arm is still commanded the one the torque controller it replaces in
  // this same switch left (cho_controller_base::live_held_torque()): a torque
  // controller's first command fades from that, as from a goal's end.
  torque_fade_.reset();
  torque_tracking_ = false;
  Eigen::VectorXd handed;
  if (cho_controller_base::live_held_torque(
        command_interfaces_, arm_joints_, state_.q_arm, get_node()->get_name(), handed) &&
      handed.size() == num_dof_) {
    torque_fade_.carry_on_from(handed);
  }

  activity_.activated();

  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  activity_.deactivated();
  // What every controller here owes when it lets go, while its interfaces are
  // still claimed (cho_controller_base::release_on_deactivate): a velocity
  // command goes to 0, a position command stays as the hold and is recorded with
  // where the arm is now, so the next position controller can tell a live hold
  // from a stale one (live_held_command); an effort command is left as it is
  // and recorded with who left it and when, so the torque controller activated
  // in the same switch carries on from it (release_held_torque()).
  // A controller whose reference may be moving records its rate and
  // acceleration with it, so the controller activated in the same switch brings
  // the motion to rest (held_command_motion()) instead of stopping it in one
  // period. Without a readable state the velocities are still zeroed, nothing
  // recorded.
  Vector7d rate, acceleration;
  if (state_interfaces_in_order()) {
    update_joint_states();
    if (reference_motion(rate, acceleration)) {
      cho_controller_base::release_on_deactivate(
        command_interfaces_, arm_joints_, state_.q_arm, rate, acceleration, get_node()->get_name());
    } else {
      cho_controller_base::release_on_deactivate(command_interfaces_, arm_joints_, state_.q_arm);
    }
    cho_controller_base::release_held_torque(
      command_interfaces_, arm_joints_, state_.q_arm, get_node()->get_name());
  } else {
    cho_controller_base::release_on_deactivate(command_interfaces_, arm_joints_, Eigen::VectorXd());
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaBaseController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {

    read_robot_time();
    update_joint_states();
    compute_all_terms();

    // logging (only the arm-tracking controller; see should_publish_arm_log())
    if (should_publish_arm_log()) {
        publish_ee_state();
        publish_controller_state();
    }

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
    // arm. The order is checked once, in on_activate (state_interfaces_in_order()).
    for (auto i = 0; i < num_interface; ++i) {
        const auto& position_interface = state_interfaces_.at(2 * i);
        const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

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
        // franka_ros2's Gazebo plugin compensates the gravity of the arm and the hand
        // body; add Coriolis + the payload term (compute_hand_gravity()).
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
    // The joint torque that holds the payload ("mass" at "center_of_mass" in the
    // EE frame; config/payload.yaml, 0.73 kg for the hand, or payload_ft_sensor.yaml,
    // 0.97 kg with the FT sensor) against
    // gravity. compute_all_terms() adds it in the "gazebo" branch only, at
    // gazebo_payload_gravity m/s^2 (default 1.0), not at 9.81:
    //
    // - franka_ros2's Gazebo plugin (extern/franka_ros2/franka_gazebo/
    //   franka_ign_ros2_control, IgnitionSystem::write()) adds to every effort
    //   command the KDL gravity torque, at 9.8 m/s^2, of the chain from the URDF
    //   root to the leaf link findTipLink() picks. For the FR3 descriptions here
    //   that leaf is fr3_hand_tcp, so the chain carries link0-link7, fr3_hand
    //   (0.654 kg) and, with load_ft_sensor, the Bota sensor links: the hand body
    //   is compensated already. Only the two fingers (0.029 kg each), off that
    //   chain, are not.
    // - 1.0 m/s^2 is an empirical value with no recorded derivation or
    //   measurement; it is not the fingers' weight either (0.97 N with the FT
    //   sensor's payload, Gazebo's default, against 0.57 N for the fingers). It stays the default so behaviour is unchanged;
    //   the parameter lets a measurement replace it, or 0 drop the term.
    const double gravity =
        (bringup_type_ == "gazebo") ? gazebo_payload_gravity_ : 9.81;
    const Eigen::Vector3d gravity_world(0, 0, -gravity);

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

Vector7d FrankaBaseController::held_command_position() const
{
    // When the position interface starts a FRESH libfranka session (first-ever
    // activation, or a mode switch from effort/velocity), franka_hardware initializes
    // hw_position_commands_ to ZEROS and only re-seeds them to the measured position
    // in the read() after this on_activate (first_position_update_); a stale value
    // from an older position session survives the same way. The FCI motion generator
    // then restarts from the measured position, which held_command() falls back to:
    // fr3 joint4's range [-3.07, -0.07] never comes within its 0.05 rad of zero.
    //
    // The stale value is the dangerous one: a command left by a position
    // controller before an effort or velocity controller moved the arm, still
    // within the band. live_held_command() refuses any value a cho controller
    // left before the arm moved, which covers the switches this package makes
    // (effort/velocity <-> position). A value left by a controller from
    // elsewhere is judged by the band alone.
    return cho_controller_base::live_held_command(command_interfaces_, arm_joints_, state_.q_arm_init);
}

bool FrankaBaseController::held_command_motion(Vector7d & rate, Vector7d & acceleration) const
{
    Eigen::VectorXd held_rate, held_acceleration;
    rate.setZero();
    acceleration.setZero();
    if (!cho_controller_base::live_held_motion(
          command_interfaces_, arm_joints_, state_.q_arm_init, get_node()->get_name(),
          held_rate, held_acceleration) ||
        held_rate.size() != num_dof_ || (held_rate.array() == 0.0).all()) {
        return false;
    }
    rate = held_rate;
    acceleration = held_acceleration;
    return true;
}

double FrankaBaseController::nominal_period(const rclcpp::Duration & period)
{
  // The per-cycle period for every controller here that advances its own
  // trajectory clock or integrates an open-loop reference. Such code calls this,
  // never 1.0 / get_update_rate(), a literal dt, or period.seconds() directly.
  //
  // get_update_rate() reports the controller's OWN rate, which is 0 whenever it
  // inherits the controller_manager's. The manager's rate differs per bringup
  // (config/<env>/controllers.yaml; Isaac's equals the physics rate), so a
  // literal dt is right only where it happens to match: 1 ms under a 250 Hz
  // manager advances the trajectory clock at a quarter of sim time and stretches
  // every goal fourfold. From the outside that looks like a drive too soft to
  // hold against gravity, not like a clock bug.
  //
  // With no rate of its own the measured period is used to ESTIMATE the rate,
  // not to advance the clock directly: it jitters by up to 2x, and
  // parameterising a position trajectory by it makes each cycle's command step -
  // and so the velocity the actuator infers from it - jitter in proportion; it is
  // exactly 0 on a cycle that saw no new state, where a difference divided by it
  // is NaN. Seeded from the first sane cycle so the estimate is right from the
  // first goal, then smoothed; 1 ms until that cycle.
  //
  // The smoothing still passes some of the jitter on: one 2.2 ms period moves the
  // estimate 6 %, and the commanded rate with it in one cycle, ~18 rad/s^2 at
  // 0.3 rad/s against libfranka's 10. The FCI's period is exact, so the real
  // bringup gives every controller that calls this an update_rate of 1000
  // (config/real/controllers.yaml) and the estimate is never used there. The
  // simulations keep it: their manager rates differ and nothing rejects jitter.
  //
  // On the real FR3 the FCI's own clock comes first: the ticks it advanced since
  // the last cycle (read_robot_time()), one normally and two when the loop
  // skipped a state, a tick the robot meanwhile extrapolated. Advanced by one
  // period instead, the reference falls a tick behind the robot and the next
  // command steps back by it: v * 1 ms, 95 urad at 0.095 rad/s, twenty times the
  // step that tripped joint_motion_generator_acceleration_discontinuity on
  // 2026-10-22. Franka's own example controllers time their trajectories by it.
  if (robot_period_ > 0.0) {
    return robot_period_;
  }
  const unsigned int rate = get_update_rate();
  if (rate > 0) {
    return 1.0 / rate;
  }
  const double measured = period.seconds();
  // A cycle that saw no new state reports 0; a resume after a clock jump reports
  // something huge. Neither says anything about the nominal rate.
  if (measured > 1e-6 && measured < 0.1) {
    nominal_dt_ = (nominal_dt_ > 0.0) ? (0.95 * nominal_dt_ + 0.05 * measured) : measured;
  }
  return (nominal_dt_ > 0.0) ? nominal_dt_ : 0.001;
}


void FrankaBaseController::clip_position(Vector7d & position, const double eps)
{
    // Limit the per-step change to eps around the PREVIOUS COMMAND (q_arm_ref) and
    // advance that baseline. Rate-limiting against the command (not the measurement)
    // lets the position actuator keep enough tracking error (= drive force, esp. in
    // mujoco) while also suppressing target jumps and singularity spikes.
    position = position.array()
                   .max(state_.q_arm_ref.array() - eps)
                   .min(state_.q_arm_ref.array() + eps);
    state_.q_arm_ref = position;
}

void FrankaBaseController::read_robot_time()
{
  robot_period_ = 0.0;
  if (robot_time_index_ < 0) {
    return;
  }
  const double now = state_interfaces_[static_cast<std::size_t>(robot_time_index_)].get_value();
  const double step = now - last_robot_time_;
  last_robot_time_ = now;
  // Whole ticks within the limiter's clock-jump bound. The first reading (NaN),
  // the same state twice, the robot's clock set back or a stall past 0.1 s
  // leave nominal_period() to its own rule.
  if (std::isfinite(step) && step > 0.0 && step <= 0.1) {
    const double ticks = std::round(step / JointRateLimiter::kTick);
    if (ticks >= 1.0) {
      robot_period_ = ticks * JointRateLimiter::kTick;
    }
  }
}

void FrankaBaseController::limit_position_command(Vector7d & position)
{
  if (bringup_type_ != "real") {
    return;
  }
  const int ticks = robot_period_ > 0.0 ? static_cast<int>(std::lround(robot_period_ / JointRateLimiter::kTick)) : 1;
  if (command_limiter_.limit_position(position, ticks)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Joint rate limiter changed a position command: it passed the FR3's jerk, acceleration or "
        "velocity bound (JointRateLimiter).");
  }
}

void FrankaBaseController::limit_velocity_command(Vector7d & velocity)
{
  if (bringup_type_ != "real") {
    return;
  }
  const int ticks = robot_period_ > 0.0 ? static_cast<int>(std::lround(robot_period_ / JointRateLimiter::kTick)) : 1;
  if (command_limiter_.limit_velocity(velocity, ticks)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Joint rate limiter changed a velocity command: it passed the FR3's jerk, acceleration or "
        "velocity bound (JointRateLimiter).");
  }
}

void FrankaBaseController::clamp_to_joint_limits(Vector7d & q) const
{
    q = q.array().max(q_lower_limits_.array()).min(q_upper_limits_.array());
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

void FrankaBaseController::fade_torque_switch(
    Vector7d & torque, const bool tracking, const double dt, const bool law_changed)
{
    // A goal starting or ending (a cancel included) changes what the law tracks.
    torque_fade_.apply(torque, tracking != torque_tracking_ || law_changed, dt);
    torque_tracking_ = tracking;
}

void FrankaBaseController::publish_ee_state()
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

    // Per-controller ~/ee_state carries the three poses (ref / desired / current).
    if (ee_state_rt_pub_ && ee_state_rt_pub_->trylock()) {
        auto & e = ee_state_rt_pub_->msg_;
        e.header.stamp = get_node()->now();
        fill_pose(e.pose_ref, state_.H_ee_ref);
        fill_pose(e.pose_des, state_.H_ee_des);
        fill_pose(e.pose_curr, state_.H_ee);
        ee_state_rt_pub_->unlockAndPublish();
    }
}

void FrankaBaseController::publish_controller_state()
{
    // Per-controller ~/controller_state. Vectors and joint_names were preallocated
    // in on_configure (resize() here is a no-op, so no RT heap traffic).
    if (ctrl_state_rt_pub_ && ctrl_state_rt_pub_->trylock()) {
        auto & cs = ctrl_state_rt_pub_->msg_;
        cs.header.stamp = get_node()->now();
        cs.reference.positions.resize(num_dof_);
        cs.reference.velocities.resize(num_dof_);
        cs.feedback.positions.resize(num_dof_);
        cs.feedback.velocities.resize(num_dof_);
        Eigen::VectorXd::Map(cs.reference.positions.data(), num_dof_) = state_.q_arm_des;
        Eigen::VectorXd::Map(cs.reference.velocities.data(), num_dof_) = state_.v_arm_des;
        Eigen::VectorXd::Map(cs.feedback.positions.data(), num_dof_) = state_.q_arm;
        Eigen::VectorXd::Map(cs.feedback.velocities.data(), num_dof_) = state_.v_arm;
        ctrl_state_rt_pub_->unlockAndPublish();
    }
}

std::vector<std::string> FrankaBaseController::arm_joint_names() const
{
    std::vector<std::string> joints;
    for (int i = 1; i <= num_dof_; ++i) {
        joints.push_back(robot_type_ + "_joint" + std::to_string(i));
    }
    return joints;
}

cho_controller::common::trajectory::JointMotionLimits FrankaBaseController::joint_motion_limits()
{
    return cho_controller::common::trajectory::load_joint_motion_limits(get_node(), arm_joint_names());
}

cho_controller::common::trajectory::CartesianMotionLimits FrankaBaseController::cartesian_motion_limits()
{
    return cho_controller::common::trajectory::load_cartesian_motion_limits(get_node());
}

} // namespace franka
} // namespace cho_controller
