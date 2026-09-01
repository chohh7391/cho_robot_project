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
#include "cho_controller_openarm/base_controller.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <string>
#include <vector>

#include <Eigen/Eigen>

using cho_controller::common::robot::RobotWrapper;

namespace cho_controller {
namespace openarm {

namespace {
// Margin on the URDF position limits for clamp_to_joint_limits(). Matches the
// value cho_controller_franka settled on after measuring reference overshoot in
// simulation; re-measure on OpenArm hardware before trusting it there.
constexpr double kJointLimitMargin = 0.05;  // [rad]
}  // namespace

controller_interface::InterfaceConfiguration
OpenArmBaseController::command_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
OpenArmBaseController::arm_command_interface_configuration(const std::string & interface) const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        // On MIT hardware the controller must own all three interfaces whatever
        // it drives - see write_arm_torque(). Simulators enable exactly one mode
        // per joint, so there we claim only the one being driven.
        if (mit_command_) {
            config.names.push_back(name + "/position");
            config.names.push_back(name + "/velocity");
            config.names.push_back(name + "/effort");
        } else {
            config.names.push_back(name + "/" + interface);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
OpenArmBaseController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
        config.names.push_back(name + "/velocity");
    }
    // The description also exposes an effort state interface on every joint. It
    // is not claimed here because nothing in the minimal controller set reads
    // measured torque; claiming it would add an activation failure mode for no
    // benefit. Add it (and widen the stride in update_joint_states) when a
    // controller actually needs it.
    if (has_gripper_) {
        config.names.push_back(gripper_joint_ + "/position");
        config.names.push_back(gripper_joint_ + "/velocity");
    }
    return config;
}

CallbackReturn OpenArmBaseController::on_init()
{
    try {
        auto_declare<std::vector<std::string>>("joints", {
            "openarm_joint1", "openarm_joint2", "openarm_joint3", "openarm_joint4",
            "openarm_joint5", "openarm_joint6", "openarm_joint7",
        });
        auto_declare<std::string>("gripper_joint", "openarm_finger_joint1");
        // Joints that follow gripper_joint through a URDF <mimic>. Pinocchio's
        // parser ignores <mimic>, so the model carries them as free DOFs and
        // nothing would ever move them; listing them here is what keeps the
        // dynamics consistent. Config-driven rather than parsed out of the URDF
        // so a differently-built gripper needs no code change.
        auto_declare<std::vector<std::string>>("gripper_mimic_joints",
                                               {"openarm_finger_joint2"});
        auto_declare<std::string>("ee_name", "openarm_hand_tcp");
        // Empty means "use the URDF root". Pin it explicitly when the root is a
        // world link, so /ee_state/pose keeps the same meaning across bringups.
        auto_declare<std::string>("base_frame", "");
        // Rotor inertia reflected onto each joint, kg m^2, added to the model's
        // mass-matrix diagonal (Pinocchio's Model::armature, honoured by crba).
        //
        // It belongs to the plant, not to the controller, so it is a parameter
        // rather than a constant: the Isaac bringup authors the Damiao rotor
        // inertias onto its drives, while the v1 MuJoCo model declares none. A
        // controller whose M(q) disagrees with the simulator's under-commands by
        // exactly that ratio - on openarm_joint1 the rotor is 0.0081 against a
        // link inertia of 0.0118, so ignoring it means asking for 40 % too little
        // torque. Empty means none, which is what the MuJoCo configs want.
        auto_declare<std::vector<double>>("rotor_inertia", {});
        auto_declare<std::string>("bringup_type", "");
        auto_declare<std::string>("control_mode", "effort");
        // Declared only so the shared controllers.yaml keys do not trip ROS 2's
        // undeclared-parameter check.
        auto_declare<std::string>("robot_type", "");
    } catch (const std::exception & e) {
        fprintf(stderr, "Exception during init: %s\n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmBaseController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!get_node()->has_parameter("robot_description")) {
        get_node()->declare_parameter<std::string>("robot_description", "");
    }
    robot_description_ = get_node()->get_parameter("robot_description").as_string();

    // Gazebo and MuJoCo start the controller_manager without a robot_description
    // parameter of its own; robot_state_publisher always has one. This fallback
    // is what lets the same controller run unchanged in every environment.
    if (robot_description_.empty()) {
        RCLCPP_INFO(get_node()->get_logger(),
            "robot_description empty, requesting it from robot_state_publisher...");
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(
            get_node(), "robot_state_publisher");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_node()->get_logger(), "robot_state_publisher not available");
            return CallbackReturn::FAILURE;
        }
        auto result = client->get_parameters({"robot_description"}).get();
        if (result.empty() || result[0].value_to_string().empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description");
            return CallbackReturn::FAILURE;
        }
        robot_description_ = result[0].value_to_string();
    }

    robot_type_ = get_node()->get_parameter("robot_type").as_string();
    bringup_type_ = get_node()->get_parameter("bringup_type").as_string();
    control_mode_ = get_node()->get_parameter("control_mode").as_string();

    // The MIT packet is a property of the hardware plugin, not of the control
    // mode: OpenArmHW always writes {kp, kd, pos, vel, tau}. "mock" is included
    // because mock_components stands in for real hardware in the interface-shape
    // smoke test, and the two must claim the same set for that test to mean
    // anything.
    mit_command_ = (bringup_type_ == "real" || bringup_type_ == "mock");

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    num_dof_ = static_cast<int>(joint_names_.size());
    if (num_dof_ == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
        return CallbackReturn::FAILURE;
    }
    gripper_joint_ = get_node()->get_parameter("gripper_joint").as_string();
    gripper_mimic_joints_ = get_node()->get_parameter("gripper_mimic_joints").as_string_array();

    robot_ = std::make_shared<RobotWrapper>(robot_description_, true, false);
    model_ = robot_->model();
    data_ = pinocchio::Data(model_);
    nq_ = robot_->nq();
    nv_ = robot_->nv();
    na_ = robot_->na();

    // Resolve every controlled joint to its place in the model. A name that is
    // not in the URDF is a configuration error worth failing on: silently
    // shifting every index would look like a control bug later.
    auto resolve = [&](const std::string & name, int & q_idx, int & v_idx) {
        const auto joint_id = model_.getJointId(name);
        if (joint_id >= static_cast<pinocchio::JointIndex>(model_.joints.size())) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "joint '%s' is not in the URDF", name.c_str());
            return false;
        }
        q_idx = model_.joints[joint_id].idx_q();
        v_idx = model_.joints[joint_id].idx_v();
        return true;
    };

    arm_q_index_.assign(num_dof_, 0);
    arm_v_index_.assign(num_dof_, 0);
    for (int i = 0; i < num_dof_; ++i) {
        if (!resolve(joint_names_[i], arm_q_index_[i], arm_v_index_[i])) {
            return CallbackReturn::FAILURE;
        }
    }

    // The gripper is only claimable if the model actually carries its DOF; a
    // description built with hand:=false does not.
    gripper_q_index_.clear();
    gripper_v_index_.clear();
    has_gripper_ = false;
    if (!gripper_joint_.empty()) {
        int q_idx = 0, v_idx = 0;
        if (model_.existJointName(gripper_joint_) && resolve(gripper_joint_, q_idx, v_idx)) {
            has_gripper_ = true;
            gripper_q_index_.push_back(q_idx);
            gripper_v_index_.push_back(v_idx);
            for (const auto & mimic : gripper_mimic_joints_) {
                if (!model_.existJointName(mimic)) {
                    continue;
                }
                if (!resolve(mimic, q_idx, v_idx)) {
                    return CallbackReturn::FAILURE;
                }
                gripper_q_index_.push_back(q_idx);
                gripper_v_index_.push_back(v_idx);
            }
        }
    }

    ee_name_ = get_node()->get_parameter("ee_name").as_string();
    ee_id_ = model_.getFrameId(ee_name_);
    if (ee_id_ == static_cast<pinocchio::FrameIndex>(model_.frames.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "End-effector frame '%s' does not exist in the URDF", ee_name_.c_str());
        return CallbackReturn::FAILURE;
    }

    base_frame_ = get_node()->get_parameter("base_frame").as_string();
    if (base_frame_.empty()) {
        // frames[0] is Pinocchio's "universe"; frames[1] is the URDF root link.
        base_frame_ = model_.frames.size() > 1 ? model_.frames[1].name : "world";
    }
    if (model_.getFrameId(base_frame_) == static_cast<pinocchio::FrameIndex>(model_.frames.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "base_frame '%s' does not exist in the URDF", base_frame_.c_str());
        return CallbackReturn::FAILURE;
    }

    const auto rotor_inertia = get_node()->get_parameter("rotor_inertia").as_double_array();
    if (!rotor_inertia.empty()) {
        if (static_cast<int>(rotor_inertia.size()) != num_dof_) {
            RCLCPP_FATAL(get_node()->get_logger(),
                "rotor_inertia has %zu entries, expected %d", rotor_inertia.size(), num_dof_);
            return CallbackReturn::FAILURE;
        }
        rotor_inertia_ = Eigen::Map<const Eigen::VectorXd>(rotor_inertia.data(), num_dof_);
        // model_ is this controller's own copy, so this does not disturb anything
        // else; RobotWrapper reads the model through the same object.
        model_.armature.setZero(model_.nv);
        for (int i = 0; i < num_dof_; ++i) {
            model_.armature(arm_v_index_[i]) = rotor_inertia_(i);
        }
        robot_->model() = model_;
        data_ = pinocchio::Data(model_);
        RCLCPP_INFO(get_node()->get_logger(), "rotor inertia applied to the model");
    }

    torque_limits_.setZero(num_dof_);
    q_lower_limits_.setZero(num_dof_);
    q_upper_limits_.setZero(num_dof_);
    for (int i = 0; i < num_dof_; ++i) {
        torque_limits_(i) = model_.effortLimit(arm_v_index_[i]);
        q_lower_limits_(i) = model_.lowerPositionLimit(arm_q_index_[i]) + kJointLimitMargin;
        q_upper_limits_(i) = model_.upperPositionLimit(arm_q_index_[i]) - kJointLimitMargin;
    }

    state_.q.setZero(nq_);
    state_.v.setZero(nv_);
    state_.q_init.setZero(nq_);
    state_.v_init.setZero(nv_);
    state_.q_arm.setZero(num_dof_);
    state_.v_arm.setZero(num_dof_);
    state_.q_arm_init.setZero(num_dof_);
    state_.v_arm_init.setZero(num_dof_);
    state_.q_arm_des.setZero(num_dof_);
    state_.v_arm_des.setZero(num_dof_);
    state_.q_arm_ref.setZero(num_dof_);
    state_.nle.setZero(num_dof_);
    state_.J.setZero(6, nv_);
    state_.J_world.setZero(6, nv_);
    state_.J_arm.setZero(6, num_dof_);
    state_.J_arm_world.setZero(6, num_dof_);
    state_.M.setZero(nv_, nv_);
    state_.M_arm.setZero(num_dof_, num_dof_);
    dq_filtered_.setZero(num_dof_);

    // Per-controller namespaced logs. Relative names give each controller node its
    // own topic; on a bimanual build that is one pair per arm.
    ctrl_state_pub_ = get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "~/controller_state", 10);
    ee_state_pub_ = get_node()->create_publisher<cho_interfaces::msg::PoseLog>("~/ee_state", 10);
    ctrl_state_rt_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>(
            ctrl_state_pub_);
    ee_state_rt_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>>(ee_state_pub_);
    // Preallocate the controller_state vectors (and fill joint_names once) so the
    // per-cycle fill is heap-free, matching the joint-log publisher above.
    ctrl_state_rt_pub_->msg_.joint_names = joint_names_;
    ctrl_state_rt_pub_->msg_.reference.positions.resize(num_dof_);
    ctrl_state_rt_pub_->msg_.reference.velocities.resize(num_dof_);
    ctrl_state_rt_pub_->msg_.feedback.positions.resize(num_dof_);
    ctrl_state_rt_pub_->msg_.feedback.velocities.resize(num_dof_);

    RCLCPP_INFO(get_node()->get_logger(),
        "OpenArmBaseController configured: %d DOF, ee=%s, base=%s, bringup=%s, mit_command=%s",
        num_dof_, ee_name_.c_str(), base_frame_.c_str(), bringup_type_.c_str(),
        mit_command_ ? "true" : "false");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmBaseController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    update_joint_states();
    compute_all_terms();
    state_.q_init = state_.q;
    state_.v_init = state_.v;
    state_.q_arm_init = state_.q_arm;
    state_.v_arm_init = state_.v_arm;
    state_.H_ee_init = state_.H_ee;
    state_.H_ee_ref = state_.H_ee;
    state_.H_ee_des = state_.H_ee;
    state_.q_arm_ref = state_.q_arm;
    state_.q_arm_des = state_.q_arm;

    // One-time sanity check on the interface layout. update_joint_states() and
    // write_arm_command() index by position, which is only valid because the
    // controller_manager hands interfaces back in the order this controller
    // asked for them. assert() compiles out in Release, so log it instead: a
    // silent reordering would look like a control bug, not a wiring one.
    {
        std::string cmd, st;
        for (const auto & iface : command_interfaces_) {
            cmd += iface.get_name() + " ";
        }
        for (const auto & iface : state_interfaces_) {
            st += iface.get_name() + " ";
        }
        RCLCPP_INFO(get_node()->get_logger(), "command interfaces: %s", cmd.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "state interfaces:   %s", st.c_str());
    }

    controller_active_.store(true, std::memory_order_release);
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmBaseController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    controller_active_.store(false, std::memory_order_release);
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type OpenArmBaseController::update(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    update_joint_states();
    compute_all_terms();
    if (should_publish_arm_log()) {
        log_ee_pose();
        log_joint_pos();
    }
    return controller_interface::return_type::OK;
}

void OpenArmBaseController::update_joint_states()
{
    for (int i = 0; i < num_dof_; ++i) {
        const auto & position_interface = state_interfaces_.at(2 * i);
        const auto & velocity_interface = state_interfaces_.at(2 * i + 1);
        assert(position_interface.get_interface_name() == "position");
        assert(velocity_interface.get_interface_name() == "velocity");
        state_.q_arm(i) = position_interface.get_value();
        state_.v_arm(i) = velocity_interface.get_value();
        state_.q(arm_q_index_[i]) = state_.q_arm(i);
        state_.v(arm_v_index_[i]) = state_.v_arm(i);
    }

    if (has_gripper_) {
        const double gq = state_interfaces_.at(2 * num_dof_).get_value();
        const double gv = state_interfaces_.at(2 * num_dof_ + 1).get_value();
        // The actuated finger and every joint mimicking it get the same value;
        // see gripper_mimic_joints.
        for (size_t k = 0; k < gripper_q_index_.size(); ++k) {
            state_.q(gripper_q_index_[k]) = gq;
            state_.v(gripper_v_index_[k]) = gv;
        }
    }
}

void OpenArmBaseController::compute_all_terms()
{
    robot_->computeAllTerms(data_, state_.q, state_.v);
    state_.M = robot_->mass(data_);

    // Full non-linear effects (Coriolis + gravity) in every environment.
    //
    // Unlike Franka, nothing downstream of us compensates gravity: OpenArmHW
    // hands a raw MIT packet to the Damiao drivers, and MuJoCo, Gazebo and Isaac
    // all simulate gravity in full. cho_controller_franka branches on
    // bringup_type here precisely because libfranka pre-compensates arm and hand
    // gravity on real hardware - copying that branch would double-compensate.
    const Eigen::VectorXd nle_full = robot_->nonLinearEffects(data_);

    state_.H_ee = robot_->framePosition(data_, ee_id_);
    robot_->frameJacobianLocal(data_, ee_id_, state_.J);
    robot_->frameJacobianWorldAligned(data_, ee_id_, state_.J_world);

    // Gather this arm's rows and columns by index rather than slicing the
    // leading block, so a bimanual model works unchanged.
    for (int i = 0; i < num_dof_; ++i) {
        state_.nle(i) = nle_full(arm_v_index_[i]);
        state_.J_arm.col(i) = state_.J.col(arm_v_index_[i]);
        state_.J_arm_world.col(i) = state_.J_world.col(arm_v_index_[i]);
        for (int j = 0; j < num_dof_; ++j) {
            state_.M_arm(i, j) = state_.M(arm_v_index_[i], arm_v_index_[j]);
        }
    }
}

std::string OpenArmBaseController::action_server_name() const
{
    return std::string("/controller_action_server/") + get_node()->get_name();
}

void OpenArmBaseController::write_arm_torque(const Eigen::VectorXd & torque)
{
    if (!mit_command_) {
        for (int i = 0; i < num_dof_; ++i) {
            command_interfaces_[i].set_value(torque(i));
        }
        return;
    }

    // MIT hardware. OpenArmHW::write() sends
    //     tau = kp*(pos_cmd - pos) + kd*(vel_cmd - vel) + tau_cmd
    // every cycle regardless of which interfaces we wrote, and its command
    // buffers are zero-initialised rather than NaN. Leaving pos_cmd at its
    // default therefore means "hold q = 0 at full stiffness" - which for
    // openarm_joint4 (range [0, 2.443]) is a joint limit, and which fights every
    // torque we command.
    //
    // The description already sets kp to 0 in torque mode, so this is belt and
    // braces: writing pos_cmd = q_measured makes the stiffness term vanish
    // identically even if someone restores a non-zero kp, leaving only -kd*v,
    // i.e. joint damping at the motor's own rate. That is a safe residual.
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[3 * i].set_value(state_.q_arm(i));
        command_interfaces_[3 * i + 1].set_value(0.0);
        command_interfaces_[3 * i + 2].set_value(torque(i));
    }
}

void OpenArmBaseController::write_arm_position(const Eigen::VectorXd & position)
{
    if (!mit_command_) {
        for (int i = 0; i < num_dof_; ++i) {
            command_interfaces_[i].set_value(position(i));
        }
        return;
    }

    // MIT hardware in position mode: the motor's own kp/kd close the loop, so
    // the feed-forward torque must be zero and the velocity target zero. This
    // only tracks if the description was built with kp_scale:=1.0 - with the
    // torque-mode default of 0.0 the stiffness term vanishes and the arm simply
    // does not follow.
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[3 * i].set_value(position(i));
        command_interfaces_[3 * i + 1].set_value(0.0);
        command_interfaces_[3 * i + 2].set_value(0.0);
    }
}

void OpenArmBaseController::write_arm_velocity(const Eigen::VectorXd & velocity)
{
    if (!mit_command_) {
        for (int i = 0; i < num_dof_; ++i) {
            command_interfaces_[i].set_value(velocity(i));
        }
        return;
    }

    // MIT hardware in velocity mode: kd closes the loop on velocity, so pin the
    // position target to the measurement (kp * 0) and leave the feed-forward at
    // zero. Requires kp_scale:=0.0, the default.
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[3 * i].set_value(state_.q_arm(i));
        command_interfaces_[3 * i + 1].set_value(velocity(i));
        command_interfaces_[3 * i + 2].set_value(0.0);
    }
}

double OpenArmBaseController::nominal_period(const rclcpp::Duration & period)
{
    // get_update_rate() reports the controller's OWN rate, which is 0 whenever it
    // inherits the controller_manager's - and none of this project's configs set
    // a per-controller update_rate, so it is 0 in every bringup. Falling back to
    // a hardcoded 1 kHz was therefore only ever correct for MuJoCo, which embeds
    // the controller_manager and runs it at 1000 Hz. Under Isaac the
    // controller_manager runs at 250 Hz, so a 1 ms nominal advanced the
    // trajectory clock at a QUARTER of sim time and every goal took four times
    // its requested duration - which looks exactly like the arm tracking slowly
    // against a weak drive, not like a clock bug.
    //
    // The measured period is used to estimate the true rate rather than to
    // advance the clock directly: it jitters by up to 2x, and parameterising a
    // position trajectory by it makes each cycle's command step - and so the
    // velocity the actuator infers from it - jitter in proportion. Seeded from
    // the first sane cycle so the estimate is right from the first goal, then
    // smoothed.
    const unsigned int rate = get_update_rate();
    if (rate > 0) {
        return 1.0 / rate;
    }
    const double measured = period.seconds();
    // A cycle that saw no new state reports 0; a resume after a clock jump
    // reports something huge. Neither says anything about the nominal rate.
    if (measured > 1e-6 && measured < 0.1) {
        nominal_dt_ = (nominal_dt_ > 0.0) ? (0.95 * nominal_dt_ + 0.05 * measured)
                                          : measured;
    }
    return (nominal_dt_ > 0.0) ? nominal_dt_ : 0.001;
}


void OpenArmBaseController::clip_position(Eigen::VectorXd & q_cmd, double eps)
{
    if (state_.q_arm_ref.size() != q_cmd.size()) {
        state_.q_arm_ref = state_.q_arm;
    }
    q_cmd = q_cmd.array()
                .max(state_.q_arm_ref.array() - eps)
                .min(state_.q_arm_ref.array() + eps);
    state_.q_arm_ref = q_cmd;
}

void OpenArmBaseController::clip_torque(Eigen::VectorXd & torque) const
{
    // Eigen's array max/min propagate NaN, so saturation alone does not sanitize
    // a non-finite command. On MIT hardware a NaN would be packed into a CAN
    // frame and handed to the drivers, so it must never leave here.
    if (!torque.allFinite()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Non-finite torque command detected; commanding zero torque.");
        torque.setZero();
    }
    torque = torque.array().max(-torque_limits_.array()).min(torque_limits_.array());
}

void OpenArmBaseController::clamp_to_joint_limits(Eigen::VectorXd & q) const
{
    q = q.array().max(q_lower_limits_.array()).min(q_upper_limits_.array());
}

void OpenArmBaseController::log_ee_pose()
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
        fill_pose(ee_state_rt_pub_->msg_.pose_ref, state_.H_ee_ref);
        fill_pose(ee_state_rt_pub_->msg_.pose_des, state_.H_ee_des);
        fill_pose(ee_state_rt_pub_->msg_.pose_curr, state_.H_ee);
        ee_state_rt_pub_->unlockAndPublish();
    }
}

void OpenArmBaseController::log_joint_pos()
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

}  // namespace openarm
}  // namespace cho_controller
