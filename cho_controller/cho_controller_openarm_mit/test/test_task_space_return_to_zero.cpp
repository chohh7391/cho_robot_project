#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>

#include <cho_openarm_mit_core/mit_protocol.hpp>

#include "cho_controller_openarm_mit/task_space_impedance_mit_controller.hpp"

namespace cho_controller_openarm_mit
{
struct TaskSpaceImpedanceMitControllerTestAccess
{
  static bool ready(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.task_ready_.load(std::memory_order_acquire);
  }
  static double position_command(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint].get_value();
  }
  static double stiffness_command(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 2].get_value();
  }
  static double damping_command(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 3].get_value();
  }
  static double effort_command(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 4].get_value();
  }
  static double measured_position(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.state_interfaces_[2 * joint].get_value();
  }
  static double configured_damping(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.kd_[joint].load(std::memory_order_acquire);
  }
  static double configured_stiffness(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.kp_[joint];
  }
  static double reference_offset_limit(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.reference_offset_limit_[joint];
  }
  static bool drive_side_impedance(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.drive_side_impedance_;
  }
  static void friction(
    const TaskSpaceImpedanceMitController & controller,
    const std::array<double, 7> & dq, const std::array<double, 7> & dq_des,
    Eigen::Matrix<double, 7, 1> & torque)
  {
    controller.friction_torque(dq, dq_des, torque);
  }
  static void set_friction(
    TaskSpaceImpedanceMitController & controller, const std::array<double, 7> & level,
    const double scale, const double epsilon)
  {
    for (std::size_t joint = 0; joint < 7; ++joint) {
      controller.friction_level_[joint].store(level[joint], std::memory_order_release);
    }
    controller.friction_scale_.store(scale, std::memory_order_release);
    controller.friction_velocity_epsilon_ = epsilon;
  }
  static void set_friction_stribeck(
    TaskSpaceImpedanceMitController & controller, const double ratio, const double velocity)
  {
    controller.friction_kinetic_ratio_.store(ratio, std::memory_order_release);
    controller.friction_stribeck_velocity_.store(velocity, std::memory_order_release);
  }
  static void set_friction_source_measured(
    TaskSpaceImpedanceMitController & controller, const bool measured)
  {
    controller.friction_velocity_source_ = measured ?
      TaskSpaceImpedanceMitController::FrictionVelocity::MEASURED :
      TaskSpaceImpedanceMitController::FrictionVelocity::REFERENCE;
  }
  static double position_lower(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.position_lower_[joint];
  }
  static double position_upper(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.position_upper_[joint];
  }
  static ArmCommand clamp_positions(
    const TaskSpaceImpedanceMitController & controller, ArmCommand command)
  {
    controller.clamp_command_positions(command);
    return command;
  }
  static bool return_to_zero_handoff_active(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.return_to_zero_handoff_active_;
  }
  static bool startup_active(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.startup_active_;
  }
  static double q_reference(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.task_q_reference_observed_[joint].load(std::memory_order_acquire);
  }
  static void stage_unbounded_relative_goal(TaskSpaceImpedanceMitController & controller)
  {
    TaskSpaceImpedanceMitController::Goal goal;
    goal.id = controller.task_last_started_id_ + 1;
    goal.duration = 5.0;
    goal.relative = true;
    // Deliberately exceeds the former 50 mm translation and 0.35 rad
    // orientation guards. The Cartesian torque and MIT safety limits remain
    // the runtime boundary.
    goal.translation = Eigen::Vector3d(0.30, 0.0, 0.0);
    goal.rotation = Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitY());
    controller.task_goal_buffer_.writeFromNonRT(goal);
  }
  static rclcpp_action::GoalResponse accept_unbounded_relative_goal(
    TaskSpaceImpedanceMitController & controller)
  {
    auto goal = std::make_shared<TaskSpaceImpedanceMitController::Action::Goal>();
    goal->duration = 5.0F;
    goal->relative = true;
    // 300 mm/2 rad deliberately exceeds the retired Cartesian goal guards.
    goal->target_pose.position.x = 0.30;
    goal->target_pose.orientation.y = std::sin(1.0);
    goal->target_pose.orientation.w = std::cos(1.0);
    return controller.goal_callback({}, goal);
  }
  static void stage_absolute_goal_without_cartesian_caps(
    TaskSpaceImpedanceMitController & controller)
  {
    TaskSpaceImpedanceMitController::Goal goal;
    goal.id = controller.task_last_started_id_ + 1;
    goal.duration = 5.0;
    goal.relative = false;
    goal.translation = Eigen::Vector3d(0.30, 0.0, 0.0);
    goal.rotation = Eigen::Quaterniond::Identity();
    controller.task_goal_buffer_.writeFromNonRT(goal);
  }
  static void stage_stationary_relative_goal(TaskSpaceImpedanceMitController & controller)
  {
    TaskSpaceImpedanceMitController::Goal goal;
    goal.id = controller.task_last_started_id_ + 1;
    goal.duration = 0.25;
    goal.relative = true;
    goal.translation = Eigen::Vector3d::Zero();
    goal.rotation = Eigen::Quaterniond::Identity();
    controller.task_goal_buffer_.writeFromNonRT(goal);
  }
  static bool capacity_rejected(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.task_capacity_rejected_;
  }
  static std::uint64_t active_task_id(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.task_public_id_.load(std::memory_order_acquire);
  }
  static void set_nonzero_idle_orientation_error(
    TaskSpaceImpedanceMitController & controller, const double radians)
  {
    pinocchio::SE3 measured_pose;
    Eigen::Matrix<double, 6, 7> jacobian;
    ASSERT_TRUE(controller.task_pose_and_jacobian(
      controller.measured(), measured_pose, jacobian));
    controller.idle_pose_ = pinocchio::SE3(
      Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      measured_pose.rotation(),
      measured_pose.translation());
    controller.idle_pose_valid_ = true;
  }
  static pinocchio::SE3 idle_pose(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.idle_pose_;
  }
  static pinocchio::SE3 measured_pose(TaskSpaceImpedanceMitController & controller)
  {
    pinocchio::SE3 pose;
    Eigen::Matrix<double, 6, 7> jacobian;
    EXPECT_TRUE(controller.task_pose_and_jacobian(controller.measured(), pose, jacobian));
    return pose;
  }
  static pinocchio::SE3 task_start_pose(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.task_start_pose_;
  }
  static bool write_task_cycle(
    TaskSpaceImpedanceMitController & controller, const double time,
    const double dt, DirectMitTarget & target)
  {
    return controller.write_task_target(time, dt, target);
  }
  static double velocity_command(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 1].get_value();
  }
  static double command_velocity_limit(const TaskSpaceImpedanceMitController & controller,
    const std::size_t joint)
  {
    return controller.command_velocity_[joint];
  }
  static bool release_active(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.idle_release_active_;
  }
  static double mit_status(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.state_interfaces_[18].get_value();
  }
  static bool safe_stopped(const TaskSpaceImpedanceMitController & controller)
  {
    return controller.safe_stopped_.load(std::memory_order_acquire);
  }
  static void request_cancel(TaskSpaceImpedanceMitController & controller, const std::uint64_t id)
  {
    controller.task_cancel_id_.store(id, std::memory_order_release);
  }
  static bool pose_and_jacobian(
    TaskSpaceImpedanceMitController & controller, const std::array<double, 7> & q,
    pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian)
  {
    return controller.task_pose_and_jacobian(q, pose, jacobian);
  }
  static std::array<double, 7> velocity_reference(
    const TaskSpaceImpedanceMitController & controller,
    const Eigen::Matrix<double, 6, 7> & jacobian, const Eigen::Matrix<double, 6, 1> & twist)
  {
    std::array<double, 7> out{};
    controller.joint_velocity_reference(jacobian, twist, out);
    return out;
  }
  static bool nullspace_torque(
    TaskSpaceImpedanceMitController & controller, const std::array<double, 7> & posture,
    const double kp_null, const double kd_null, Eigen::Matrix<double, 7, 1> & torque,
    Eigen::Matrix<double, 6, 7> & jacobian, Eigen::Matrix<double, 7, 7> & mass)
  {
    const auto q = controller.measured();
    std::array<double, 7> dq{};
    pinocchio::SE3 pose;
    if (!controller.task_pose_and_jacobian(q, pose, jacobian)) return false;
    if (!controller.task_dynamics(jacobian)) return false;
    controller.nullspace_posture_ = posture;
    controller.nullspace_posture_explicit_ = true;
    controller.kp_null_ = kp_null;
    controller.kd_null_ = kd_null;
    mass = controller.task_arm_mass_;
    return controller.nullspace_posture_torque(jacobian, q, dq, torque);
  }
  static void set_joint_limit_guard(
    TaskSpaceImpedanceMitController & controller, const std::array<double, 7> & stiffness,
    const double margin)
  {
    controller.joint_limit_stiffness_ = stiffness;
    controller.joint_limit_margin_ = margin;
  }
  static Eigen::Matrix<double, 7, 1> joint_limit_torque(
    const TaskSpaceImpedanceMitController & controller, const std::array<double, 7> & q)
  {
    Eigen::Matrix<double, 7, 1> torque;
    controller.joint_limit_torque(q, torque);
    return torque;
  }
  static double feedforward_slew(
    const TaskSpaceImpedanceMitController & controller, const std::size_t joint)
  {
    return controller.feedforward_slew_[joint];
  }
  static void set_torque_limit(
    TaskSpaceImpedanceMitController & controller, const double limit)
  {
    controller.torque_limit_.fill(limit);
  }
  static double slew_model_feedforward(
    TaskSpaceImpedanceMitController & controller, const std::size_t joint,
    const double desired, const double dt)
  {
    return controller.slew_model_feedforward(joint, desired, dt);
  }
  static double last_model_feedforward(
    const TaskSpaceImpedanceMitController & controller, const std::size_t joint)
  {
    return controller.task_last_model_feedforward_[joint];
  }
  static void stage_moving_relative_goal(
    TaskSpaceImpedanceMitController & controller)
  {
    TaskSpaceImpedanceMitController::Goal goal;
    goal.id = controller.task_last_started_id_ + 1;
    goal.duration = 1.0;
    goal.relative = true;
    goal.translation = Eigen::Vector3d(0.30, 0.0, 0.0);
    goal.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY());
    controller.task_goal_buffer_.writeFromNonRT(goal);
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
std::string urdf()
{
  std::ostringstream out;
  out << "<robot name='task_mit'><link name='base'/>";
  for (int i = 1; i <= 7; ++i) {
    out << "<link name='link" << i << "'><inertial><mass value='1.0'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01'/></inertial></link>"
        << "<joint name='openarm_joint" << i << "' type='revolute'><parent link='"
        << (i == 1 ? "base" : "link" + std::to_string(i - 1)) << "'/><child link='link" << i
        << "'/><axis xyz='0 1 0'/><limit lower='-3.14' upper='3.14' effort='20' velocity='5'/></joint>";
  }
  out << "<link name='openarm_hand_tcp'/><joint name='tcp_fixed' type='fixed'><parent link='link7'/><child link='openarm_hand_tcp'/></joint>"
      << "<ros2_control name='fake' type='system'><hardware><plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
      << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param><param name='max_stiffness'>500</param><param name='max_damping'>50</param><param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param><param name='safe_hold_damping'>2</param><param name='initial_position'>0.1</param></hardware>";
  for (int i = 1; i <= 7; ++i) {
    out << "<joint name='openarm_joint" << i << "'>";
    for (const auto * name : {"position", "velocity", "stiffness", "damping", "effort"})
      out << "<command_interface name='" << name << "'/>";
    for (const auto * name : {"position", "velocity", "effort"})
      out << "<state_interface name='" << name << "'/>";
    out << "</joint>";
  }
  out << "<gpio name='openarm_arm'>";
  for (const auto * name : {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation", "mit_safe_request_generation"}) out << "<command_interface name='" << name << "'/>";
  for (const auto * name : {"mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation", "mit_status"}) out << "<state_interface name='" << name << "'/>";
  return out.str() + "</gpio></ros2_control></robot>";
}

std::string bimanual_model_urdf()
{
  std::ostringstream out;
  out << "<robot name='openarm_bimanual'><link name='openarm_body_link0'/>";
  for (const auto * side : {"left", "right"}) {
    const std::string prefix = std::string("openarm_") + side + "_";
    out << "<link name='" << prefix << "link0'/>"
        << "<joint name='" << prefix << "mount' type='fixed'><parent link='openarm_body_link0'/><child link='"
        << prefix << "link0'/></joint>";
    for (int i = 1; i <= 7; ++i) {
      out << "<link name='" << prefix << "link" << i
          << "'><inertial><mass value='1.0'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01'/></inertial></link>"
          << "<joint name='" << prefix << "joint" << i << "' type='revolute'><parent link='"
          << prefix << "link" << (i - 1) << "'/><child link='" << prefix << "link" << i
          << "'/><axis xyz='0 1 0'/><limit lower='-3.14' upper='3.14' effort='40' velocity='21'/></joint>";
    }
    out << "<link name='" << prefix << "hand_tcp'/><joint name='" << prefix
        << "tcp_fixed' type='fixed'><parent link='" << prefix << "link7'/><child link='"
        << prefix << "hand_tcp'/></joint>";
  }
  return out.str() + "</robot>";
}

std::string right_bimanual_control_urdf()
{
  auto description = bimanual_model_urdf();
  description.erase(description.rfind("</robot>"));
  std::ostringstream out;
  out << description
      << "<ros2_control name='right_fake' type='system'><hardware>"
      << "<plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
      << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>21</param>"
      << "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
      << "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
      << "<param name='safe_hold_damping'>2</param><param name='initial_position'>0.1</param>"
      << "<param name='arm_side'>right</param></hardware>";
  for (int i = 1; i <= 7; ++i) {
    out << "<joint name='openarm_right_joint" << i << "'>";
    for (const auto * name : {"position", "velocity", "stiffness", "damping", "effort"})
      out << "<command_interface name='" << name << "'/>";
    for (const auto * name : {"position", "velocity", "effort"})
      out << "<state_interface name='" << name << "'/>";
    out << "</joint>";
  }
  out << "<gpio name='openarm_right_arm'>";
  for (const auto * name : {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation", "mit_safe_request_generation"})
    out << "<command_interface name='" << name << "'/>";
  for (const auto * name : {"mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation", "mit_status"})
    out << "<state_interface name='" << name << "'/>";
  return out.str() + "</gpio></ros2_control></robot>";
}

class Fixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}}
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true), executor_, "controller_manager", "/task_zero_test");
    controller_ = std::make_shared<cho_controller_openarm_mit::TaskSpaceImpedanceMitController>();
    ASSERT_TRUE(manager_->add_controller(controller_, "task_space_impedance_mit_controller", "cho_controller_openarm_mit/TaskSpaceImpedanceMitController"));
    const auto set = [&](const char * name, const auto & value) {ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);};
    set("arm", "single"); set("kp", std::vector<double>(7, 5.0)); set("kd", std::vector<double>(7, 0.4)); set("torque_limit", std::vector<double>(7, 3.0));
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE); set("safety_profile_name", "mujoco_sim_safe"); set("robot_description", urdf()); set("ee_frame", "openarm_hand_tcp");
    set("kp_task", std::vector<double>(6, 1.0)); set("kd_task", std::vector<double>(6, 0.1)); set("max_task_wrench", std::vector<double>(6, 1.0));
    set("startup_posture", std::vector<double>{0.0, -0.5, 0.0, 1.2, 0.0, 0.4, 0.0}); set("startup_kp", std::vector<double>{70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0}); set("startup_kd", std::vector<double>{2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5});
    set("startup_duration", 0.30); set("startup_tolerance", 0.05); set("return_to_zero", true); set("return_to_zero_duration", 0.30); set("return_to_zero_tolerance", 0.05);
    set("release_duration", 0.3);
    ASSERT_EQ(manager_->configure_controller("task_space_impedance_mit_controller"), controller_interface::return_type::OK);
    running_ = true; worker_ = std::thread([this] {while (running_) {const auto now = manager_->now(); const auto period = rclcpp::Duration::from_seconds(0.001); manager_->read(now, period); manager_->update(now, period); manager_->write(now, period); std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    ASSERT_EQ(manager_->switch_controller({"task_space_impedance_mit_controller"}, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT), controller_interface::return_type::OK);
  }
  void TearDown() override {running_ = false; if (worker_.joinable()) worker_.join();}
  void stop_control_loop() {running_ = false; if (worker_.joinable()) worker_.join();}
  void cycle(int count) {for (int i = 0; i < count; ++i) {executor_->spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<cho_controller_openarm_mit::TaskSpaceImpedanceMitController> controller_;
  std::atomic<bool> running_{false}; std::thread worker_;
};

TEST_F(Fixture, EmittedJointReferenceIsClampedIntoTheProfilePositionWindow)
{
  // The consumer rejects the entire seven-axis tuple when one q_des leaves the
  // profile window, and joint 4's lower bound is its mechanical zero. A
  // measured position slightly below it must not take the whole arm to SAFE.
  cho_controller_openarm_mit::ArmCommand command;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    command.joints[joint].position =
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_lower(
      *controller_, joint) - 1e-3;
  }
  command.joints[0].position =
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_upper(
    *controller_, 0) + 5.0;
  const auto clamped =
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::clamp_positions(
    *controller_, command);
  EXPECT_DOUBLE_EQ(
    clamped.joints[0].position,
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_upper(
      *controller_, 0));
  for (std::size_t joint = 1; joint < 7; ++joint) {
    EXPECT_DOUBLE_EQ(
      clamped.joints[joint].position,
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_lower(
        *controller_, joint));
  }
}

TEST_F(Fixture, ReturnToZeroSuppressesTheOrdinaryTaskStartupPostureUntilGainHandoffCompletes)
{
  bool observed_handoff = false;
  for (int i = 0; i < 700; ++i) {
    cycle(1);
    if (!cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::return_to_zero_handoff_active(*controller_)) continue;
    observed_handoff = true;
    EXPECT_FALSE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_));
    // The fake begins at 0.1 rad, so a seed-command regression is observable.
    // Every handoff tuple must instead retain the converged nominal-zero q_des.
    for (std::size_t joint = 0; joint < 7; ++joint) {
      EXPECT_NEAR(
        cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_command(*controller_, joint),
        cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::q_reference(*controller_, joint),
        0.005);
      EXPECT_NEAR(
        cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_command(*controller_, joint),
        joint == 3 ? 0.001 : 0.0, 0.03);
      // The handoff ramps stiffness to zero but damping only down to the
      // controller's own kd, which the damped Cartesian mode keeps applying.
      // Ramping damping to zero here would hand the Cartesian law an arm whose
      // null space has no dissipation at all.
      EXPECT_GE(
        cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::damping_command(
          *controller_, joint),
        cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::configured_damping(
          *controller_, joint) - 1e-9);
    }
  }
  EXPECT_TRUE(observed_handoff);
  cycle(100);
  EXPECT_TRUE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_));
}

class ConfigureFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }
};

TEST_F(ConfigureFixture, ExactRealRightReturnToZeroParametersConfigureAgainstBimanualModel)
{
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<controller_manager::ControllerManager>(
    std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true),
    executor, "configure_controller_manager", "/task_zero_configure_test");
  auto controller = std::make_shared<cho_controller_openarm_mit::TaskSpaceImpedanceMitController>();
  ASSERT_TRUE(manager->add_controller(
    controller, "right_real_task_configure_regression",
    "cho_controller_openarm_mit/TaskSpaceImpedanceMitController"));
  const auto set = [&](const char * name, const auto & value) {
      ASSERT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
    };
  set("arm", "right");
  set("safety_backend", "real");
  set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
  set("safety_profile_name", "real_return_to_zero_commissioning");
  set("robot_description", bimanual_model_urdf());
  set("ee_frame", "openarm_right_hand_tcp");
  // Drive-side impedance needs real joint gains: they ARE the Cartesian
  // stiffness now. Kept inside real_return_to_zero_commissioning's ceilings,
  // which is the profile this regression selects.
  set("kp", std::vector<double>{32.0, 30.0, 50.0, 49.0, 10.0, 10.0, 5.0});
  set("kd", std::vector<double>{2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5});
  set("torque_limit", std::vector<double>{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0});
  set("kp_task", std::vector<double>{50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
  set("kd_task", std::vector<double>{6.32, 6.32, 6.32, 0.45, 0.45, 0.45});
  set("max_task_wrench", std::vector<double>{8.0, 8.0, 8.0, 1.0, 1.0, 1.0});
  set("startup_posture", std::vector<double>{0.0, -0.15, 0.0, 1.14, 0.0, 0.38, 0.0});
  set("startup_kp", std::vector<double>{70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0});
  set("startup_kd", std::vector<double>{2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5});
  set("startup_duration", 15.0);
  set("startup_tolerance", 0.05);
  set("return_to_zero", true);
  set("return_to_zero_duration", 2.0);
  set("return_to_zero_tolerance", 0.05);

  EXPECT_EQ(
    manager->configure_controller("right_real_task_configure_regression"),
    controller_interface::return_type::OK);
}

TEST_F(ConfigureFixture, LegacyTauFfLawStillConfiguresWithZeroJointGains)
{
  // The escape hatch back to the pre-redesign law. It has to keep configuring
  // with kp=0: that combination is meaningless under drive-side impedance (the
  // drive would have nothing to push against) and is rejected there, so
  // without this the fallback would be unreachable on hardware exactly when it
  // is wanted - after a bad tuning session.
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<controller_manager::ControllerManager>(
    std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true),
    executor, "legacy_controller_manager", "/task_zero_legacy_test");
  auto controller = std::make_shared<cho_controller_openarm_mit::TaskSpaceImpedanceMitController>();
  ASSERT_TRUE(manager->add_controller(
    controller, "legacy_tau_ff_regression",
    "cho_controller_openarm_mit/TaskSpaceImpedanceMitController"));
  const auto set = [&](const char * name, const auto & value) {
      ASSERT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
    };
  set("arm", "right");
  set("safety_backend", "real");
  set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
  set("safety_profile_name", "real_conservative_commissioning");
  set("robot_description", bimanual_model_urdf());
  set("ee_frame", "openarm_right_hand_tcp");
  set("drive_side_impedance", false);
  set("kp", std::vector<double>(7, 0.0));
  set("kd", std::vector<double>{1.0, 1.0, 0.8, 0.8, 0.3, 0.25, 0.2});
  set("torque_limit", std::vector<double>{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0});
  set("kp_task", std::vector<double>{20.0, 20.0, 20.0, 2.0, 2.0, 2.0});
  set("kd_task", std::vector<double>{10.0, 10.0, 10.0, 0.6, 0.6, 0.6});
  set("max_task_wrench", std::vector<double>{10.0, 10.0, 10.0, 1.5, 1.5, 1.5});
  set("return_to_zero", false);

  EXPECT_EQ(
    manager->configure_controller("legacy_tau_ff_regression"),
    controller_interface::return_type::OK);
}

TEST_F(ConfigureFixture, DriveSideImpedanceRefusesGainsItCannotPushAgainst)
{
  // kp=0 with drive-side impedance is a silent dead loop: q_des would carry the
  // Cartesian error and nothing would act on it. Fail configure instead.
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<controller_manager::ControllerManager>(
    std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true),
    executor, "dead_loop_controller_manager", "/task_zero_dead_loop_test");
  auto controller = std::make_shared<cho_controller_openarm_mit::TaskSpaceImpedanceMitController>();
  ASSERT_TRUE(manager->add_controller(
    controller, "dead_loop_regression",
    "cho_controller_openarm_mit/TaskSpaceImpedanceMitController"));
  const auto set = [&](const char * name, const auto & value) {
      ASSERT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
    };
  set("arm", "right");
  set("safety_backend", "real");
  set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
  set("safety_profile_name", "real_conservative_commissioning");
  set("robot_description", bimanual_model_urdf());
  set("ee_frame", "openarm_right_hand_tcp");
  set("kp", std::vector<double>(7, 0.0));
  set("kd", std::vector<double>{1.0, 1.0, 0.8, 0.8, 0.3, 0.25, 0.2});
  set("torque_limit", std::vector<double>{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0});
  set("return_to_zero", false);

  EXPECT_NE(
    manager->configure_controller("dead_loop_regression"),
    controller_interface::return_type::OK);
}

class OptOutFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
        right_bimanual_control_urdf(), true, true),
      executor_, "opt_out_controller_manager", "/task_zero_opt_out_test");
    controller_ = std::make_shared<cho_controller_openarm_mit::TaskSpaceImpedanceMitController>();
    ASSERT_TRUE(manager_->add_controller(
      controller_, "right_task_space_impedance_mit_controller",
      "cho_controller_openarm_mit/TaskSpaceImpedanceMitController"));
    const auto set = [&](const char * name, const auto & value) {
        ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
      };
    set("arm", "right");
    set("safety_backend", "real");
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "real_conservative_commissioning");
    set("robot_description", right_bimanual_control_urdf());
    set("ee_frame", "openarm_right_hand_tcp");
    set("kp", std::vector<double>{32.0, 30.0, 50.0, 49.0, 15.0, 10.0, 5.0});
    set("kd", std::vector<double>{5.0, 5.0, 5.0, 5.0, 0.7, 0.6, 0.5});
    set("torque_limit", std::vector<double>{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0});
    set("kp_task", std::vector<double>{50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
    set("kd_task", std::vector<double>{6.32, 6.32, 6.32, 0.45, 0.45, 0.45});
    set("max_task_wrench", std::vector<double>{8.0, 8.0, 8.0, 1.0, 1.0, 1.0});
    // These legacy posture values remain in the YAML for compatibility, but
    // opting out of RTZ must not validate or execute them.
    set("startup_posture", std::vector<double>{0.0, -0.15, 0.0, 1.14, 0.0, 0.38, 0.0});
    set("startup_kp", std::vector<double>{3.0, 3.0, 3.0, 2.0, 0.5, 0.5, 0.5});
    set("startup_kd", std::vector<double>{0.4, 0.4, 0.3, 0.3, 0.1, 0.1, 0.08});
    set("startup_duration", 15.0);
    set("startup_tolerance", 0.05);
    set("return_to_zero", false);
    ASSERT_EQ(
      manager_->configure_controller("right_task_space_impedance_mit_controller"),
      controller_interface::return_type::OK);
    running_ = true;
    worker_ = std::thread([this] {
      while (running_) {
        const auto now = manager_->now();
        const auto period = rclcpp::Duration::from_seconds(0.001);
        manager_->read(now, period);
        manager_->update(now, period);
        manager_->write(now, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    ASSERT_EQ(
      manager_->switch_controller(
        {"right_task_space_impedance_mit_controller"}, {},
        controller_manager_msgs::srv::SwitchController::Request::STRICT),
      controller_interface::return_type::OK);
  }

  void TearDown() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

  void cycle(int count)
  {
    for (int i = 0; i < count; ++i) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<cho_controller_openarm_mit::TaskSpaceImpedanceMitController> controller_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

TEST_F(OptOutFixture, SkipsJointStartupAndHoldsCurrentCartesianPoseWithDriveSideStiffness)
{
  for (int i = 0; i < 300 &&
       !cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(
         *controller_); ++i) {
    cycle(1);
  }
  ASSERT_TRUE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(
    *controller_));
  EXPECT_FALSE(
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::startup_active(
      *controller_));

  const auto idle =
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::idle_pose(*controller_);
  const auto measured =
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::measured_pose(*controller_);
  EXPECT_TRUE(idle.translation().isApprox(measured.translation(), 1e-9));
  EXPECT_TRUE(idle.rotation().isApprox(measured.rotation(), 1e-9));
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_NEAR(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::measured_position(
        *controller_, joint),
      0.1, 1e-6);
    EXPECT_DOUBLE_EQ(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::stiffness_command(
        *controller_, joint),
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::configured_stiffness(
        *controller_, joint));
    EXPECT_DOUBLE_EQ(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::damping_command(
        *controller_, joint),
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::configured_damping(
        *controller_, joint));
  }
}

TEST_F(Fixture, RelativeGoalBeyondFormerCartesianCapsIsAcceptedAndExecuted)
{
  for (int i = 0; i < 900 &&
       !cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_); ++i) {
    cycle(1);
  }
  ASSERT_TRUE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_));

  EXPECT_EQ(
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::
      accept_unbounded_relative_goal(*controller_),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::stage_unbounded_relative_goal(*controller_);
  cycle(20);

  EXPECT_FALSE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::capacity_rejected(*controller_));
  EXPECT_NE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::active_task_id(*controller_), 0U);
}

TEST_F(Fixture, AbsoluteGoalWithoutCartesianCapsIsAcceptedAndExecuted)
{
  for (int i = 0; i < 900 &&
       !cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_); ++i) {
    cycle(1);
  }
  ASSERT_TRUE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_));

  // Absolute goals use the same direct Cartesian-torque path as relative goals.
  cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::
    stage_absolute_goal_without_cartesian_caps(*controller_);
  cycle(20);

  EXPECT_FALSE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::capacity_rejected(*controller_));
  EXPECT_NE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::active_task_id(*controller_), 0U);
}

TEST_F(Fixture, ActiveTaskDrivesCartesianErrorThroughTheDriveReferenceOffset)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  ASSERT_TRUE(Access::drive_side_impedance(*controller_));

  std::array<double, 7> idle_reference{};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    idle_reference[joint] = Access::q_reference(*controller_, joint);
  }

  Access::stage_unbounded_relative_goal(*controller_);
  cycle(100);

  ASSERT_NE(Access::active_task_id(*controller_), 0U);
  bool observed_velocity_reference = false;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    // The Cartesian error rides q_des as J^+ (x_des ominus x) and the drive's
    // own kp closes the loop, so the impedance never crosses the 200 Hz CAN
    // cycle. That offset is the only thing bounding the impedance torque:
    // torque_limit clamps the effort field, and kp*(q_des - q) is added by the
    // drive downstream of it.
    const double offset = Access::position_command(*controller_, joint) -
      Access::measured_position(*controller_, joint);
    EXPECT_TRUE(std::isfinite(offset));
    EXPECT_LE(
      std::abs(offset), Access::reference_offset_limit(*controller_, joint) + 1e-12);

    EXPECT_DOUBLE_EQ(
      Access::stiffness_command(*controller_, joint),
      Access::configured_stiffness(*controller_, joint));
    EXPECT_DOUBLE_EQ(
      Access::damping_command(*controller_, joint),
      Access::configured_damping(*controller_, joint));

    // dq_des = J^+ v_des keeps the in-motor kd term tracking the commanded
    // motion instead of braking it, and stays inside the profile window the
    // consumer checks.
    const double velocity = Access::velocity_command(*controller_, joint);
    EXPECT_TRUE(std::isfinite(velocity));
    EXPECT_LE(
      std::abs(velocity), Access::command_velocity_limit(*controller_, joint) + 1e-12);
    observed_velocity_reference = observed_velocity_reference || std::abs(velocity) > 1e-9;

    // No joint trajectory is generated: task_q_ref_ is still the latched idle
    // reference, so nothing but the Cartesian error moves the arm.
    EXPECT_DOUBLE_EQ(Access::q_reference(*controller_, joint), idle_reference[joint]);
    // tau_ff must no longer carry the Cartesian PD. In this fixture the model
    // term is negligible, so what used to make this field large was precisely
    // the J^T (Kx e + Dx edot) that has moved into the drive.
    EXPECT_LE(
      std::abs(Access::effort_command(*controller_, joint)),
      std::abs(Access::configured_stiffness(*controller_, joint)) *
      Access::reference_offset_limit(*controller_, joint));
  }
  EXPECT_TRUE(observed_velocity_reference);
}

TEST_F(Fixture, FrictionTermIsSilentUntilItIsDeliberatelyTurnedOn)
{
  // friction_scale defaults to 0, so a config that carries an unidentified
  // friction_level must behave exactly as if the term did not exist. Anything
  // else would put an unexplained constant push on the arm the moment someone
  // wrote a number into the config.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  Eigen::Matrix<double, 7, 1> torque;
  const std::array<double, 7> moving{1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
  Access::friction(*controller_, moving, moving, torque);
  EXPECT_DOUBLE_EQ(torque.cwiseAbs().maxCoeff(), 0.0);
}

TEST_F(Fixture, FrictionTermNeverExceedsTheIdentifiedLevelAndVanishesAtRest)
{
  // The two properties that keep this from being a torque source of its own:
  // tanh saturates at the identified level so it can never exceed it, and it
  // passes through zero so a stationary joint gets no push.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  const std::array<double, 7> level{1.2, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2};
  Access::set_friction(*controller_, level, 0.7, 0.1);
  Eigen::Matrix<double, 7, 1> torque;

  const std::array<double, 7> still{};
  Access::friction(*controller_, still, still, torque);
  EXPECT_DOUBLE_EQ(torque.cwiseAbs().maxCoeff(), 0.0) << "a joint at rest must not be pushed";

  const std::array<double, 7> fast{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  Access::friction(*controller_, fast, fast, torque);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_LE(std::abs(torque[joint]), 0.7 * level[joint] + 1e-9);
    EXPECT_GT(torque[joint], 0.0) << "must oppose motion, not assist it";
  }

  // Sign follows the velocity, and the term is odd through zero.
  const std::array<double, 7> back{-5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0};
  Eigen::Matrix<double, 7, 1> reversed;
  Access::friction(*controller_, back, back, reversed);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_NEAR(reversed[joint], -torque[joint], 1e-12);
  }
}

TEST_F(Fixture, StribeckShapingBacksOffOnceTheJointIsSliding)
{
  // The hardware failure this exists for: a level identified at breakaway is
  // larger than sliding friction, so a flat law over-compensates while moving
  // and the excess acts as negative damping. Hand-guided at scale 0.7 the arm
  // slid away under a harder push while every stationary check passed.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  const std::array<double, 7> level{1.2, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2};
  Access::set_friction(*controller_, level, 1.0, 0.05);
  Access::set_friction_stribeck(*controller_, 0.6, 0.15);
  Eigen::Matrix<double, 7, 1> slow, fast;
  Access::friction(*controller_, {}, std::array<double, 7>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, slow);
  Access::friction(*controller_, {}, std::array<double, 7>{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0}, fast);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    // Sliding settles at the kinetic level, which is what stops the runaway.
    EXPECT_NEAR(fast[joint], 0.6 * level[joint], 1e-3);
    // Near breakaway the term is still above the kinetic level, which is what
    // keeps the assist that made hand guiding smooth in the first place.
    EXPECT_GT(slow[joint], fast[joint]);
    // And it never exceeds the identified breakaway level.
    EXPECT_LE(std::abs(slow[joint]), level[joint] + 1e-9);
  }
}

TEST_F(Fixture, KineticRatioOfOneIsExactlyTheUnshapedLaw)
{
  // Default, and what every other friction test assumes. Shaping has to be
  // opt-in or enabling it would silently change an already-tuned arm.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  const std::array<double, 7> level{1.2, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2};
  Access::set_friction(*controller_, level, 0.7, 0.1);
  Access::set_friction_stribeck(*controller_, 1.0, 0.15);
  Eigen::Matrix<double, 7, 1> torque;
  Access::friction(*controller_, {}, std::array<double, 7>{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}, torque);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_NEAR(torque[joint], 0.7 * level[joint], 1e-6);
  }
}

TEST_F(Fixture, FrictionSourceDecidesWhichVelocitySignIsUsed)
{
  // Hand guiding has dq_des identically zero, so a reference-sourced term is
  // silent there and the mode would test nothing. This is why the source is
  // selectable rather than fixed.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  Access::set_friction(*controller_, {1.2, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2}, 0.7, 0.1);
  const std::array<double, 7> pushed{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const std::array<double, 7> none{};
  Eigen::Matrix<double, 7, 1> torque;

  Access::set_friction_source_measured(*controller_, false);
  Access::friction(*controller_, pushed, none, torque);
  EXPECT_DOUBLE_EQ(torque.cwiseAbs().maxCoeff(), 0.0)
    << "reference source must ignore measured motion";

  Access::set_friction_source_measured(*controller_, true);
  Access::friction(*controller_, pushed, none, torque);
  EXPECT_GT(torque.cwiseAbs().maxCoeff(), 0.0)
    << "measured source is what makes hand guiding testable";
}

TEST_F(Fixture, ReferenceOffsetSaturationKeepsItsCartesianDirection)
{
  // A saturating offset must shrink, not tilt. Clipping each joint against its
  // own bound changes which way the arm goes, and on hardware that drove the
  // arm 100 mm opposite the command: the Lambda-weighted wrench asked for
  // [-19.9, 6.2, 118.8] N, per-axis clipping at 10 N returned [-10, 6.2, 10],
  // which is 40 degrees away from where it was pointing.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  // A large error saturates; a small one along the same direction does not.
  Access::set_nonzero_idle_orientation_error(*controller_, 2.5);
  cho_controller_openarm_mit::DirectMitTarget big;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.0, 0.001, big));
  Access::set_nonzero_idle_orientation_error(*controller_, 0.02);
  cho_controller_openarm_mit::DirectMitTarget small;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.0, 0.001, small));

  Eigen::Matrix<double, 7, 1> a, b;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    a[joint] = big.position[joint] - Access::measured_position(*controller_, joint);
    b[joint] = small.position[joint] - Access::measured_position(*controller_, joint);
    EXPECT_LE(std::abs(a[joint]), Access::reference_offset_limit(*controller_, joint) + 1e-12);
  }
  ASSERT_GT(a.norm(), 1e-9);
  ASSERT_GT(b.norm(), 1e-9);
  // Same direction: the saturated one is a scaled copy of the unsaturated one.
  const double cosine = a.dot(b) / (a.norm() * b.norm());
  EXPECT_GT(cosine, 0.999) << "saturation tilted the offset by "
                           << std::acos(std::min(1.0, cosine)) * 180.0 / M_PI << " deg";
  EXPECT_GT(a.norm(), b.norm());
}

TEST_F(Fixture, CartesianErrorReachesTheDriveReferenceAndStaysBounded)
{
  // The claim the redesign rests on: a Cartesian error becomes a bounded joint
  // reference offset, which is what the drive's kp acts on. Injected rather
  // than waited for, so the test does not depend on where an interpolated goal
  // happens to be after N cycles.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  ASSERT_TRUE(Access::drive_side_impedance(*controller_));
  stop_control_loop();

  Access::set_nonzero_idle_orientation_error(*controller_, 0.5);
  cho_controller_openarm_mit::DirectMitTarget target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.0, 0.001, target));

  bool moved = false;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const double offset = target.position[joint] - Access::measured_position(*controller_, joint);
    EXPECT_TRUE(std::isfinite(offset));
    EXPECT_LE(std::abs(offset), Access::reference_offset_limit(*controller_, joint) + 1e-12);
    moved = moved || std::abs(offset) > 1e-9;
  }
  EXPECT_TRUE(moved);
}

TEST_F(Fixture, ReferenceOffsetSaturatesRatherThanTrackingAHugeCartesianError)
{
  // A large error must not become a large q_des: the drive multiplies that
  // offset by kp and adds the result after this controller's torque_limit, so
  // the clamp is the only bound on the impedance torque.
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  Access::set_nonzero_idle_orientation_error(*controller_, 2.5);
  cho_controller_openarm_mit::DirectMitTarget target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.0, 0.001, target));

  bool saturated = false;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const double limit = Access::reference_offset_limit(*controller_, joint);
    const double offset = target.position[joint] - Access::measured_position(*controller_, joint);
    EXPECT_LE(std::abs(offset), limit + 1e-12);
    saturated = saturated || std::abs(std::abs(offset) - limit) < 1e-9;
  }
  EXPECT_TRUE(saturated);
}

TEST_F(Fixture, CompletedTaskLatchesCartesianIdleWithoutSnappingToOldJointReference)
{
  for (int i = 0; i < 900 &&
       !cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_); ++i) {
    cycle(1);
  }
  ASSERT_TRUE(cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::ready(*controller_));

  std::array<double, 7> idle_reference{};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    idle_reference[joint] =
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::q_reference(
      *controller_, joint);
  }
  cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::
    stage_stationary_relative_goal(*controller_);
  cycle(400);

  EXPECT_EQ(
    cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::active_task_id(*controller_),
    0U);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_DOUBLE_EQ(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::stiffness_command(
        *controller_, joint),
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::configured_stiffness(
        *controller_, joint));
    EXPECT_DOUBLE_EQ(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::damping_command(
        *controller_, joint),
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::configured_damping(
        *controller_, joint));
    EXPECT_DOUBLE_EQ(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::q_reference(
        *controller_, joint),
      idle_reference[joint]);
    EXPECT_NEAR(
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::position_command(
        *controller_, joint),
      cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess::measured_position(
        *controller_, joint),
      1e-6);
  }
}

TEST_F(Fixture, NonzeroCartesianErrorTransitionsPreserveReferenceAndReleaseByBlending)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  constexpr double dt = 0.001;
  Access::set_nonzero_idle_orientation_error(*controller_, 0.5);
  const auto prior_reference = Access::idle_pose(*controller_);

  cho_controller_openarm_mit::DirectMitTarget idle_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.0, dt, idle_target));
  std::array<double, 7> model_before{};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    model_before[joint] = Access::last_model_feedforward(*controller_, joint);
  }

  Access::stage_stationary_relative_goal(*controller_);
  cho_controller_openarm_mit::DirectMitTarget goal_start_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.001, dt, goal_start_target));
  const auto action_start = Access::task_start_pose(*controller_);
  EXPECT_NEAR((action_start.translation() - prior_reference.translation()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
    pinocchio::log3(action_start.rotation().transpose() * prior_reference.rotation()).norm(),
    0.0, 1e-12);
  // Only the model feed-forward is rate-limited across the idle/action
  // boundary; the Cartesian feedback follows the (continuous) reference.
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const double allowed = Access::feedforward_slew(*controller_, joint) * dt;
    EXPECT_LE(
      std::abs(Access::last_model_feedforward(*controller_, joint) - model_before[joint]),
      allowed + 1e-12);
  }

  // The 0.5 rad error exceeds terminal tolerance, so this cycle takes the
  // timeout path. The reference is not snapped to the measured pose: it
  // starts a bounded blend from the loaded reference.
  cho_controller_openarm_mit::DirectMitTarget timeout_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 12.30, dt, timeout_target));
  EXPECT_EQ(Access::active_task_id(*controller_), 0U);
  EXPECT_TRUE(Access::release_active(*controller_));
  EXPECT_NEAR(
    pinocchio::log3(
      Access::idle_pose(*controller_).rotation().transpose() * prior_reference.rotation()).norm(),
    0.0, 1e-9);

  cho_controller_openarm_mit::DirectMitTarget first_release_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 12.301, dt, first_release_target));
  EXPECT_TRUE(Access::release_active(*controller_));
  EXPECT_LT(
    pinocchio::log3(
      Access::idle_pose(*controller_).rotation().transpose() * prior_reference.rotation()).norm(),
    1e-2);

  // After the release window the reference coincides with the measured pose,
  // idle resumes, and the action server stayed available: no SAFE request.
  cho_controller_openarm_mit::DirectMitTarget settled_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 12.302, 1.0, settled_target));
  EXPECT_FALSE(Access::release_active(*controller_));
  const auto measured = Access::measured_pose(*controller_);
  const auto idle = Access::idle_pose(*controller_);
  EXPECT_NEAR((idle.translation() - measured.translation()).norm(), 0.0, 1e-9);
  EXPECT_NEAR(pinocchio::log3(idle.rotation().transpose() * measured.rotation()).norm(), 0.0, 1e-9);
  EXPECT_TRUE(Access::ready(*controller_));
}

TEST_F(Fixture, ModelFeedforwardSlewStaysInsideControllerTorqueLimit)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  constexpr double dt = 0.001;
  constexpr double controller_limit = 0.20;
  Access::set_torque_limit(*controller_, controller_limit);
  const double allowed = Access::feedforward_slew(*controller_, 0) * dt;
  ASSERT_LT(allowed, controller_limit);

  double value = 0.0;
  for (int cycle_index = 0; cycle_index < 12; ++cycle_index) {
    value = Access::slew_model_feedforward(*controller_, 0, 1.0, dt);
  }
  // The tracker is bounded by the controller torque_limit as well as the
  // profile tau_ff magnitude, so a saturated model term cannot wind it up
  // past what the final clamp emits.
  EXPECT_NEAR(value, controller_limit, 1e-12);
  EXPECT_NEAR(Access::last_model_feedforward(*controller_, 0), controller_limit, 1e-12);

  double previous = value;
  for (int cycle_index = 0; cycle_index < 4; ++cycle_index) {
    value = Access::slew_model_feedforward(*controller_, 0, -1.0, dt);
    EXPECT_LE(std::abs(value - previous), allowed + 1e-12);
    EXPECT_NEAR(Access::last_model_feedforward(*controller_, 0), value, 1e-12);
    previous = value;
  }
  EXPECT_NEAR(value, -controller_limit, 1e-12);
}

TEST_F(Fixture, MovingGoalPreemptionContinuesFromCommandedReferenceWithVelocityReference)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  constexpr double dt = 0.001;
  cho_controller_openarm_mit::DirectMitTarget idle_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.0, dt, idle_target));
  // Idle has zero desired twist, so the MIT velocity field is zero and the
  // in-motor kd term is pure dissipation.
  for (std::size_t joint = 0; joint < 7; ++joint) EXPECT_DOUBLE_EQ(idle_target.velocity[joint], 0.0);

  Access::stage_moving_relative_goal(*controller_);
  cho_controller_openarm_mit::DirectMitTarget start_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.001, dt, start_target));

  // At u=0.5 the trajectory has nonzero desired angular velocity, so
  // dq_des = J^+ v_des is nonzero and bounded by the profile command velocity.
  cho_controller_openarm_mit::DirectMitTarget moving_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.501, dt, moving_target));
  bool observed_velocity_reference = false;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_TRUE(std::isfinite(moving_target.velocity[joint]));
    EXPECT_LE(
      std::abs(moving_target.velocity[joint]),
      Access::command_velocity_limit(*controller_, joint) + 1e-12);
    observed_velocity_reference =
      observed_velocity_reference || std::abs(moving_target.velocity[joint]) > 1e-9;
  }
  EXPECT_TRUE(observed_velocity_reference);
  const auto commanded_reference = Access::idle_pose(*controller_);

  // The preempting trajectory begins from the exact commanded reference, not
  // from measured pose, and at u=0 its desired twist and velocity reference
  // are zero.
  Access::stage_stationary_relative_goal(*controller_);
  cho_controller_openarm_mit::DirectMitTarget preempt_target;
  ASSERT_TRUE(Access::write_task_cycle(*controller_, 10.502, dt, preempt_target));
  const auto replacement_start = Access::task_start_pose(*controller_);
  EXPECT_NEAR(
    (replacement_start.translation() - commanded_reference.translation()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
    pinocchio::log3(
      replacement_start.rotation().transpose() * commanded_reference.rotation()).norm(),
    0.0, 1e-12);
  for (std::size_t joint = 0; joint < 7; ++joint) EXPECT_NEAR(preempt_target.velocity[joint], 0.0, 1e-12);
}

TEST_F(Fixture, CancelReleasesReferenceToMeasuredPoseWithoutSafeStop)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));

  Access::stage_moving_relative_goal(*controller_);
  cycle(100);
  const auto id = Access::active_task_id(*controller_);
  ASSERT_NE(id, 0U);
  Access::request_cancel(*controller_, id);
  cycle(50);
  // Cancel terminalizes the goal and releases the reference; it no longer
  // hands the arm to the hardware SAFE hold, so the session stays ACTIVE and
  // the action server stays ready.
  EXPECT_EQ(Access::active_task_id(*controller_), 0U);
  EXPECT_TRUE(Access::ready(*controller_));
  EXPECT_FALSE(Access::safe_stopped(*controller_));
  EXPECT_DOUBLE_EQ(
    Access::mit_status(*controller_),
    static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE));

  // release_duration is 0.3 s in this fixture and the loop runs at ~1 kHz.
  cycle(1200);
  stop_control_loop();
  EXPECT_FALSE(Access::release_active(*controller_));
  EXPECT_DOUBLE_EQ(
    Access::mit_status(*controller_),
    static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE));
  const auto measured = Access::measured_pose(*controller_);
  const auto idle = Access::idle_pose(*controller_);
  EXPECT_NEAR((idle.translation() - measured.translation()).norm(), 0.0, 1e-6);
  EXPECT_NEAR(pinocchio::log3(idle.rotation().transpose() * measured.rotation()).norm(), 0.0, 1e-6);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_DOUBLE_EQ(
      Access::stiffness_command(*controller_, joint),
      Access::configured_stiffness(*controller_, joint));
    EXPECT_DOUBLE_EQ(
      Access::damping_command(*controller_, joint), Access::configured_damping(*controller_, joint));
    EXPECT_NEAR(
      Access::position_command(*controller_, joint), Access::measured_position(*controller_, joint),
      1e-6);
  }
}

TEST_F(Fixture, VelocityReferenceSolvesRangeSpaceTwistWithinCommandVelocity)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  std::array<double, 7> q{};
  for (std::size_t joint = 0; joint < 7; ++joint) q[joint] = Access::measured_position(*controller_, joint);
  pinocchio::SE3 pose;
  Eigen::Matrix<double, 6, 7> jacobian;
  ASSERT_TRUE(Access::pose_and_jacobian(*controller_, q, pose, jacobian));
  const Eigen::Matrix<double, 7, 1> seed = Eigen::Matrix<double, 7, 1>::Constant(0.1);
  const Eigen::Matrix<double, 6, 1> twist = jacobian * seed;
  ASSERT_GT(twist.norm(), 1e-6);

  // A range-space twist is reproduced by J * dq_des up to the DLS damping.
  const auto dq_des = Access::velocity_reference(*controller_, jacobian, twist);
  Eigen::Matrix<double, 7, 1> dq;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    dq[joint] = dq_des[joint];
    EXPECT_LE(std::abs(dq_des[joint]), Access::command_velocity_limit(*controller_, joint) + 1e-12);
  }
  EXPECT_LT((jacobian * dq - twist).norm(), 1e-2 * twist.norm());

  const auto zero = Access::velocity_reference(
    *controller_, jacobian, Eigen::Matrix<double, 6, 1>::Zero());
  for (std::size_t joint = 0; joint < 7; ++joint) EXPECT_DOUBLE_EQ(zero[joint], 0.0);
}

TEST_F(Fixture, NullspacePostureTorqueProducesNoTaskAcceleration)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  std::array<double, 7> posture{};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    posture[joint] = Access::measured_position(*controller_, joint) + 0.2;
  }
  Eigen::Matrix<double, 7, 1> torque;
  Eigen::Matrix<double, 6, 7> jacobian;
  Eigen::Matrix<double, 7, 7> mass;
  ASSERT_TRUE(Access::nullspace_torque(*controller_, posture, 10.0, 1.0, torque, jacobian, mass));
  EXPECT_GT(torque.norm(), 1e-6);

  // Dynamically consistent: the posture torque accelerates only the null
  // space, so J M^-1 tau_null vanishes up to the Lambda regularization, while
  // the unprojected posture acceleration would move the task frame.
  const Eigen::Matrix<double, 6, 1> task_acceleration = jacobian * mass.llt().solve(torque);
  const Eigen::Matrix<double, 7, 1> unprojected = Eigen::Matrix<double, 7, 1>::Constant(10.0 * 0.2);
  const double reference = (jacobian * unprojected).norm();
  ASSERT_GT(reference, 1e-6);
  EXPECT_LT(task_acceleration.norm(), 1e-3 * reference);
}

TEST_F(Fixture, JointLimitSpringActsOnlyInsideTheMarginBand)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  for (int i = 0; i < 900 && !Access::ready(*controller_); ++i) cycle(1);
  ASSERT_TRUE(Access::ready(*controller_));
  stop_control_loop();

  std::array<double, 7> stiffness{};
  stiffness.fill(10.0);
  Access::set_joint_limit_guard(*controller_, stiffness, 0.05);
  std::array<double, 7> q{};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    q[joint] = 0.5 * (Access::position_lower(*controller_, joint) + Access::position_upper(*controller_, joint));
  }
  auto torque = Access::joint_limit_torque(*controller_, q);
  for (std::size_t joint = 0; joint < 7; ++joint) EXPECT_DOUBLE_EQ(torque[joint], 0.0);

  // Inside the lower band the spring pushes back toward the free range and is
  // continuous at the band edge (0.03 rad of penetration at 10 N*m/rad).
  q[3] = Access::position_lower(*controller_, 3) + 0.02;
  torque = Access::joint_limit_torque(*controller_, q);
  EXPECT_NEAR(torque[3], 10.0 * 0.03, 1e-12);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    if (joint != 3) EXPECT_DOUBLE_EQ(torque[joint], 0.0);
  }
  q[3] = Access::position_upper(*controller_, 3) - 0.02;
  torque = Access::joint_limit_torque(*controller_, q);
  EXPECT_NEAR(torque[3], -10.0 * 0.03, 1e-12);
}

// The real adapter validates every emitted tuple against the per-joint TASK
// ceilings (kp_max/kd_max) and drops the whole arm to SAFE on the first tuple
// above them. This controller validates the homing gains against the separate
// return_to_zero ceilings, and under real_conservative_commissioning the two
// disagree on joints 2, 3, 4 and 7 (kp_max 45/55/50/5 against homing
// 70/70/60/10). On hardware, 2026-09-05, the joint 7 ramp crossed kp_max = 5
// exactly 2.5 s after activation, the tuple was rejected and the controller
// latched FAULT. Homing must therefore never emit above the task ceiling.
class HardwareCeilingFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}}
  void SetUp() override
  {
    profile_ = cho_openarm_mit_core::load_safety_profile_file(
      OPENARM_SAFETY_PROFILE_SOURCE, "real_conservative_commissioning",
      cho_openarm_mit_core::SafetyBackend::REAL, "single");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true), executor_,
      "controller_manager", "/task_zero_ceiling_test");
    controller_ = std::make_shared<cho_controller_openarm_mit::TaskSpaceImpedanceMitController>();
    ASSERT_TRUE(manager_->add_controller(
      controller_, "task_space_impedance_mit_controller",
      "cho_controller_openarm_mit/TaskSpaceImpedanceMitController"));
    const auto set = [&](const char * name, const auto & value) {
        ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
      };
    set("arm", "single");
    set("safety_backend", "real");
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "real_conservative_commissioning");
    set("robot_description", urdf());
    set("ee_frame", "openarm_hand_tcp");
    // Task gains sit exactly on the task ceilings, so the homing ramp toward
    // the (higher) upstream homing gains would leave the ceiling on its very
    // first cycle on joints 2 and 7 if nothing clamped it.
    set("kp", std::vector<double>(profile_.kp_max.begin(), profile_.kp_max.end()));
    set("kd", std::vector<double>(profile_.kd_max.begin(), profile_.kd_max.end()));
    set("torque_limit", std::vector<double>{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0});
    set("kp_task", std::vector<double>{40.0, 40.0, 40.0, 3.0, 3.0, 3.0});
    set("kd_task", std::vector<double>(6, 0.0));
    set("max_task_wrench", std::vector<double>{10.0, 10.0, 10.0, 1.5, 1.5, 1.5});
    set("startup_posture", std::vector<double>{0.0, -0.15, 0.0, 1.14, 0.0, 0.38, 0.0});
    set("startup_kp", std::vector<double>{70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0});
    set("startup_kd", std::vector<double>{2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5});
    set("startup_duration", 0.30); set("startup_tolerance", 0.05);
    set("return_to_zero", true); set("return_to_zero_duration", 0.30);
    set("return_to_zero_tolerance", 0.05);
    set("release_duration", 0.3);
    ASSERT_EQ(manager_->configure_controller("task_space_impedance_mit_controller"),
      controller_interface::return_type::OK);
    running_ = true;
    worker_ = std::thread([this] {
        while (running_) {
          const auto now = manager_->now();
          const auto period = rclcpp::Duration::from_seconds(0.001);
          manager_->read(now, period); manager_->update(now, period); manager_->write(now, period);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    ASSERT_EQ(manager_->switch_controller({"task_space_impedance_mit_controller"}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT),
      controller_interface::return_type::OK);
  }
  void TearDown() override {running_ = false; if (worker_.joinable()) worker_.join();}
  void cycle(int count) {for (int i = 0; i < count; ++i) {executor_->spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
  cho_openarm_mit_core::SafetyProfile profile_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<cho_controller_openarm_mit::TaskSpaceImpedanceMitController> controller_;
  std::atomic<bool> running_{false}; std::thread worker_;
};

TEST_F(HardwareCeilingFixture, ReturnToZeroRampNeverEmitsAboveTheHardwarePerJointCeiling)
{
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceMitControllerTestAccess;
  // Sanity: the profile really does disagree with the homing gains, otherwise
  // this test would pass without exercising the clamp.
  ASSERT_LT(profile_.kp_max[1], 70.0);
  ASSERT_LT(profile_.kp_max[6], 10.0);
  bool observed_startup = false, observed_handoff = false;
  for (int i = 0; i < 1500 && !Access::ready(*controller_); ++i) {
    cycle(1);
    observed_startup = observed_startup || Access::startup_active(*controller_);
    observed_handoff = observed_handoff || Access::return_to_zero_handoff_active(*controller_);
    for (std::size_t joint = 0; joint < 7; ++joint) {
      EXPECT_LE(Access::stiffness_command(*controller_, joint), profile_.kp_max[joint] + 1e-9)
        << "joint " << joint + 1 << " cycle " << i;
      EXPECT_LE(Access::damping_command(*controller_, joint), profile_.kd_max[joint] + 1e-9)
        << "joint " << joint + 1 << " cycle " << i;
    }
  }
  EXPECT_TRUE(observed_startup);
  EXPECT_TRUE(observed_handoff);
  // Clamping the ramp must not have cost the controller its way into Cartesian
  // control: the arm still homes and the action server still opens.
  EXPECT_TRUE(Access::ready(*controller_));
  EXPECT_EQ(Access::mit_status(*controller_), 1.0);
}
}  // namespace
