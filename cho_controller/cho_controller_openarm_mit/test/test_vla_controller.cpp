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

// controller_manager fixture for VlaController.
//
// What this proves is the wiring, not the chunk arithmetic: splicing, sampling,
// validation and watchdog transitions are covered exhaustively in cho_vla_core's
// own gtests without a fixture. Here the questions are the ones only a real
// controller_manager can answer -- does the MIT session/ACK/lease protocol still
// hold with a VLA reference in it, does the startup ramp still gate the action
// server, does a NaN chunk get refused before it reaches a command interface,
// does a quiet stream hold instead of dropping to SAFE.
//
// Also the host-side halves of the 2026-10-08 fixes (tests named AC<n>_...):
// joint chunks refused under the legacy law (AC2), ActionChunk.relative_frame
// parsed and composed (AC3), max_task_ang_acc wired to the limiter (AC5), and
// the limiter re-seeded when a goal switches action space (AC8), that re-seed
// starting from the COMMANDED joints and rates rather than the measurement or
// rest (AC11), the legacy-law refusal reported in telemetry under its own
// reason (AC13), and that re-seed kept inside the joint window and stamped
// with the previous cycle's time (AC18); and the 2026-10-13 stream rules: the
// watchdog waits out the buffered horizon (AC10, S15), the observation-stamp
// window and chunks with nothing left to play (AC5, S8), control_dt <= 0
// refused (AC6, C16), the relative-chunk anchor taken from the command as
// emitted, against a kinematic wall (S14, WallPlant), and bounded_dt()'s
// period rule (BoundedDt_). Expected values come from
// hand-worked kinematics of the test URDF (tcp_pose(), tcp_twist()), not from
// the controller's own model.
#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cho_interfaces/action/vision_language_action.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <cho_interfaces/msg/vla_telemetry.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcutils/logging.h>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/vla_controller.hpp"

namespace cho_controller_openarm_mit
{
struct VlaControllerTestAccess
{
  static bool ready(const VlaController & controller)
  {
    return controller.task_ready_.load(std::memory_order_acquire);
  }
  static double command(const VlaController & controller, const std::size_t index)
  {
    return controller.command_interfaces_[index].get_value();
  }
  static double position_command(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint].get_value();
  }
  // The MIT tuple's dq_des for `joint`.
  static double velocity_command(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 1].get_value();
  }
  // Whether the 35 joint commands and 14 joint states are loaned (false once
  // deactivated, when the vectors are released).
  static bool loaned(const VlaController & controller)
  {
    return controller.command_interfaces_.size() >= 35 && controller.state_interfaces_.size() >= 14;
  }
  static double stiffness(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 2].get_value();
  }
  static double effort(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 4].get_value();
  }
  static double measured(const VlaController & controller, const std::size_t joint)
  {
    return controller.state_interfaces_[2 * joint].get_value();
  }
  static int stream_state(const VlaController & controller)
  {
    return static_cast<int>(controller.session_.telemetry().stream_state);
  }
  static std::uint64_t accepted(const VlaController & controller)
  {
    return controller.session_.telemetry().chunks_accepted;
  }
  static std::uint64_t rejected(const VlaController & controller)
  {
    return controller.session_.telemetry().chunks_rejected;
  }
  // Node clock minus action clock [s], as the control thread last refreshed it.
  static double action_time_offset(const VlaController & controller)
  {
    return controller.action_time_offset_.load(std::memory_order_acquire);
  }
  static double reference_offset_limit(const VlaController & controller, std::size_t joint)
  {
    return controller.reference_offset_limit_[joint];
  }
  // Inherited from TaskSpaceImpedanceController: which law the fixture really
  // configured, so a legacy-law test cannot silently run the drive-side one.
  static bool drive_side_impedance(const VlaController & controller)
  {
    return controller.drive_side_impedance_;
  }
  // The commanded Cartesian reference (TaskSpaceImpedanceController::idle_pose_),
  // base-frame z. Control thread only (Fixture::record_cycle()).
  static double reference_z(const VlaController & controller)
  {
    return controller.idle_pose_.translation().z();
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using VlaAction = cho_interfaces::action::VisionLanguageAction;
using cho_controller_openarm_mit::VlaController;
using cho_controller_openarm_mit::VlaControllerTestAccess;
using Access = VlaControllerTestAccess;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr const char * kControllerName = "vla_mit_controller";
constexpr const char * kNamespace = "/mit_vla_test";

// `initial_position` (rad, every joint) is FakeMitSystem's start posture.
// `mirror_offset` (rad, every joint) is its mirror_position_offset: the next
// measured position is the accepted q_des plus this, so measurement and
// command differ by it while the arm holds. Zero leaves a parameter out, which
// is the URDF every pre-existing test used.
std::string urdf(const double initial_position = 0.0, const double mirror_offset = 0.0)
{
  std::ostringstream x;
  // Seven named revolute joints with non-zero distal mass, so nle is a real
  // Pinocchio result rather than zero, plus the TCP frame the Cartesian path
  // resolves its Jacobian at.
  x << "<robot name='vla_mit'><link name='base'/>";
  for (int i = 1; i <= 7; ++i) {
    x << "<link name='link" << i << "'><inertial><origin xyz='0.10 0 0'/>"
      << "<mass value='1.0'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' "
      << "iyz='0' izz='0.01'/></inertial></link><joint name='openarm_joint" << i
      << "' type='revolute'><parent link='"
      << (i == 1 ? "base" : "link" + std::to_string(i - 1))
      << "'/><child link='link" << i << "'/><origin xyz='0.20 0 0' rpy='0 0 0'/>"
      << "<axis xyz='0 1 0'/><limit lower='-3.14' upper='3.14' effort='20' velocity='5'/>"
      << "</joint>";
  }
  x << "<link name='openarm_hand_tcp'/>"
    << "<joint name='tcp_fixed' type='fixed'><parent link='link7'/>"
    << "<child link='openarm_hand_tcp'/><origin xyz='0.05 0 0'/></joint>";
  x << "<ros2_control name='fake' type='system'><hardware>"
    << "<plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
    << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param>"
    << "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
    << "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
    << "<param name='safe_hold_damping'>2</param>";
  if (initial_position != 0.0) {
    x << "<param name='initial_position'>" << initial_position << "</param>";
  }
  if (mirror_offset != 0.0) {
    x << "<param name='mirror_position_offset'>" << mirror_offset << "</param>";
  }
  x << "</hardware>";
  for (int i = 1; i <= 7; ++i) {
    x << "<joint name='openarm_joint" << i << "'>";
    for (const auto * n : {"position", "velocity", "stiffness", "damping", "effort", "hold_torque"}) {
      x << "<command_interface name='" << n << "'/>";
    }
    for (const auto * n : {"position", "velocity", "effort"}) {
      x << "<state_interface name='" << n << "'/>";
    }
    x << "</joint>";
  }
  x << "<gpio name='openarm_arm'>";
  for (const auto * n : {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation",
      "mit_safe_request_generation"})
  {
    x << "<command_interface name='" << n << "'/>";
  }
  for (const auto * n : {"mit_session_id", "mit_ack_generation", "mit_safe_generation",
      "mit_safe_ack_generation", "mit_status"})
  {
    x << "<state_interface name='" << n << "'/>";
  }
  return x.str() + "</gpio></ros2_control></robot>";
}

// Records what the controller under test logs, so a test can assert that a
// warning or a rejection reason was actually emitted. Chains to the handler it
// replaces, so console output is unchanged. Only the controller's own logger is
// kept (its name carries kControllerName).
struct LogRecord
{
  int severity;
  std::string text;
};

class LogCapture
{
public:
  static void install()
  {
    std::lock_guard<std::mutex> lock(mutex());
    if (installed()) {return;}
    previous() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&LogCapture::handle);
    installed() = true;
  }

  static void clear()
  {
    std::lock_guard<std::mutex> lock(mutex());
    records().clear();
  }

  // Records at `min_severity` or above whose text contains every needle,
  // case-insensitively.
  static std::size_t count(const int min_severity, const std::vector<std::string> & needles)
  {
    std::lock_guard<std::mutex> lock(mutex());
    std::size_t n = 0;
    for (const auto & record : records()) {
      if (record.severity < min_severity) {continue;}
      const auto text = lower(record.text);
      bool all = true;
      for (const auto & needle : needles) {
        all = all && text.find(lower(needle)) != std::string::npos;
      }
      if (all) {++n;}
    }
    return n;
  }

private:
  static std::string lower(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return text;
  }

  static void handle(
    const rcutils_log_location_t * location, int severity, const char * name,
    rcutils_time_point_value_t timestamp, const char * format, va_list * args)
  {
    if (name != nullptr && std::strstr(name, kControllerName) != nullptr && format != nullptr) {
      va_list copy;
      va_copy(copy, *args);
      char buffer[4096];
      std::vsnprintf(buffer, sizeof(buffer), format, copy);
      va_end(copy);
      std::lock_guard<std::mutex> lock(mutex());
      if (records().size() < 50000) {records().push_back({severity, buffer});}
    }
    if (previous() != nullptr) {previous()(location, severity, name, timestamp, format, args);}
  }

  static std::mutex & mutex() {static std::mutex m; return m;}
  static std::vector<LogRecord> & records() {static std::vector<LogRecord> r; return r;}
  static rcutils_logging_output_handler_t & previous()
  {
    static rcutils_logging_output_handler_t p = nullptr;
    return p;
  }
  static bool & installed() {static bool i = false; return i;}
};

// Forward kinematics of urdf(), worked by hand rather than through the
// controller's Pinocchio model. Every joint turns about +y and every origin is
// +0.2 m along the parent's x, so the chain is planar in x-z: with
// theta_k = q_1 + ... + q_k and Ry(t)*(d, 0, 0) = (d cos t, 0, -d sin t),
//   joint 1 sits at (0.2, 0, 0), joint k+1 = joint k + Ry(theta_k)*(0.2, 0, 0),
//   tcp = joint 7 + Ry(theta_7)*(0.05, 0, 0),  R_tcp = Ry(theta_7).
// At q = 0 that is (1.45, 0, 0) with the identity rotation.
using Joints = std::array<double, 7>;

struct PlanarPose
{
  double x;
  double y;
  double z;
  double pitch;  // rotation angle about +y; R = Ry(pitch)
};

PlanarPose tcp_pose(const Joints & q)
{
  PlanarPose pose {0.2, 0.0, 0.0, 0.0};
  for (std::size_t k = 0; k < 7; ++k) {
    pose.pitch += q[k];
    const double length = k < 6 ? 0.2 : 0.05;
    pose.x += length * std::cos(pose.pitch);
    pose.z -= length * std::sin(pose.pitch);
  }
  return pose;
}

// The TCP twist of urdf() for joint rates dq at q, differentiated by hand from
// tcp_pose(): segment k (length L_k, angle theta_k) turns with every joint
// j <= k, so
//   d(x, z)/dq_j = sum over k >= j of L_k * (-sin theta_k, -cos theta_k),
//   d(pitch)/dq_j = 1.
// Linear velocity of the TCP origin and angular velocity about +y, both in
// the base frame -- the LOCAL_WORLD_ALIGNED twist, with y, roll and yaw zero.
struct PlanarTwist
{
  double vx;
  double vz;
  double wy;
};

PlanarTwist tcp_twist(const Joints & q, const Joints & dq)
{
  Joints theta {};
  double angle = 0.0;
  for (std::size_t k = 0; k < 7; ++k) {
    angle += q[k];
    theta[k] = angle;
  }
  PlanarTwist twist {0.0, 0.0, 0.0};
  for (std::size_t j = 0; j < 7; ++j) {
    double dx = 0.0;
    double dz = 0.0;
    for (std::size_t k = j; k < 7; ++k) {
      const double length = k < 6 ? 0.2 : 0.05;
      dx -= length * std::sin(theta[k]);
      dz -= length * std::cos(theta[k]);
    }
    twist.vx += dq[j] * dx;
    twist.vz += dq[j] * dz;
    twist.wy += dq[j];
  }
  return twist;
}

// One control cycle as the worker saw it right after write(): the MIT tuple's
// q_des, dq_des and tau_ff, the measurement the fake mirrored from it, the
// controller's accepted-chunk count at that moment, the commanded Cartesian
// reference's z, and the update count before this cycle's increment.
struct CycleSample
{
  Joints q_cmd;
  Joints dq_cmd;
  Joints tau_cmd;
  Joints q_meas;
  std::uint64_t accepted;
  double reference_z;
  unsigned long long update;
};

// A kinematic wall in front of FakeMitSystem (imported into the resource
// manager directly, not loaded from the URDF). The fake mirrors every accepted
// q_des into its measured state; this wrapper reports that as the measurement
// unless the TCP (tcp_pose()) would cross below the plane z = `wall_z`. Then
// the measured joints stop where the straight joint path from the last
// measurement meets the plane (bisection), with zero velocity. NaN is no wall.
// The 46 command interfaces, the protocol state and the switch rule are the
// fake's own.
class WallPlant : public hardware_interface::SystemInterface
{
public:
  explicit WallPlant(std::atomic<double> & wall_z)
  : wall_z_(wall_z) {}

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {return CallbackReturn::ERROR;}
    inner_ = loader_.createUniqueInstance(info.hardware_class_type);
    return inner_->on_init(info);
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State & s) override {return inner_->on_configure(s);}
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & s) override {return inner_->on_cleanup(s);}
  CallbackReturn on_activate(const rclcpp_lifecycle::State & s) override {return inner_->on_activate(s);}
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & s) override
  {
    return inner_->on_deactivate(s);
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override {return inner_->on_shutdown(s);}
  CallbackReturn on_error(const rclcpp_lifecycle::State & s) override {return inner_->on_error(s);}

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> out;
    for (auto & state : inner_->export_state_interfaces()) {
      std::size_t joint = 7;
      for (std::size_t j = 0; j < 7 && j < info_.joints.size(); ++j) {
        if (state.get_prefix_name() == info_.joints[j].name) {joint = j;}
      }
      const auto & name = state.get_interface_name();
      const std::size_t field = name == "position" ? 0 : (name == "velocity" ? 1 : 2);
      if (joint == 7) {
        out.push_back(state);
        continue;
      }
      inner_joint_[joint][field] = std::make_unique<hardware_interface::StateInterface>(state);
      state_[joint][field] = state.get_value();
      out.emplace_back(state.get_prefix_name(), name, &state_[joint][field]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    return inner_->export_command_interfaces();
  }
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    return inner_->prepare_command_mode_switch(start, stop);
  }
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    return inner_->perform_command_mode_switch(start, stop);
  }
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period)
  override
  {
    return inner_->write(time, period);
  }

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period)
  override
  {
    const auto result = inner_->read(time, period);
    Joints mirrored {}, previous {};
    for (std::size_t joint = 0; joint < 7; ++joint) {
      mirrored[joint] = inner_joint_[joint][0]->get_value();
      previous[joint] = state_[joint][0];
    }
    const double wall = wall_z_.load(std::memory_order_acquire);
    const bool free = std::isnan(wall) || tcp_pose(mirrored).z >= wall;
    double reached = 1.0;
    if (!free) {
      // `previous` is on the free side (it was measured there); find how far
      // along the step the TCP can go.
      double low = 0.0;
      double high = 1.0;
      for (int i = 0; i < 50; ++i) {
        const double mid = 0.5 * (low + high);
        Joints q {};
        for (std::size_t joint = 0; joint < 7; ++joint) {
          q[joint] = previous[joint] + mid * (mirrored[joint] - previous[joint]);
        }
        (tcp_pose(q).z >= wall ? low : high) = mid;
      }
      reached = low;
    }
    for (std::size_t joint = 0; joint < 7; ++joint) {
      state_[joint][0] = free ? mirrored[joint] :
        previous[joint] + reached * (mirrored[joint] - previous[joint]);
      state_[joint][1] = free ? inner_joint_[joint][1]->get_value() : 0.0;
      state_[joint][2] = inner_joint_[joint][2]->get_value();
    }
    return result;
  }

private:
  std::atomic<double> & wall_z_;
  // Declared before inner_, which it must outlive.
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_ {
    "hardware_interface", "hardware_interface::SystemInterface"};
  pluginlib::UniquePtr<hardware_interface::SystemInterface> inner_;
  std::array<std::array<std::unique_ptr<hardware_interface::StateInterface>, 3>, 7> inner_joint_;
  std::array<std::array<double, 3>, 7> state_ {};
};

class Fixture : public ::testing::Test
{
protected:
  virtual void extra_parameters() {}
  // Called once right after configure_controller(), whatever it returned.
  virtual void after_configure() {}
  // FakeMitSystem's start posture for every joint [rad].
  virtual double initial_position() const {return 0.0;}
  // FakeMitSystem's mirror_position_offset [rad]: measured = q_des + this.
  virtual double mirror_position_offset() const {return 0.0;}
  // The hardware: FakeMitSystem from urdf(), every component active.
  virtual std::unique_ptr<hardware_interface::ResourceManager> make_resources()
  {
    return std::make_unique<hardware_interface::ResourceManager>(
      urdf(initial_position(), mirror_position_offset()), true, true);
  }

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
    LogCapture::install();
  }

  void set(const char * name, const auto & value)
  {
    ASSERT_TRUE(
      controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful)
      << name;
  }

  void SetUp() override
  {
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = make_resources();
    manager = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor, "controller_manager", kNamespace);
    controller = std::make_shared<VlaController>();
    ASSERT_TRUE(manager->add_controller(
        controller, kControllerName, "cho_controller_openarm_mit/VlaController"));

    set("arm", "single");
    set("kp", std::vector<double>(7, 5.0));
    set("kd", std::vector<double>(7, 0.4));
    set("torque_limit", std::vector<double>(7, 3.0));
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "mujoco_sim_safe");
    set("robot_description", urdf(initial_position(), mirror_position_offset()));
    set("ee_frame", "openarm_hand_tcp");
    // Validated as 6 positive values even though drive-side impedance makes it
    // inert on the commanded torque; see the base class on_configure.
    set("max_task_wrench", std::vector<double>{50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
    set("max_reference_offset", std::vector<double>(7, 0.15));
    set("chunk_topic", std::string("/vla_test/chunks"));
    set("stream_timeout_sec", 0.2);
    set("hold_timeout_sec", 0.6);
    set("chunk_blend_duration", 0.0);
    set("chunk_ema_factor", 1.0);
    extra_parameters();

    configured = manager->configure_controller(kControllerName);
    after_configure();
    if (configured != controller_interface::return_type::OK) {
      return;  // a configure-refusal test asserts on `configured` itself
    }
    client_node = std::make_shared<rclcpp::Node>("vla_mit_test_client");
    chunk_pub = client_node->create_publisher<cho_interfaces::msg::ActionChunk>(
      "/vla_test/chunks", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    executor->add_node(client_node);

    running = true;
    worker = std::thread([this] {
        while (running) {
          const auto now = manager->now();
          const auto period =
            rclcpp::Duration::from_seconds(worker_period.load(std::memory_order_acquire));
          manager->read(now, period);
          manager->update(now, period);
          manager->write(now, period);
          if (recording.load(std::memory_order_acquire)) {record_cycle();}
          update_count.fetch_add(1, std::memory_order_release);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    ASSERT_EQ(
      manager->switch_controller({kControllerName}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT),
      controller_interface::return_type::OK);
    cycle(30);
  }

  void TearDown() override
  {
    // Before the deactivation below releases the interfaces the recorder reads.
    recording.store(false, std::memory_order_release);
    if (controller && controller->get_node()->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      EXPECT_TRUE(safe_stop());
      EXPECT_EQ(
        manager->switch_controller({}, {kControllerName},
        controller_manager_msgs::srv::SwitchController::Request::STRICT),
        controller_interface::return_type::OK);
    }
    running = false;
    if (worker.joinable()) {worker.join();}
    if (executor && client_node) {executor->remove_node(client_node);}
  }

  // Advance `count` CONTROL CYCLES, not milliseconds: the ramps and timeouts
  // these tests wait on advance once per update(), which the worker drives
  // independently of this thread.
  void cycle(int count = 1)
  {
    if (count <= 0) {return;}
    if (!running.load()) {
      for (int i = 0; i < count; ++i) {
        executor->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return;
    }
    const auto target = update_count.load(std::memory_order_acquire) +
      static_cast<unsigned long long>(count);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(1000 + 20 * count);
    while (update_count.load(std::memory_order_acquire) < target) {
      executor->spin_some();
      if (std::chrono::steady_clock::now() > deadline) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool safe_stop()
  {
    auto client = client_node->create_client<std_srvs::srv::Trigger>(
      std::string(kNamespace) + "/" + kControllerName + "/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) {return false;}
    for (int i = 0; i < 300; ++i) {
      auto future =
        client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        cycle();
      }
      if (future.get()->success) {return true;}
      cycle(2);
    }
    return false;
  }

  // A joint-space chunk holding one waypoint per step, all at `value`.
  cho_interfaces::msg::ActionChunk joint_chunk(
    const double value, const int steps = 8, const double control_dt = 0.05) const
  {
    cho_interfaces::msg::ActionChunk message;
    message.action_space = "joint";
    message.relative_mode = "absolute";
    message.gripper_mode = "continuous";
    message.chunk_size = steps;
    message.control_dt = control_dt;
    message.arm_actions.assign(static_cast<std::size_t>(steps) * 7, 0.0);
    for (int step = 0; step < steps; ++step) {
      message.arm_actions[static_cast<std::size_t>(step) * 7] = value;
    }
    return message;
  }

  std::shared_ptr<VlaAction::Goal> vla_goal(const double stream_timeout = 0.0) const
  {
    auto goal = std::make_shared<VlaAction::Goal>();
    goal->model_name = "test";
    goal->task = "do the thing";
    goal->inference_frequency = 15.0f;
    goal->stream_timeout = stream_timeout;
    return goal;
  }

  // Send a goal and return the accepted handle (or nullptr).
  auto send_goal(const std::shared_ptr<VlaAction::Goal> & goal)
  {
    auto client = rclcpp_action::create_client<VlaAction>(
      client_node, std::string(kNamespace) + "/" + kControllerName + "/vla");
    EXPECT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
    auto future = client->async_send_goal(*goal);
    for (int i = 0; i < 400 &&
      future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cycle();
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return decltype(future.get())(nullptr);
    }
    return future.get();
  }

  void wait_ready(const int max_cycles = 4000)
  {
    for (int i = 0; i < max_cycles && !Access::ready(*controller); ++i) {cycle(5);}
  }

  void publish(const cho_interfaces::msg::ActionChunk & message, const int spin = 20)
  {
    chunk_pub->publish(message);
    cycle(spin);
  }

  // An absolute joint chunk holding every joint at `q` for `steps` waypoints.
  cho_interfaces::msg::ActionChunk joint_target_chunk(
    const Joints & q, const int steps = 8, const double control_dt = 0.05) const
  {
    cho_interfaces::msg::ActionChunk message;
    message.action_space = "joint";
    message.relative_mode = "absolute";
    message.gripper_mode = "continuous";
    message.chunk_size = steps;
    message.control_dt = control_dt;
    for (int step = 0; step < steps; ++step) {
      message.arm_actions.insert(message.arm_actions.end(), q.begin(), q.end());
    }
    return message;
  }

  // A task chunk in axis_angle (rotation vector) form, one row
  // (x, y, z, rx, ry, rz) per waypoint.
  cho_interfaces::msg::ActionChunk task_chunk(
    const std::string & relative_mode, const std::vector<std::array<double, 6>> & rows,
    const std::string & relative_frame, const double control_dt = 0.05) const
  {
    cho_interfaces::msg::ActionChunk message;
    message.action_space = "task";
    message.relative_mode = relative_mode;
    message.relative_frame = relative_frame;
    message.rotation_type = "axis_angle";
    message.gripper_mode = "continuous";
    message.chunk_size = static_cast<std::int32_t>(rows.size());
    message.control_dt = control_dt;
    for (const auto & row : rows) {
      message.arm_actions.insert(message.arm_actions.end(), row.begin(), row.end());
    }
    return message;
  }

  // An absolute task chunk holding the TCP at `pose` (model root frame).
  cho_interfaces::msg::ActionChunk pose_chunk(
    const PlanarPose & pose, const std::string & relative_frame = "", const int steps = 8) const
  {
    return task_chunk(
      "absolute",
      std::vector<std::array<double, 6>>(
        static_cast<std::size_t>(steps), {pose.x, pose.y, pose.z, 0.0, pose.pitch, 0.0}),
      relative_frame);
  }

  // A per_step task chunk: `steps` increments of `dz` along the delta frame's z,
  // with no rotation.
  cho_interfaces::msg::ActionChunk per_step_z_chunk(
    const double dz, const std::string & relative_frame, const int steps = 8) const
  {
    return task_chunk(
      "per_step",
      std::vector<std::array<double, 6>>(
        static_cast<std::size_t>(steps), {0.0, 0.0, dz, 0.0, 0.0, 0.0}),
      relative_frame);
  }

  Joints measured_q() const
  {
    Joints q {};
    for (std::size_t joint = 0; joint < 7; ++joint) {q[joint] = Access::measured(*controller, joint);}
    return q;
  }

  Joints commanded_q() const
  {
    Joints q {};
    for (std::size_t joint = 0; joint < 7; ++joint) {
      q[joint] = Access::position_command(*controller, joint);
    }
    return q;
  }

  // The 35 per-joint MIT fields (position, velocity, stiffness, damping, effort
  // for each of the 7 joints). The four protocol GPIOs after them change every
  // cycle by design and are left out.
  std::vector<double> joint_commands() const
  {
    std::vector<double> values(35);
    for (std::size_t index = 0; index < values.size(); ++index) {
      values[index] = Access::command(*controller, index);
    }
    return values;
  }

  static double max_abs_difference(const Joints & a, const Joints & b)
  {
    double worst = 0.0;
    for (std::size_t joint = 0; joint < 7; ++joint) {
      worst = std::max(worst, std::abs(a[joint] - b[joint]));
    }
    return worst;
  }

  // Cycle at least `min_cycles`, then until the measured joints have stopped
  // moving (under `tolerance` across three consecutive 10-cycle windows) or
  // `max_cycles` have run. True if they stopped.
  bool settle(const int min_cycles, const int max_cycles = 5000, const double tolerance = 1e-6)
  {
    cycle(min_cycles);
    auto previous = measured_q();
    int still = 0;
    for (int done = min_cycles; done < max_cycles; done += 10) {
      cycle(10);
      const auto now = measured_q();
      still = max_abs_difference(now, previous) < tolerance ? still + 1 : 0;
      previous = now;
      if (still >= 3) {return true;}
    }
    return false;
  }

  // An absolute joint chunk through `path`, one waypoint per entry.
  cho_interfaces::msg::ActionChunk joint_path_chunk(
    const std::vector<Joints> & path, const double control_dt = 0.05) const
  {
    cho_interfaces::msg::ActionChunk message;
    message.action_space = "joint";
    message.relative_mode = "absolute";
    message.gripper_mode = "continuous";
    message.chunk_size = static_cast<std::int32_t>(path.size());
    message.control_dt = control_dt;
    for (const auto & q : path) {
      message.arm_actions.insert(message.arm_actions.end(), q.begin(), q.end());
    }
    return message;
  }

  // An absolute task chunk through `path` (model root frame), one waypoint per entry.
  cho_interfaces::msg::ActionChunk pose_path_chunk(
    const std::vector<PlanarPose> & path, const double control_dt = 0.05) const
  {
    std::vector<std::array<double, 6>> rows;
    for (const auto & pose : path) {rows.push_back({pose.x, pose.y, pose.z, 0.0, pose.pitch, 0.0});}
    return task_chunk("absolute", rows, "", control_dt);
  }

  // Per-cycle recording, taken on the worker thread right after write(), so
  // no cycle is missed and no value is read mid-update.
  void start_recording()
  {
    {
      std::lock_guard<std::mutex> lock(trace_mutex);
      trace.clear();
      trace.reserve(20000);
    }
    recording.store(true, std::memory_order_release);
  }

  std::vector<CycleSample> recorded()
  {
    std::lock_guard<std::mutex> lock(trace_mutex);
    return trace;
  }

  std::vector<CycleSample> stop_recording()
  {
    recording.store(false, std::memory_order_release);
    return recorded();
  }

  void record_cycle()
  {
    if (!Access::loaned(*controller)) {return;}
    CycleSample sample {};
    for (std::size_t joint = 0; joint < 7; ++joint) {
      sample.q_cmd[joint] = Access::position_command(*controller, joint);
      sample.dq_cmd[joint] = Access::velocity_command(*controller, joint);
      sample.tau_cmd[joint] = Access::effort(*controller, joint);
      sample.q_meas[joint] = Access::measured(*controller, joint);
    }
    sample.accepted = Access::accepted(*controller);
    sample.reference_z = Access::reference_z(*controller);
    sample.update = update_count.load(std::memory_order_acquire);
    std::lock_guard<std::mutex> lock(trace_mutex);
    if (trace.size() < 50000) {trace.push_back(sample);}
  }

  // The controller's VlaTelemetry, ~/vla_telemetry (cho_interfaces/CONTRACT.md).
  static std::string telemetry_topic()
  {
    return std::string(kNamespace) + "/" + kControllerName + "/vla_telemetry";
  }

  // Every topic of type VlaTelemetry that exists, for a failure message.
  std::string telemetry_topics() const
  {
    std::string names;
    for (const auto & entry : client_node->get_topic_names_and_types()) {
      for (const auto & type : entry.second) {
        if (type == "cho_interfaces/msg/VlaTelemetry") {names += entry.first + " ";}
      }
    }
    return names.empty() ? std::string("(none)") : names;
  }

  // Keeps the newest telemetry message. Best effort, so it matches either
  // reliability on the publisher's side.
  void watch_telemetry()
  {
    telemetry_sub = client_node->create_subscription<cho_interfaces::msg::VlaTelemetry>(
      telemetry_topic(), rclcpp::SensorDataQoS(),
      [this](const cho_interfaces::msg::VlaTelemetry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        latest_telemetry = *message;
        telemetry_seen = true;
      });
  }

  // Cycles until the newest telemetry message satisfies `done`, at most
  // `max_cycles`. True if one did.
  template<typename Predicate>
  bool wait_telemetry(Predicate done, const int max_cycles = 4000)
  {
    for (int i = 0; i < max_cycles; i += 5) {
      {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        if (telemetry_seen && done(latest_telemetry)) {return true;}
      }
      cycle(5);
    }
    return false;
  }

  cho_interfaces::msg::VlaTelemetry telemetry()
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex);
    return latest_telemetry;
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  std::shared_ptr<VlaController> controller;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Publisher<cho_interfaces::msg::ActionChunk>::SharedPtr chunk_pub;
  controller_interface::return_type configured {controller_interface::return_type::OK};
  std::atomic<bool> running {false};
  std::atomic<unsigned long long> update_count {0};
  // The period the worker passes to every read/update/write [s].
  std::atomic<double> worker_period {0.001};
  std::thread worker;
  std::atomic<bool> recording {false};
  std::mutex trace_mutex;
  std::vector<CycleSample> trace;
  rclcpp::Subscription<cho_interfaces::msg::VlaTelemetry>::SharedPtr telemetry_sub;
  std::mutex telemetry_mutex;
  cho_interfaces::msg::VlaTelemetry latest_telemetry;
  bool telemetry_seen {false};
};

// max_reference_offset unset: the base class would derive a default, this
// controller must refuse.
class MissingOffsetFixture : public Fixture
{
protected:
  void extra_parameters() override
  {
    set("max_reference_offset", std::vector<double>(7, 0.0));
  }
};

class NoWatchdogFixture : public Fixture
{
protected:
  void extra_parameters() override {set("stream_timeout_sec", 0.0);}
};

// The historical law (real bimanual config): drive_side_impedance false and
// zero MIT stiffness, so TRACKING_DAMPED_TORQUE pins q_des to the measurement
// and nothing closes a joint position loop. Long stream timeouts: before the
// first ACCEPTED chunk the watchdog runs from the goal start and a rejected
// chunk does not feed it, so with the default 0.2 s / 0.6 s a loaded machine
// could abort the goal before a test's later chunk lands.
class LegacyLawFixture : public Fixture
{
protected:
  void extra_parameters() override
  {
    set("drive_side_impedance", false);
    set("kp", std::vector<double>(7, 0.0));
    set("stream_timeout_sec", 10.0);
    set("hold_timeout_sec", 20.0);
    // Frequent telemetry, so the AC13 checks do not wait on the 10 Hz default.
    set("telemetry_period_sec", 0.05);
    LogCapture::clear();
  }
  void after_configure() override
  {
    configure_task_only_warnings =
      LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"task", "chunk"});
  }
  std::size_t configure_task_only_warnings {0};
};

// A non-singular start (every joint at 0.3 rad, so the TCP is pitched by
// 2.1 rad and x is reachable) and a stream watchdog long enough that a single
// chunk plays out and settles without a hold. Explicit limiter bounds so the
// timing numbers below do not depend on the controller's defaults.
class CurledFixture : public Fixture
{
protected:
  double initial_position() const override {return 0.3;}
  void extra_parameters() override
  {
    set("stream_timeout_sec", 10.0);
    set("hold_timeout_sec", 20.0);
    set("max_task_ang_vel", 1.5);
    set("max_joint_ref_acc", std::vector<double>(7, 4.0));
    LogCapture::clear();
  }

  static constexpr double kStartPitch = 7 * 0.3;  // theta_7 at the start posture [rad]

  // On a fresh goal, sends one per_step chunk of 8 increments of dz = -0.01 m
  // along `frame`'s z, lets the arm settle, and returns how far the TCP moved
  // (hand FK of the measured joints; pitch is the change of rotation about y).
  PlanarPose per_step_displacement(const std::string & frame)
  {
    wait_ready();
    EXPECT_TRUE(Access::ready(*controller));
    EXPECT_NE(send_goal(vla_goal()), nullptr);
    const auto start = tcp_pose(measured_q());
    EXPECT_NEAR(start.pitch, kStartPitch, 1e-9);
    chunk_pub->publish(per_step_z_chunk(-0.01, frame));
    EXPECT_TRUE(settle(300)) << "the arm did not come to rest";
    const auto end = tcp_pose(measured_q());
    return {end.x - start.x, end.y - start.y, end.z - start.z, end.pitch - start.pitch};
  }

  struct PitchSample
  {
    double elapsed;  // control time since the chunk was published, upper bound [s]
    double pitch;    // measured TCP pitch minus the pitch at publication [rad]
  };

  // Publishes one absolute chunk that keeps the TCP where it is and pitches it
  // by `step` rad about y, then samples the measured pitch once per cycle.
  // The worker advances control time by exactly 1 ms per update and the chunk
  // cannot be spliced before the update that follows the publication, so
  // (updates since publication) * 1 ms bounds the time the reference has had.
  // The joints are read BEFORE the update count, so the count can only be late.
  std::vector<PitchSample> pitch_response(const double step, const int cycles)
  {
    auto target = tcp_pose(measured_q());
    const double start_pitch = target.pitch;
    target.pitch += step;
    const auto first = update_count.load(std::memory_order_acquire);
    chunk_pub->publish(pose_chunk(target));
    std::vector<PitchSample> samples;
    samples.reserve(static_cast<std::size_t>(cycles));
    for (int i = 0; i < cycles; ++i) {
      cycle();
      const double pitch = tcp_pose(measured_q()).pitch - start_pitch;
      const auto updates = update_count.load(std::memory_order_acquire) - first;
      samples.push_back({1e-3 * static_cast<double>(updates), pitch});
    }
    return samples;
  }
};

constexpr double kAngularAcceleration = 2.0;  // rad/s^2, AngularAccelerationFixture

class AngularAccelerationFixture : public CurledFixture
{
protected:
  void extra_parameters() override
  {
    CurledFixture::extra_parameters();
    set("max_task_ang_acc", kAngularAcceleration);
  }
};

// Sets max_task_ang_acc to `value()` without asserting that the set itself
// succeeds: a range-checked declaration may refuse it at set time, which is as
// good a refusal as configure failing. Whether the parameter is declared at all
// is recorded separately, so an undeclared parameter cannot pass as "refused".
class AngularAccelerationRefusalFixture : public Fixture
{
protected:
  virtual double value() const = 0;
  void extra_parameters() override
  {
    declared = controller->get_node()->has_parameter("max_task_ang_acc");
    set_accepted = controller->get_node()->set_parameter(
      rclcpp::Parameter("max_task_ang_acc", value())).successful;
  }
  bool declared {false};
  bool set_accepted {false};
};

class NegativeAngularAccelerationFixture : public AngularAccelerationRefusalFixture
{
protected:
  double value() const override {return -1.0;}
};

class NanAngularAccelerationFixture : public AngularAccelerationRefusalFixture
{
protected:
  double value() const override {return kNaN;}
};

// ---------------------------------------------------------------------------
// Stream-watchdog and chunk-admission tests (2026-10-13: S15, S8, C16).
//
// Control time is the wall clock here (the worker passes manager->now()), so
// these tests measure the wall time between a publish and the telemetry that
// reports the hold. Telemetry is fast (20 Hz) and only ever LATE, so a lower
// bound on "hold no sooner than X after the chunk" cannot be violated by
// delivery delay, and the upper bounds below are generous.
using Clock = std::chrono::steady_clock;

struct StreamSample
{
  Clock::time_point at;
  std::string state;
  std::string last_reject;
  std::uint64_t accepted;
  std::uint64_t rejected;
};

struct OpenGoal
{
  rclcpp_action::Client<VlaAction>::SharedPtr client;
  rclcpp_action::ClientGoalHandle<VlaAction>::SharedPtr handle;
};

class StreamFixture : public Fixture
{
protected:
  void extra_parameters() override {set("telemetry_period_sec", 0.05);}

  void watch_stream()
  {
    stream_sub = client_node->create_subscription<cho_interfaces::msg::VlaTelemetry>(
      telemetry_topic(), rclcpp::SensorDataQoS(),
      [this](const cho_interfaces::msg::VlaTelemetry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(stream_mutex);
        samples.push_back(
          {Clock::now(), message->stream_state, message->last_reject, message->chunks_accepted,
            message->chunks_rejected});
      });
  }

  OpenGoal open_goal()
  {
    OpenGoal goal;
    goal.client = rclcpp_action::create_client<VlaAction>(
      client_node, std::string(kNamespace) + "/" + kControllerName + "/vla");
    EXPECT_TRUE(goal.client->wait_for_action_server(std::chrono::seconds(2)));
    auto future = goal.client->async_send_goal(*vla_goal());
    for (int i = 0; i < 400 &&
      future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cycle();
    }
    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      goal.handle = future.get();
    }
    return goal;
  }

  // Cycle for `seconds` of wall time, calling `each` once per 5 cycles.
  template<typename Callback>
  void for_seconds(const double seconds, Callback each)
  {
    const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
    while (Clock::now() < end) {
      each();
      cycle(5);
    }
  }

  std::vector<StreamSample> stream_samples()
  {
    std::lock_guard<std::mutex> lock(stream_mutex);
    return samples;
  }

  // Number of times the reported state went from something else into `state`.
  int entries_into(const std::string & state)
  {
    int count = 0;
    std::string previous;
    for (const auto & sample : stream_samples()) {
      if (sample.state == state && previous != state) {++count;}
      previous = sample.state;
    }
    return count;
  }

  // Cycles until a sample at or after `since` reports `state`; its time, or
  // nullopt-like (false) after `timeout_sec`.
  bool wait_state(const std::string & state, const Clock::time_point since, double timeout_sec,
    Clock::time_point * when)
  {
    const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(timeout_sec));
    while (Clock::now() < end) {
      for (const auto & sample : stream_samples()) {
        if (sample.at >= since && sample.state == state) {
          *when = sample.at;
          return true;
        }
      }
      cycle(5);
    }
    return false;
  }

  bool wait_reject(const std::string & name, const int max_cycles = 4000)
  {
    for (int i = 0; i < max_cycles; i += 5) {
      const auto all = stream_samples();
      if (!all.empty() && all.back().last_reject == name) {return true;}
      cycle(5);
    }
    return false;
  }

  std::string last_reject()
  {
    const auto all = stream_samples();
    return all.empty() ? std::string("(no telemetry)") : all.back().last_reject;
  }

  static double seconds_between(const Clock::time_point from, const Clock::time_point to)
  {
    return std::chrono::duration<double>(to - from).count();
  }

  rclcpp::Subscription<cho_interfaces::msg::VlaTelemetry>::SharedPtr stream_sub;
  std::mutex stream_mutex;
  std::vector<StreamSample> samples;
};

// A dead policy is cut off at `stream_horizon_credit_sec` of open-loop replay.
class CreditFixture : public StreamFixture
{
protected:
  void extra_parameters() override
  {
    StreamFixture::extra_parameters();
    set("stream_horizon_credit_sec", 0.5);
  }
};

// Chunks placed by their observation stamp, which this fixture sets on the
// controller's own clock (client_node and the controller manager are both on
// the system-backed ROS clock).
class ObservationFixture : public StreamFixture
{
protected:
  void extra_parameters() override
  {
    StreamFixture::extra_parameters();
    set("chunk_time_source", std::string("observation"));
  }

  // `message` stamped as an observation taken `age_sec` ago (negative: ahead
  // of the controller clock).
  cho_interfaces::msg::ActionChunk stamped(
    cho_interfaces::msg::ActionChunk message, const double age_sec) const
  {
    const rclcpp::Time stamp = client_node->now() - rclcpp::Duration::from_seconds(age_sec);
    message.header.stamp = stamp;
    return message;
  }

  // 20 waypoints at 0.05 s: reaches 0.95 s past the observation.
  cho_interfaces::msg::ActionChunk long_chunk(const double value = 0.2) const
  {
    return joint_chunk(value, 20);
  }
};

// The age window narrowed to 0.3 s, to show chunk_max_observation_age is wired.
class NarrowAgeFixture : public ObservationFixture
{
protected:
  void extra_parameters() override
  {
    ObservationFixture::extra_parameters();
    set("chunk_max_observation_age", 0.3);
  }
};

// The curled start (every joint at 0.3 rad) behind WallPlant, with the real
// single-arm VLA block's chunk shaping (blend 0.2 s, 0.10 m/s, 0.4 m/s^2) and
// a 5 mrad reference-offset bound. That bound is what the anti-windup leaves
// between the anchor and the measurement: at this posture the solve maps 1 m
// of z error to at most 1.87 rad on one joint, so it saturates at about
// 2.7 mm of Cartesian error.
class WallFixture : public Fixture
{
protected:
  double initial_position() const override {return 0.3;}
  std::unique_ptr<hardware_interface::ResourceManager> make_resources() override
  {
    auto resources = std::make_unique<hardware_interface::ResourceManager>();
    const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(initial_position()));
    resources->import_component(std::make_unique<WallPlant>(wall_z), infos.at(0));
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    EXPECT_EQ(
      resources->set_component_state(infos.at(0).name, active),
      hardware_interface::return_type::OK);
    return resources;
  }
  void extra_parameters() override
  {
    set("max_reference_offset", std::vector<double>(7, kOffsetBound));
    set("stream_timeout_sec", 10.0);
    set("hold_timeout_sec", 20.0);
    set("chunk_blend_duration", 0.2);
    set("max_task_lin_vel", 0.10);
    set("max_task_lin_acc", 0.4);
    set("max_task_ang_vel", 0.5);
  }

  static constexpr double kOffsetBound = 0.005;  // rad, every joint

  // A from_anchor base-frame task chunk from a policy that sees only the
  // MEASURED pose: 10 steps of at most 2 mm (30 mm/s at 15 Hz) along base z
  // from the observed height toward `target_z`.
  cho_interfaces::msg::ActionChunk policy_chunk(const double observed_z, const double target_z) const
  {
    std::vector<std::array<double, 6>> rows;
    double offset = 0.0;
    for (int step = 0; step < 10; ++step) {
      offset += std::clamp(target_z - observed_z - offset, -kPolicyStep, kPolicyStep);
      rows.push_back({0.0, 0.0, offset, 0.0, 0.0, 0.0});
    }
    return task_chunk("from_anchor", rows, "base", kPolicyPeriod);
  }

  static constexpr double kPolicyPeriod = 1.0 / 15.0;  // chunk period and control_dt [s]
  static constexpr double kPolicyStep = 0.002;  // m per waypoint

  std::atomic<double> wall_z {std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace

// ---------------------------------------------------------------------------

TEST_F(MissingOffsetFixture, ConfigureRefusesWithoutAnExplicitReferenceOffsetBound)
{
  // max_reference_offset is the ONLY bound on kp*(q_des - q), which the drive
  // applies downstream of torque_limit. Accepting a derived default here would
  // let untrusted policy output drive an unbounded reference offset.
  EXPECT_NE(configured, controller_interface::return_type::OK);
}

TEST_F(NoWatchdogFixture, ConfigureRefusesWithoutAStreamWatchdog)
{
  // Without it a dead policy leaves the arm holding a mid-motion Cartesian
  // reference indefinitely with the drive's full stiffness behind it.
  EXPECT_NE(configured, controller_interface::return_type::OK);
}

TEST_F(Fixture, ConfiguresAndKeepsTheInheritedReferenceOffsetBound)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_NEAR(Access::reference_offset_limit(*controller, joint), 0.15, 1e-12);
  }
}

TEST_F(Fixture, ExposesNoTaskSpaceActionServer)
{
  // Two servers on the same 46 interfaces could both drive; uses_task_space_action()
  // is false so only the VLA API exists.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  const std::string vla_prefix = std::string(kNamespace) + "/" + kControllerName + "/vla/_action/";
  const std::string task_space_prefix = std::string(kNamespace) + "/" + kControllerName + "/task_space/_action/";
  bool have_vla_server = false;
  bool have_task_space_server = false;
  cycle(20);
  for (const auto & entry : client_node->get_service_names_and_types()) {
    if (entry.first.rfind(vla_prefix, 0) == 0) {have_vla_server = true;}
    if (entry.first.rfind(task_space_prefix, 0) == 0) {have_task_space_server = true;}
  }
  EXPECT_TRUE(have_vla_server);
  EXPECT_FALSE(have_task_space_server);
}

TEST_F(Fixture, GoalIsRejectedBeforeTheStartupRampSettles)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  // task_ready_ gates the server; without return_to_zero the ramp settles almost
  // immediately, so drive the check off the flag rather than off wall time.
  if (Access::ready(*controller)) {
    GTEST_SKIP() << "startup already settled; covered by the ready-path tests";
  }
  auto handle = send_goal(vla_goal());
  EXPECT_TRUE(handle == nullptr || !handle->is_result_aware());
}

TEST_F(Fixture, AcceptsAGoalAndFollowsAJointChunkAsImpedance)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));

  auto handle = send_goal(vla_goal());
  ASSERT_NE(handle, nullptr);

  const double start = Access::position_command(*controller, 0);
  publish(joint_chunk(0.5), 120);

  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);

  // Joint-space impedance: the drive gets a nonzero stiffness and a q_des that
  // moved toward the target, bounded by max_reference_offset against measured.
  EXPECT_GT(Access::stiffness(*controller, 0), 0.0);
  const double commanded = Access::position_command(*controller, 0);
  EXPECT_GT(commanded, start);
  EXPECT_LE(
    std::abs(commanded - Access::measured(*controller, 0)),
    Access::reference_offset_limit(*controller, 0) + 1e-6);

  // tau_ff carries the model term, so it is not identically zero on a loaded arm.
  bool any_effort = false;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    any_effort = any_effort || std::abs(Access::effort(*controller, joint)) > 1e-9;
  }
  EXPECT_TRUE(any_effort);
}

TEST_F(Fixture, EveryCommandInterfaceStaysFiniteThroughAChunkWithNaN)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  auto poisoned = joint_chunk(0.3);
  poisoned.arm_actions[3] = kNaN;
  publish(poisoned, 60);

  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)))
      << "command interface " << index;
  }
}

TEST_F(Fixture, AnUnknownRotationTypeIsRefusedRatherThanGuessed)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  cho_interfaces::msg::ActionChunk message;
  message.action_space = "task";
  message.rotation_type = "made_up";
  message.chunk_size = 1;
  message.control_dt = 0.05;
  // Deliberately empty: with the historical dim = 0 fallback this passed the
  // size check and then read an iterator range backwards over an empty vector.
  message.arm_actions.clear();
  publish(message, 40);

  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
}

TEST_F(StreamFixture, AQuietStreamHoldsAndThenAbortsWithoutRequestingSafe)
{
  // AC10 (S15) moved WHEN the hold starts, not whether it does. The watchdog
  // holds once the stream has been quiet for stream_timeout (0.2 s) AND the
  // buffered horizon has run out: this chunk's 8 waypoints at 0.05 s reach
  // about 0.35 s past its arrival, so the hold begins there instead of at 0.2 s.
  // Quiet -> hold -> abort after hold_timeout, never a SAFE request: unchanged.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();

  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const auto published = Clock::now();
  publish(joint_chunk(0.2), 40);
  ASSERT_EQ(Access::accepted(*controller), 1u);

  // Stop publishing. Hold when the horizon ends (~0.35 s); hold_timeout 0.6 s -> abort.
  auto result_future = goal.client->async_get_result(goal.handle);
  for (int i = 0; i < 2000 &&
    result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle(2);
  }
  ASSERT_EQ(result_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->is_completed);
  // The abort must say why, and must not have gone through SAFE: the controller
  // is still ACTIVE and still commanding a finite tuple.
  EXPECT_NE(result.result->message.find("stream"), std::string::npos)
    << "message was: " << result.result->message;
  EXPECT_EQ(
    controller->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }

  // The hold was reported, and not before the horizon (0.35 s, bound loosened to
  // 0.3 s) had passed: under the pre-S15 rule it began at 0.2 s.
  Clock::time_point held;
  ASSERT_TRUE(wait_state("hold", published, 1.0, &held)) << "no hold in telemetry";
  EXPECT_GE(seconds_between(published, held), 0.3);
}

TEST_F(Fixture, CancelReleasesTheReferenceAndLeavesTheServerAvailable)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));

  auto client = rclcpp_action::create_client<VlaAction>(
    client_node, std::string(kNamespace) + "/" + kControllerName + "/vla");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  auto goal_future = client->async_send_goal(*vla_goal());
  for (int i = 0; i < 400 &&
    goal_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  auto handle = goal_future.get();
  ASSERT_NE(handle, nullptr);
  publish(joint_chunk(0.2), 40);

  auto cancel_future = client->async_cancel_goal(handle);
  for (int i = 0; i < 400 &&
    cancel_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  cycle(60);

  // Still ACTIVE, still commanding finite values, and a second goal is accepted:
  // a cancel is not a SAFE request.
  EXPECT_EQ(
    controller->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
  EXPECT_NE(send_goal(vla_goal()), nullptr);
}

TEST_F(Fixture, ASafeStopEndsTheGoalWithTheReason)
{
  // The controller stops computing references once STOPPING, so nothing else
  // would ever finish the goal: it used to stay accepted with no result and
  // block every later one.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  auto client = rclcpp_action::create_client<VlaAction>(
    client_node, std::string(kNamespace) + "/" + kControllerName + "/vla");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  auto goal_future = client->async_send_goal(*vla_goal());
  for (int i = 0; i < 400 &&
    goal_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  auto handle = goal_future.get();
  ASSERT_NE(handle, nullptr);
  publish(joint_chunk(0.2), 40);
  auto result = client->async_get_result(handle);
  ASSERT_TRUE(safe_stop());
  for (int i = 0; i < 400 &&
    result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(wrapped.result->message.find("SAFE stop"), std::string::npos)
    << wrapped.result->message;
}

TEST_F(Fixture, ChunksAreIgnoredWithoutAnActiveGoal)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  publish(joint_chunk(1.0), 60);
  // A chunk is not authorised to drive the arm on its own.
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
}

TEST_F(Fixture, TheNonRealtimeTimerSurvivesTicksBetweenConfigureAndActivate)
{
  // Regression for a crash that took down the whole controller_manager.
  //
  // The 5 ms non-RT timer is created in on_configure, but the telemetry stamp it
  // subtracts from used to be set in on_activate. A default-constructed
  // rclcpp::Time carries RCL_SYSTEM_TIME; the node's clock under sim time is
  // RCL_ROS_TIME; subtracting across sources THROWS, and a throw out of a timer
  // callback is std::terminate, not a reported error. Any gap longer than one
  // tick between configure and activate aborted the process -- reproduced in
  // MuJoCo on the bimanual torso, where ros2_control_node died with SIGABRT
  // immediately after the controller activated.
  //
  // The fixture already configures, spins for many ticks, and then activates, so
  // reaching this assertion at all means the timer ran while inactive without
  // terminating. The explicit spin below makes that the point of the test rather
  // than an accident of the fixture's ordering.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  for (int i = 0; i < 40; ++i) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(
    controller->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // And it must still work afterwards, not merely have survived.
  wait_ready();
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  publish(joint_chunk(0.1), 40);
  EXPECT_GE(Access::accepted(*controller), 1u);
}

TEST_F(Fixture, ProtocolGenerationKeepsAdvancingUnderAVlaReference)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  publish(joint_chunk(0.2), 20);

  // command_interfaces_[37] is the MIT commit generation. The hardware faults a
  // producer that stops writing, so a VLA path with a quiet stream must still
  // advance it every cycle -- this is the difference between an action-path
  // controller and the raw-topic producer, which would request SAFE instead.
  const double before = Access::command(*controller, 37);
  cycle(200);
  const double after = Access::command(*controller, 37);
  EXPECT_GT(after, before);
}

// ---------------------------------------------------------------------------
// AC2 (M3). Under drive_side_impedance: false the law sends kp = 0 with q_des
// pinned to the measurement, so a joint chunk would have no position loop
// behind it. The controller takes task chunks only there.

TEST_F(LegacyLawFixture, AC2_ConfigureWarnsOnceThatOnlyTaskChunksAreAccepted)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  ASSERT_FALSE(Access::drive_side_impedance(*controller));
  // At launch, not first at the first refused chunk.
  EXPECT_GE(configure_task_only_warnings, 1u)
    << "no WARN mentioning task chunks while configuring the legacy law";
  // Once: running the control loop does not repeat it.
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  cycle(300);
  EXPECT_EQ(
    LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"task", "chunk"}),
    configure_task_only_warnings);
}

TEST_F(LegacyLawFixture, AC2_AJointChunkIsRejectedWithItsReasonAndChangesNoCommand)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  ASSERT_FALSE(Access::drive_side_impedance(*controller));
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  // Long enough for the model feed-forward slew to finish, so the snapshot is
  // a steady state and any later change is the chunk's doing.
  cycle(300);
  const auto commands_before = joint_commands();
  const auto q_before = measured_q();
  LogCapture::clear();

  publish(joint_chunk(0.5), 150);

  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 1u);
  // Nothing reached the drive. Measured on the pre-fix controller, an ACCEPTED
  // joint chunk under this law left all 35 fields unchanged too -- kp = 0 and
  // q_des = measured, so it was taken and silently did nothing, which is the
  // defect. The counts and the logged reason above are what tell the two apart.
  const auto commands_after = joint_commands();
  const char * fields[] = {"position", "velocity", "stiffness", "damping", "effort"};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    for (std::size_t field = 0; field < 5; ++field) {
      const auto index = 5 * joint + field;
      EXPECT_NEAR(commands_after[index], commands_before[index], field == 4 ? 1e-6 : 1e-9)
        << "joint " << joint + 1 << " " << fields[field];
    }
  }
  EXPECT_LE(max_abs_difference(measured_q(), q_before), 1e-9);
  // Refused WITH the reason, which names the setting that would allow it.
  EXPECT_GE(
    LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"drive_side_impedance"}) +
    LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"drive-side impedance"}), 1u)
    << "the refusal was not logged with its reason";
  // AC13: and telemetry names it as what it is -- a well-formed chunk in an
  // action space this configuration does not take -- not as a bad field.
  watch_telemetry();
  ASSERT_TRUE(wait_telemetry([](const auto & t) {return t.chunks_rejected >= 1;}))
    << "no telemetry with the rejection on " << telemetry_topic()
    << "; VlaTelemetry topics: " << telemetry_topics();
  EXPECT_EQ(telemetry().last_reject, "action_space_not_accepted");

  // The refusal does not end the goal: a task chunk on it is still taken.
  publish(pose_chunk(tcp_pose(measured_q())), 60);
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 1u);
}

TEST_F(LegacyLawFixture, AC2_ATaskChunkIsAccepted)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  ASSERT_FALSE(Access::drive_side_impedance(*controller));
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  // Straight arm at q = 0: the TCP is at (1.45, 0, 0), unrotated.
  const auto here = tcp_pose(measured_q());
  ASSERT_NEAR(here.x, 1.45, 1e-9);
  publish(pose_chunk(here), 80);

  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
}

// ---------------------------------------------------------------------------
// AC3 (M4), host side: ActionChunk.relative_frame is parsed fail-closed and
// reaches the composition.

TEST_F(CurledFixture, AC3_AnUnknownRelativeFrameIsRejectedInEverySpace)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  const auto q_before = measured_q();

  // Joint space does not use the field, but an unknown value is still refused.
  Joints nudged = q_before;
  nudged[0] += 0.1;
  auto joint = joint_target_chunk(nudged);
  joint.relative_frame = "bogus";
  publish(joint, 60);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 1u);

  // Absolute task chunk: likewise unused, likewise refused.
  publish(pose_chunk(tcp_pose(q_before), "bogus"), 60);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 2u);

  // Relative task chunk: the one place the value would matter.
  publish(per_step_z_chunk(-0.01, "bogus"), 60);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 3u);

  EXPECT_LE(max_abs_difference(measured_q(), q_before), 1e-9);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
}

// The start posture pitches the TCP by theta_7 = 2.1 rad, so the tool z axis
// Ry(2.1)*z = (sin 2.1, 0, cos 2.1) = (0.863, 0, -0.505) is far from the base z
// axis, and 8 per_step increments of (0, 0, -0.01) part ways:
//   base:  p = p_a + sum(dp)        -> (0, 0, -0.08)
//   ee:    p = p_a + R_a * sum(dp)  -> (-0.08 sin 2.1, 0, -0.08 cos 2.1)
//                                    = (-0.0691, 0, +0.0404)
// Orientation is unchanged either way (every rotation increment is zero).
TEST_F(CurledFixture, AC3_ABaseFramePerStepChunkMovesAlongTheBaseAxes)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  const auto moved = per_step_displacement("base");
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_NEAR(moved.x, 0.0, 5e-3);
  EXPECT_NEAR(moved.y, 0.0, 1e-9);
  EXPECT_NEAR(moved.z, -0.08, 5e-3);
  EXPECT_NEAR(moved.pitch, 0.0, 5e-3);
  // A bridge that names its frame is not nagged.
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"relative_frame"}), 0u);
}

TEST_F(CurledFixture, AC3_AnEmptyRelativeFrameIsComposedInTheEeFrameAndWarned)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  const auto moved = per_step_displacement("");
  // Accepted as before the field existed: the four external bridges must keep
  // working after a rebuild.
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_NEAR(moved.x, -0.08 * std::sin(kStartPitch), 5e-3);
  EXPECT_NEAR(moved.z, -0.08 * std::cos(kStartPitch), 5e-3);
  EXPECT_NEAR(moved.pitch, 0.0, 5e-3);
  EXPECT_GE(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"relative_frame"}), 1u)
    << "a relative task chunk without relative_frame was taken silently";
}

TEST_F(CurledFixture, AC3_AnExplicitEeRelativeFrameIsComposedInTheEeFrame)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  const auto moved = per_step_displacement("ee");
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_NEAR(moved.x, -0.08 * std::sin(kStartPitch), 5e-3);
  EXPECT_NEAR(moved.z, -0.08 * std::cos(kStartPitch), 5e-3);
  EXPECT_NEAR(moved.pitch, 0.0, 5e-3);
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"relative_frame"}), 0u);
}

TEST_F(CurledFixture, AC3_AbsoluteTaskChunksIgnoreTheRelativeFrame)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  const auto q_before = measured_q();
  const auto here = tcp_pose(q_before);

  // Each holds the TCP where it is. If a frame were applied to an absolute
  // target (p_a + p, or R_a * p) the arm would leave by tens of centimetres.
  for (const char * frame : {"", "base", "ee"}) {
    publish(pose_chunk(here, frame), 150);
  }
  EXPECT_EQ(Access::accepted(*controller), 3u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_LE(max_abs_difference(measured_q(), q_before), 2e-3);
  // An empty field on an absolute chunk is the normal case for absolute
  // bridges (pi0 ALOHA, GR00T, LeRobot), not something to warn about.
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"relative_frame"}), 0u);
}

TEST_F(CurledFixture, AC3_JointChunksIgnoreTheRelativeFrame)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  const auto q_before = measured_q();

  for (const char * frame : {"", "base", "ee"}) {
    auto message = joint_target_chunk(q_before);
    message.relative_frame = frame;
    publish(message, 150);
  }
  EXPECT_EQ(Access::accepted(*controller), 3u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_LE(max_abs_difference(measured_q(), q_before), 2e-3);
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, {"relative_frame"}), 0u);
}

// ---------------------------------------------------------------------------
// AC8 (C12). When the sampled reference changes action space inside one goal,
// the limiter is re-seeded from what is being commanded NOW in the new space,
// never from the anchor the old chunk carried. Both tests move the arm away
// from where the first chunk was anchored, then switch to a chunk whose target
// is exactly where the arm already is: a correct re-seed commands no motion at
// all, while a limiter resuming from the stale anchor puts the reference back
// there and q_des leaps toward it by up to max_reference_offset (0.15 rad).

TEST_F(CurledFixture, AC8_AJointToTaskSwitchInOneGoalStartsFromTheCommandedPose)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  // ONE joint chunk, so its observation-time anchor stays the start pose while
  // the arm turns 0.4 rad away from it on joint 1 (TCP pitch 2.1 -> 2.5 rad).
  const Joints start = measured_q();
  Joints target = start;
  target[0] += 0.4;
  chunk_pub->publish(joint_target_chunk(target));
  ASSERT_TRUE(settle(300)) << "the joint chunk did not come to rest";
  ASSERT_EQ(Access::accepted(*controller), 1u);
  const Joints q_switch = measured_q();
  ASSERT_LE(max_abs_difference(q_switch, target), 1e-3) << "the joint chunk was not followed";
  const Joints commanded_switch = commanded_q();

  // Same goal, task space, holding the TCP where the joint reference put it.
  chunk_pub->publish(pose_chunk(tcp_pose(q_switch)));
  double command_jump = 0.0;
  double drift = 0.0;
  for (int i = 0; i < 400; ++i) {
    cycle();
    command_jump = std::max(command_jump, max_abs_difference(commanded_q(), commanded_switch));
    drift = std::max(drift, max_abs_difference(measured_q(), q_switch));
  }
  EXPECT_EQ(Access::accepted(*controller), 2u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_LE(command_jump, 2e-3) << "q_des jumped at the joint -> task switch";
  EXPECT_LE(drift, 2e-3) << "the arm moved although the task target is where it stood";
}

TEST_F(CurledFixture, AC8_ATaskToJointSwitchInOneGoalStartsFromTheCommandedJoints)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  // ONE absolute task chunk 8 cm below the start: its anchor joints stay the
  // start posture while the arm moves away from it.
  const Joints start = measured_q();
  auto lowered = tcp_pose(start);
  lowered.z -= 0.08;
  chunk_pub->publish(pose_chunk(lowered));
  ASSERT_TRUE(settle(300)) << "the task chunk did not come to rest";
  ASSERT_EQ(Access::accepted(*controller), 1u);
  const Joints q_switch = measured_q();
  ASSERT_NEAR(tcp_pose(q_switch).z, lowered.z, 5e-3) << "the task chunk was not followed";
  ASSERT_GE(max_abs_difference(q_switch, start), 0.02);
  const Joints commanded_switch = commanded_q();

  // Same goal, joint space, holding the joints where the task reference put them.
  chunk_pub->publish(joint_target_chunk(q_switch));
  double command_jump = 0.0;
  double drift = 0.0;
  for (int i = 0; i < 400; ++i) {
    cycle();
    command_jump = std::max(command_jump, max_abs_difference(commanded_q(), commanded_switch));
    drift = std::max(drift, max_abs_difference(measured_q(), q_switch));
  }
  EXPECT_EQ(Access::accepted(*controller), 2u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_LE(command_jump, 2e-3) << "q_des jumped at the task -> joint switch";
  EXPECT_LE(drift, 2e-3) << "the arm moved although the joint target is where it stood";
}

// ---------------------------------------------------------------------------
// AC5, host side: max_task_ang_acc reaches ReferenceLimiter's
// max_angular_acceleration. Default 0 = off.

TEST_F(Fixture, AC5_MaxTaskAngAccIsDeclaredAndDefaultsToOff)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  ASSERT_TRUE(controller->get_node()->has_parameter("max_task_ang_acc"));
  EXPECT_EQ(controller->get_node()->get_parameter("max_task_ang_acc").as_double(), 0.0);
}

TEST_F(NegativeAngularAccelerationFixture, AC5_ANegativeMaxTaskAngAccIsRefused)
{
  EXPECT_TRUE(declared) << "max_task_ang_acc is not declared";
  EXPECT_TRUE(!set_accepted || configured != controller_interface::return_type::OK)
    << "max_task_ang_acc = -1 was taken";
}

TEST_F(NanAngularAccelerationFixture, AC5_ANonFiniteMaxTaskAngAccIsRefused)
{
  EXPECT_TRUE(declared) << "max_task_ang_acc is not declared";
  EXPECT_TRUE(!set_accepted || configured != controller_interface::return_type::OK)
    << "max_task_ang_acc = NaN was taken";
}

// A 0.3 rad pitch step with the TCP held in place. With the limit a, the
// emitted angular velocity starts at rest and changes by at most a*dt per
// cycle, so after n cycles the reference has turned at most
// a*dt^2*n(n+1)/2; the measured pitch follows that reference and cannot be
// ahead of it. Bang-bang over 0.3 rad at a = 2 takes 2*sqrt(0.3/2) = 0.77 s
// with a peak of 0.77 rad/s, inside max_task_ang_vel = 1.5.
TEST_F(AngularAccelerationFixture, AC5_ARotationStepIsAccelerationLimited)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  const auto samples = pitch_response(0.3, 1000);
  double worst_excess = -1.0;
  double worst_at = 0.0;
  for (const auto & sample : samples) {
    const double n = sample.elapsed / 1e-3;
    const double bound = 0.5 * kAngularAcceleration * 1e-6 * n * (n + 1.0) + 1e-3;
    if (sample.pitch - bound > worst_excess) {
      worst_excess = sample.pitch - bound;
      worst_at = sample.elapsed;
    }
  }
  EXPECT_LE(worst_excess, 0.0)
    << "pitch ran ahead of the a*t^2/2 envelope by " << worst_excess << " rad at t = "
    << worst_at << " s";
  // Limited, not stuck: it still converges onto the step.
  ASSERT_TRUE(settle(0, 4000));
  EXPECT_NEAR(tcp_pose(measured_q()).pitch, kStartPitch + 0.3, 5e-3);
  EXPECT_EQ(Access::accepted(*controller), 1u);
}

// The same step with the parameter left at its default: the velocity clamp
// alone, so the pitch leaves the a = 2 envelope at once (1.5 rad/s from the
// first cycles). This is what makes the test above mean something.
TEST_F(CurledFixture, AC5_ByDefaultARotationStepIsOnlyVelocityClamped)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  const auto samples = pitch_response(0.3, 250);
  double best_excess = -1.0;
  for (const auto & sample : samples) {
    if (sample.elapsed > 0.25) {break;}
    const double envelope = 0.5 * kAngularAcceleration * sample.elapsed * sample.elapsed;
    best_excess = std::max(best_excess, sample.pitch - envelope);
  }
  EXPECT_GT(best_excess, 0.05)
    << "with max_task_ang_acc unset the rotation was already acceleration-limited";
  ASSERT_TRUE(settle(0, 4000));
  EXPECT_NEAR(tcp_pose(measured_q()).pitch, kStartPitch + 0.3, 5e-3);
}

// ---------------------------------------------------------------------------
// AC11 (A). The space-switch re-seed starts from what is being COMMANDED in the
// new space, at the rate it is being commanded: joint -> task from
// FK(commanded q_des) with twist J(q_des) * dq_des, task -> joint from the
// commanded q_des with dq_des. Not from the measurement, and not from rest.
//
// The AC8 tests above cannot tell those apart. They switch at rest, so every
// rate is zero, and the fake mirrors each accepted q_des into the next
// measurement, so FK of the measured joints IS FK of the last command. Here the
// fake's mirror_position_offset keeps the measurement kMirrorOffset from q_des
// on every joint (a plant that never quite reaches its command), and the
// moving tests switch in mid-motion with the per-cycle trace recorded on the
// worker thread.

namespace
{
constexpr double kMirrorOffset = 0.005;  // rad, measured = q_des + this on every joint
constexpr double kJointRampRate = 0.1;   // rad/s on joint 1
constexpr double kTcpRampSpeed = 0.05;   // m/s, TCP straight down
// The time from the last recorded cycle to the cycle that splices the next
// chunk, which a continuation chunk is advanced by. A few cycles; an error in
// it is a gap the limiter closes, not a step.
constexpr double kContinuationLead = 0.002;  // s

// Largest change of any joint's commanded position from one recorded cycle to
// the next (starting from `before`), and largest distance from `before`.
struct CommandMotion
{
  double step;
  double drift;
};

CommandMotion command_motion(const std::vector<CycleSample> & trace, const Joints & before)
{
  CommandMotion motion {0.0, 0.0};
  const Joints * previous = &before;
  for (const auto & sample : trace) {
    for (std::size_t joint = 0; joint < 7; ++joint) {
      motion.step = std::max(motion.step, std::abs(sample.q_cmd[joint] - (*previous)[joint]));
      motion.drift = std::max(motion.drift, std::abs(sample.q_cmd[joint] - before[joint]));
    }
    previous = &sample.q_cmd;
  }
  return motion;
}

double dot(const Joints & a, const Joints & b)
{
  double sum = 0.0;
  for (std::size_t joint = 0; joint < 7; ++joint) {sum += a[joint] * b[joint];}
  return sum;
}

// First trace index at which the accepted-chunk count reached `count`, or the
// trace size if it never did.
std::size_t first_with_accepted(const std::vector<CycleSample> & trace, const std::uint64_t count)
{
  for (std::size_t i = 0; i < trace.size(); ++i) {
    if (trace[i].accepted >= count) {return i;}
  }
  return trace.size();
}

class MirrorOffsetFixture : public CurledFixture
{
protected:
  double mirror_position_offset() const override {return kMirrorOffset;}
};

// Rate bounds at which a reference restarted from rest stays visibly slower
// than the motion it replaces for many cycles: from zero, 0.5 m/s^2 needs
// 0.21 s to regain the joint ramp's 0.107 m/s TCP speed and 2 rad/s^2 needs
// 0.05 s to regain its 0.1 rad/s pitch rate; 1 rad/s^2 per joint needs ~0.1 s
// to regain the task ramp's joint rates. A re-seed WITH the commanded rates
// is unaffected by any of them on the switch cycle.
class MovingSwitchFixture : public CurledFixture
{
protected:
  void extra_parameters() override
  {
    CurledFixture::extra_parameters();
    set("max_task_lin_vel", 0.4);
    set("max_task_lin_acc", 0.5);
    set("max_task_ang_acc", 2.0);
    set("max_joint_ref_acc", std::vector<double>(7, 1.0));
    set("chunk_time_source", std::string("arrival"));
  }

  // One switch in mid-motion: the per-cycle trace from the first chunk's
  // publication to 300 cycles after the second's, `from` the first index
  // recorded after the second publication, `last` the cycle just before it.
  struct SwitchRun
  {
    std::vector<CycleSample> trace;
    std::size_t from {0};
    CycleSample last {};
  };

  // Joint 1 alone ramps at kJointRampRate; mid-ramp, one task chunk in the
  // same goal continues that exact path, FK(q_des + dq_des * t).
  void joint_ramp_then_task_path(SwitchRun & run)
  {
    ASSERT_EQ(configured, controller_interface::return_type::OK);
    wait_ready();
    ASSERT_TRUE(Access::ready(*controller));
    ASSERT_NE(send_goal(vla_goal()), nullptr);

    const Joints q0 = commanded_q();
    std::vector<Joints> ramp;
    for (int k = 0; k < 30; ++k) {
      Joints q = q0;
      q[0] += kJointRampRate * 0.05 * k;
      ramp.push_back(q);
    }
    start_recording();
    chunk_pub->publish(joint_path_chunk(ramp));
    cycle(750);  // mid-ramp: the ramp lasts 1.45 s
    const auto before = recorded();
    ASSERT_GE(before.size(), 600u);
    run.last = before.back();
    const CycleSample & last = run.last;
    ASSERT_NEAR(last.dq_cmd[0], kJointRampRate, 0.1 * kJointRampRate)
      << "the joint ramp is not being commanded at its rate";
    for (std::size_t joint = 1; joint < 7; ++joint) {
      ASSERT_NEAR(last.dq_cmd[joint], 0.0, 1e-3) << "joint " << joint + 1 << " moves in the joint ramp";
    }
    const auto twist = tcp_twist(last.q_cmd, last.dq_cmd);
    ASSERT_GT(twist.vx * twist.vx + twist.vz * twist.vz, 0.05 * 0.05);
    ASSERT_GT(twist.wy, 0.05);

    std::vector<PlanarPose> path;
    for (int k = 0; k < 20; ++k) {
      Joints q = last.q_cmd;
      for (std::size_t joint = 0; joint < 7; ++joint) {
        q[joint] += last.dq_cmd[joint] * (kContinuationLead + 0.05 * k);
      }
      path.push_back(tcp_pose(q));
    }
    run.from = before.size();
    chunk_pub->publish(pose_path_chunk(path));
    cycle(300);
    run.trace = stop_recording();
    EXPECT_EQ(Access::accepted(*controller), 2u);
    EXPECT_EQ(Access::rejected(*controller), 0u);
    ASSERT_LT(first_with_accepted(run.trace, 2), run.trace.size() - 200)
      << "the task chunk came too late to judge";
  }

  // The TCP goes straight down at kTcpRampSpeed; mid-ramp, one joint chunk in
  // the same goal continues the commanded joints along q_des + dq_des * t.
  void task_ramp_then_joint_path(SwitchRun & run)
  {
    ASSERT_EQ(configured, controller_interface::return_type::OK);
    wait_ready();
    ASSERT_TRUE(Access::ready(*controller));
    ASSERT_NE(send_goal(vla_goal()), nullptr);

    const auto start = tcp_pose(commanded_q());
    std::vector<PlanarPose> ramp;
    for (int k = 0; k < 30; ++k) {
      auto pose = start;
      pose.z -= kTcpRampSpeed * 0.05 * k;
      ramp.push_back(pose);
    }
    start_recording();
    chunk_pub->publish(pose_path_chunk(ramp));
    cycle(750);
    const auto before = recorded();
    ASSERT_GE(before.size(), 600u);
    run.last = before.back();
    const CycleSample & last = run.last;
    ASSERT_GT(dot(last.dq_cmd, last.dq_cmd), 0.05 * 0.05) << "the task ramp is not moving the joints";
    const auto twist = tcp_twist(last.q_cmd, last.dq_cmd);
    ASSERT_NEAR(twist.vz, -kTcpRampSpeed, 0.2 * kTcpRampSpeed) << "the task ramp is not being commanded";

    std::vector<Joints> path;
    for (int k = 0; k < 20; ++k) {
      Joints q = last.q_cmd;
      for (std::size_t joint = 0; joint < 7; ++joint) {
        q[joint] += last.dq_cmd[joint] * (kContinuationLead + 0.05 * k);
      }
      path.push_back(q);
    }
    run.from = before.size();
    chunk_pub->publish(joint_path_chunk(path));
    cycle(300);
    run.trace = stop_recording();
    EXPECT_EQ(Access::accepted(*controller), 2u);
    EXPECT_EQ(Access::rejected(*controller), 0u);
    ASSERT_LT(first_with_accepted(run.trace, 2), run.trace.size() - 200)
      << "the joint chunk came too late to judge";
  }
};
}  // namespace

// Joint -> task at rest, measurement 5 mrad off the command on every joint.
// The task law emits q_des = q + J^+ (x_ref - FK(q)) (drive-side DLS offset,
// lambda = 0.01). Seeded at FK(q_des), the first task cycle commands, to first
// order, q + J^+ J (q_des - q): the row-space part of the previous q_des - q is
// kept and its null-space part (I - J^+ J)(q_des - q) is dropped in that one
// cycle. This offset is uniform, and the pitch row of J is all ones, so it is
// nearly all row space: worked through this posture's kinematics, its
// null-space part is 0.04 mrad (from the DLS damping), and the 0.43 mrad the
// first cycle moves q_des on the worst joint is the second-order residual of
// the one Newton step (it scales with the offset squared); 0.49 mrad of drift
// to the law's fixed point. What this test checks is therefore row-space
// continuity. Seeded at FK(q), the reference sits on the measurement, the
// offset term vanishes and q_des = q: a 5 mrad step on every joint at once.
TEST_F(MirrorOffsetFixture, AC11_AJointToTaskSwitchStartsFromFkOfTheCommandedJoints)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  Joints target = commanded_q();
  target[0] += 0.2;
  chunk_pub->publish(joint_target_chunk(target));
  ASSERT_TRUE(settle(300)) << "the joint chunk did not come to rest";
  ASSERT_EQ(Access::accepted(*controller), 1u);
  const Joints q_des = commanded_q();
  const Joints q = measured_q();
  // The joint law commands the reference itself (q + clamp(q_ref - q)), so
  // the command sits on the target and the measurement is offset from it.
  ASSERT_LE(max_abs_difference(q_des, target), 1e-4) << "the joint chunk was not followed";
  for (std::size_t joint = 0; joint < 7; ++joint) {
    ASSERT_NEAR(q[joint] - q_des[joint], kMirrorOffset, 1e-6)
      << "the fake's mirror offset is not in effect on joint " << joint + 1;
  }

  // Same goal, task space, holding the TCP at FK of the COMMANDED joints.
  start_recording();
  chunk_pub->publish(pose_chunk(tcp_pose(q_des)));
  cycle(400);
  const auto trace = stop_recording();
  ASSERT_GE(trace.size(), 300u);
  EXPECT_EQ(Access::accepted(*controller), 2u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  ASSERT_LT(first_with_accepted(trace, 2), trace.size() - 200) << "the task chunk came too late to judge";

  const auto motion = command_motion(trace, q_des);
  EXPECT_LE(motion.step, kMirrorOffset / 4)
    << "q_des stepped by " << motion.step << " rad in one cycle at the joint -> task switch "
    << "(a re-seed from the measured pose steps it by the full " << kMirrorOffset << ")";
  EXPECT_LE(motion.drift, kMirrorOffset / 2)
    << "q_des left the commanded joints by " << motion.drift << " rad after the switch";
}

// Task -> joint at rest with the same offset. The task law has settled to its
// fixed point, the measurement 5 mrad off it; the joint chunk holds exactly
// the commanded joints. Seeded from the commanded q_des the joint law emits
// that same q_des (q + clamp(q_ref - q) with q_ref = q_des): no motion at all.
// Seeded from the measurement it emits the measurement, a 5 mrad step.
TEST_F(MirrorOffsetFixture, AC11_ATaskToJointSwitchStartsFromTheCommandedJoints)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  auto lowered = tcp_pose(commanded_q());
  lowered.z -= 0.05;
  chunk_pub->publish(pose_chunk(lowered));
  ASSERT_TRUE(settle(300)) << "the task chunk did not come to rest";
  ASSERT_EQ(Access::accepted(*controller), 1u);
  const Joints q_des = commanded_q();
  const Joints q = measured_q();
  // One DLS step from a measurement offset by 5 mrad leaves FK(q_des) within
  // about 0.25 mm of the target at this posture.
  ASSERT_NEAR(tcp_pose(q_des).z, lowered.z, 2e-3) << "the task chunk was not followed";
  for (std::size_t joint = 0; joint < 7; ++joint) {
    ASSERT_NEAR(q[joint] - q_des[joint], kMirrorOffset, 1e-6)
      << "the fake's mirror offset is not in effect on joint " << joint + 1;
  }

  start_recording();
  chunk_pub->publish(joint_target_chunk(q_des));
  cycle(400);
  const auto trace = stop_recording();
  ASSERT_GE(trace.size(), 300u);
  EXPECT_EQ(Access::accepted(*controller), 2u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  ASSERT_LT(first_with_accepted(trace, 2), trace.size() - 200) << "the joint chunk came too late to judge";

  const auto motion = command_motion(trace, q_des);
  EXPECT_LE(motion.step, kMirrorOffset / 4)
    << "q_des stepped by " << motion.step << " rad in one cycle at the task -> joint switch";
  EXPECT_LE(motion.drift, kMirrorOffset / 2)
    << "q_des left the commanded joints by " << motion.drift << " rad after the switch";
}

// Joint -> task in mid-motion. Joint 1 alone turns at 0.1 rad/s, so the
// commanded TCP twist is J(q_des) * dq_des = 0.1 * (dx/dq_1, dz/dq_1) at
// 0.107 m/s with a pitch rate of 0.1 rad/s (tcp_twist()). The task chunk
// continues that exact path, FK(q_des + dq_des * t). With the commanded twist
// in the seed, the switch cycle already commands it (dq_des = J^+ twist, whose
// image J dq_des is the twist up to the DLS damping, a few percent here) and
// the acceleration bounds keep it there. Seeded at rest, dq_des is zero on the
// switch cycle and regains the motion only at 0.5 m/s^2 and 2 rad/s^2.
TEST_F(MovingSwitchFixture, AC11_AJointToTaskSwitchInMotionKeepsTheCommandedTwist)
{
  SwitchRun run;
  ASSERT_NO_FATAL_FAILURE(joint_ramp_then_task_path(run));
  const auto & trace = run.trace;
  const std::size_t from = run.from;
  const auto twist = tcp_twist(run.last.q_cmd, run.last.dq_cmd);
  const double linear_sq = twist.vx * twist.vx + twist.vz * twist.vz;

  // Along the commanded twist, every cycle from the publication on: the
  // linear part projected on the joint ramp's, and the pitch rate.
  double worst_linear = std::numeric_limits<double>::infinity();
  double worst_angular = std::numeric_limits<double>::infinity();
  double worst_step = 0.0;
  bool spread = false;
  for (std::size_t i = from; i < trace.size(); ++i) {
    const auto now = tcp_twist(trace[i].q_cmd, trace[i].dq_cmd);
    worst_linear = std::min(worst_linear, (now.vx * twist.vx + now.vz * twist.vz) / linear_sq);
    worst_angular = std::min(worst_angular, now.wy / twist.wy);
    worst_step = std::max(worst_step, max_abs_difference(trace[i].q_cmd, trace[i - 1].q_cmd));
    for (std::size_t joint = 1; joint < 7; ++joint) {
      spread = spread || std::abs(trace[i].dq_cmd[joint]) > 5e-3;
    }
  }
  // The task law's dq_des = J^+ twist spreads over every joint (joint 2 gets
  // about 0.036 rad/s here), so this proves the window saw the task law.
  EXPECT_TRUE(spread) << "the commanded joint rates never left the joint ramp";
  EXPECT_GE(worst_linear, 0.5)
    << "the commanded TCP velocity fell to " << worst_linear << " of the joint ramp's at the switch";
  EXPECT_GE(worst_angular, 0.5)
    << "the commanded pitch rate fell to " << worst_angular << " of the joint ramp's at the switch";
  EXPECT_LE(worst_step, 2e-3) << "q_des stepped by " << worst_step << " rad at the switch";
}

// Task -> joint in mid-motion. The TCP goes straight down at 0.05 m/s, which
// the task law commands as dq_des = J^+ v (about 0.15 rad/s in norm here,
// spread over all seven joints). The joint chunk continues those joints along
// q_des + dq_des * t. Seeded with dq_des, the joint law's dq_des carries on;
// seeded at rest it is zero on the switch cycle and regains the motion only
// at 1 rad/s^2 per joint.
TEST_F(MovingSwitchFixture, AC11_ATaskToJointSwitchInMotionKeepsTheCommandedJointRates)
{
  SwitchRun run;
  ASSERT_NO_FATAL_FAILURE(task_ramp_then_joint_path(run));
  const auto & trace = run.trace;
  const std::size_t from = run.from;
  const CycleSample & last = run.last;
  const double speed_sq = dot(last.dq_cmd, last.dq_cmd);

  double worst = std::numeric_limits<double>::infinity();
  double worst_step = 0.0;
  for (std::size_t i = from; i < trace.size(); ++i) {
    worst = std::min(worst, dot(trace[i].dq_cmd, last.dq_cmd) / speed_sq);
    worst_step = std::max(worst_step, max_abs_difference(trace[i].q_cmd, trace[i - 1].q_cmd));
  }
  EXPECT_GE(worst, 0.5)
    << "the commanded joint rates fell to " << worst << " of the task ramp's at the switch";
  EXPECT_LE(worst_step, 2e-3) << "q_des stepped by " << worst_step << " rad at the switch";
}

// ---------------------------------------------------------------------------
// AC13 (C). Under drive_side_impedance: false a joint chunk is refused as a
// well-formed chunk in an action space this configuration does not take
// (Reject::kActionSpaceNotAccepted, "action_space_not_accepted"), not as an
// unparseable field -- which is still what an unknown field value is.

TEST_F(LegacyLawFixture, AC13_TheJointRefusalHasItsOwnReasonDistinctFromABadField)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  ASSERT_FALSE(Access::drive_side_impedance(*controller));
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  watch_telemetry();

  // A task chunk with an unknown relative_frame: a bad field, as before.
  publish(per_step_z_chunk(-0.01, "bogus"), 40);
  ASSERT_TRUE(wait_telemetry([](const auto & t) {return t.chunks_rejected >= 1;}))
    << "no telemetry with the first rejection on " << telemetry_topic()
    << "; VlaTelemetry topics: " << telemetry_topics();
  EXPECT_EQ(telemetry().last_reject, "unparseable_field");

  // A well-formed joint chunk this law cannot run.
  publish(joint_chunk(0.5), 40);
  ASSERT_TRUE(wait_telemetry([](const auto & t) {return t.chunks_rejected >= 2;}))
    << "no telemetry with the second rejection";
  const auto latest = telemetry();
  EXPECT_EQ(latest.last_reject, "action_space_not_accepted");
  EXPECT_EQ(latest.chunks_rejected, 2u);
  EXPECT_EQ(latest.chunks_accepted, 0u);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 2u);
}

// The new reason is the legacy law's alone: a drive-side host takes a joint
// chunk, and refuses one with an unknown relative_frame as a bad field.
// CurledFixture for its long stream watchdog: a rejected chunk does not feed
// it, and the telemetry wait may outlast the default 0.2 s.
TEST_F(CurledFixture, AC13_ADriveSideHostTakesJointChunksAndReportsABadFieldAsSuch)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  ASSERT_TRUE(Access::drive_side_impedance(*controller));
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  watch_telemetry();

  const Joints here = measured_q();
  auto bad = joint_target_chunk(here);
  bad.relative_frame = "bogus";
  publish(bad, 40);
  ASSERT_TRUE(wait_telemetry([](const auto & t) {return t.chunks_rejected >= 1;}))
    << "no telemetry with the rejection on " << telemetry_topic()
    << "; VlaTelemetry topics: " << telemetry_topics();
  EXPECT_EQ(telemetry().last_reject, "unparseable_field");

  publish(joint_target_chunk(here), 40);
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 1u);
}

// ---------------------------------------------------------------------------
// AC18 (review 20261013-140008 C1, C4).
//
// C1: the limiter is never seeded outside the joint window. Under the task law
// q_des = q + offset can lie past a joint's window edge; the joint is sent at
// the edge. A task -> joint switch (or a goal start or resume) that seeded the
// joint limiter with the unclamped q_des left the limiter's window clamp to
// pull the joint back on the next cycle, and that cycle's rate came out as the
// whole distance over one period.
//
// C4: a space switch seeds the limiter at the previous cycle's time, so the
// switch cycle advances the command by one cycle of the commanded motion, as
// every other cycle does. Seeded at the switch cycle's own time, the limiter
// has no time to integrate on that cycle and returns the seed: q_des stands
// still for one cycle (0.1 mrad at 0.1 rad/s and 1 kHz) and stays that far
// behind.

namespace
{
constexpr double kCycle = 1e-3;  // s, one update of the fixture's worker
// Joint 4 (index 3). Its mujoco_sim_safe window is [0, 2.443461] rad, so its
// lower edge is its zero.
constexpr std::size_t kEdgeJoint = 3;
constexpr double kNearEdge = 0.02;   // rad, where the joint chunk parks joint 4
constexpr double kPastEdge = -0.1;   // rad, where the task target needs it
constexpr double kJointRefAcc = 4.0;  // rad/s^2, CurledFixture's max_joint_ref_acc

// The task law's DLS offset weighted onto joint 4 (reference_offset_weight 1
// there, 1e6 on the others): it resolves the task error with joint 4 and
// leaves the other joints practically where they are. A task target that
// needs joint 4 at kPastEdge then holds q_des_4 = 0 + offset_4 about 0.1 rad
// below the window for as long as the target stands (worked through this
// posture's kinematics: -0.099 rad after 0.6 s), while joint 4 is sent, and
// measured, at the edge. With equal weights the other joints take the error
// up within a few cycles and the push past the edge lasts only that long.
class WindowEdgeFixture : public CurledFixture
{
protected:
  void extra_parameters() override
  {
    CurledFixture::extra_parameters();
    std::vector<double> weight(7, 1e6);
    weight[kEdgeJoint] = 1.0;
    set("reference_offset_weight", weight);
  }
};

// The slowest and the fastest of a per-cycle series, and where they were.
struct Extremes
{
  double slowest {std::numeric_limits<double>::infinity()};
  double fastest {-std::numeric_limits<double>::infinity()};
  std::size_t slowest_at {0};
  std::size_t fastest_at {0};

  void note(const double value, const std::size_t at)
  {
    if (value < slowest) {
      slowest = value;
      slowest_at = at;
    }
    if (value > fastest) {
      fastest = value;
      fastest_at = at;
    }
  }
};
}  // namespace

// Task -> joint with joint 4 pushed past its window edge by the task law. The
// target has been reached as far as the edge allows, so nothing moves; the
// joint chunk holds the commanded joints, joint 4 at its edge. Seeded inside
// the window, at the commanded rates, every dq_des stays within what the
// acceleration bound allows from the rates before the switch, two cycles of
// 4 rad/s^2. Seeded at the unclamped q_des_4, the joint limiter's window clamp
// reports the 0.1 rad back to the edge as one cycle's rate, 100 rad/s, which
// the host passes on at command_velocity (5.4 rad/s on joint 4).
TEST_F(WindowEdgeFixture, AC18_ATaskToJointSwitchAtAWindowEdgeKeepsDqDesWithinTheBounds)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  // Joint 4 to just inside its lower edge.
  Joints parked = commanded_q();
  parked[kEdgeJoint] = kNearEdge;
  chunk_pub->publish(joint_target_chunk(parked));
  ASSERT_TRUE(settle(300)) << "the joint chunk did not come to rest";
  ASSERT_EQ(Access::accepted(*controller), 1u);
  ASSERT_LE(max_abs_difference(commanded_q(), parked), 1e-4) << "joint 4 was not parked";

  // Same goal, a task target that needs joint 4 at -0.1 rad.
  Joints past = commanded_q();
  past[kEdgeJoint] = kPastEdge;
  const PlanarPose target = tcp_pose(past);
  chunk_pub->publish(pose_chunk(target));
  cycle(1500);
  ASSERT_EQ(Access::accepted(*controller), 2u);
  ASSERT_EQ(Access::rejected(*controller), 0u);

  start_recording();
  cycle(30);
  const auto before = recorded();
  ASSERT_GE(before.size(), 20u);
  const CycleSample last = before.back();
  ASSERT_NEAR(last.q_cmd[kEdgeJoint], 0.0, 1e-9) << "joint 4 is not sent at its window edge";
  // Held off the target pitch by the edge: the task law is still pushing.
  ASSERT_GE(tcp_pose(last.q_cmd).pitch - target.pitch, 0.05)
    << "the task target does not need joint 4 past its edge";
  double rate_before = 0.0;
  for (const auto & sample : before) {
    for (std::size_t joint = 0; joint < 7; ++joint) {
      rate_before = std::max(rate_before, std::abs(sample.dq_cmd[joint]));
    }
  }
  ASSERT_LE(rate_before, 0.05) << "the arm is not at rest before the switch";

  // Same goal, joint space, holding the commanded joints.
  const std::size_t from = before.size();
  chunk_pub->publish(joint_target_chunk(last.q_cmd));
  cycle(300);
  const auto trace = stop_recording();
  EXPECT_EQ(Access::accepted(*controller), 3u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  ASSERT_LT(first_with_accepted(trace, 3), trace.size() - 200) << "the joint chunk came too late to judge";

  const double rate_bound = rate_before + 2.0 * kJointRefAcc * kCycle + 1e-3;
  double worst_rate = 0.0;
  std::size_t worst_joint = 0;
  std::size_t worst_at = 0;
  double worst_step = 0.0;
  for (std::size_t i = from; i < trace.size(); ++i) {
    for (std::size_t joint = 0; joint < 7; ++joint) {
      const double rate = std::abs(trace[i].dq_cmd[joint]);
      if (rate > worst_rate) {
        worst_rate = rate;
        worst_joint = joint;
        worst_at = i - from;
      }
    }
    worst_step = std::max(worst_step, max_abs_difference(trace[i].q_cmd, trace[i - 1].q_cmd));
  }
  RecordProperty("rate_before", std::to_string(rate_before));
  RecordProperty("worst_rate", std::to_string(worst_rate));
  RecordProperty("worst_step", std::to_string(worst_step));
  EXPECT_LE(worst_rate, rate_bound)
    << "dq_des reached " << worst_rate << " rad/s on joint " << worst_joint + 1 << ", "
    << worst_at << " cycles after the joint chunk was published";
  EXPECT_LE(worst_step, 1e-4) << "q_des stepped by " << worst_step << " rad at the switch";
  EXPECT_NEAR(trace.back().q_cmd[kEdgeJoint], 0.0, 1e-6) << "joint 4 left its edge";
}

// Joint -> task in mid-motion (the AC11 scenario): per cycle from the task
// chunk's publication on, how far the commanded TCP moved along the twist
// commanded before it, in cycles of that twist (1 = as before, 0 = stood
// still). Measured on the TCP because the task law keeps only the row-space
// part of the joint motion (AC11): the joints' own progress changes there by
// design.
TEST_F(MovingSwitchFixture, AC18_AJointToTaskSwitchInMotionAdvancesTheCommandOnTheSwitchCycle)
{
  SwitchRun run;
  ASSERT_NO_FATAL_FAILURE(joint_ramp_then_task_path(run));
  const auto twist = tcp_twist(run.last.q_cmd, run.last.dq_cmd);
  const double linear_sq = twist.vx * twist.vx + twist.vz * twist.vz;

  Extremes linear;
  Extremes angular;
  for (std::size_t i = run.from; i < run.trace.size(); ++i) {
    const auto a = tcp_pose(run.trace[i - 1].q_cmd);
    const auto b = tcp_pose(run.trace[i].q_cmd);
    linear.note(
      ((b.x - a.x) * twist.vx + (b.z - a.z) * twist.vz) / (linear_sq * kCycle), i - run.from);
    angular.note((b.pitch - a.pitch) / (twist.wy * kCycle), i - run.from);
  }
  RecordProperty("linear_progress", std::to_string(linear.slowest) + " .. " + std::to_string(linear.fastest));
  RecordProperty("angular_progress", std::to_string(angular.slowest) + " .. " + std::to_string(angular.fastest));
  EXPECT_GE(linear.slowest, 0.5)
    << "the commanded TCP advanced " << linear.slowest << " of a cycle of its velocity, "
    << linear.slowest_at << " cycles after the task chunk was published";
  EXPECT_LE(linear.fastest, 1.5)
    << "the commanded TCP advanced " << linear.fastest << " cycles of its velocity in one, "
    << linear.fastest_at << " cycles after the task chunk was published";
  EXPECT_GE(angular.slowest, 0.5)
    << "the commanded pitch advanced " << angular.slowest << " of a cycle of its rate, "
    << angular.slowest_at << " cycles after the task chunk was published";
  EXPECT_LE(angular.fastest, 1.5)
    << "the commanded pitch advanced " << angular.fastest << " cycles of its rate in one, "
    << angular.fastest_at << " cycles after the task chunk was published";
}

// Task -> joint in mid-motion (the AC11 scenario): per cycle from the joint
// chunk's publication on, how far q_des moved along the joint rates commanded
// before it, in cycles of those rates.
TEST_F(MovingSwitchFixture, AC18_ATaskToJointSwitchInMotionAdvancesTheCommandOnTheSwitchCycle)
{
  SwitchRun run;
  ASSERT_NO_FATAL_FAILURE(task_ramp_then_joint_path(run));
  const Joints & rate = run.last.dq_cmd;
  const double speed_sq = dot(rate, rate);

  Extremes progress;
  for (std::size_t i = run.from; i < run.trace.size(); ++i) {
    Joints step {};
    for (std::size_t joint = 0; joint < 7; ++joint) {
      step[joint] = run.trace[i].q_cmd[joint] - run.trace[i - 1].q_cmd[joint];
    }
    progress.note(dot(step, rate) / (speed_sq * kCycle), i - run.from);
  }
  RecordProperty("progress", std::to_string(progress.slowest) + " .. " + std::to_string(progress.fastest));
  EXPECT_GE(progress.slowest, 0.5)
    << "q_des advanced " << progress.slowest << " of a cycle of its rates, "
    << progress.slowest_at << " cycles after the joint chunk was published";
  EXPECT_LE(progress.fastest, 1.5)
    << "q_des advanced " << progress.fastest << " cycles of its rates in one, "
    << progress.fastest_at << " cycles after the joint chunk was published";
}

// ---------------------------------------------------------------------------
// AC10 (S15). The stream watchdog holds when the stream has been quiet for
// stream_timeout AND the buffered horizon has run out, with open-loop replay
// capped at stream_horizon_credit_sec (1.0 s default). Here stream_timeout is
// the fixture's 0.2 s and the goal sets none.

TEST_F(StreamFixture, AC10_ABufferedHorizonKeepsTheWatchdogOffBetweenChunks)
{
  // A re-inference every 0.5 s with a 0.95 s horizon: a chunk always arrives
  // while the previous one still has half a second to play. The quiet gap is
  // 2.5x stream_timeout, so the pre-S15 watchdog entered hold after every chunk
  // (about 0.3 s of each 0.5 s).
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  int published = 0;
  auto next = Clock::now();
  for_seconds(3.0, [&] {
      if (Clock::now() >= next) {
        chunk_pub->publish(joint_chunk(published % 2 == 0 ? 0.2 : 0.25, 20));
        ++published;
        next += std::chrono::milliseconds(500);
      }
    });

  ASSERT_GE(published, 5);
  EXPECT_EQ(entries_into("hold"), 0) << "the watchdog held while a horizon was buffered";
  EXPECT_GE(entries_into("running"), 1);
  EXPECT_GE(Access::accepted(*controller), static_cast<std::uint64_t>(published - 1));
  const auto all = stream_samples();
  ASSERT_FALSE(all.empty());
  EXPECT_EQ(all.back().state, "running");
}

TEST_F(StreamFixture, AC10_AQuietStreamHoldsOnlyWhenTheHorizonHasEnded)
{
  // One chunk reaching 0.95 s, then silence: hold no sooner than the horizon
  // (bound loosened to 0.85 s), and still before the goal is left forever.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const auto published = Clock::now();
  publish(joint_chunk(0.2, 20), 10);
  ASSERT_EQ(Access::accepted(*controller), 1u);

  Clock::time_point held;
  ASSERT_TRUE(wait_state("hold", published, 3.0, &held));
  EXPECT_GE(seconds_between(published, held), 0.85);
  EXPECT_LE(seconds_between(published, held), 2.5);
  // Silence stays silence: hold_timeout 0.6 s later the goal is aborted.
  auto result = goal.client->async_get_result(goal.handle);
  for (int i = 0; i < 2000 &&
    result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle(2);
  }
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(CreditFixture, AC10_OpenLoopReplayIsCappedByTheHorizonCredit)
{
  // A 3.95 s horizon from a policy that then dies. With stream_horizon_credit_sec
  // 0.5 the hold begins about 0.5 s after the chunk, not after 3.95 s.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const auto published = Clock::now();
  publish(joint_chunk(0.2, 80), 10);
  ASSERT_EQ(Access::accepted(*controller), 1u);

  Clock::time_point held;
  ASSERT_TRUE(wait_state("hold", published, 3.0, &held));
  EXPECT_GE(seconds_between(published, held), 0.45);
  EXPECT_LE(seconds_between(published, held), 1.5);
}

// ---------------------------------------------------------------------------
// AC5 (S8). With chunk_time_source: observation, a chunk whose observation
// stamp is in the future (more than chunk_max_future_skew, 0.05 s) or older than
// chunk_max_observation_age (1.0 s) at arrival is rejected under its own
// reason, and a chunk that leaves no waypoint to play does not count as
// accepted and does not feed the watchdog.

TEST_F(ObservationFixture, AC5_AnObservationInsideTheWindowIsAccepted)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  // 0.3 s old (limit 1.0) and a 0.95 s horizon: 0.65 s of it still ahead.
  publish(stamped(long_chunk(), 0.3), 40);
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
  EXPECT_EQ(last_reject(), "none");

  // Slightly ahead of the controller clock, inside the 0.05 s skew allowance.
  publish(stamped(long_chunk(0.25), -0.02), 40);
  EXPECT_EQ(Access::accepted(*controller), 2u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
}

TEST_F(ObservationFixture, AC5_AnObservationOlderThanTheMaximumAgeIsRejected)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  publish(stamped(long_chunk(), 10.0), 40);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  EXPECT_TRUE(wait_reject("observation_too_old")) << "last_reject was: " << last_reject();
}

TEST_F(ObservationFixture, AC5_AnObservationInTheFutureIsRejected)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  publish(stamped(long_chunk(), -1.0), 40);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  EXPECT_TRUE(wait_reject("observation_in_future")) << "last_reject was: " << last_reject();
}

TEST_F(NarrowAgeFixture, AC5_TheMaximumObservationAgeIsAParameter)
{
  // chunk_max_observation_age 0.3: a 0.5 s old stamp is refused, a 0.1 s old
  // one is not (both are well inside the 1.0 s default).
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  publish(stamped(long_chunk(), 0.5), 40);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_TRUE(wait_reject("observation_too_old")) << "last_reject was: " << last_reject();

  publish(stamped(long_chunk(), 0.1), 40);
  EXPECT_EQ(Access::accepted(*controller), 1u);
}

TEST_F(StreamFixture, AC5_TheArrivalClockDoesNotJudgeTheObservationStamp)
{
  // The default chunk_time_source is arrival: the stamp is not the time base, so
  // a stamp from another clock domain (here 10 s old) must not get a chunk
  // refused by the observation window.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  auto message = joint_chunk(0.2, 20);
  const rclcpp::Time stamp = client_node->now() - rclcpp::Duration::from_seconds(10.0);
  message.header.stamp = stamp;
  publish(message, 40);
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
}

TEST_F(ObservationFixture, AC5_AChunkWithNoWaypointLeftDoesNotCountOrFeedTheWatchdog)
{
  // A valid chunk (0.35 s horizon), then a stream of chunks observed 0.8 s ago
  // (inside the age window) whose 8 waypoints all lie 0.45 s or more in the
  // past: nothing in them is playable. They are not "accepted", and they must
  // not keep the stream looking alive -- the watchdog still holds.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const auto first = Clock::now();
  publish(stamped(joint_chunk(0.2, 8), 0.0), 10);
  ASSERT_EQ(Access::accepted(*controller), 1u);

  bool held = false;
  Clock::time_point when;
  for_seconds(2.5, [&] {
      if (held) {return;}
      chunk_pub->publish(stamped(joint_chunk(0.3, 8), 0.8));
      for (const auto & sample : stream_samples()) {
        if (sample.at >= first && sample.state == "hold") {held = true;}
      }
    });
  EXPECT_TRUE(held) << "stale chunks kept the watchdog alive";
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_TRUE(wait_state("hold", first, 0.5, &when));
}

// ---------------------------------------------------------------------------
// AC6 (C16). A chunk with control_dt <= 0 is rejected, not played at an
// invented spacing derived from the goal's inference_frequency (15 Hz here,
// which used to make it a perfectly playable chunk).

TEST_F(StreamFixture, AC6_AZeroControlDtChunkIsRejectedAndMovesNothing)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const auto before = commanded_q();
  publish(joint_chunk(0.5, 8, 0.0), 60);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  EXPECT_TRUE(wait_reject("control_dt")) << "last_reject was: " << last_reject();
  EXPECT_LT(max_abs_difference(commanded_q(), before), 1e-3);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
}

TEST_F(StreamFixture, AC6_ANegativeControlDtChunkIsRejected)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  publish(joint_chunk(0.5, 8, -0.05), 60);
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  EXPECT_TRUE(wait_reject("control_dt")) << "last_reject was: " << last_reject();
}

TEST_F(StreamFixture, AC6_APositiveControlDtChunkIsStillAccepted)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_stream();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  publish(joint_chunk(0.2, 8, 0.05), 40);
  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
}

// ---------------------------------------------------------------------------
// Telemetry stamps (VLA session, AC6). last_chunk_stamp and playback_stamp are
// ROS time (the node clock), not seconds since activation, so a bridge that
// sets its RTC from them reads the same clock as the chunk headers.

// Arrival-timed: last_chunk_stamp is the arrival instant and playback_stamp a
// recent cycle, both on the clock client_node reads.
TEST_F(StreamFixture, AC6_TelemetryStampsAreRosTime)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_telemetry();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const double before = client_node->now().seconds();
  publish(joint_chunk(0.2, 20), 40);
  ASSERT_TRUE(wait_telemetry([](const auto & t) {
      return t.chunks_accepted >= 1 && t.playback_stamp > 0.0 && t.stream_state == "running";
    }));
  const auto t = telemetry();
  const double after = client_node->now().seconds();
  const double header = rclcpp::Time(t.header.stamp).seconds();
  EXPECT_GE(t.last_chunk_stamp, before - 0.01);
  EXPECT_LE(t.last_chunk_stamp, after + 0.01);
  EXPECT_GE(t.playback_stamp, before - 0.01);
  EXPECT_LE(t.playback_stamp, header + 0.01);
  EXPECT_LE(header - t.playback_stamp, 0.05);
}

// Observation-timed: last_chunk_stamp is the chunk's own stamp, as sent.
TEST_F(ObservationFixture, AC6_TheObservationStampComesBackAsSent)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_telemetry();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  const auto message = stamped(long_chunk(), 0.3);
  const double stamp = rclcpp::Time(message.header.stamp).seconds();
  const double offset_before = Access::action_time_offset(*controller);
  publish(message, 40);
  ASSERT_TRUE(wait_telemetry([](const auto & t) {return t.chunks_accepted >= 1;}));
  const auto t = telemetry();
  const double offset_after = Access::action_time_offset(*controller);
  // The stamp goes to the action clock at ingest and back at publish, each through
  // the offset of the last cycle. This worker advances the action clock 1 ms per
  // cycle but sleeps at least 1 ms of node time, so the offset only grows, by a
  // load-dependent amount: the stamp returns late by at most that growth over
  // the test (offset_after - offset_before), never early. An arrival stamp
  // would be 0.3 s off. 1e-5 s is double rounding at ~1.7e9 s.
  const double returned_late = t.last_chunk_stamp - stamp;
  EXPECT_GE(returned_late, -1e-5);
  EXPECT_LE(returned_late, offset_after - offset_before + 1e-5);
  EXPECT_NEAR(t.last_chunk_latency, 0.3, 0.1);
}

// Before any chunk of this goal there is no stamp: 0, as on the Franka host.
TEST_F(StreamFixture, AC6_AnUnsetStampIsZero)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  watch_telemetry();
  auto goal = open_goal();
  ASSERT_NE(goal.handle, nullptr);

  ASSERT_TRUE(wait_telemetry([](const auto & t) {return t.stream_state == "waiting_first_chunk";}));
  EXPECT_EQ(telemetry().last_chunk_stamp, 0.0);
}

// ---------------------------------------------------------------------------
// bounded_dt() (2026-10-13) is the gripper's rule. The model
// feed-forward slews at the profile's tau_ff_slew_per_s, 17.5 N m/s on joint 5
// in mujoco_sim_safe, per bounded_dt(): a zero period moves it not at all (the
// former 1e-4 s floor moved it 1.75 mN m per cycle), a 1 s one by 0.1 s worth,
// 1.75 N m (the former uncapped step crossed the whole 3 N m in one cycle).
TEST_F(Fixture, BoundedDt_TheModelSlewGetsNoTimeFromAZeroPeriodAndAtMostATenthOfASecond)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  // Joint 5 carries three 1 kg links 0.1-0.5 m out, 8.8 N m at q = 0, so its
  // model term rises to the 3 N m torque_limit within 0.2 s and stays there.
  cycle(400);
  const double held = Access::effort(*controller, 4);
  ASSERT_NEAR(std::abs(held), 3.0, 1e-9);
  constexpr double kSlew = 17.5;  // N m/s, joint 5

  worker_period.store(0.0, std::memory_order_release);
  cycle(5);
  set("gravity_scale", 0.0);
  start_recording();
  cycle(100);
  auto trace = stop_recording();
  ASSERT_GE(trace.size(), 50u);
  for (const auto & sample : trace) {
    ASSERT_EQ(sample.tau_cmd[4], held) << "a zero period moved the model term";
  }

  start_recording();
  worker_period.store(1.0, std::memory_order_release);
  cycle(20);
  worker_period.store(0.001, std::memory_order_release);
  trace = stop_recording();
  ASSERT_GE(trace.size(), 3u);
  double previous = held;
  double largest_step = 0.0;
  for (const auto & sample : trace) {
    largest_step = std::max(largest_step, std::abs(sample.tau_cmd[4] - previous));
    previous = sample.tau_cmd[4];
  }
  EXPECT_NEAR(largest_step, 0.1 * kSlew, 1e-9) << "a 1 s period was not capped at 0.1 s";
  EXPECT_NEAR(trace.back().tau_cmd[4], 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// S14 (2026-10-13): the relative-chunk anchor is the command as emitted.
//
// The kinematic wall of review 20261007-212552 (scratch/repro_S14_all.cpp),
// here through the controller: a policy that sees only the measured pose
// presses 10 mm into a wall 10 mm below the start for 5 s, then asks for 50 mm
// above it. Anchored on the commanded reference, every chunk built on a
// reference already past the wall, which walked on at the policy's speed: this
// test measured it 207 mm past the wall at the reversal and the arm 5.1 s
// leaving it (the repro: 181 mm, 4.9 s). Anchored on the command as emitted,
// the reference stays within the offset bound (~2.7 mm here) plus one chunk's
// 10 mm of the wall, and the arm leaves about as soon as with a measured
// anchor (8.5 mm and 339 ms here; the repro's measured anchor: 332-353 ms).
TEST_F(WallFixture, S14_PressingAWallDoesNotWindTheRelativeAnchorPastIt)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  const double wall = tcp_pose(measured_q()).z - 0.010;
  wall_z.store(wall, std::memory_order_release);

  constexpr double kPress = 5.0;   // s of control time from the first chunk
  constexpr double kWindow = 2.5;  // s after the reversal before giving up
  start_recording();
  const auto first = update_count.load(std::memory_order_acquire);
  const auto elapsed = [&]() {
      return 1e-3 * static_cast<double>(update_count.load(std::memory_order_acquire) - first);
    };
  double next_chunk = 0.0;
  unsigned long long reversal = 0;  // update count at the first retreat chunk
  unsigned long long left = 0;  // update count once the arm is off the wall
  while (elapsed() < kPress + kWindow) {
    const double now = elapsed();
    const double z = tcp_pose(measured_q()).z;
    if (now >= next_chunk) {
      const bool press = now < kPress;
      if (!press && reversal == 0) {reversal = update_count.load(std::memory_order_acquire);}
      chunk_pub->publish(policy_chunk(z, press ? wall - 0.010 : wall + 0.050));
      next_chunk = std::max(next_chunk + kPolicyPeriod, now);
    }
    if (reversal != 0 && z > wall + 1e-4) {
      left = update_count.load(std::memory_order_acquire);
      break;
    }
    cycle();
  }
  const auto trace = stop_recording();
  ASSERT_NE(reversal, 0u);
  ASSERT_FALSE(trace.empty());

  // The scenario held: the arm reached the wall and stayed on it, and the
  // command was saturated against it, not merely tracking.
  double lowest = std::numeric_limits<double>::infinity();
  double widest_offset = 0.0;
  double depth_at_reversal = std::numeric_limits<double>::quiet_NaN();
  double z_at_reversal = std::numeric_limits<double>::quiet_NaN();
  for (const auto & sample : trace) {
    const double z = tcp_pose(sample.q_meas).z;
    lowest = std::min(lowest, z);
    if (sample.update < reversal) {
      for (std::size_t joint = 0; joint < 7; ++joint) {
        widest_offset = std::max(widest_offset, std::abs(sample.q_cmd[joint] - sample.q_meas[joint]));
      }
    }
    if (sample.update == reversal) {
      depth_at_reversal = wall - sample.reference_z;
      z_at_reversal = z;
    }
  }
  EXPECT_GE(lowest, wall - 1e-6) << "the plant let the TCP through the wall";
  ASSERT_TRUE(std::isfinite(depth_at_reversal)) << "no sample at the reversal cycle";
  EXPECT_NEAR(z_at_reversal, wall, 1e-4) << "the arm was not on the wall when the policy reversed";
  EXPECT_NEAR(widest_offset, kOffsetBound, 0.1 * kOffsetBound)
    << "the command never saturated against the wall";

  const double delay = left != 0 ? 1e-3 * static_cast<double>(left - reversal) :
    std::numeric_limits<double>::infinity();
  std::printf(
    "[S14] after %.1f s against the wall: commanded reference %.1f mm past it at the reversal, "
    "off the wall %.0f ms after the reversal (offset bound %.3f rad)\n",
    kPress, 1e3 * depth_at_reversal, 1e3 * delay, kOffsetBound);
  RecordProperty("reference_depth_at_reversal_mm", std::to_string(1e3 * depth_at_reversal));
  RecordProperty("release_delay_ms", std::to_string(1e3 * delay));
  // One chunk's 10 mm and the ~2.7 mm saturation distance, with room for the
  // limiter's overshoot; wound up, it is over 100 mm.
  EXPECT_LT(depth_at_reversal, 0.020) << "the reference wound up past the wall";
  // The measured-anchor level is ~0.34 s; wound up, it is several seconds.
  EXPECT_LT(delay, 0.8) << "the arm stayed on the wall after the policy reversed";
}
