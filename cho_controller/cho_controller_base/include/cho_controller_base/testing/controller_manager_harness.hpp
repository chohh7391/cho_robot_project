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

// TEST SUPPORT, header-only: a controller_manager over
// mock_components/GenericSystem that a test drives one control period at a
// time, with an action client node on the same executor. For an adapter test
// that needs the real lifecycle -- switches, interface claims, a goal sent
// over ROS -- rather than a hand-called compute(). Nothing in this package
// includes it; a test that does links controller_manager itself.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cho_controller_base/held_command.hpp"

namespace cho_controller_base::testing
{

// One GenericSystem joint per entry of `arm`, exporting position, velocity and
// effort as both command and state so any control mode can claim its own.
// GenericSystem copies every command into the state of the same name on the
// next read(), so a test reads what a controller commanded from the states. A
// position command therefore also moves the joint; effort and velocity do not,
// which holds the arm still under a torque or velocity controller.
// `passive` joints export position and velocity states only.
//
// `hardware` adds GenericSystem parameters. Two change what the arm does:
//   {"position_state_following_offset", "0.01"} -- the position state reads the
//     command plus the offset, a steady tracking error like a gravity droop, so
//     a held command and the measurement differ;
//   {"calculate_dynamics", "true"} -- a velocity command moves the joint (the
//     position integrates it), so a velocity controller can move the arm.
inline std::string mock_ros2_control(
  const std::vector<std::string> & arm, const std::vector<double> & initial,
  const std::vector<std::string> & passive = {},
  const std::vector<std::pair<std::string, std::string>> & hardware = {})
{
  std::ostringstream out;
  out << "<ros2_control name='mock_arm' type='system'><hardware>"
      << "<plugin>mock_components/GenericSystem</plugin>";
  for (const auto & parameter : hardware) {
    out << "<param name='" << parameter.first << "'>" << parameter.second << "</param>";
  }
  out << "</hardware>";
  for (std::size_t i = 0; i < arm.size(); ++i) {
    out << "<joint name='" << arm[i] << "'>";
    for (const char * name : {"position", "velocity", "effort"}) {
      out << "<command_interface name='" << name << "'/>";
    }
    out << "<state_interface name='position'><param name='initial_value'>"
        << (i < initial.size() ? initial[i] : 0.0) << "</param></state_interface>"
        << "<state_interface name='velocity'/><state_interface name='effort'/></joint>";
  }
  for (const auto & joint : passive) {
    out << "<joint name='" << joint << "'><state_interface name='position'/>"
        << "<state_interface name='velocity'/></joint>";
  }
  out << "</ros2_control>";
  return out.str();
}

// `urdf` with every <ros2_control> block replaced by `block`.
inline std::string with_ros2_control(std::string urdf, const std::string & block)
{
  for (auto start = urdf.find("<ros2_control"); start != std::string::npos;
    start = urdf.find("<ros2_control"))
  {
    const auto end = urdf.find("</ros2_control>", start);
    if (end == std::string::npos) {
      break;
    }
    urdf.erase(start, end + std::string("</ros2_control>").size() - start);
  }
  const auto close = urdf.rfind("</robot>");
  return close == std::string::npos ? urdf : urdf.insert(close, block);
}

class ControllerManagerHarness
{
public:
  using Strictness = controller_manager_msgs::srv::SwitchController::Request;
  static constexpr double kPeriod = 0.001;

  ControllerManagerHarness(const std::string & urdf, const std::string & ns)
  {
    // The ledger is process-wide, and every test builds a fresh mock arm with
    // the same joint names: a release recorded by an earlier test is not about
    // this arm.
    cho_controller_base::HeldCommandLedger::clear();
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(urdf, true, true);
    resources_ = resources.get();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor_, "controller_manager", ns);
    clock_ns_ = manager_->now().nanoseconds();
    client_node_ = std::make_shared<rclcpp::Node>("harness_client", ns);
    executor_->add_node(client_node_);
  }

  ControllerManagerHarness(const ControllerManagerHarness &) = delete;
  ControllerManagerHarness & operator=(const ControllerManagerHarness &) = delete;

  // Loads `type` (a pluginlib name) as `name` and sets `parameters` on it.
  controller_interface::ControllerInterfaceBaseSharedPtr load(
    const std::string & name, const std::string & type,
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto controller = manager_->load_controller(name, type);
    if (!controller) {
      return nullptr;
    }
    const auto node = controller->get_node();
    for (const auto & parameter : parameters) {
      if (!node->has_parameter(parameter.get_name())) {
        node->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
      } else if (!node->set_parameter(parameter).successful) {
        return nullptr;
      }
    }
    return controller;
  }

  bool configure(const std::string & name)
  {
    return manager_->configure_controller(name) == controller_interface::return_type::OK;
  }

  // Humble performs a switch at the end of update(), so periods have to run
  // while the request waits. They run here, one at a time, and stop as soon as
  // the request returns: the next period is the first of the new state, and it
  // is the test's to run with cycle(). The executor is NOT spun meanwhile, so
  // no action-server timer runs during a switch.
  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate)
  {
    auto request = std::async(std::launch::async, [&]() {
        return manager_->switch_controller(activate, deactivate, Strictness::STRICT);
      });
    for (int i = 0; i < 1000; ++i) {
      if (request.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
        break;
      }
      step();
    }
    return request.get() == controller_interface::return_type::OK;
  }

  // `count` control periods; the executor is spun after each, so timers and
  // action callbacks keep up with the control loop as they would in a bringup.
  void cycle(const int count, const bool spin = true)
  {
    for (int i = 0; i < count; ++i) {
      step();
      if (spin) {
        executor_->spin_some(std::chrono::milliseconds(0));
      }
    }
  }

  // Callbacks only, no control period; true once `future` is ready.
  template<typename FutureT>
  bool spin_until(FutureT & future, const int budget_ms = 3000)
  {
    for (int i = 0; i < budget_ms; ++i) {
      executor_->spin_some(std::chrono::milliseconds(0));
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  // A state interface's value, e.g. "fr3_joint1/effort". With GenericSystem
  // that is the last command written to the interface of the same name.
  double state(const std::string & name)
  {
    return resources_->claim_state_interface(name).get_value();
  }

  std::vector<double> states(const std::vector<std::string> & joints, const std::string & interface)
  {
    std::vector<double> values;
    for (const auto & joint : joints) {
      values.push_back(state(joint + "/" + interface));
    }
    return values;
  }

  rclcpp::Node::SharedPtr client_node() const {return client_node_;}

  // The time the next period will be stamped with, minus one period: the time
  // the controllers saw on the last one. What a bridge echoing the controller's
  // clock would stamp an observation with.
  rclcpp::Time now() const {return rclcpp::Time(clock_ns_, RCL_ROS_TIME);}

private:
  // One period on the harness's own clock, which advances exactly kPeriod per
  // period: the periods run far faster than real time, and a controller that
  // times its trajectory from update()'s `time` would otherwise see almost
  // none of it pass.
  void step()
  {
    const auto period = rclcpp::Duration::from_seconds(kPeriod);
    clock_ns_ += period.nanoseconds();
    const rclcpp::Time now(clock_ns_, RCL_ROS_TIME);
    manager_->read(now, period);
    manager_->update(now, period);
    manager_->write(now, period);
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  hardware_interface::ResourceManager * resources_{nullptr};
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  rclcpp::Node::SharedPtr client_node_;
  std::int64_t clock_ns_{0};
};

inline double max_abs_difference(const std::vector<double> & a, const std::vector<double> & b)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

}  // namespace cho_controller_base::testing
