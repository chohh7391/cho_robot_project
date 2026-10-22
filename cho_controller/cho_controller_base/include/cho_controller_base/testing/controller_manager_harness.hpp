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
// over ROS -- rather than a hand-called compute(). In this package only its
// own test includes it; a test that does links controller_manager itself.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
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
//
// Each `clocks` entry, "<gpio>/<interface>", adds a gpio with a command and a
// state interface of that name: a robot's own clock, which reads 0 unless
// ControllerManagerHarness::drive_clock() writes it.
inline std::string mock_ros2_control(
  const std::vector<std::string> & arm, const std::vector<double> & initial,
  const std::vector<std::string> & passive = {},
  const std::vector<std::pair<std::string, std::string>> & hardware = {},
  const std::vector<std::string> & clocks = {})
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
  for (const auto & clock : clocks) {
    const auto slash = clock.find('/');
    out << "<gpio name='" << clock.substr(0, slash) << "'><command_interface name='" << clock.substr(slash + 1)
        << "'/><state_interface name='" << clock.substr(slash + 1) << "'/></gpio>";
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
  // How long a load, configure or switch may take, periods included, before it
  // is given up as failed (see run_with_periods()).
  static constexpr std::chrono::seconds kRequestBudget{20};

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
  // Before and after cycle(), as configure().
  controller_interface::ControllerInterfaceBaseSharedPtr load(
    const std::string & name, const std::string & type,
    const std::vector<rclcpp::Parameter> & parameters)
  {
    const auto manager = manager_;
    auto controller = list_request(
      [manager, name, type]() {return manager->load_controller(name, type);},
      controller_interface::ControllerInterfaceBaseSharedPtr());
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

  // Configures `name`; one that is inactive is cleaned up first, by the
  // controller_manager. Before the first period it runs on this thread and runs
  // no period, as it always did. After one it runs periods while it waits, as a
  // switch does (run_with_periods()): an active controller sees them.
  bool configure(const std::string & name)
  {
    const auto manager = manager_;
    return list_request(
      [manager, name]() {return manager->configure_controller(name);},
      controller_interface::return_type::ERROR) == controller_interface::return_type::OK;
  }

  // Humble performs a switch at the end of update(), so periods have to run
  // while the request waits: run_with_periods().
  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate)
  {
    const auto manager = manager_;
    return run_with_periods(
      [manager, activate, deactivate]() {
        return manager->switch_controller(activate, deactivate, Strictness::STRICT);
      },
      controller_interface::return_type::ERROR) == controller_interface::return_type::OK;
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

  // One period stamped `late` seconds after its place on the schedule, as a
  // cycle a busy PC ran late is: the manager hands update() the time it ran, and
  // the measured periods on either side of it stretch and shrink by as much.
  // The period after it is on time again.
  void late_cycle(const double late, const bool spin = true)
  {
    late_ns_ = static_cast<std::int64_t>(std::llround(late * 1e9));
    cycle(1, spin);
  }

  // One period that comes two periods after the last, as when a stalled control
  // loop skips a hardware state: the robot clock (drive_clock()) and the
  // manager's time both move on by two periods, and update() runs once.
  void skipped_cycle(const bool spin = true)
  {
    skip_ = true;
    cycle(1, spin);
  }

  // Writes the hardware's own clock, the periods run so far times kPeriod (two
  // for a skipped_cycle()), into `name` before every read(): a clock gpio of
  // mock_ros2_control(), whose state GenericSystem copies from that command, as
  // a robot that reports its own time does (franka_hardware's robot_time).
  void drive_clock(const std::string & name)
  {
    clock_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      resources_->claim_command_interface(name));
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

  // Called after every period, the ones a request runs while it waits
  // (run_with_periods()) included, so a test that samples the command
  // interfaces through it sees every period, a switch's too. Empty: none.
  void on_period(std::function<void()> observer) {observer_ = std::move(observer);}

  // The time the next period will be stamped with, minus one period: the time
  // the controllers saw on the last one. What a bridge echoing the controller's
  // clock would stamp an observation with.
  rclcpp::Time now() const {return rclcpp::Time(clock_ns_, RCL_ROS_TIME);}

  // The hardware clock's periods so far (drive_clock()): one per period, two
  // for a skipped_cycle().
  std::int64_t ticks() const {return ticks_;}

private:
  // A load or configure. Humble's controller_manager ends either by handing
  // update() a new controller list and waiting until update() has taken it
  // (switch_updated_list()); until update() has first run there is nothing to
  // wait for. Here update() runs only in step(), on this thread, so after the
  // first period a direct call never returns: it goes through
  // run_with_periods(). Before, it runs here, with no period, as it always did.
  template<typename Request>
  std::invoke_result_t<Request &> list_request(
    Request request, std::invoke_result_t<Request &> failed)
  {
    if (!stepped_) {
      return request();
    }
    return run_with_periods(std::move(request), std::move(failed));
  }

  // `request`, a controller_manager call, on a thread of its own while periods
  // run here, one at a time, 20 ms apart, and stop as soon as it returns: the
  // next period is the first of the new state, and it is the test's to run with
  // cycle(). The executor is NOT spun meanwhile, so no action-server timer runs
  // during a request. Its exception is rethrown here. One still running after
  // kRequestBudget returns `failed` and is left running on its own thread (a
  // std::async future would wait for it on destruction), holding the manager.
  template<typename Request>
  std::invoke_result_t<Request &> run_with_periods(
    Request request, std::invoke_result_t<Request &> failed)
  {
    using Result = std::invoke_result_t<Request &>;
    auto promise = std::make_shared<std::promise<Result>>();
    auto result = promise->get_future();
    std::thread(
      [promise, request = std::make_unique<Request>(std::move(request))]() mutable {
        // The request, and the manager it holds, goes before the result is
        // handed over: once the caller has the result this thread owns no
        // manager, and the harness destroys its manager on its own thread.
        try {
          Result value = (*request)();
          request.reset();
          promise->set_value(std::move(value));
        } catch (...) {
          request.reset();
          promise->set_exception(std::current_exception());
        }
      }).detach();
    const auto deadline = std::chrono::steady_clock::now() + kRequestBudget;
    while (result.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_ERROR(
          client_node_->get_logger(), "a controller_manager request did not return within %lld s",
          static_cast<long long>(kRequestBudget.count()));
        return failed;
      }
      step();
    }
    return result.get();
  }

  // One period on the harness's own clock, which advances exactly kPeriod per
  // period: the periods run far faster than real time, and a controller that
  // times its trajectory from update()'s `time` would otherwise see almost
  // none of it pass.
  void step()
  {
    stepped_ = true;
    // skipped_cycle(): two periods since the last one.
    const int periods = skip_ ? 2 : 1;
    skip_ = false;
    const auto scheduled = rclcpp::Duration::from_seconds(periods * kPeriod);
    clock_ns_ += scheduled.nanoseconds();
    ticks_ += periods;
    if (clock_interface_) {
      clock_interface_->set_value(static_cast<double>(ticks_) * kPeriod);
    }
    // late_cycle(): this period runs late_ns_ behind its place, the one before
    // ran previous_late_ns_ behind.
    const rclcpp::Time now(clock_ns_ + late_ns_, RCL_ROS_TIME);
    const rclcpp::Duration period(std::chrono::nanoseconds(scheduled.nanoseconds() + late_ns_ - previous_late_ns_));
    previous_late_ns_ = late_ns_;
    late_ns_ = 0;
    manager_->read(now, period);
    manager_->update(now, period);
    manager_->write(now, period);
    if (observer_) {
      observer_();
    }
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  hardware_interface::ResourceManager * resources_{nullptr};
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  rclcpp::Node::SharedPtr client_node_;
  std::int64_t clock_ns_{0};
  // late_cycle(): how late the next period runs, and how late the last one ran.
  std::int64_t late_ns_{0};
  std::int64_t previous_late_ns_{0};
  // skipped_cycle() and drive_clock(): the next period comes two after the
  // last; the periods run so far; the clock command written before each read().
  bool skip_{false};
  std::int64_t ticks_{0};
  std::unique_ptr<hardware_interface::LoanedCommandInterface> clock_interface_;
  // update() has run: a load or configure now waits for it (list_request()).
  bool stepped_{false};
  std::function<void()> observer_;
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
