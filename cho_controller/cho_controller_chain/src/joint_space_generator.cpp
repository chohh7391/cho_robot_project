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

#include "cho_controller_chain/joint_space_generator.hpp"

#include <exception>
#include <memory>

#include "cho_controller_common/trajectory/motion_limits_params.hpp"

namespace cho_controller_chain
{

JointSpaceGenerator::CallbackReturn JointSpaceGenerator::on_init()
{
  try {
    // Declares and range-checks every parameter in joint_space_generator_parameters.yaml.
    param_listener_ = std::make_shared<chain_joint_space_generator::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

JointSpaceGenerator::CallbackReturn JointSpaceGenerator::on_configure(const rclcpp_lifecycle::State &)
{
  const auto params = param_listener_->get_params();
  if (!configure_chain(params.joints, params.law_name)) {
    return CallbackReturn::FAILURE;
  }
  server_state_.q.setZero(n_);
  server_state_.start.setZero(n_);

  server_ = std::make_shared<ChainJointSpaceServer>(get_node(), "~/joint_space", static_cast<int>(n_));
  server_->init();
  server_->trajectory_->setLimits(cho_controller::common::trajectory::load_joint_motion_limits(get_node(), joints_));
  server_->set_joint_names(joints_);
  server_->set_joint_limits(q_lower_, q_upper_);
  server_->attach_activity(&activity_);
  RCLCPP_INFO(
    get_node()->get_logger(), "configured on %ld joints, writing %s's references", static_cast<long>(n_),
    law_name_.c_str());
  return CallbackReturn::SUCCESS;
}

void JointSpaceGenerator::compute_reference(const rclcpp::Time & clock, double)
{
  server_state_.q = q_;
  server_state_.start = q_ref_;
  // False with no goal, or one that ended this cycle (canceled, aborted, or one that outlived a
  // deactivation): the hold stays.
  if (!server_->compute(clock, server_state_)) {
    return;
  }
  const auto & sample = server_->trajectory_->computeNext();
  if (sample.pos.size() != n_ || sample.vel.size() != n_ || sample.acc.size() != n_) {
    return;
  }
  q_ref_ = sample.pos;
  v_ref_ = sample.vel;
  a_ref_ = sample.acc;
}

}  // namespace cho_controller_chain

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller_chain::JointSpaceGenerator, controller_interface::ControllerInterface)
