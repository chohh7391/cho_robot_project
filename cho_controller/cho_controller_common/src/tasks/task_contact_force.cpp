//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include "cho_controller_common/tasks/task_contact_force.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

TaskContactForce::TaskContactForce(const std::string & name, cho_controller::common::robot::RobotWrapper & robot)
: TaskBase(name, robot) {}

} // namespace tasks
} // namespace common
} // namespace cho_controller