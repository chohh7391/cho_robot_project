//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include "cho_controller_common/contact/contact_base.hpp"

namespace cho_controller {
namespace common {
namespace contact {

ContactBase::ContactBase(const std::string & name, RobotWrapper & robot)
: m_name(name), m_robot(robot) {}

const std::string & ContactBase::name() const
{
  return m_name;
}

void ContactBase::name(const std::string & name)
{
  m_name = name;
}

} // namespace contact
} // namespace common
} // namespace cho_controller