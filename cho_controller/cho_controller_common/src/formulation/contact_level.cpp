//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include "cho_controller_common/formulation/contact_level.hpp"

namespace cho_controller {
namespace common {
namespace formulation {

ContactLevel::ContactLevel(cho_controller::common::contact::ContactBase & contact)
: contact(contact) {}

} // namespace formulation
} // namespace common
} // namespace cho_controller