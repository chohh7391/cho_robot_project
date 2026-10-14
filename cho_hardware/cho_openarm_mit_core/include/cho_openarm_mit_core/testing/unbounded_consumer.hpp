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

// TEST SUPPORT, header-only: consumers that never bound a SAFE hold in motion,
// stated explicitly (hold band and motion velocity +inf), for a test of
// something else. ArmConsumer, PairedConsumer and BimanualCommandRouter have no
// constructor that leaves the band implicit, and none may grow one here. Each
// helper puts the given damping on every joint, stiffness 0 unless given, and
// the hold's effort limit `limits.max_abs_effort` unless given.
#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_openarm_mit_core::testing
{
using JointValues = std::array<double, kJointsPerArm>;

inline constexpr double kUnbounded = std::numeric_limits<double>::infinity();

inline ArmConsumer unbounded_arm_consumer(
  const ValidationLimits & limits, const JointValues & damping, const JointValues & stiffness,
  const JointValues & effort_limit)
{
  return ArmConsumer(
    limits, damping, stiffness, effort_limit, detail::every_joint(kUnbounded), kUnbounded);
}

inline ArmConsumer unbounded_arm_consumer(
  const ValidationLimits & limits, const JointValues & damping, const JointValues & stiffness)
{
  return unbounded_arm_consumer(
    limits, damping, stiffness, detail::every_joint(limits.max_abs_effort));
}

inline ArmConsumer unbounded_arm_consumer(
  const ValidationLimits & limits, double damping, double stiffness, const JointValues & effort_limit)
{
  return unbounded_arm_consumer(
    limits, detail::every_joint(damping), detail::every_joint(stiffness), effort_limit);
}

inline ArmConsumer unbounded_arm_consumer(
  const ValidationLimits & limits, double damping = 1.0, double stiffness = 0.0)
{
  return unbounded_arm_consumer(
    limits, detail::every_joint(damping), detail::every_joint(stiffness));
}

inline PairedConsumer unbounded_paired_consumer(
  const ValidationLimits & limits, std::uint64_t session, double damping = 1.0)
{
  return PairedConsumer(limits, session, damping, detail::every_joint(kUnbounded), kUnbounded);
}

inline BimanualCommandRouter unbounded_command_router(
  const ValidationLimits & limits, double damping = 1.0)
{
  return BimanualCommandRouter(limits, damping, detail::every_joint(kUnbounded), kUnbounded);
}
}  // namespace cho_openarm_mit_core::testing
