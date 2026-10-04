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

// Tripwire for the build flags, not for the logic.
//
// -ffinite-math-only (implied by -Ofast)
// folds std::isfinite() to true. Measured on g++ 11.4: an -Ofast build reports a
// NaN-carrying vector as all-finite. Every rejection this library exists to make
// would silently stop happening, with no compile error and no behavioural clue
// until a policy emitted a NaN on hardware.
//
// The CMakeLists' -fno-finite-math-only beats -Ofast whichever comes last, so
// what this test actually guards is that flag still being there. Verified both
// ways: it fails with the line removed under -Ofast, and passes with the line
// present even when -Ofast is added after it. If it fails, fix the flags, not
// the test.
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "cho_vla_core/chunk_validator.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

TEST(FiniteMathGuard, IsfiniteStillRejectsNanAndInfinity) {
  volatile double nan_value = std::numeric_limits<double>::quiet_NaN();
  volatile double inf_value = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(std::isfinite(nan_value));
  EXPECT_FALSE(std::isfinite(inf_value));
}

TEST(FiniteMathGuard, ValidatorRejectsNanThroughTheRealCodePath) {
  Chunk chunk;
  chunk.space = ActionSpace::kJoint;
  chunk.chunk_size = 2;
  chunk.control_dt = 0.02;
  chunk.arm_actions.assign(2 * kJoints, 0.0);
  chunk.arm_actions[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate(chunk, ValidationLimits{}), Reject::kNonFinite);

  chunk.arm_actions[5] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(validate(chunk, ValidationLimits{}), Reject::kNonFinite);
}
}  // namespace
