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

// GripperOutcome, the executor -> update() handoff of a franka_gripper result:
// an outcome completes a goal only if it answers that goal's command.
#include <gtest/gtest.h>

#include "cho_controller_franka/gripper_outcome.hpp"

namespace
{
using cho_controller::franka::GripperOutcome;

TEST(GripperOutcome, AnOutcomeIsTakenForItsOwnCommand) {
  GripperOutcome outcome;
  bool success = false;
  EXPECT_FALSE(outcome.take(1, success)) << "nothing was posted";
  outcome.post(1, true);
  ASSERT_TRUE(outcome.take(1, success));
  EXPECT_TRUE(success);
  EXPECT_FALSE(outcome.take(1, success)) << "an outcome is taken once";
  outcome.post(2, false);
  ASSERT_TRUE(outcome.take(2, success));
  EXPECT_FALSE(success);
}

// The race the old pair of flags lost: the executor checked that command 1 was
// the newest, update() then moved on to command 2, and the executor stored
// command 1's result -- which completed command 2's goal.
TEST(GripperOutcome, AnOutcomeForAnEarlierCommandNeverCompletesTheCurrentOne) {
  GripperOutcome outcome;
  outcome.post(1, true);  // checked against command 1 just before update() moved on
  bool success = false;
  EXPECT_FALSE(outcome.take(2, success)) << "command 1's result completed command 2";
  // And it is gone: command 2's own outcome is the next thing taken.
  EXPECT_FALSE(outcome.take(2, success));
  outcome.post(2, false);
  ASSERT_TRUE(outcome.take(2, success));
  EXPECT_FALSE(success);
}

TEST(GripperOutcome, ClearDropsAPendingOutcome) {
  GripperOutcome outcome;
  outcome.post(3, true);
  outcome.clear();
  bool success = false;
  EXPECT_FALSE(outcome.take(3, success));
}

}  // namespace
