// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "cho_vla_core/gripper_dispatch.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

TEST(GripperDispatch, BinaryModeEdgeTriggers) {
  GripperDispatch dispatch;
  EXPECT_EQ(dispatch.update(1.0, GripperMode::kBinary), GripperCommand::kOpen);
  EXPECT_EQ(dispatch.update(1.0, GripperMode::kBinary), GripperCommand::kNone);
  EXPECT_EQ(dispatch.update(-1.0, GripperMode::kBinary), GripperCommand::kGrasp);
  EXPECT_EQ(dispatch.update(-0.2, GripperMode::kBinary), GripperCommand::kNone);
  EXPECT_EQ(dispatch.update(0.5, GripperMode::kBinary), GripperCommand::kOpen);
}

TEST(GripperDispatch, BinaryZeroMeansNoChange) {
  GripperDispatch dispatch;
  EXPECT_EQ(dispatch.update(0.0, GripperMode::kBinary), GripperCommand::kNone);
  EXPECT_FALSE(dispatch.initialized());
}

TEST(GripperDispatch, ContinuousModeUsesADeadband) {
  GripperDispatch dispatch;
  EXPECT_EQ(dispatch.update(0.9, GripperMode::kContinuous), GripperCommand::kOpen);
  // Inside the deadband nothing happens, so a policy hovering near the middle
  // does not chatter the gripper.
  EXPECT_EQ(dispatch.update(0.5, GripperMode::kContinuous), GripperCommand::kNone);
  EXPECT_EQ(dispatch.update(0.4, GripperMode::kContinuous), GripperCommand::kNone);
  EXPECT_EQ(dispatch.update(0.1, GripperMode::kContinuous), GripperCommand::kGrasp);
  EXPECT_EQ(dispatch.update(0.5, GripperMode::kContinuous), GripperCommand::kNone);
  EXPECT_EQ(dispatch.update(0.8, GripperMode::kContinuous), GripperCommand::kOpen);
}

TEST(GripperDispatch, RetryUndoesTheLatchSoARefusedRequestFiresAgain) {
  GripperDispatch dispatch;
  ASSERT_EQ(dispatch.update(-1.0, GripperMode::kBinary), GripperCommand::kGrasp);
  // Same value again is normally a no-op.
  ASSERT_EQ(dispatch.update(-1.0, GripperMode::kBinary), GripperCommand::kNone);
  // The gripper server refused the goal (it settles for ~1 s after a result).
  dispatch.retry();
  EXPECT_EQ(dispatch.update(-1.0, GripperMode::kBinary), GripperCommand::kGrasp);
}

TEST(GripperDispatch, NonFiniteValueIsIgnored) {
  GripperDispatch dispatch;
  EXPECT_EQ(
    dispatch.update(std::numeric_limits<double>::quiet_NaN(), GripperMode::kBinary),
    GripperCommand::kNone);
  EXPECT_FALSE(dispatch.initialized());
}

TEST(GripperDispatch, FirstSampleAlwaysCommands) {
  // Nothing is known about the physical gripper at goal start, so the first
  // decisive sample must command rather than assume a matching state.
  GripperDispatch open_first;
  EXPECT_EQ(open_first.update(1.0, GripperMode::kBinary), GripperCommand::kOpen);

  GripperDispatch close_first;
  EXPECT_EQ(close_first.update(-1.0, GripperMode::kBinary), GripperCommand::kGrasp);
}

TEST(GripperDispatch, ResetForgetsTheLatch) {
  GripperDispatch dispatch;
  ASSERT_EQ(dispatch.update(-1.0, GripperMode::kBinary), GripperCommand::kGrasp);
  dispatch.reset();
  EXPECT_FALSE(dispatch.initialized());
  EXPECT_EQ(dispatch.update(-1.0, GripperMode::kBinary), GripperCommand::kGrasp);
}
}  // namespace
