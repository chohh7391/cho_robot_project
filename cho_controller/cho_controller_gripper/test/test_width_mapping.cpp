// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include "cho_controller_gripper/width_mapping.hpp"

using cho_controller_gripper::StallDetector;
using cho_controller_gripper::WidthMapping;
using cho_controller_gripper::grasp_succeeded;

TEST(WidthMapping, OpenArmParallelJawDoublesTheFingerTravel)
{
  // One finger travels 0 to 44 mm while the other mimics it, so the object sees
  // 88 mm of opening. Confusing the two is how a task ends up commanding half
  // the width it asked for.
  WidthMapping mapping{0.0, 0.044, 0.0, 0.088};
  ASSERT_TRUE(mapping.valid());
  EXPECT_DOUBLE_EQ(mapping.to_width(0.0), 0.0);
  EXPECT_DOUBLE_EQ(mapping.to_width(0.044), 0.088);
  EXPECT_DOUBLE_EQ(mapping.to_width(0.022), 0.044);
  EXPECT_DOUBLE_EQ(mapping.to_joint(0.088), 0.044);
  EXPECT_NEAR(mapping.to_joint(mapping.to_width(0.031)), 0.031, 1e-12);
  EXPECT_DOUBLE_EQ(mapping.to_width_rate(0.01), 0.02);
}

TEST(WidthMapping, RevoluteGripperThatClosesAsTheAngleGrowsIsJustReversedEndpoints)
{
  // Robotiq 2F-85: 0 rad is open, 0.7929 rad is closed. No sign flag is needed,
  // the endpoints carry the direction.
  WidthMapping mapping{0.7929, 0.0, 0.0, 0.085};
  ASSERT_TRUE(mapping.valid());
  EXPECT_NEAR(mapping.to_width(0.7929), 0.0, 1e-12);
  EXPECT_NEAR(mapping.to_width(0.0), 0.085, 1e-12);
  EXPECT_NEAR(mapping.to_joint(0.085), 0.0, 1e-12);
  // Opening (width increasing) means the joint angle decreases, so a negative
  // joint rate has to read as a positive width rate.
  EXPECT_GT(mapping.to_width_rate(-0.1), 0.0);
}

TEST(WidthMapping, DegenerateConfigurationsAreRejectedRatherThanDividingByZero)
{
  EXPECT_FALSE((WidthMapping{0.0, 0.0, 0.0, 0.044}).valid());
  EXPECT_FALSE((WidthMapping{0.0, 0.044, 0.044, 0.044}).valid());
  EXPECT_FALSE((WidthMapping{0.0, 0.044, 0.05, 0.0}).valid());
  EXPECT_TRUE((WidthMapping{0.0, 0.044, 0.0, 0.044}).valid());
}

TEST(WidthMapping, ClampKeepsACommandInsideThePhysicalOpening)
{
  WidthMapping mapping{0.0, 0.044, 0.0, 0.088};
  EXPECT_DOUBLE_EQ(mapping.clamp_width(-0.01), 0.0);
  EXPECT_DOUBLE_EQ(mapping.clamp_width(0.5), 0.088);
  EXPECT_DOUBLE_EQ(mapping.clamp_width(0.04), 0.04);
}

TEST(StallDetector, NeedsBothALaggingCommandAndAStillFinger)
{
  StallDetector stall;
  stall.configure(0.003, 0.002, 0.1);
  // Lagging but still moving: a fast command the hardware is chasing.
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(stall.update(0.02, 0.03, 0.01, 0.01));
  }
  stall.reset();
  // Still but not lagging: simply sitting at the commanded width.
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(stall.update(0.030, 0.0301, 0.0, 0.01));
  }
}

TEST(StallDetector, FiresOnlyAfterTheDwellTimeAndResetsOnMotion)
{
  StallDetector stall;
  stall.configure(0.003, 0.002, 0.1);
  // 0.09 s of blocked motion is not yet a stall.
  for (int i = 0; i < 9; ++i) {
    EXPECT_FALSE(stall.update(0.01, 0.02, 0.0, 0.01));
  }
  // It fires on the cycle that crosses the dwell time. Summing 0.01 ten times
  // lands a bit under 0.1 in binary floating point, so allow the crossing to
  // take one extra cycle rather than pinning it to an exact tick.
  bool fired = false;
  for (int i = 0; i < 3 && !fired; ++i) {
    fired = stall.update(0.01, 0.02, 0.0, 0.01);
  }
  EXPECT_TRUE(fired);
  EXPECT_TRUE(stall.stalled());
  // The object slips and the fingers move again: the dwell restarts, so a
  // momentary block cannot latch a grasp.
  EXPECT_FALSE(stall.update(0.01, 0.02, 0.05, 0.01));
  EXPECT_FALSE(stall.stalled());
}

TEST(GraspSucceeded, AcceptsAnObjectInsideTheToleranceBandAndRejectsEmptyClosure)
{
  // A 15 mm marker commanded to 15 mm with Franka's default tolerances.
  EXPECT_TRUE(grasp_succeeded(0.015, 0.015, 0.005, 0.010));
  EXPECT_TRUE(grasp_succeeded(0.012, 0.015, 0.005, 0.010));
  EXPECT_TRUE(grasp_succeeded(0.024, 0.015, 0.005, 0.010));
  // An 8 mm pen slips through the inner tolerance; a 30 mm flashlight stops the
  // fingers too early.
  EXPECT_FALSE(grasp_succeeded(0.008, 0.015, 0.005, 0.010));
  EXPECT_FALSE(grasp_succeeded(0.030, 0.015, 0.005, 0.010));
}
