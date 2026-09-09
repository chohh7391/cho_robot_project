// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cho_vla_core/reference_history.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

void push_ramp(ReferenceHistory & history, const int count, const double dt)
{
  for (int step = 0; step < count; ++step) {
    const double time = static_cast<double>(step) * dt;
    Vector7 joints;
    joints.setConstant(time);
    history.push(
      time,
      SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(time, 0.0, 0.0)),
      joints);
  }
}

TEST(ReferenceHistory, EmptyHistoryHasNoAnchor) {
  ReferenceHistory history(16);
  Anchor anchor;
  EXPECT_FALSE(history.at(1.0, anchor));
  EXPECT_FALSE(history.newest(anchor));
}

TEST(ReferenceHistory, ReturnsTheNewestEntryAtOrBeforeTheRequestedTime) {
  ReferenceHistory history(64);
  push_ramp(history, 20, 0.01);   // 0.00 .. 0.19

  Anchor anchor;
  bool exact = false;
  ASSERT_TRUE(history.at(0.125, anchor, &exact));
  EXPECT_TRUE(exact);
  // Entry at 0.12 is the newest at-or-before 0.125.
  EXPECT_NEAR(anchor.joints(0), 0.12, 1e-9);
  EXPECT_NEAR(anchor.pose.translation()(0), 0.12, 1e-9);

  ASSERT_TRUE(history.at(0.13, anchor, &exact));
  EXPECT_TRUE(exact);
  EXPECT_NEAR(anchor.joints(0), 0.13, 1e-9);
}

TEST(ReferenceHistory, AnchoringAtObservationTimeDiffersFromArrivalTime) {
  // This is the whole reason the ring exists: a chunk observed at 0.05 and
  // arriving at 0.15 must be anchored to the 0.05 state, not the 0.15 one.
  ReferenceHistory history(256);
  push_ramp(history, 20, 0.01);

  Anchor observed;
  Anchor arrived;
  ASSERT_TRUE(history.at(0.05, observed));
  ASSERT_TRUE(history.at(0.15, arrived));
  EXPECT_NEAR(observed.joints(0), 0.05, 1e-9);
  EXPECT_NEAR(arrived.joints(0), 0.15, 1e-9);
  EXPECT_GT(arrived.joints(0) - observed.joints(0), 0.09);
}

TEST(ReferenceHistory, ObservationOlderThanTheRingFallsBackToTheOldestAndSaysSo) {
  ReferenceHistory history(8);
  push_ramp(history, 40, 0.01);   // only the last 8 entries survive

  Anchor anchor;
  bool exact = true;
  ASSERT_TRUE(history.at(0.0, anchor, &exact));
  EXPECT_FALSE(exact);
  // Oldest surviving entry, not the requested one.
  EXPECT_GT(anchor.joints(0), 0.3);
}

TEST(ReferenceHistory, RequestNewerThanTheRingReturnsTheNewestEntry) {
  ReferenceHistory history(16);
  push_ramp(history, 10, 0.01);

  Anchor anchor;
  bool exact = false;
  ASSERT_TRUE(history.at(99.0, anchor, &exact));
  EXPECT_TRUE(exact);
  EXPECT_NEAR(anchor.joints(0), 0.09, 1e-9);
}

TEST(ReferenceHistory, WrapsWithoutLosingOrdering) {
  ReferenceHistory history(4);
  push_ramp(history, 100, 0.01);

  Anchor newest;
  ASSERT_TRUE(history.newest(newest));
  EXPECT_NEAR(newest.joints(0), 0.99, 1e-9);

  Anchor anchor;
  ASSERT_TRUE(history.at(0.98, anchor));
  EXPECT_NEAR(anchor.joints(0), 0.98, 1e-9);
}

TEST(ReferenceHistory, NonFiniteTimeIsIgnored) {
  ReferenceHistory history(8);
  push_ramp(history, 3, 0.01);
  history.push(
    std::numeric_limits<double>::quiet_NaN(), SE3::Identity(), Vector7::Zero());

  Anchor anchor;
  ASSERT_TRUE(history.newest(anchor));
  EXPECT_NEAR(anchor.joints(0), 0.02, 1e-9);
}

TEST(ReferenceHistory, ResetClearsTheRing) {
  ReferenceHistory history(8);
  push_ramp(history, 5, 0.01);
  history.reset();
  Anchor anchor;
  EXPECT_FALSE(history.newest(anchor));
  EXPECT_FALSE(history.at(0.0, anchor));
}

TEST(ReferenceHistory, ConcurrentWriterNeverHandsTheReaderATornPose) {
  // The writer is the control loop and the reader is the executor, so a torn
  // read is the realistic failure: a pose whose rotation came from one cycle and
  // translation from another would anchor a chunk to a frame that never existed.
  ReferenceHistory history(512);
  std::atomic<bool> stop {false};
  std::atomic<int> torn {0};
  std::atomic<int> reads {0};

  std::thread writer([&history, &stop] {
      double time = 0.0;
      while (!stop.load(std::memory_order_relaxed)) {
        time += 0.001;
        // Every entry is self-consistent: translation, rotation and joints all
        // encode the same value, so any mismatch means a torn read. The rotation
        // angle is wrapped into [0, 1) rather than being `time` itself, because
        // `time` grows without bound and a rotation only carries its angle
        // modulo 2*pi -- comparing against the unwrapped value would report every
        // read past pi as torn.
        Vector7 joints;
        joints.setConstant(time);
        history.push(
          time,
          SE3(
            Eigen::AngleAxisd(std::fmod(time, 1.0), Eigen::Vector3d::UnitZ())
            .toRotationMatrix(),
            Eigen::Vector3d(time, 0.0, 0.0)),
          joints);
      }
    });

  // Wait for the writer to publish something before timing the read loop. The
  // loop below is non-blocking and 200k iterations of it take milliseconds, so
  // on a loaded machine it used to finish before the writer thread was ever
  // scheduled -- every newest() returned false and the test failed with zero
  // reads while the library was fine. Observed for real with a simulator running
  // alongside.
  {
    Anchor ready;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!history.newest(ready) && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(history.newest(ready)) << "writer thread never produced an entry";
  }

  // Bounded by time as well as by iterations, so the read count stays meaningful
  // whether this runs on an idle machine or a busy one.
  const auto read_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  for (int attempt = 0; attempt < 200000; ++attempt) {
    if (reads.load(std::memory_order_relaxed) >= 5000 &&
      std::chrono::steady_clock::now() > read_deadline)
    {
      break;
    }
    Anchor anchor;
    if (!history.newest(anchor)) {continue;}
    reads.fetch_add(1, std::memory_order_relaxed);
    const double from_translation = anchor.pose.translation()(0);
    const double from_joints = anchor.joints(0);
    // Compare cos/sin of the wrapped angle directly rather than extracting an
    // AngleAxis, whose axis is undefined at a zero angle.
    const double wrapped = std::fmod(from_translation, 1.0);
    const double cos_error = std::abs(anchor.pose.rotation()(0, 0) - std::cos(wrapped));
    const double sin_error = std::abs(anchor.pose.rotation()(1, 0) - std::sin(wrapped));
    if (std::abs(from_translation - from_joints) > 1e-9 ||
      cos_error > 1e-9 || sin_error > 1e-9)
    {
      torn.fetch_add(1, std::memory_order_relaxed);
    }
  }

  stop.store(true, std::memory_order_relaxed);
  writer.join();

  EXPECT_GT(reads.load(), 1000);
  EXPECT_EQ(torn.load(), 0);
}
}  // namespace
