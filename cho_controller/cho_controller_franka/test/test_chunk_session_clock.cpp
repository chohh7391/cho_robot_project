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

// ChunkSessionClock alone: the clock ActionChunkController steps its chunk session on.
// Expected values from the header's rule and hand arithmetic.

#include <cstdint>

#include <gtest/gtest.h>
#include <rclcpp/time.hpp>

#include "cho_controller_franka/chunk_session_clock.hpp"

namespace
{
using cho_controller::franka::ChunkSessionClock;

constexpr double kDt = 0.001;
constexpr std::int64_t kStep = 1'000'000;
// Seconds since the epoch, as on the robot PC: where a double of seconds would
// round every 1 ms step.
constexpr std::int64_t kStart = 1'792'674'935'000'000'000;

rclcpp::Time at(const std::int64_t nanoseconds) {return rclcpp::Time(nanoseconds, RCL_ROS_TIME);}

TEST(ChunkSessionClock, StartsAtTheManagersTimeAndStepsByThePeriod)
{
  ChunkSessionClock clock;
  std::int64_t tick = kStart + 123;
  EXPECT_EQ(clock.advance(at(tick), kDt).nanoseconds(), tick);
  // A minute on schedule: exactly one period a cycle, nothing gained or lost.
  for (int i = 0; i < 60000; ++i) {
    tick += kStep;
    ASSERT_EQ(clock.advance(at(tick), kDt).nanoseconds(), tick) << "cycle " << i;
  }
}

// The FR3's case: a cycle 0.95 ms late, then one on time.
TEST(ChunkSessionClock, ALateCycleIsTimedAtItsTick)
{
  ChunkSessionClock clock;
  std::int64_t tick = kStart;
  for (int i = 0; i < 10; ++i, tick += kStep) {
    clock.advance(at(tick), kDt);
  }
  // Pulled by dt / kPullSeconds of the 0.95 ms: 950 ns.
  const std::int64_t late = clock.advance(at(tick + 950'000), kDt).nanoseconds();
  EXPECT_EQ(late - tick, 950);
  tick += kStep;
  // And by a thousandth of what is left, 49 ns, on the cycle after it.
  const std::int64_t next = clock.advance(at(tick + 50'000), kDt).nanoseconds();
  EXPECT_EQ(next - late, kStep + 49);
}

// A manager whose period is 100 ns long (the PC's clock 100 ppm ahead of the
// hardware's tick): followed about drift * kPullSeconds = 100 us behind.
TEST(ChunkSessionClock, FollowsADriftWithinItsPull)
{
  ChunkSessionClock clock;
  std::int64_t manager = kStart;
  std::int64_t last = clock.advance(at(manager), kDt).nanoseconds();
  for (int i = 0; i < 20000; ++i) {
    manager += kStep + 100;
    last = clock.advance(at(manager), kDt).nanoseconds();
  }
  EXPECT_NEAR(static_cast<double>(manager - last), 100'000.0, 2'000.0);
}

// Farther than the step the session's limiter would integrate, it starts again
// from the manager's time; nearer, it is pulled.
TEST(ChunkSessionClock, StartsAgainOnAJumpTheLimiterWouldNotIntegrate)
{
  ChunkSessionClock clock;
  clock.advance(at(kStart), kDt);
  const std::int64_t ahead = kStart + kStep + 200'000'000;
  EXPECT_EQ(clock.advance(at(ahead), kDt).nanoseconds(), ahead);
  const std::int64_t back = ahead - 5'000'000'000;
  EXPECT_EQ(clock.advance(at(back), kDt).nanoseconds(), back);
  EXPECT_EQ(clock.advance(at(back + kStep + 50'000'000), kDt).nanoseconds(), back + kStep + 50'000);
}

TEST(ChunkSessionClock, AResetStartsAtTheNextTime)
{
  ChunkSessionClock clock;
  clock.advance(at(kStart), kDt);
  clock.advance(at(kStart + kStep), kDt);
  clock.reset();
  EXPECT_EQ(clock.advance(at(kStart + 7 * kStep + 3), kDt).nanoseconds(), kStart + 7 * kStep + 3);
}
}  // namespace
