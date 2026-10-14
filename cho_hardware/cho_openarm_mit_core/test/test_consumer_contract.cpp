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

// The consumers' API and the one hold-torque validity rule.
//
// AC3: no consumer can be built or fed without saying how its SAFE hold is
// bounded in motion. An ArmConsumer or PairedConsumer constructor that takes no
// band and motion velocity does not exist, and neither does an observe() that
// records no velocity (it would record "at rest"). The checks are runtime
// EXPECT_* on type traits, so a mismatch is a red test, not a broken build.
//
// AC5: a commit's hold_torque is valid, on every consumer, iff each joint's h is
// NaN or finite with |h| <= that joint's hold effort limit (ArmConsumer: the
// hold_effort_limit it was built with; Limiter: the profile's tau_ff_max). Anything
// else, an infinity included, rejects the whole commit and changes nothing.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

#include "cho_openarm_mit_core/mit_limiter.hpp"
#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace
{
using cho_openarm_mit_core::ArmCommand;
using cho_openarm_mit_core::ArmConsumer;
using cho_openarm_mit_core::kJointsPerArm;
using cho_openarm_mit_core::Limiter;
using cho_openarm_mit_core::MitStatus;
using cho_openarm_mit_core::PairedConsumer;
using cho_openarm_mit_core::SafetyBackend;
using cho_openarm_mit_core::ValidationLimits;
using A = std::array<double, kJointsPerArm>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr ValidationLimits kLimits{3.5, 10.0, 200.0, 10.0, 50.0, 100};

A filled(double value)
{
  A a{};
  a.fill(value);
  return a;
}

// -- detection idiom for observe() ---------------------------------------------

template<class Consumer, class Args, class = void>
struct CanObserve : std::false_type {};

template<class Consumer, class... Args>
struct CanObserve<
  Consumer, std::tuple<Args...>,
  std::void_t<decltype(std::declval<Consumer &>().observe(std::declval<Args>()...))>>
  : std::true_type {};

using Arr = const A &;

// -- AC3 -------------------------------------------------------------------------

TEST(ConsumerApi, AC3_ArmConsumerHasOnlyTheConstructorThatTakesBandAndVelocity)
{
  // The one that remains: limits, damping, stiffness, effort limit, band, velocity.
  EXPECT_TRUE((std::is_constructible_v<ArmConsumer, ValidationLimits, A, A, A, A, double>));
  // Removed: everything that leaves the band and the motion velocity implicit.
  EXPECT_FALSE((std::is_constructible_v<ArmConsumer, ValidationLimits>));
  EXPECT_FALSE((std::is_constructible_v<ArmConsumer, ValidationLimits, double, double>));
  EXPECT_FALSE((std::is_constructible_v<ArmConsumer, ValidationLimits, A, A>));
  EXPECT_FALSE((std::is_constructible_v<ArmConsumer, ValidationLimits, A, A, A>));
  // A band without a velocity, or a velocity without a band, is no way around it either.
  EXPECT_FALSE((std::is_constructible_v<ArmConsumer, ValidationLimits, A, A, A, A>));
  EXPECT_FALSE((std::is_constructible_v<ArmConsumer, ValidationLimits, A, A, A, double>));
  // No default constructor.
  EXPECT_FALSE(std::is_default_constructible_v<ArmConsumer>);
}

TEST(ConsumerApi, AC3_PairedConsumerHasOnlyTheConstructorThatTakesBandAndVelocity)
{
  EXPECT_TRUE((std::is_constructible_v<PairedConsumer, ValidationLimits, std::uint64_t, double, A, double>));
  EXPECT_FALSE((std::is_constructible_v<PairedConsumer, ValidationLimits, std::uint64_t, double>));
  EXPECT_FALSE((std::is_constructible_v<PairedConsumer, ValidationLimits, std::uint64_t>));
  EXPECT_FALSE((std::is_constructible_v<PairedConsumer, ValidationLimits, std::uint64_t, double, A>));
  EXPECT_FALSE(std::is_default_constructible_v<PairedConsumer>);
}

TEST(ConsumerApi, AC3_ArmConsumerObserveNeedsTheVelocityToRecordOne)
{
  // Positions only (leaves the velocity and torque samples untouched): kept.
  EXPECT_TRUE((CanObserve<ArmConsumer, std::tuple<Arr>>::value));
  // Positions, velocities and torques: kept.
  EXPECT_TRUE((CanObserve<ArmConsumer, std::tuple<Arr, Arr, Arr>>::value));
  // Positions and torques, which recorded the velocity as 0 (at rest): removed.
  EXPECT_FALSE((CanObserve<ArmConsumer, std::tuple<Arr, Arr>>::value));
}

TEST(ConsumerApi, AC3_PairedConsumerObserveNeedsTheVelocitiesToRecordThem)
{
  EXPECT_TRUE((CanObserve<PairedConsumer, std::tuple<Arr, Arr>>::value));
  EXPECT_TRUE((CanObserve<PairedConsumer, std::tuple<Arr, Arr, Arr, Arr, Arr, Arr>>::value));
  // lq, rq, ltau, rtau: removed.
  EXPECT_FALSE((CanObserve<PairedConsumer, std::tuple<Arr, Arr, Arr, Arr>>::value));
}

TEST(ConsumerApi, AC3_TheDetectionIdiomSeesWhatIsThere)
{
  // The negative checks above are only worth something if the idiom says yes to a
  // call that exists and no to one that cannot exist.
  EXPECT_TRUE((CanObserve<ArmConsumer, std::tuple<A>>::value));
  EXPECT_FALSE((CanObserve<ArmConsumer, std::tuple<>>::value));
  EXPECT_FALSE((CanObserve<ArmConsumer, std::tuple<Arr, Arr, Arr, Arr>>::value));
}

TEST(ConsumerApi, AC3_AnExplicitInfiniteBoundIsStillLegal)
{
  // "Do not bound" is something the caller says: +inf band and +inf velocity build.
  EXPECT_NO_THROW(ArmConsumer(kLimits, filled(1.0), filled(0.0), filled(40.0), filled(kInf), kInf));
  EXPECT_NO_THROW(PairedConsumer(kLimits, 1, 1.0, filled(kInf), kInf));
  // And a bound that is not one is refused (a velocity of 0, a negative or NaN band).
  EXPECT_THROW(
    ArmConsumer(kLimits, filled(1.0), filled(0.0), filled(40.0), filled(0.5), 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    ArmConsumer(kLimits, filled(1.0), filled(0.0), filled(40.0), filled(-0.5), 0.1),
    std::invalid_argument);
  EXPECT_THROW(
    ArmConsumer(kLimits, filled(1.0), filled(0.0), filled(40.0), filled(kNaN), 0.1),
    std::invalid_argument);
  EXPECT_THROW(
    ArmConsumer(kLimits, filled(1.0), filled(0.0), filled(40.0), filled(0.5), kNaN),
    std::invalid_argument);
}

TEST(ConsumerApi, AC3_APositionsOnlyObserveLeavesTheVelocitySampleAlone)
{
  // observe(q) keeps the last velocity: a moving read, then a positions-only read,
  // still latches in motion. observe(q, dq, tau) with dq = 0 is what means "at rest".
  const A hold = filled(2.0);
  auto build = []() {
      return ArmConsumer(kLimits, filled(2.0), filled(3.0), filled(40.0), filled(0.5), 0.1);
    };
  auto commit = [&hold](std::uint64_t generation) {
      ArmCommand c;
      for (auto & joint : c.joints) {
        joint = {0.4, 0.0, 20.0, 1.0, 0.5};
      }
      c.session_echo = 1.0;
      c.lease_cycles = 10.0;
      c.generation = static_cast<double>(generation);
      c.hold_torque = hold;
      return c;
    };
  A moving{};
  moving[3] = 0.5;
  {
    auto consumer = build();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(commit(1)));
    consumer.observe(filled(0.35), moving, filled(5.5));  // moving, h = 2 -> bound to 2.5
    consumer.observe(filled(0.34));                       // positions only
    consumer.request_safe_transition(true);
    ASSERT_TRUE(consumer.submit_safe_transition(true));
    for (const auto & joint : consumer.submitted().joints) {
      EXPECT_DOUBLE_EQ(joint.effort, 2.5);
    }
  }
  {
    auto consumer = build();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(commit(1)));
    consumer.observe(filled(0.35), moving, filled(5.5));
    consumer.observe(filled(0.34), A{}, filled(5.5));  // an explicit read at rest
    consumer.request_safe_transition(true);
    ASSERT_TRUE(consumer.submit_safe_transition(true));
    for (const auto & joint : consumer.submitted().joints) {
      EXPECT_DOUBLE_EQ(joint.effort, 5.5);
    }
  }
}

// -- AC5: ArmConsumer ------------------------------------------------------------

// Hold effort limit per joint, deliberately below limits.max_abs_effort (50) and
// different on every joint.
constexpr A kEffortLimit{40.0, 30.0, 20.0, 10.0, 5.0, 4.0, 3.0};

ArmConsumer limited_consumer()
{
  return ArmConsumer(kLimits, filled(2.0), filled(3.0), kEffortLimit, filled(0.5), 0.1);
}

ArmCommand commit(std::uint64_t generation, const A & hold)
{
  ArmCommand c;
  for (auto & joint : c.joints) {
    joint = {0.4, 0.0, 20.0, 1.0, 0.5};
  }
  c.session_echo = 1.0;
  c.lease_cycles = 10.0;
  c.generation = static_cast<double>(generation);
  c.hold_torque = hold;
  return c;
}

TEST(HoldTorqueValidity, AC5_ArmConsumerRejectsAnEstimateOverTheJointsHoldEffortLimit)
{
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    // Over the limit, and no more than max_abs_effort: the old scalar bound accepted these.
    ASSERT_LT(kEffortLimit[j] + 0.5, kLimits.max_abs_effort);
    for (const double sign : {1.0, -1.0}) {
      auto consumer = limited_consumer();
      ASSERT_TRUE(consumer.configure(1, filled(0.0)));
      const A good = filled(1.0);
      ASSERT_TRUE(consumer.accept_and_write(commit(1, good)));
      ASSERT_EQ(consumer.status(), MitStatus::ACTIVE);
      ASSERT_EQ(consumer.ack_generation(), 1u);

      A bad = good;
      bad[j] = sign * (kEffortLimit[j] + 0.5);
      EXPECT_FALSE(consumer.accept_and_write(commit(2, bad))) << "joint " << j << " sign " << sign;
      EXPECT_NE(consumer.status(), MitStatus::ACTIVE) << "joint " << j;
      // The accepted state is unchanged: no new acknowledgement, the estimate kept.
      EXPECT_EQ(consumer.ack_generation(), 1u) << "joint " << j;
      for (std::size_t i = 0; i < kJointsPerArm; ++i) {
        EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[i], good[i]) << "joint " << i;
      }
    }
  }
}

TEST(HoldTorqueValidity, AC5_ArmConsumerAcceptsAnEstimateAtTheLimitAndNaN)
{
  auto consumer = limited_consumer();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  A hold{};
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    hold[j] = (j % 2 == 0) ? kEffortLimit[j] : -kEffortLimit[j];
  }
  EXPECT_TRUE(consumer.accept_and_write(commit(1, hold)));
  EXPECT_EQ(consumer.status(), MitStatus::ACTIVE);
  EXPECT_EQ(consumer.ack_generation(), 1u);
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[j], hold[j]) << j;
  }
  // NaN on every joint: "no estimate", valid.
  EXPECT_TRUE(consumer.accept_and_write(commit(2, filled(kNaN))));
  EXPECT_EQ(consumer.status(), MitStatus::ACTIVE);
  EXPECT_EQ(consumer.ack_generation(), 2u);
  // NaN on one joint beside a valid estimate on the others.
  A mixed = filled(1.0);
  mixed[4] = kNaN;
  EXPECT_TRUE(consumer.accept_and_write(commit(3, mixed)));
  EXPECT_EQ(consumer.status(), MitStatus::ACTIVE);
}

TEST(HoldTorqueValidity, AC5_ArmConsumerRejectsInfinitiesAndOnlyJustOverTheLimit)
{
  for (const double bad : {kInf, -kInf}) {
    auto consumer = limited_consumer();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    A hold = filled(1.0);
    hold[0] = bad;
    EXPECT_FALSE(consumer.accept_and_write(commit(1, hold))) << bad;
    EXPECT_NE(consumer.status(), MitStatus::ACTIVE) << bad;
    EXPECT_EQ(consumer.ack_generation(), 0u) << bad;
  }
  // The smallest step over the wrist limit (3 N m) is already invalid.
  auto consumer = limited_consumer();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  A hold = filled(1.0);
  hold[6] = std::nextafter(kEffortLimit[6], kInf);
  EXPECT_FALSE(consumer.accept_and_write(commit(1, hold)));
}

TEST(HoldTorqueValidity, AC5_ArmConsumerLimitIsPerJointNotTheScalarMaximum)
{
  // 5 N m is valid on joint 0 (limit 40) and over joint 5 (limit 4) and 6 (limit 3).
  {
    auto consumer = limited_consumer();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    A hold = filled(1.0);
    hold[0] = 5.0;
    EXPECT_TRUE(consumer.accept_and_write(commit(1, hold)));
  }
  for (const std::size_t j : {std::size_t{5}, std::size_t{6}}) {
    auto consumer = limited_consumer();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    A hold = filled(1.0);
    hold[j] = 5.0;
    EXPECT_FALSE(consumer.accept_and_write(commit(1, hold))) << j;
  }
}

// The tuple's effort is bounded by the same per-joint array as the hold torque: the
// joint's tau_ff bound, below limits.max_abs_effort (which validate_tuple() checks).

TEST(TupleEffortLimit, ArmConsumerRejectsATupleEffortOverTheJointsLimit)
{
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    // Over the limit, and under max_abs_effort: validate_tuple() alone accepts these.
    ASSERT_LT(kEffortLimit[j] + 0.5, kLimits.max_abs_effort);
    for (const double sign : {1.0, -1.0}) {
      auto consumer = limited_consumer();
      ASSERT_TRUE(consumer.configure(1, filled(0.0)));
      ASSERT_TRUE(consumer.accept_and_write(commit(1, filled(kNaN))));

      auto bad = commit(2, filled(kNaN));
      bad.joints[j].effort = sign * (kEffortLimit[j] + 0.5);
      EXPECT_FALSE(consumer.accept_and_write(bad)) << "joint " << j << " sign " << sign;
      EXPECT_NE(consumer.status(), MitStatus::ACTIVE) << "joint " << j;
      // The accepted state is unchanged: no new acknowledgement, the old tuple kept.
      EXPECT_EQ(consumer.ack_generation(), 1u) << "joint " << j;
      EXPECT_DOUBLE_EQ(consumer.submitted().joints[j].effort, 0.5) << "joint " << j;
    }
  }
}

TEST(TupleEffortLimit, ArmConsumerAcceptsATupleEffortAtTheJointsLimit)
{
  auto consumer = limited_consumer();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  auto at_limit = commit(1, filled(kNaN));
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    at_limit.joints[j].effort = (j % 2 == 0) ? kEffortLimit[j] : -kEffortLimit[j];
  }
  EXPECT_TRUE(consumer.accept_and_write(at_limit));
  EXPECT_EQ(consumer.status(), MitStatus::ACTIVE);
  EXPECT_EQ(consumer.ack_generation(), 1u);
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    EXPECT_DOUBLE_EQ(consumer.submitted().joints[j].effort, at_limit.joints[j].effort) << j;
  }
}

TEST(HoldTorqueValidity, AC5_ARejectedEstimateSendsTheArmToSafeLikeAnyInvalidCommit)
{
  // The outcome of an invalid h is the outcome of any invalid commit (here an
  // out-of-range tau_ff): the same status.
  auto reference = limited_consumer();
  ASSERT_TRUE(reference.configure(1, filled(0.0)));
  auto bad_tau = commit(1, filled(kNaN));
  bad_tau.joints[0].effort = 50.5;
  ASSERT_FALSE(reference.accept_and_write(bad_tau));

  auto consumer = limited_consumer();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  A hold = filled(1.0);
  hold[3] = 10.5;
  ASSERT_FALSE(consumer.accept_and_write(commit(1, hold)));
  EXPECT_EQ(consumer.status(), reference.status());
}

TEST(HoldTorqueValidity, AC5_PairedConsumerChecksBothArmsAgainstTheirEffortLimit)
{
  // PairedConsumer's hold effort limit is limits.max_abs_effort on both arms.
  PairedConsumer pair(kLimits, 1, 1.0, filled(0.5), 0.1);
  ASSERT_TRUE(pair.configure(1, filled(0.0), filled(0.0)));
  A at_limit = filled(kLimits.max_abs_effort);
  at_limit[1] = -kLimits.max_abs_effort;
  at_limit[2] = kNaN;
  EXPECT_TRUE(pair.write_pair(commit(1, at_limit), commit(1, at_limit)));

  for (const bool right : {false, true}) {
    PairedConsumer p(kLimits, 1, 1.0, filled(0.5), 0.1);
    ASSERT_TRUE(p.configure(1, filled(0.0), filled(0.0)));
    A over = filled(1.0);
    over[4] = kLimits.max_abs_effort + 0.5;
    const A ok = filled(1.0);
    EXPECT_FALSE(p.write_pair(commit(1, right ? ok : over), commit(1, right ? over : ok)))
      << (right ? "right" : "left");
  }
}

// -- AC5: Limiter ----------------------------------------------------------------

constexpr std::size_t N = kJointsPerArm;
using Tuple = Limiter::Tuple;

cho_openarm_mit_core::SafetyProfile mujoco_profile()
{
  return cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", SafetyBackend::MUJOCO, "single");
}

std::array<Tuple, N> zero_tuples()
{
  std::array<Tuple, N> c{};
  for (auto & t : c) {
    t = {0.0, 0.0, 0.0, 0.0, 0.0};
  }
  return c;
}

TEST(HoldTorqueValidity, AC5_LimiterRejectsAnEstimateOverTauFfMagnitudeAndAcceptsItAtTheBound)
{
  const auto profile = mujoco_profile();
  for (std::size_t j = 0; j < N; ++j) {
    const double bound = profile.tau_ff_max[j];
    ASSERT_GT(bound, 0.0);
    for (const double sign : {1.0, -1.0}) {
      Limiter limiter(profile, 1000);
      limiter.reset(A{});
      A h = filled(0.0);
      h[j] = sign * (bound + 0.5);
      EXPECT_FALSE(limiter.submit(zero_tuples(), h, 1, 20)) << "joint " << j << " sign " << sign;
      EXPECT_EQ(limiter.ack(), 0u) << "joint " << j;

      // Nothing changed: the same generation is still the next one and succeeds.
      h[j] = sign * bound;
      EXPECT_TRUE(limiter.submit(zero_tuples(), h, 1, 20)) << "joint " << j << " sign " << sign;
      EXPECT_EQ(limiter.ack(), 1u) << "joint " << j;
    }
  }
}

TEST(HoldTorqueValidity, AC5_LimiterAcceptsNaNAndRejectsInfinities)
{
  const auto profile = mujoco_profile();
  {
    Limiter limiter(profile, 1000);
    limiter.reset(A{});
    EXPECT_TRUE(limiter.submit(zero_tuples(), filled(kNaN), 1, 20));
  }
  {
    Limiter limiter(profile, 1000);
    limiter.reset(A{});
    A h = filled(kNaN);
    h[2] = 1.0;
    EXPECT_TRUE(limiter.submit(zero_tuples(), h, 1, 20));
  }
  for (const double bad : {kInf, -kInf}) {
    Limiter limiter(profile, 1000);
    limiter.reset(A{});
    A h = filled(kNaN);
    h[5] = bad;
    EXPECT_FALSE(limiter.submit(zero_tuples(), h, 1, 20)) << bad;
    EXPECT_EQ(limiter.ack(), 0u) << bad;
  }
}

TEST(HoldTorqueValidity, AC5_LimiterAndArmConsumerAgreeWhenTheirLimitsAreTheSame)
{
  // A consumer built with the profile's tau_ff_max as its hold effort limit and the
  // Limiter built from that profile give the same verdict on every estimate.
  const auto profile = mujoco_profile();
  A limit{};
  for (std::size_t j = 0; j < N; ++j) {
    limit[j] = profile.tau_ff_max[j];
  }
  const ValidationLimits limits = kLimits;
  for (std::size_t j = 0; j < N; ++j) {
    for (const double scale : {0.0, 0.99, 1.0, 1.01, -1.0, -1.01}) {
      A h = filled(kNaN);
      h[j] = scale * limit[j];
      ArmConsumer consumer(limits, filled(1.0), filled(0.0), limit, filled(kInf), kInf);
      ASSERT_TRUE(consumer.configure(1, filled(0.0)));
      ArmCommand c;
      for (auto & joint : c.joints) {
        joint = {0.0, 0.0, 0.0, 0.0, 0.0};
      }
      c.session_echo = 1.0;
      c.lease_cycles = 10.0;
      c.generation = 1.0;
      c.hold_torque = h;
      Limiter limiter(profile, 1000);
      limiter.reset(A{});
      EXPECT_EQ(consumer.accept_and_write(c), limiter.submit(zero_tuples(), h, 1, 20))
        << "joint " << j << " scale " << scale;
    }
  }
}

}  // namespace
