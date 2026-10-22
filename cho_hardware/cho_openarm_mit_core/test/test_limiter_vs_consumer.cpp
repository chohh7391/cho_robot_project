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

// The MuJoCo limiter against the real consumer on the same tuples. The
// sequence is a direct producer's SEEDING right after activation: q_des the
// seed pose, tau_ff the held torque, the task gains in one step from the
// hold's (kp 3 to 32 on joint 1), with the arm measured kOffset off the seed
// pose. The real consumer (ArmConsumer) passes a commit on as it came and the
// drive evaluates the MIT equation; the limiter slews gains and tau_ff from
// what it applied last. Both run real_conservative_commissioning at its
// 750 Hz, the limiter's copy relabelled MUJOCO (which its constructor needs).
//
// Then the measured SAFE hold both latch after it, at rest and pushed, and
// the bound both put on a hold latched in motion.
//
// Measured per joint, printed and recorded as test properties: when the two
// torques agree for good, and their largest difference before that.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "cho_openarm_mit_core/mit_limiter.hpp"
#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace
{
using cho_openarm_mit_core::ArmCommand;
using cho_openarm_mit_core::ArmConsumer;
using cho_openarm_mit_core::JointTuple;
using cho_openarm_mit_core::kJointsPerArm;
using cho_openarm_mit_core::Limiter;
using cho_openarm_mit_core::SafetyBackend;
using cho_openarm_mit_core::SafetyProfile;
using cho_openarm_mit_core::ValidationLimits;
using Joints = std::array<double, kJointsPerArm>;

constexpr std::size_t kRate = 750;
constexpr double kDt = 1.0 / kRate;
// The drive-side TaskSpace and ActionChunk gains of cho_bringup_openarm's
// config/real/controllers_mit.yaml.
constexpr Joints kKp{32.0, 30.0, 50.0, 49.0, 15.0, 10.0, 5.0};
constexpr Joints kKd{5.0, 5.0, 5.0, 5.0, 0.7, 0.6, 0.5};
constexpr Joints kSeed{0.2, 0.3, 0.1, 1.2, 0.1, 0.2, 0.1};
// The measured error, constant and against the held torques (>= 0 below), so
// the two consumers differ only by the kp they apply: the real one is
// kp * 0.05 above the held torque at once.
constexpr double kOffset = -0.05;
// A held arm's torque (gravity-like); inside every tau_ff_magnitude.
constexpr Joints kHeld{1.0, 8.0, 1.5, 4.0, 0.4, 0.6, 0.1};
constexpr double kAgree = 1e-9;  // N m

struct JointResult
{
  double agree_s{0.0};     // from the phase's first cycle until the torques agree for good
  double expected_s{0.0};  // when the limiter's slew should have finished
  double max_diff{0.0};    // N m, |real - limiter|
  double max_diff_s{0.0};  // when
};
using Results = std::array<JointResult, kJointsPerArm>;

SafetyProfile real_profile()
{
  return cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "real_conservative_commissioning", SafetyBackend::REAL,
    "single");
}

SafetyProfile mujoco_copy(SafetyProfile p)
{
  p.backend = SafetyBackend::MUJOCO;
  return p;
}

// As OpenArmMitRealSystem::on_configure() builds them.
ValidationLimits real_adapter_limits(const SafetyProfile & p)
{
  const auto max_of = [](const Joints & v) { return *std::max_element(v.begin(), v.end()); };
  return {
    std::max(
      std::abs(*std::min_element(p.position_lower.begin(), p.position_lower.end())),
      std::abs(max_of(p.position_upper))),
    max_of(p.command_velocity),
    max_of(p.kp_max),
    max_of(p.kd_max),
    max_of(p.final_torque),
    p.lease_cap};
}

// The real adapter's own per-joint gate (window, velocity, gains): a commit outside
// it is rejected, not sent. The tuple's tau_ff against tau_ff_max is the consumer's
// check (accept_and_write), made right after this one.
bool real_adapter_accepts(const SafetyProfile & p, const ArmCommand & c)
{
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    const auto & t = c.joints[j];
    if (
      t.position < p.position_lower[j] || t.position > p.position_upper[j] ||
      std::abs(t.velocity) > p.command_velocity[j] || t.stiffness > p.kp_max[j] ||
      t.damping > p.kd_max[j])
      return false;
  }
  return true;
}

// What the drive applies for a tuple, in the limiter's order of terms.
double mit_torque(const JointTuple & t, double q, double dq)
{
  return t.stiffness * (t.position - q) + t.damping * (t.velocity - dq) + t.effort;
}

std::array<Limiter::Tuple, kJointsPerArm> limiter_tuples(const ArmCommand & c)
{
  std::array<Limiter::Tuple, kJointsPerArm> out{};
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    const auto & t = c.joints[j];
    out[j] = {t.position, t.velocity, t.stiffness, t.damping, t.effort};
  }
  return out;
}

Joints offset_from(const Joints & pose, double offset)
{
  Joints q{};
  for (std::size_t j = 0; j < kJointsPerArm; ++j) q[j] = pose[j] + offset;
  return q;
}

// Both consumers on one arm. The real side is the real adapter's cycle around
// ArmConsumer: read() observes the pose and the torque the motor applied in
// the previous cycle, write() accepts the commit or latches the SAFE hold and
// sends submitted(), whose MIT torque the motor applies. The limiter side is
// the MuJoCo backend's submit() and update().
class Bench
{
public:
  Bench()
  : real_(real_profile()),
    sim_(mujoco_copy(real_)),
    limiter_(sim_, kRate),
    consumer_(
      real_adapter_limits(real_), real_.safe_damping, real_.safe_stiffness, real_.tau_ff_max,
      real_.safe_hold_torque_band, real_.safe_hold_motion_velocity)
  {
  }
  const SafetyProfile & profile() const { return real_; }

  // The limiter applying `held` at the seed pose, then its SAFE hold latched
  // there: an INACTIVE hold, before a reactivation.
  void hold_at_seed(const Joints & held)
  {
    limiter_.reset(kSeed);
    std::array<Limiter::Tuple, kJointsPerArm> c{};
    for (std::size_t j = 0; j < kJointsPerArm; ++j)
      c[j] = {kSeed[j], 0.0, real_.safe_stiffness[j], real_.safe_damping[j], held[j]};
    const Joints dq{};
    const ArmCommand no_estimate;  // its hold_torque: none
    for (std::uint64_t k = 1; k <= 2 * kRate; ++k) {
      ASSERT_TRUE(limiter_.submit(c, no_estimate.hold_torque, k, real_.lease_default));
      (void)limiter_.update(kSeed, dq, kDt);
    }
    limiter_.request_safe();
    (void)limiter_.update(kSeed, dq, kDt);
  }

  // The activation: each backend's first hold at the seed pose, with the
  // torque it was applying (the limiter) or measured at the seed read (real).
  // Returns the producer's SEEDING commit, its tau_ff the published hold.
  ArmCommand activate(const Joints & held)
  {
    limiter_.reset(kSeed);
    EXPECT_TRUE(consumer_.configure(1, kSeed, held));
    applied_ = held;
    ArmCommand c;
    c.session_echo = 1;
    c.lease_cycles = static_cast<double>(real_.lease_default);
    for (std::size_t j = 0; j < kJointsPerArm; ++j) {
      EXPECT_NEAR(limiter_.held_effort(j), held[j], 1e-12) << j;
      EXPECT_NEAR(consumer_.hold_effort()[j], held[j], 1e-12) << j;
      c.joints[j] = {kSeed[j], 0.0, kKp[j], kKd[j], held[j]};
    }
    return c;
  }

  // One control cycle at the measured pose `q` (dq = 0). `commit`: a new
  // generation of it, as a direct producer commits every cycle; null: none.
  void step(const Joints & q, const ArmCommand * commit, Joints & tau_real, Joints & tau_sim)
  {
    const Joints dq{};
    consumer_.observe(q, dq, applied_);
    if (safe_requested_) {
      consumer_.request_safe_transition(true);
      ASSERT_TRUE(consumer_.submit_safe_transition(true));
      limiter_.request_safe();
      safe_requested_ = false;
    }
    if (commit != nullptr) {
      ArmCommand c = *commit;
      c.generation = static_cast<double>(++generation_);
      ASSERT_TRUE(real_adapter_accepts(real_, c));
      ASSERT_TRUE(consumer_.accept_and_write(c));
      ASSERT_TRUE(limiter_.submit(
        limiter_tuples(c), c.hold_torque, generation_, static_cast<std::uint64_t>(c.lease_cycles)));
    }
    tau_sim = limiter_.update(q, dq, kDt);
    if (commit != nullptr) {
      ASSERT_TRUE(consumer_.successful_write_cycle());
    }
    for (std::size_t j = 0; j < kJointsPerArm; ++j) {
      tau_real[j] = mit_torque(consumer_.submitted().joints[j], q[j], dq[j]);
      // Inside the final torque bound, where the real adapter sends the tuple
      // unchanged.
      ASSERT_LE(std::abs(tau_real[j]), real_.final_torque[j]) << j;
    }
    applied_ = tau_real;
  }

  // Both consumers latch their SAFE hold in the next step().
  void request_safe() { safe_requested_ = true; }

private:
  SafetyProfile real_, sim_;
  Limiter limiter_;
  ArmConsumer consumer_;
  Joints applied_{};  // the real motor's torque last cycle: what the next read measures
  std::uint64_t generation_{0};
  bool safe_requested_{false};
};

// `cycles` steps at `q`, recording per joint when the torques agree for good
// and their largest difference; `check(j, t, real, limiter)` is the phase's
// invariant on every cycle.
template <class Check>
void measure(
  Bench & bench, const Joints & q, const ArmCommand * commit, std::uint64_t cycles,
  Results & results, Check check)
{
  std::array<std::uint64_t, kJointsPerArm> last_disagree{};
  Joints real{}, sim{};
  for (std::uint64_t k = 1; k <= cycles; ++k) {
    bench.step(q, commit, real, sim);
    if (::testing::Test::HasFatalFailure()) return;
    const double t = static_cast<double>(k - 1) * kDt;
    for (std::size_t j = 0; j < kJointsPerArm; ++j) {
      check(j, t, real[j], sim[j]);
      const double diff = std::abs(real[j] - sim[j]);
      if (diff > kAgree) last_disagree[j] = k;
      if (diff > results[j].max_diff) {
        results[j].max_diff = diff;
        results[j].max_diff_s = t;
      }
    }
  }
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    ASSERT_LT(last_disagree[j], cycles) << "joint " << j << " never agreed";
    results[j].agree_s = static_cast<double>(last_disagree[j]) * kDt;
    // One cycle of rounding either way on top of the slew.
    EXPECT_LE(results[j].agree_s, results[j].expected_s + 2 * kDt) << j;
  }
}

std::uint64_t cycles_past(const Results & results, double margin_s)
{
  double horizon_s = 0.0;
  for (const auto & r : results) horizon_s = std::max(horizon_s, r.expected_s);
  return static_cast<std::uint64_t>(std::ceil((horizon_s + margin_s) * kRate));
}

// The SEEDING phase from `held`: the limiter lies between the hold and the
// real consumer -- it never applies more than the real arm does, nor drops
// below the hold -- until its kp has slewed to the commit's.
void seed(Bench & bench, const Joints & held, Results & results)
{
  const auto & p = bench.profile();
  const ArmCommand commit = bench.activate(held);
  if (::testing::Test::HasFailure()) return;
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    ASSERT_GT(kKp[j], p.safe_stiffness[j]) << j;
    results[j].expected_s = (kKp[j] - p.safe_stiffness[j]) / p.kp_slew[j];
  }
  measure(
    bench, offset_from(kSeed, kOffset), &commit, cycles_past(results, 1.0), results,
    [&](std::size_t j, double t, double real, double sim) {
      ASSERT_GE(real, held[j]) << j;  // the premise of the sign convention
      EXPECT_LE(sim, real + kAgree) << "joint " << j << " t " << t;
      EXPECT_GE(sim, held[j] - kAgree) << "joint " << j << " t " << t;
    });
}

std::string number(double value)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.4f", value);
  return text;
}

void report(const std::string & phase, const std::string & what, const Results & results)
{
  std::printf(
    "[limiter_vs_consumer] %s: %s, %zu Hz\n"
    "  joint  agree_s  expected_s  max_dtau_Nm  at_s\n",
    phase.c_str(), what.c_str(), kRate);
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    const auto & r = results[j];
    std::printf(
      "  j%zu     %7.3f  %10.3f  %11.4f  %.3f\n", j + 1, r.agree_s, r.expected_s, r.max_diff,
      r.max_diff_s);
    const auto key = phase + "_j" + std::to_string(j + 1);
    ::testing::Test::RecordProperty(key + "_agree_s", number(r.agree_s));
    ::testing::Test::RecordProperty(key + "_max_dtau_Nm", number(r.max_diff));
  }
  std::fflush(stdout);
}

std::string seeding_label(const SafetyProfile & p)
{
  char head[96];
  std::snprintf(
    head, sizeof(head), "kp safe -> commit at kp_slew_per_s, arm %.2f rad off the seed pose (",
    std::abs(kOffset));
  std::string text = head;
  for (std::size_t j = 0; j < kJointsPerArm; ++j) {
    char item[48];
    std::snprintf(
      item, sizeof(item), "%sj%zu %.1f->%.0f @%.0f/s", j ? ", " : "", j + 1, p.safe_stiffness[j],
      kKp[j], p.kp_slew[j]);
    text += item;
  }
  return text + ")";
}
}  // namespace

TEST(LimiterVsConsumer, SeedingAfterAFreshEnable)
{
  // A motor enabled just now: nothing held, both holds carry zero torque.
  Bench bench;
  Results results{};
  seed(bench, Joints{}, results);
  if (HasFatalFailure()) return;
  report("fresh_enable", seeding_label(bench.profile()), results);
}

TEST(LimiterVsConsumer, SeedingAfterAHeldArm)
{
  // A reactivation from an INACTIVE hold: both holds carry the held torque.
  Bench bench;
  bench.hold_at_seed(kHeld);
  if (HasFatalFailure()) return;
  Results results{};
  seed(bench, kHeld, results);
  if (HasFatalFailure()) return;
  report("held_arm", seeding_label(bench.profile()), results);
}

TEST(LimiterVsConsumer, SafeHoldAfterSeeding)
{
  // After the SEEDING above, both consumers latch their measured SAFE hold.
  Bench bench;
  bench.hold_at_seed(kHeld);
  if (HasFatalFailure()) return;
  Results seeding{};
  seed(bench, kHeld, seeding);
  if (HasFatalFailure()) return;
  const auto & p = bench.profile();
  const Joints q_hold = offset_from(kSeed, kOffset);
  Joints latched{};  // what both holds latch: the torque applied when SAFE happens
  for (std::size_t j = 0; j < kJointsPerArm; ++j)
    latched[j] = mit_torque({kSeed[j], 0.0, kKp[j], kKd[j], kHeld[j]}, q_hold[j], 0.0);

  // At rest. The real hold carries the latched torque at once, and so does
  // the limiter's: its spring is gone at once (q_des is the measured pose) and
  // its tau_ff is the latched torque in the latching cycle, not a slew to it
  // from the commit's, so the two agree from the first SAFE cycle. (The slew
  // dipped the limiter by up to the spring's share, kp * 0.05 here.)
  Results at_rest{};  // expected_s 0: no slew to wait for
  constexpr std::uint64_t kRestCycles = kRate / 2;
  bench.request_safe();
  measure(
    bench, q_hold, nullptr, kRestCycles, at_rest,
    [&](std::size_t j, double t, double real, double sim) {
      EXPECT_NEAR(real, latched[j], 1e-9) << j;
      EXPECT_NEAR(sim, real, kAgree) << "joint " << j << " t " << t;
    });
  if (HasFatalFailure()) return;
  report("safe_at_rest", "SAFE latched at rest, tau_ff the latched torque at once", at_rest);

  // Then pushed a further 0.02 rad: the real hold resists at the safe kp, the
  // limiter at a kp still slewing down from the commit's since the latch, so
  // it is the stiffer one until that slew ends. (0.02 rad: small enough that
  // the final torque slew passes the real hold's step in one cycle.)
  constexpr double kPush = -0.02;
  const double since_latch_s = static_cast<double>(kRestCycles) * kDt;
  Results pushed{};
  for (std::size_t j = 0; j < kJointsPerArm; ++j)
    pushed[j].expected_s =
      std::max(0.0, (kKp[j] - p.safe_stiffness[j]) / p.kp_slew[j] - since_latch_s);
  measure(
    bench, offset_from(q_hold, kPush), nullptr, cycles_past(pushed, 1.0), pushed,
    [&](std::size_t j, double t, double real, double sim) {
      EXPECT_GE(sim, real - kAgree) << "joint " << j << " t " << t;
      EXPECT_LE(sim, latched[j] - kKp[j] * kPush + kAgree) << "joint " << j << " t " << t;
    });
  if (HasFatalFailure()) return;
  report(
    "safe_pushed", "the SAFE hold pushed 0.02 rad, 0.5 s after the latch (kp slewing down)",
    pushed);
}

TEST(LimiterVsConsumer, BothBoundAHoldLatchedInMotionTheSameWay)
{
  // A commit carrying the producer's hold torque h (kHeld), applied as a pure
  // tau_ff of h + 3.5 N m: the motion's inertial share on top of gravity. SAFE
  // latches with joint 4 moving at twice the profile's motion velocity, or at
  // rest. Moving, both backends hold h + band; at rest, the torque applied.
  constexpr double kInertial = 3.5;
  const SafetyProfile real = real_profile();
  for (const bool moving : {true, false}) {
    Joints dq{};
    if (moving) dq[3] = 2.0 * real.safe_hold_motion_velocity;
    ArmCommand commit;
    commit.session_echo = 1;
    commit.lease_cycles = static_cast<double>(real.lease_default);
    Joints applied{};
    for (std::size_t j = 0; j < kJointsPerArm; ++j) {
      ASSERT_LT(real.safe_hold_torque_band[j], kInertial) << j;
      applied[j] = kHeld[j] + kInertial;
      commit.joints[j] = {kSeed[j], 0.0, 0.0, 0.0, applied[j]};
      commit.hold_torque[j] = kHeld[j];
    }

    // MuJoCo: the limiter applies the commit until every slew is done.
    Limiter limiter(mujoco_copy(real), kRate);
    limiter.reset(kSeed);
    Joints tau{};
    for (std::uint64_t k = 1; k <= 2 * kRate; ++k) {
      ASSERT_TRUE(limiter.submit(
        limiter_tuples(commit), commit.hold_torque, k,
        static_cast<std::uint64_t>(commit.lease_cycles)));
      tau = limiter.update(kSeed, Joints{}, kDt);
    }
    for (std::size_t j = 0; j < kJointsPerArm; ++j) ASSERT_NEAR(tau[j], applied[j], 1e-9) << j;
    limiter.request_safe();
    (void)limiter.update(kSeed, dq, kDt);

    // Real: the consumer accepts the commit and reads the torque applied.
    ArmConsumer consumer(
      real_adapter_limits(real), real.safe_damping, real.safe_stiffness, real.tau_ff_max,
      real.safe_hold_torque_band, real.safe_hold_motion_velocity);
    ASSERT_TRUE(consumer.configure(1, kSeed));
    commit.generation = 1;
    ASSERT_TRUE(consumer.accept_and_write(commit));
    consumer.observe(kSeed, dq, applied);
    consumer.request_safe_transition(true);
    ASSERT_TRUE(consumer.submit_safe_transition(true));

    for (std::size_t j = 0; j < kJointsPerArm; ++j) {
      const double expected = moving ? kHeld[j] + real.safe_hold_torque_band[j] : applied[j];
      EXPECT_NEAR(consumer.submitted().joints[j].effort, expected, 1e-12)
        << "joint " << j << (moving ? " moving" : " at rest");
      EXPECT_NEAR(limiter.held_effort(j), expected, 1e-9)
        << "joint " << j << (moving ? " moving" : " at rest");
    }
  }
}
