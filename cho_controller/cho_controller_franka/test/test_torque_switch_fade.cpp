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

// TorqueSwitchFade, what the Franka torque controllers run when the reference
// their law tracks changes at once (a goal's start, end or cancel), at 1 kHz. A
// law that jumps from one torque to another goes out as the old torque on the
// period of the switch and reaches the new one 0.1 s later, never changing by more
// than the raised cosine's steepest rate in a period.
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "cho_controller_franka/torque_switch_fade.hpp"

namespace
{

using cho_controller::franka::TorqueSwitchFade;
using Vector7d = TorqueSwitchFade::Vector7d;

constexpr double kDt = 0.001;
constexpr double kPi = 3.14159265358979323846;

Vector7d torque(const double scale)
{
  Vector7d t;
  t << 1.0, -2.0, 0.5, 3.0, -0.25, 0.1, 0.0;
  return scale * t;
}

// One period: the law's torque in, the command out.
Vector7d period(TorqueSwitchFade & fade, Vector7d law, const bool switched)
{
  fade.apply(law, switched, kDt);
  fade.commit(law);
  return law;
}

// A controller that commanded `from` for a while, then whose law jumps to `to`.
std::vector<Vector7d> run(const Vector7d & from, const Vector7d & to, const int periods)
{
  TorqueSwitchFade fade;
  fade.reset();
  for (int k = 0; k < 10; ++k) {
    period(fade, from, false);
  }
  std::vector<Vector7d> out;
  for (int k = 0; k < periods; ++k) {
    out.push_back(period(fade, to, k == 0));
  }
  return out;
}

TEST(TorqueSwitchFade, TheSwitchPeriodCommandsTheLastCommand)
{
  const auto out = run(torque(2.0), torque(-1.0), 1);
  EXPECT_LT((out[0] - torque(2.0)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(TorqueSwitchFade, TheChangeFadesOutOverTheFadeTimeNoFasterThanTheCosine)
{
  const Vector7d from = torque(2.0);
  const Vector7d to = torque(-1.0);
  const auto out = run(from, to, 300);
  const double change = (from - to).cwiseAbs().maxCoeff();
  const double steepest = kPi / (2.0 * TorqueSwitchFade::kSeconds) * change * kDt;
  for (std::size_t k = 1; k < out.size(); ++k) {
    EXPECT_LE((out[k] - out[k - 1]).cwiseAbs().maxCoeff(), steepest * (1.0 + 1e-9)) << "period " << k;
    // From `from` toward `to`, never past either.
    for (int i = 0; i < 7; ++i) {
      EXPECT_LE(std::abs(out[k](i) - to(i)), std::abs(out[k - 1](i) - to(i)) + 1e-12) << "period " << k;
    }
  }
  const auto done = static_cast<std::size_t>(TorqueSwitchFade::kSeconds / kDt) + 1;
  EXPECT_LT((out[done] - to).cwiseAbs().maxCoeff(), 1e-12) << "still fading after kSeconds";
  EXPECT_LT((out.back() - to).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(TorqueSwitchFade, AFirstSwitchAfterResetFadesNothing)
{
  // On activation the interface still holds another controller's command, or
  // none: nothing of this controller's own to go on from.
  TorqueSwitchFade fade;
  fade.reset();
  EXPECT_LT((period(fade, torque(1.0), true) - torque(1.0)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_FALSE(fade.fading());
  // And reset() forgets what was committed before it.
  period(fade, torque(1.0), false);
  fade.reset();
  EXPECT_LT((period(fade, torque(-1.0), true) - torque(-1.0)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(TorqueSwitchFade, ASwitchDuringAFadeGoesOnFromWhereItHadGot)
{
  TorqueSwitchFade fade;
  fade.reset();
  for (int k = 0; k < 10; ++k) {
    period(fade, torque(2.0), false);
  }
  Vector7d last;
  for (int k = 0; k < 30; ++k) {
    last = period(fade, torque(-1.0), k == 0);
  }
  ASSERT_TRUE(fade.fading());
  const Vector7d first = period(fade, torque(0.5), true);
  EXPECT_LT((first - last).cwiseAbs().maxCoeff(), 1e-12);
  Vector7d out;
  for (int k = 0; k < 200; ++k) {
    out = period(fade, torque(0.5), false);
  }
  EXPECT_LT((out - torque(0.5)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(TorqueSwitchFade, ANonFiniteCommandStartsNoFade)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // A non-finite last command (clip_torque() zeroes it, but say it went out).
  TorqueSwitchFade fade;
  fade.reset();
  Vector7d bad = torque(1.0);
  bad(2) = nan;
  fade.commit(bad);
  EXPECT_LT((period(fade, torque(1.0), true) - torque(1.0)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_FALSE(fade.fading());
  // A non-finite law: left as it is for clip_torque(), and no fade from it.
  period(fade, torque(1.0), false);
  Vector7d law = bad;
  fade.apply(law, true, kDt);
  EXPECT_TRUE(std::isnan(law(2)));
  EXPECT_FALSE(fade.fading());
}

}  // namespace
