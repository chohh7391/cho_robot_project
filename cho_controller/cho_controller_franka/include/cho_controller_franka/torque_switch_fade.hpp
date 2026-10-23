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

#pragma once

#include <Eigen/Core>

namespace cho_controller {
namespace franka {

// Carries a torque controller's command across a change of what its law tracks
// -- a goal starting, ending or being canceled -- and across a switch from the
// torque controller it replaces, instead of letting it jump.
//
// Each torque law computes its command from the reference it tracks, and that
// reference changes at once when a goal starts or ends: the task-space laws hold
// the pose (or, the task-space QP, the joints) measured at the end, the
// joint-space ones start the next goal from the measured joints, and every one
// drops the reference velocity to 0 on a cancel. With the arm away from the new
// reference (on the FR3, joint friction stops a 1 cm move a few mm short) or
// still moving, the command changed in one period by the push the old reference
// made, or by kd * v: ~1 Nm at a goal's end on the arm, at libfranka's 1000 Nm/s
// torque rate limit. apply() puts that change back on the period of the switch
// and fades it out over kSeconds with a raised cosine, whose steepest rate,
// pi / (2 * kSeconds) = 52 /s times the change, stays under that limit for a
// change of up to 19 Nm (libfranka's own limiter spreads a larger one further).
// The law decides where the arm ends up; for kSeconds a canceled motion is
// braked more gently than its law alone would, which is why the fade is short:
// over 0.1 s a joint impedance cancel at 0.83 rad/s in MuJoCo went on 36 mrad
// where its law alone stopped it in 23.
//
// A controller switched in for another starts its law from its own reference,
// seeded from the measured arm, and so drops whatever the other pushed with:
// on the FR3, the effort action-chunk hold after a circle, which joint friction
// had left the arm short of, pushed j2 with 0.6 Nm, gone in the one period of
// the switch to the joint impedance controller (577 Nm/s). carry_on_from()
// starts the same fade from that push.
class TorqueSwitchFade
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  static constexpr double kSeconds = 0.03;

  // Forgets the last command: on activation there is no torque of this
  // controller's own to go on from, so a switch before the first commit() fades
  // nothing.
  void reset();

  // After reset(), on activation: the command the controller switched out in
  // the same switch left on the arm, which the arm is commanded until this
  // controller's first. The next apply() fades from it, switch or not.
  void carry_on_from(const Vector7d & torque);

  // Before clip_torque(), every period: `switched` when the reference the law
  // tracks changed this period. Starts a fade from the last committed command
  // (or the one carried on from) when both are finite, and adds what is left of
  // the fade to `torque`.
  void apply(Vector7d & torque, bool switched, double dt);

  // After clip_torque(): the command that goes out.
  void commit(const Vector7d & torque);

  // Whether a fade is still being added.
  bool fading() const {return left_ > 0.0;}

private:
  Vector7d last_{Vector7d::Zero()};
  Vector7d fade_{Vector7d::Zero()};
  double left_{0.0};
  bool has_last_{false};
  bool carried_{false};
};

}  // namespace franka
}  // namespace cho_controller
