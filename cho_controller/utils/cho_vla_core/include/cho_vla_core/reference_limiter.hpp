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

#include <ruckig/ruckig.hpp>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Reference-side velocity, acceleration and jerk bound: the single choke point
// on how fast ANY control law is asked to move.
//
// Ported from the Franka pipeline's saturate_des(), with two fixes. It runs only
// on the ACTIVE action space -- the historical version limited pose and joints
// on every cycle regardless, so the inactive representation's rate state froze
// at goal start. That alone does NOT make a change of action space continuous.
// A sampled reference's inactive representation is the running chunk's anchor,
// passed through unlimited, so when a chunk switches space the last value this
// limiter emitted in the new space is that anchor, not what the arm is being
// commanded to, and the first cycle would ramp from it. The host re-seeds at a
// switch, with what it commands in the new space and how fast: see
// seed_with_rates(). And the joint bounds are per-joint rather than one scalar,
// because the MIT safety profile's command_velocity differs per joint and is
// the authority there.
//
// The limiter TRACKS the sampled reference rather than chasing it. Each cycle
// it asks for the velocity the reference itself has (feed-forward) plus a
// correction that closes any gap between the two, and Ruckig's velocity
// interface takes that request to the bounds: velocity (next paragraph),
// acceleration and jerk, each bounded in BOTH directions. A reference that
// stays inside the bounds is therefore followed with no lag, and only the part
// of it that does not is shaped -- which is what a safety bound downstream of a
// smooth interpolation should do.
//
// The emitted rate in the active space stays within its velocity bound whatever
// the sampled rate does, to a relative 1e-9: each joint's joint_velocity within
// its max_joint_velocity, the linear and angular parts of the twist within
// max_linear_velocity and max_angular_velocity as norms, and one cycle's
// displacement within the bound times the step. Ruckig's velocity interface
// does not bound velocity, so two things do. Near the bound the feed-forward
// acceleration is capped to what still levels off inside it (none for a
// request clipped onto it), so the profile does not commit to passing it; and
// Ruckig's output is projected into the bound, per joint and on the norms, with
// the position following the projected rate. Both act only near the bound, so a
// reference followed inside the bounds is not affected. Not covered: the
// inactive space's fields, which pass through. The joint window clamp can move
// a joint further than that only from outside the window. A seed's joints are
// clamped into it (seed()), so that happens only on a joint cycle that follows
// task space without the re-seed a switch requires, starting from the joints
// task space passed through.
//
// The two limiters this replaced -- the historical trapezoid, then a Ruckig
// planning to come to rest on the sampled position -- both CHASED: every cycle
// they planned to stop at where the reference was, so a moving reference was
// followed v^2/(2a) behind. Measured in MuJoCo against a policy's own path that
// was 44-80 ms of lag, and on a clean policy the arm moved most smoothly with no
// limiter at all; the smoothing those limiters added was a side effect of the
// lag. The trapezoid also allowed instant deceleration, a velocity step that
// was the largest acceleration spike in the reference (+-80 rad/s^2 against a 4
// rad/s^2 bound).
//
// The gap correction is proportional for a small gap (time constant
// 1/tracking_gain) and capped by the braking envelope sqrt(2 a_brake gap) for a
// large one, with a_brake half the acceleration bound, so a far stationary
// target is approached without being overshot. A target that jumps BEHIND a
// moving reference is still overshot by up to v^2/(2a): bounded deceleration
// cannot stop faster, and on a torque-controlled arm the arm cannot either.
//
// Ruckig needs a velocity and an acceleration bound:
//
//   - both set: tracked as above. With no jerk bound configured, the
//     acceleration ramps over 40 ms (jerk = a / 0.04 s); an acceleration-only
//     Ruckig switches bang-bang between +-a and measured rougher than the
//     trapezoid did.
//   - velocity only: a per-cycle step clamp at that velocity.
//   - neither: the target passes through.
//
// Translation is tracked the same way on three axes. Its bounds are on NORMS:
// the requested velocity is clamped to max_linear_velocity as a vector, and the
// per-axis acceleration and jerk bounds are the norm bounds over sqrt(3), so the
// vector never exceeds them.
//
// Rotation, by default, keeps a velocity clamp alone, which never lagged: a
// rotation inside its bound passes through unchanged -- and so does any step in
// angular rate, a sampled reference that turns faster than the clamp included.
// With max_angular_acceleration set, rotation is tracked like translation
// instead, on the world-aligned angular velocity: the gap is the rotation vector
// from the emitted orientation to the sampled one, the request is the sampled
// rate plus the same proportional-then-braking-envelope correction, and Ruckig's
// velocity interface brings the emitted rate to it within the acceleration bound
// (norm, as for translation) and a jerk of a / 0.04 s. The emitted rate never
// exceeds max_angular_velocity when that is set; the orientation advances by the
// trapezoid of the previous and the new rate.
//
// A cycle on which time does not advance (now - previous time zero, negative
// or not finite) emits exactly what the previous cycle did -- pose, joints,
// twist and joint_velocity -- and leaves every rate state alone, so the next
// cycle that does advance continues as though that one had not happened. It
// used to invent a 1 ms step, which moved the reference on every
// repeated-state cycle Isaac reports. So does a cycle on which the clock JUMPS
// forward by more than kClockRebaseSec: that step is not a control period, and
// integrating it would move the reference by up to the velocity bound times
// the jump in one cycle. The previous time stays on a cycle that does not
// advance, except when the clock has been set back by more than
// kClockRebaseSec; there, and after a forward jump, `now` becomes the time
// base. Kept after a set-back, the output would hold until the clock passed
// the old time again -- after a 50 s sim-time reset, 50 s of an arm standing
// still while its policy streams accepted chunks. See apply().
class ReferenceLimiter
{
public:
  // A step larger than this, back or forward, is the clock being SET (a
  // sim-time reset, a system clock step), not a control period or jitter:
  // apply() integrates nothing over it and re-bases on it. The hosts' control
  // periods are 1-4 ms, and a period of 0.1 s or more is already one the
  // OpenArm task-space controller refuses to advance its clock by. One
  // threshold serves both directions, so one stray sample from another clock
  // costs two held cycles, out and back, and a clock that keeps alternating
  // holds the output for as long as it does.
  static constexpr double kClockRebaseSec = 0.1;

  struct Params
  {
    // <= 0 disables that bound.
    double max_linear_velocity {0.0};
    double max_angular_velocity {0.0};
    double max_linear_acceleration {0.0};
    // <= 0 means a / 0.04 s, as for the joints.
    double max_linear_jerk {0.0};
    // [rad/s^2], a bound on the norm of the emitted angular velocity's rate of
    // change. <= 0 (the default) disables it, leaving the velocity clamp alone,
    // with output identical to before it existed. See the class comment.
    double max_angular_acceleration {0.0};
    // How fast a gap between the emitted and the sampled reference is closed
    // [1/s]; see the class comment.
    double tracking_gain {10.0};
    Vector7 max_joint_velocity {Vector7::Zero()};
    Vector7 max_joint_acceleration {Vector7::Zero()};
    // Only applies where both of the above are set; <= 0 means a / 0.04 s. See
    // the class comment.
    Vector7 max_joint_jerk {Vector7::Zero()};

    // Hard clamp on the emitted joint reference. Unlike the rate bounds this is
    // a position window, and it is the last thing between a policy and a joint
    // stop.
    Vector7 joint_lower {Vector7::Zero()};
    Vector7 joint_upper {Vector7::Zero()};
    bool clamp_joint_window {false};
  };

  ReferenceLimiter() = default;
  explicit ReferenceLimiter(const Params & params) : params_(params) {}

  void set_params(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  // Seed AT REST from what the host is currently commanding, so the first
  // limited cycle is continuous: pose and joints from `reference`, every rate and
  // acceleration zero (its twist and joint_velocity are not read). For a change
  // of source while the arm is held: goal start, or a resume out of a hold
  // the host made without come_to_rest().
  //
  // With clamp_joint_window the joints are clamped into [joint_lower,
  // joint_upper], where every joint cycle's output lies; a joint inside it is
  // taken unchanged. A command outside the window (a host law's q + offset
  // before its own window clamp) is therefore seeded on the window's edge, and
  // the first joint cycle's joint_velocity, its displacement over the step,
  // stays within the velocity bound.
  void seed(const Reference & reference, double now);

  // As seed(), but the rate state starts at what the host is commanding instead
  // of at rest: joint space from reference.joint_velocity, task space from
  // reference.twist (linear, then world-aligned angular, the convention apply()
  // emits). Both are taken; the space the next apply() runs in continues from
  // its own, and the other is reset there as on every cycle. Each rate is
  // clamped into its configured velocity bound (per joint; translation and
  // rotation on their norms) and a non-finite one starts at zero. Acceleration
  // state starts at zero. The joints are clamped into the window as in seed();
  // the rates are not changed by that. A cycle at the seed's own time emits the
  // seed with these rates.
  //
  // `now` is the time the seeded value was commanded at. A host seeding with
  // what the previous cycle commanded passes that cycle's time (when it is
  // finite and earlier than this cycle's), so this cycle's apply() advances from
  // the seed at the seeded rate. Seeded at this cycle's time, apply() sees a
  // zero step and emits the seed itself: a moving reference stands still for
  // one cycle.
  //
  // The host calls it when the sampled reference changes action space within a
  // goal (Reference::space differs from the previous cycle's; the class comment
  // says why the limiter cannot bridge that itself). `reference` carries the NEW
  // space's commanded value and rate -- not the measured state, which differs
  // by whatever the law is pushing against: for joint -> task the forward
  // kinematics of the commanded joints and their Jacobian times the commanded
  // joint rate. Seeding at rest there would drop a moving reference's rate to
  // zero in one cycle and re-accelerate it, the arm falling behind meanwhile.
  void seed_with_rates(const Reference & reference, double now);

  // Limit `reference` in place. Does nothing until seed() or seed_with_rates()
  // has been called. In the active space, the emitted rates and one cycle's
  // displacement keep to the velocity bounds for any sampled input (class
  // comment).
  //
  // Only a step = now - previous time with 0 < step <= kClockRebaseSec is
  // integrated. On any other, the reference's pose, joints, twist and
  // joint_velocity are set to the previously EMITTED ones (right after a seed:
  // the seed's pose and joints with the seeded rates, zero after seed()) and no
  // rate state changes. Then:
  //   - NaN or +-inf: the previous time is kept too.
  //   - -kClockRebaseSec <= step <= 0: the previous time is kept too, so the
  //     next cycle that advances is measured from the last one that did.
  //   - step < -kClockRebaseSec: the previous time becomes `now`, and the next
  //     positive step continues from there.
  //   - step > kClockRebaseSec: the previous time becomes `now` too, so the
  //     jump is never integrated; the next step continues from there.
  // A seed at a non-finite time is replaced the same way by the first finite
  // `now`. A clock that goes back or forward and STAYS there costs one held
  // cycle. One stray sample from another clock, followed by the old one again,
  // costs two, the second a jump as long as the first; a clock alternating
  // between two times holds the output throughout.
  void apply(double now, Reference & reference);

  // One cycle of a stop, for a host whose stream is in hold: the reference
  // comes to rest from the limiter's own state, in the space it last ran (the
  // last apply() that advanced, or the seed). There is no target, so no gap
  // correction and no feed-forward: each rate goes from where apply() left it
  // (rate and acceleration) to zero within the bounds apply() uses, and once
  // at zero it stays there with the position held exactly.
  //   - Joints with a velocity and an acceleration bound, translation with
  //     both, and rotation under max_angular_acceleration: Ruckig's velocity
  //     interface to rest, per joint and per axis (a/sqrt(3), j/sqrt(3) per
  //     axis, as apply()), jerk as apply() (a / 0.04 s when unset). From rate v
  //     at zero acceleration the stop takes v/a + a/j and covers
  //     v^2/(2a) + v*a/(2j); when v < a^2/j, 2*sqrt(v/j) and v*sqrt(v/j).
  //   - Anything without an acceleration bound stops at once: apply() keeps no
  //     acceleration state there and passes rate steps through.
  // The rate stays inside its velocity bound and the joints inside the window,
  // as in apply(). The other space's representation is held as last emitted.
  //
  // Returns the stop: space, pose, joints, twist and joint_velocity as apply()
  // emits them (joint_velocity the step's displacement over the step, the
  // twist the stop's own rate; zero in the space not running), valid true, no
  // gripper. Time as in apply(): a step outside (0, kClockRebaseSec] repeats
  // the previous output. Before any seed it changes nothing and returns a
  // default Reference with valid false; seeded() stays false.
  //
  // It leaves no sampled-rate memory behind, so the next apply() has no
  // feed-forward on its first cycle, as after a seed, and continues from the
  // stop's position, rate and time: a host that calls this on every held
  // cycle can resume with apply() alone.
  Reference come_to_rest(double now);

  bool seeded() const {return seeded_;}

private:
  // The clock rule of apply() (see there): true, with `step`, on a cycle that
  // integrates; otherwise moves the time base as that rule says.
  bool advance_clock(double now, double & step);
  // What the previous cycle emitted, for a cycle that integrates nothing.
  void repeat_previous(Reference & reference) const;

  // Task-space rotation under max_angular_acceleration: sets
  // reference.pose.rotation() and reference.twist.tail<3>().
  void track_rotation(double step, Reference & reference);

  // come_to_rest() per space. Each writes its fields of `reference` and its
  // own rate state.
  void stop_joints(double step, Reference & reference);
  void stop_translation(double step, Reference & reference);
  void stop_rotation(double step, Reference & reference);

  Params params_ {};
  bool seeded_ {false};
  // The space the rate state is in: the seed's, then each integrating apply()'s.
  ActionSpace space_ {ActionSpace::kTask};
  double previous_time_ {0.0};
  SE3 previous_pose_ {SE3::Identity()};
  Vector7 previous_joints_ {Vector7::Zero()};
  // What the previous cycle (or the seed) emitted besides pose and joints, for a
  // cycle on which time does not advance.
  Vector6 emitted_twist_ {Vector6::Zero()};
  Vector7 emitted_joint_velocity_ {Vector7::Zero()};
  Eigen::Vector3d previous_linear_velocity_ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_linear_acceleration_ {Eigen::Vector3d::Zero()};
  // World-aligned; only rate state when max_angular_acceleration is set.
  Eigen::Vector3d previous_angular_velocity_ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_angular_acceleration_ {Eigen::Vector3d::Zero()};
  Vector7 previous_joint_velocity_ {Vector7::Zero()};
  Vector7 previous_joint_acceleration_ {Vector7::Zero()};
  // The SAMPLED rates of the previous cycle, whose difference is the
  // feed-forward acceleration. Invalid until a cycle in that space has run.
  Vector7 previous_joint_rate_ {Vector7::Zero()};
  Eigen::Vector3d previous_linear_rate_ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_angular_rate_ {Eigen::Vector3d::Zero()};
  bool joint_rate_valid_ {false};
  bool linear_rate_valid_ {false};
  bool angular_rate_valid_ {false};

  // Fixed-size, so update() does not allocate: safe on the control thread.
  ruckig::Ruckig<kJoints> joint_generator_ {};
  ruckig::InputParameter<kJoints> joint_input_ {};
  ruckig::OutputParameter<kJoints> joint_output_ {};
  ruckig::Ruckig<3> linear_generator_ {};
  ruckig::InputParameter<3> linear_input_ {};
  ruckig::OutputParameter<3> linear_output_ {};
  ruckig::Ruckig<3> angular_generator_ {};
  ruckig::InputParameter<3> angular_input_ {};
  ruckig::OutputParameter<3> angular_output_ {};
};

}  // namespace cho_vla_core
