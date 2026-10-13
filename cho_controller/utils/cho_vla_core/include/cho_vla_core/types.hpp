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

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>

// Robot-independent VLA action-chunk pipeline.
//
// Deliberately free of ROS: nothing here includes rclcpp or a message type, and
// time is a plain double on the HOST's control clock (ROS time, sim time, or a
// test's synthetic clock -- the pipeline never reads a clock of its own). Hosts
// convert their transport into Chunk and hand it in. That is what lets the whole
// pipeline be tested without a controller_manager fixture.
namespace cho_vla_core
{
// Both robots this serves (Franka FR3, OpenArm v1.0 single arm) are 7-DoF, and
// the Franka pipeline this is extracted from was already fixed at 7. Bimanual
// (14+2) needs dynamic sizing and is deliberately out of scope for v1.
constexpr std::size_t kJoints = 7;

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using SE3 = pinocchio::SE3;

enum class ActionSpace : std::uint8_t {kJoint, kTask};

// How a chunk's arm_actions relate to the robot state.
//
//   kAbsolute   - the values ARE the targets (pi0 ALOHA, GR00T, LeRobot).
//   kFromAnchor - each value is a total offset from the observation-time anchor
//                 (the historical `relative: true` behaviour).
//   kPerStep    - each value is an increment over the previous waypoint, so the
//                 chunk integrates from the anchor. Two stacks are meant for
//                 it, and neither emits metres and radians; the bridge converts
//                 (ActionChunk.msg has the full prescription -- read from public
//                 sources, so confirm it in the version bridged):
//                   - OpenVLA-OFT LIBERO: the 7-D action is the INPUT of
//                     robosuite's OSC_POSE controller, which clips it to
//                     [-1, 1] and scales it by 0.05 m (3 position values) and
//                     0.5 rad (3 rotation values, an axis-angle vector). The
//                     bridge clips and scales and sends rotation_type
//                     "axis_angle" with relative_frame "base". That stack
//                     re-anchors every step on the measured pose; kPerStep
//                     accumulates on the previous waypoint, so the chunk
//                     reproduces the targets it commanded, not the motion it
//                     realized.
//                   - pi0 DROID: joint velocity is a normalised v in [-1, 1],
//                     which DROID turns into a joint delta of 0.2 * v rad per
//                     15 Hz step (not v * control_dt, a third of that), so the
//                     bridge sends dq_k = 0.2 * clip(v_k, -1, 1) [rad].
//
// In TASK space both relative modes also depend on the frame the delta is
// expressed in: see RelativeFrame. LIBERO's OSC_POSE applies the delta in the
// world axes, which are the robot base axes in LIBERO -- the position plus the
// delta, the delta rotation on the LEFT -- so a LIBERO bridge sends
// relative_frame "base". Read in the default "ee" frame those deltas would move
// a tool-down arm (R ~ diag(1, -1, -1)) the wrong way in y and z and invert its
// pitch and yaw.
enum class RelativeMode : std::uint8_t {kAbsolute, kFromAnchor, kPerStep};

// Frame a RELATIVE TASK-space delta (translation dp, rotation dR) is expressed
// in. Joint chunks and kAbsolute chunks ignore it.
//
//   kEe   - the body frame of the pose the delta is applied to (the pre-v3
//           behaviour, and the default), i.e. the host's EE frame (Franka's
//           ee_name, OpenArm's ee_frame), whose axes need not be those of the
//           EE frame a dataset was recorded in:
//             kFromAnchor  p = p_a + R_a * dp,        R = R_a * dR
//             kPerStep     T_k = T_{k-1} * dT_k
//   kBase - the model root frame (LIBERO's deltas, sent by an OpenVLA-OFT
//           bridge; see RelativeMode):
//             kFromAnchor  p = p_a + dp,              R = dR * R_a
//             kPerStep     p_k = p_{k-1} + dp_k,      R_k = dR_k * R_{k-1}
//
// (p_a, R_a) is Anchor::pose, T_0 = Anchor::pose, positions are in metres in the
// model root frame. kBase rotates about root-ALIGNED axes through the tool
// point; it is not dT * T_a, which would also swing the position about the root
// origin.
//
// The model root is the root frame of the model the receiving controller
// computes Anchor::pose in (the registry's model.absolute_goal_frame), which is
// not always the arm's own base link:
//   Franka               fr3_link0.
//   OpenArm single arm   world; its axes are openarm_link0's only while the
//                        bringup's base_rpy is zero (its default).
//   OpenArm bimanual     world, the torso, for each arm's controller. Each arm's
//                        link0 is rotated relative to it (by default -90 deg
//                        about x for the left arm and +90 deg about x for the
//                        right, xacro left/right_arm_base_rpy), so deltas
//                        recorded in an arm-base frame must be rotated by the
//                        bridge into world axes first.
enum class RelativeFrame : std::uint8_t {kEe, kBase};

// kBinary keeps the historical sign convention (< 0 close, > 0 open) and is what
// OpenVLA-OFT LIBERO emits. kContinuous is an opening ratio in [0, 1] and is
// what every other stack emits.
enum class GripperMode : std::uint8_t {kBinary, kContinuous};

// Encodings of a task waypoint's rotation block. decode_rotation() fixes one
// convention per encoding, and a stack that uses another is converted in its
// BRIDGE -- the decoder never guesses which one it was sent:
//
//   kEuler      3 values (a, b, c) [rad]: R = Rx(a) * Ry(b) * Rz(c), intrinsic
//               X-Y-Z (scipy from_euler("XYZ", ...)). This is NOT the extrinsic
//               R = Rz(c) * Ry(b) * Rx(a) of scipy's lower-case "xyz" or of ROS
//               / URDF roll-pitch-yaw; the two differ unless at most one angle
//               is nonzero. A stack in degrees or in another order converts.
//   kAxisAngle  3 values: the rotation vector, angle * unit axis [rad]. A zero
//               vector is "no rotation".
//   kQuaternion 4 values (x, y, z, w), scalar LAST (ROS geometry_msgs, scipy
//               as_quat()). Need not be unit length: it is normalised here. A
//               scalar-first stack (MuJoCo, Isaac Lab) reorders.
//   kRotation6D 6 values (c0, c1): the first two COLUMNS of R, column 0 first,
//               i.e. (R00, R10, R20, R01, R11, R21). Decoded by Gram-Schmidt:
//               col0 = c0 / |c0|, col1 = normalised(c1 - (col0 . c1) col0),
//               col2 = col0 x col1. A stack that flattens the first two ROWS
//               decodes here as R^T, and reordering its six numbers is not
//               enough (they lack R20 and R21). Its bridge builds R the way the
//               stack does -- orthonormalise the rows first (row0 normalised,
//               row1 made orthogonal to row0 and normalised), then
//               row2 = row0 x row1 -- and sends the first two columns of that
//               R. A raw network output is not orthonormal; skipping the first
//               step and crossing the raw rows gives a different R than the
//               stack's own, because this decoder orthogonalises columns.
enum class RotationType : std::uint8_t {kEuler, kAxisAngle, kQuaternion, kRotation6D};

// Why a chunk was refused. Reported per chunk so the host can count causes
// instead of logging one undifferentiated "bad chunk".
enum class Reject : std::uint8_t
{
  kNone, kChunkSize, kControlDt, kRotationType, kSizeMismatch, kVelocitySize,
  kGripperSize, kNonFinite, kJointOrder, kJointWindow, kWorkspace, kDegenerateRotation,
  kStaleSeq, kNonFiniteStamp,
  // A string-valued field (action_space, relative_mode, relative_frame,
  // gripper_mode) that the host could not parse. Distinct from kRotationType so
  // telemetry does not report every unparseable field as a rotation problem.
  // validate() also returns it for an out-of-range RelativeFrame, which only a
  // host that bypassed parse_relative_frame() can produce.
  kUnparseableField,
  // The chunk is well-formed, but this host, as configured, does not accept its
  // action space (the OpenArm VlaController under drive_side_impedance: false
  // takes task chunks only). The host sets it; validate() never does. Distinct
  // from kUnparseableField, which says the message is wrong: no fix to the
  // chunk's contents gets this one accepted -- the controller's configuration
  // or the bridge's choice of action space has to change.
  kActionSpaceNotAccepted,
  // t_obs outside the observation window around Chunk::t_arrival (see
  // ValidationLimits): later than arrival by more than max_future_skew, or
  // earlier by more than max_observation_age. A stamp from another clock or a
  // replay, not an observation of this arm.
  kObservationInFuture, kObservationTooOld
};
const char * reject_name(Reject reject);

// ---------------------------------------------------------------------------
// Fail-closed string parsing.
//
// Unknown strings are REJECTED, never defaulted. The historical code defaulted
// an unknown rotation_type to dim 0, which made the size check `arm_actions.size()
// == chunk_size * 0` pass for an EMPTY arm_actions and then built a std::vector
// from an iterator range running backwards over that empty vector -- undefined
// behaviour reachable from one malformed message.
// ---------------------------------------------------------------------------

// Empty string keeps the historical default ("task").
bool parse_action_space(const std::string & text, ActionSpace & out);
// Empty string falls back to the legacy bool: false -> kAbsolute, true -> kFromAnchor.
bool parse_relative_mode(const std::string & text, bool legacy_relative, RelativeMode & out);
// Empty string keeps the pre-v3 behaviour (kEe); "ee" -> kEe, "base" -> kBase.
// Anything else returns false with `out` untouched. Warning about an empty value
// on a relative task chunk is the host's job: a parser has no logger, and an
// empty value is legal.
bool parse_relative_frame(const std::string & text, RelativeFrame & out);
// Empty string keeps the historical default (kBinary).
bool parse_gripper_mode(const std::string & text, GripperMode & out);
// No empty-string default: a task-space chunk with no rotation_type is a bridge
// bug, and guessing one silently drives the arm to a guessed orientation.
bool parse_rotation_type(const std::string & text, RotationType & out);

// Size of the rotation block alone: euler 3, axis_angle 3, quaternion 4, rotation6d 6.
std::size_t rotation_dim(RotationType rotation);
// Size of one task waypoint: 3 translation + rotation_dim.
std::size_t task_waypoint_dim(RotationType rotation);

// Decode one rotation block. Returns false for a degenerate parameterisation (a
// zero-norm quaternion, a rotation6d basis whose two vectors are collinear)
// rather than falling back to identity, which would silently command "keep the
// current orientation" for corrupt input. Values are assumed already checked
// finite by the validator; this only rejects what finiteness cannot catch.
//
// The conventions are the ones listed at RotationType, in short: euler (a, b, c)
// [rad] -> Rx(a) * Ry(b) * Rz(c); axis_angle = rotation vector [rad];
// quaternion (x, y, z, w), matching the existing Franka pipeline and ROS
// geometry_msgs; rotation6d = the first two COLUMNS of R.
bool decode_rotation(const double * values, RotationType rotation, Eigen::Matrix3d & out);

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

// The robot state a chunk's offsets are measured against: the reference the host
// was commanding at the chunk's OBSERVATION time, not at its arrival time. See
// ReferenceHistory.
struct Anchor
{
  SE3 pose {SE3::Identity()};
  Vector7 joints {Vector7::Zero()};
};

// One waypoint, resolved to absolute targets, carrying the ABSOLUTE time at which
// it should be executed on the host's control clock.
struct Waypoint
{
  double t {0.0};
  Vector7 joints {Vector7::Zero()};
  SE3 pose {SE3::Identity()};
  // From the chunk's arm_velocities when supplied; otherwise the sampler finite-
  // differences the position waypoints.
  Vector7 joint_velocity {Vector7::Zero()};
  bool has_joint_velocity {false};
  double gripper {0.0};
  bool has_gripper {false};
  // Carried per waypoint rather than held as host state, so the sampler and the
  // dispatcher never have to remember which mode the producing chunk used. A
  // mid-goal mode switch then cannot mis-read waypoints from the older chunk.
  GripperMode gripper_mode {GripperMode::kBinary};
  // Slopes of the interpolating curve AT this waypoint, written by
  // ActionBuffer::splice() from the chunk the waypoint came in with: a cubic
  // spline is a property of a whole chunk, not of two neighbouring waypoints.
  // Angular is world-aligned, like Reference::twist.
  Vector7 joint_tangent {Vector7::Zero()};
  Eigen::Vector3d linear_tangent {Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_tangent {Eigen::Vector3d::Zero()};
};

// What the sampler hands the control law each cycle.
struct Reference
{
  ActionSpace space {ActionSpace::kTask};
  Vector7 joints {Vector7::Zero()};
  Vector7 joint_velocity {Vector7::Zero()};
  SE3 pose {SE3::Identity()};
  // Linear then angular, expressed in the same frame as `pose`.
  Vector6 twist {Vector6::Zero()};
  double gripper {0.0};
  bool has_gripper {false};
  GripperMode gripper_mode {GripperMode::kBinary};
  bool valid {false};
};

// Host-neutral mirror of cho_interfaces/msg/ActionChunk (v3: v2 plus
// relative_frame). Raw arrays are kept as received: every size and finiteness
// check lives in the validator, so the fragile parsing is inside the tested core
// rather than in each host's adapter.
struct Chunk
{
  // Observation time on the host's control clock. The bridge must echo the stamp
  // of the joint state it built the observation from, NOT its own wall clock:
  // this repo's multi-PC setup has no clock sync, so a bridge wall clock would be
  // in a different time domain than the controller.
  double t_obs {0.0};
  // Arrival time on the same clock, set by a host that takes t_obs from the
  // chunk's stamp; validate() then checks the observation window against it.
  // NaN (the default) or infinite skips that check: a host that places the
  // chunk at its arrival has nothing to compare.
  double t_arrival {std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t seq {0};
  ActionSpace space {ActionSpace::kTask};
  RelativeMode relative {RelativeMode::kAbsolute};
  // Read only for a TASK chunk with kFromAnchor or kPerStep; see RelativeFrame.
  // The default kEe is the pre-v3 composition.
  RelativeFrame relative_frame {RelativeFrame::kEe};
  RotationType rotation {RotationType::kQuaternion};
  GripperMode gripper_mode {GripperMode::kBinary};
  int chunk_size {0};
  // Spacing between consecutive waypoints [s]. Must be positive: it is a divisor
  // in the playback path, and the historical code let three separate invariants
  // in two threads stand in for a check at the division itself.
  double control_dt {0.0};
  std::vector<double> arm_actions;
  // Optional. Joint space only: a task-space "velocity" in the same array shape
  // would have to encode a rotation derivative, which no stack we bridge emits.
  // Task-space twists come from finite differences instead.
  std::vector<double> arm_velocities;
  std::vector<double> gripper_actions;
  // Permutation mapping this chunk's joint order onto the robot's canonical
  // order: joint_order[i] is the chunk column feeding canonical joint i. Empty
  // means identity. The host resolves joint_names -> indices (it owns the robot
  // config); the validator checks the result really is a permutation, so a
  // partial or duplicated joint_names list is refused rather than silently
  // dropping or double-driving a joint.
  std::vector<std::size_t> joint_order;
};

// Counters and stamps the host publishes. Cumulative fields are never reset by
// the pipeline; the host resets them per goal.
struct Telemetry
{
  std::uint64_t chunks_accepted {0};
  std::uint64_t chunks_rejected {0};
  // Waypoints thrown away because their execution time was already past on
  // arrival. A persistently nonzero value means inference latency exceeds the
  // chunk's horizon -- the policy is planning into a window that is gone.
  std::uint64_t waypoints_dropped_past {0};
  Reject last_reject {Reject::kNone};
  // t_arr - t_obs of the last accepted chunk [s].
  double last_chunk_latency {0.0};
  // t_obs of the last accepted chunk, and the absolute time the sampler last
  // produced a reference for. The bridge needs both: last_chunk_stamp to pace
  // observations, playback_stamp to cut its RTC prev_chunk_left_over prefix.
  double last_chunk_stamp {0.0};
  double playback_stamp {0.0};
  double remaining_horizon {0.0};
  std::size_t queue_depth {0};
};

}  // namespace cho_vla_core
