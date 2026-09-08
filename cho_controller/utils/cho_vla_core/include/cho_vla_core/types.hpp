// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
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
//                 chunk integrates from the anchor (OpenVLA-OFT LIBERO EE
//                 deltas; pi0 DROID joint velocity after the bridge multiplies
//                 by control_dt).
enum class RelativeMode : std::uint8_t {kAbsolute, kFromAnchor, kPerStep};

// kBinary keeps the historical sign convention (< 0 close, > 0 open) and is what
// OpenVLA-OFT LIBERO emits. kContinuous is an opening ratio in [0, 1] and is
// what every other stack emits.
enum class GripperMode : std::uint8_t {kBinary, kContinuous};

enum class RotationType : std::uint8_t {kEuler, kAxisAngle, kQuaternion, kRotation6D};

// Why a chunk was refused. Reported per chunk so the host can count causes
// instead of logging one undifferentiated "bad chunk".
enum class Reject : std::uint8_t
{
  kNone, kChunkSize, kControlDt, kRotationType, kSizeMismatch, kVelocitySize,
  kGripperSize, kNonFinite, kJointOrder, kJointWindow, kWorkspace, kDegenerateRotation,
  kStaleSeq, kNonFiniteStamp,
  // A string-valued field (action_space, relative_mode, gripper_mode) that the
  // host could not parse. Distinct from kRotationType so telemetry does not
  // report every unparseable field as a rotation problem.
  kUnparseableField
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
// Quaternion order is (x, y, z, w), matching the existing Franka pipeline and
// the ROS geometry_msgs convention.
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

// Host-neutral mirror of cho_interfaces/msg/ActionChunk (v2). Raw arrays are kept
// as received: every size and finiteness check lives in the validator, so the
// fragile parsing is inside the tested core rather than in each host's adapter.
struct Chunk
{
  // Observation time on the host's control clock. The bridge must echo the stamp
  // of the joint state it built the observation from, NOT its own wall clock:
  // this repo's multi-PC setup has no clock sync, so a bridge wall clock would be
  // in a different time domain than the controller.
  double t_obs {0.0};
  std::uint64_t seq {0};
  ActionSpace space {ActionSpace::kTask};
  RelativeMode relative {RelativeMode::kAbsolute};
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
