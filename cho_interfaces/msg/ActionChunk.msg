# A chunk of policy actions, as produced by a VLA inference bridge.
#
# Versions (the message definition, not a protocol handshake):
#   v1 - the original fields.
#   v2 - header.stamp read as the observation time, seq, relative_mode,
#        joint_names, arm_velocities, gripper_mode.
#   v3 - relative_frame, inserted after `relative`.
# v2 and v3 inserted fields in the middle of the message, so the CDR layout of
# the fields after each insertion moved. A bridge built against an older
# definition is not wire-compatible with a controller built against this one:
# rebuild every bridge with this cho_interfaces. What a rebuilt bridge already
# sends keeps its meaning (an empty relative_frame reads as "ee", what v2 did).
# Whether a mismatched pair fails to match or misreads the later fields was not
# tested, so do not rely on either.
#
# stamp is the OBSERVATION time: the bridge must echo the header.stamp of the
# joint state it built its observation from, verbatim. Not its own wall clock --
# this repo's multi-PC setup (FastDDS discovery server) has no clock sync, so a
# bridge clock is a different time domain than the controller's. Echoing the
# controller's own stamp makes time alignment work without any sync.
# All-zero means "unstamped": the controller falls back to arrival time, which
# reproduces the pre-v2 behaviour. A controller reads the stamp only with
# chunk_time_source: observation (the default is arrival); it then refuses a
# stamp more than chunk_max_future_skew (0.05 s) after the chunk's arrival or
# more than chunk_max_observation_age (1 s) before it, so a stamp from another
# clock is rejected instead of anchoring the chunk at the wrong instant.
std_msgs/Header header

# Monotonic per-bridge chunk counter, incremented on every publish including a
# re-inference that supersedes an earlier chunk. 0 means "not supplied" and the
# controller orders by arrival instead.
uint64 seq

string action_space  # "joint" or "task" (empty = "task")

# How arm_actions relate to the robot state:
#   "absolute"    - the values ARE the targets (pi0 ALOHA, GR00T, LeRobot)
#   "from_anchor" - each value is a total offset from the observation-time state
#   "per_step"    - each value is an increment over the previous waypoint, so the
#                   chunk integrates from the anchor. Two stacks are meant for it.
#                   Both conventions below were read from public sources (robosuite
#                   v1.4.1, OpenVLA-OFT, DROID, openpi): confirm them in the
#                   version you bridge, checkpoints differ.
#                   - OpenVLA-OFT LIBERO: the 7-D action is the INPUT of
#                     robosuite's OSC_POSE controller, not metres and radians.
#                     The controller clips it to [-1, 1] and scales it by 0.05 m
#                     (3 position values) and 0.5 rad (3 rotation values). The
#                     rotation values are an axis-angle vector, and the goal is
#                     the measured position plus the delta and the delta rotation
#                     applied on the left of the measured one, in the world
#                     axes, which are the robot base axes in LIBERO. A bridge
#                     therefore clips and scales to metres and radians and sends
#                     rotation_type "axis_angle" with relative_frame "base"
#                     (not "euler": the "roll-pitch-yaw" label in OpenVLA's
#                     dataset config does not describe how the value is used).
#                     That stack re-anchors every step on the measured pose;
#                     per_step here accumulates on the previous waypoint. A chunk
#                     sent as above reproduces the targets that stack commanded,
#                     not the motion it realized.
#                   - pi0 DROID: joint velocity is a normalised v in [-1, 1].
#                     DROID turns it into a joint delta of 0.2 * v rad per step
#                     at 15 Hz (not v * control_dt, which is a third of that), so
#                     a bridge sends dq_k = 0.2 * clip(v_k, -1, 1) [rad].
# Empty falls back to the `relative` bool below.
string relative_mode
# Pre-v2 field, kept for compatibility. Only read when relative_mode is empty:
# false -> "absolute", true -> "from_anchor".
bool relative

# Frame a relative TASK-space delta is expressed in. Ignored for joint space and
# for "absolute". (p_a, R_a) is the anchor pose, (dp, dR) a waypoint's delta:
#   "ee"   - body frame of the anchor:  p = p_a + R_a*dp,  R = R_a*dR
#   "base" - the model root frame:      p = p_a + dp,      R = dR*R_a
# For "per_step" the same formula is applied at every step to the PREVIOUS
# waypoint instead of the anchor: "ee" gives T_k = T_{k-1}*dT_k, "base" gives
# p_k = p_{k-1} + dp_k and R_k = dR_k*R_{k-1}.
# "base" is the ROOT frame of the model the receiving controller computes in
# (the registry's model.absolute_goal_frame), which is not always the arm's own
# base link:
#   Franka               fr3_link0.
#   OpenArm single arm   world; its axes are openarm_link0's only while the
#                        bringup's base_rpy is zero (its default).
#   OpenArm bimanual     world, the torso. Each arm's link0 is rotated relative
#                        to it (by default -90 deg about x for the left arm and
#                        +90 deg about x for the right, xacro left/right_arm_
#                        base_rpy), so deltas recorded in an arm-base frame must
#                        be rotated by the bridge into world axes first.
# header.frame_id is not read by either controller: it does not select the frame.
# Empty = "ee", the pre-v3 behaviour; a relative task chunk that leaves it empty
# is still accepted, with a throttled warning. An unrecognised value is REJECTED,
# never guessed -- a wrong frame drives the arm the wrong way without any error.
string relative_frame

# Task space only. One of "euler", "axis_angle", "quaternion", "rotation6d".
# An unrecognised value is REJECTED, never guessed -- guessing drives the arm to
# an invented orientation. Each one is read as follows (R is the rotation matrix
# the decoder builds; for a relative chunk it is the delta dR):
#   "euler"      - 3 values (a, b, c) in radians, R = Rx(a)*Ry(b)*Rz(c), i.e.
#                  INTRINSIC X-Y-Z (scipy: uppercase 'XYZ'). This is NOT the ROS
#                  roll-pitch-yaw convention (R = Rz(yaw)*Ry(pitch)*Rx(roll)),
#                  nor scipy's lowercase 'xyz' (extrinsic). Rotations about
#                  different axes do not commute, so a stack that uses another
#                  order is rotated to a different pose, not a slightly
#                  different one.
#   "axis_angle" - 3 values, the rotation vector (axis * angle) in radians.
#   "quaternion" - 4 values in the order (x, y, z, w).
#   "rotation6d" - 6 values (v1, v2) = the first two COLUMNS of R, concatenated
#                  column after column. v2 is orthogonalised against v1 and the
#                  third column is v1 x v2. A stack that keeps the first two ROWS
#                  (a row-major convention) would be read as the transpose of its
#                  rotation, i.e. the opposite rotation. Reordering its six
#                  numbers is not enough (they lack R20 and R21), so its bridge
#                  builds R the way the stack does: orthonormalise the rows first
#                  (row0 normalised, row1 made orthogonal to row0 and normalised,
#                  e.g. GR00T's pose.py and pytorch3d's rotation_6d_to_matrix),
#                  then row2 = row0 x row1, and sends the first two columns of
#                  that R. A raw network output is not orthonormal; skipping the
#                  first step and crossing the raw rows gives a different R than
#                  the stack's own, because this decoder orthogonalises columns.
# The decoder does not convert between conventions. A stack whose convention
# differs (row-based 6D, another euler order or an extrinsic one) must convert on
# the bridge before it publishes.
string rotation_type

# Joint names in the order arm_actions columns appear. Empty means the robot's
# canonical order. When supplied it must name every joint exactly once; a partial
# or duplicated list is rejected rather than silently dropping or double-driving
# a joint.
string[] joint_names

int32 chunk_size

# Time between consecutive waypoints in this chunk [s]. Must be positive: it is
# a divisor in the playback path. A chunk with control_dt <= 0 is rejected
# (telemetry last_reject "control_dt"), not timed from inference_frequency or
# anything else the controller could invent, so a bridge must set it.
float64 control_dt

# [chunk_size x dim] row-major. dim is 7 for joint space, or 3 + rotation size
# for task space.
float64[] arm_actions

# Optional [chunk_size x 7] joint velocity reference. Joint space only: a
# task-space entry in this shape would have to encode a rotation derivative,
# which none of the bridged stacks emits. Empty means the controller finite-
# differences the position waypoints instead.
float64[] arm_velocities

# "binary" (< 0 close, > 0 open, 0 = no change) or "continuous" (opening ratio
# in [0, 1]). Empty = "binary", the pre-v2 behaviour.
string gripper_mode
float32[] gripper_actions
