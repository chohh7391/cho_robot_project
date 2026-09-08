# A chunk of policy actions, as produced by a VLA inference bridge.
#
# stamp is the OBSERVATION time: the bridge must echo the header.stamp of the
# joint state it built its observation from, verbatim. Not its own wall clock --
# this repo's multi-PC setup (FastDDS discovery server) has no clock sync, so a
# bridge clock is a different time domain than the controller's. Echoing the
# controller's own stamp makes time alignment work without any sync.
# All-zero means "unstamped": the controller falls back to arrival time, which
# reproduces the pre-v2 behaviour.
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
#                   chunk integrates from the anchor (OpenVLA-OFT LIBERO EE
#                   deltas; pi0 DROID joint velocity times control_dt)
# Empty falls back to the `relative` bool below.
string relative_mode
# Pre-v2 field, kept for compatibility. Only read when relative_mode is empty:
# false -> "absolute", true -> "from_anchor".
bool relative

# Task space only. One of "euler", "axis_angle", "quaternion", "rotation6d".
# Quaternion order is (x, y, z, w). An unrecognised value is REJECTED, never
# guessed -- guessing drives the arm to an invented orientation.
string rotation_type

# Joint names in the order arm_actions columns appear. Empty means the robot's
# canonical order. When supplied it must name every joint exactly once; a partial
# or duplicated list is rejected rather than silently dropping or double-driving
# a joint.
string[] joint_names

int32 chunk_size

# Time between consecutive waypoints in this chunk [s]. Must be positive: it is
# a divisor in the playback path. <= 0 falls back to
# (1 / inference_frequency) / chunk_size, i.e. the chunk spans one inference
# period, and the controller warns -- bridges should set it explicitly.
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
