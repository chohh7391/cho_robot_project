"""Mount-agnostic geometry behind an AprilTag object-pose estimate.

Nothing here imports ROS, and nothing here knows where the camera is. Every
function takes poses that are already expressed in one common frame -- the
caller gets them there with tf2 -- so the same code serves a camera bolted to
the wrist and a camera on a tripod. That is the whole reason this file is
separate from ``node.py``: the part worth testing does not need a camera, a
robot, or a clock.

Quaternions are ``[x, y, z, w]`` numpy arrays, matching ``geometry_msgs`` and
the ``orientation`` lists in ``cho_robot_config`` (whose top-down default is
``[1, 0, 0, 0]``, i.e. Rx(pi)).
"""

from collections import namedtuple
import math

import numpy as np

# Tool pointing straight down: Rx(pi). Keeps the tool x-axis along base +x, so
# a yaw applied on top of it is a yaw in the base frame.
TOP_DOWN = np.array([1.0, 0.0, 0.0, 0.0])

TAG_FRAME_PREFIX = 'tag_'


def tag_frame_name(tag_id, prefix=''):
    """TF frame apriltag_ros is configured to publish for *tag_id*.

    The detector's ``tag.frames`` list is derived from the same convention in
    ``realsense_apriltag/launch/apriltag.launch.py``. Neither package writes a
    frame string by hand, so they cannot drift apart.

    *prefix* exists for a second camera. Two detector instances left at the
    default would both publish ``tag_9``, which gives one TF child two parents
    and corrupts the whole tree -- not a name clash that shows up as an error,
    but as transforms that intermittently resolve through the wrong camera.
    Give each camera its own prefix (``cam0_``) and the two sets stay apart.
    """
    return f'{prefix}{TAG_FRAME_PREFIX}{int(tag_id)}'


# ---------------------------------------------------------------- quaternions

def quat_normalize(q):
    """*q* scaled to unit length; raises if it is degenerate."""
    q = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError(f'quaternion is not usable: {q}')
    return q / norm


def quat_multiply(a, b):
    """Hamilton product, i.e. the rotation "*a* then *b* applied in a's frame"."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_conjugate(q):
    """Inverse rotation of a unit quaternion."""
    x, y, z, w = q
    return np.array([-x, -y, -z, w])


def quat_rotate(q, v):
    """Rotate the 3-vector *v* by unit quaternion *q*."""
    u = np.asarray(q[:3], dtype=float)
    w = float(q[3])
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def quat_angle(a, b):
    """Geodesic angle in radians between two unit quaternions.

    Uses ``|dot|`` so that q and -q -- the same rotation -- compare as equal.
    """
    dot = abs(float(np.dot(quat_normalize(a), quat_normalize(b))))
    return 2.0 * math.acos(min(1.0, dot))


def quat_from_yaw(yaw):
    """Rotation of *yaw* radians about the base z-axis."""
    return np.array([0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)])


# --------------------------------------------------------------- decode gate

QualityGate = namedtuple('QualityGate', 'max_hamming min_decision_margin min_edge_px')

DEFAULT_GATE = QualityGate(max_hamming=0, min_decision_margin=35.0, min_edge_px=25.0)


def corner_min_edge_px(corners):
    """Shortest side of the detected quad, in pixels.

    The shortest side, not the area: an obliquely viewed tag can cover plenty
    of pixels while one of its sides is a handful across, and it is that side
    that decides how well the pose is constrained.
    """
    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    edges = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
    return float(min(edges))


def detection_reject_reason(hamming, decision_margin, corners, gate=DEFAULT_GATE):
    """None if the detection is worth using, else why it is not.

    A rejected detection is dropped rather than published with a low score:
    ``PoseTargetBehavior`` latches the *first* pose it sees, so a bad pose that
    reaches the topic has already been obeyed by the time anything could
    reconsider it.
    """
    if hamming > gate.max_hamming:
        return f'hamming {hamming} > {gate.max_hamming} (bits had to be corrected)'
    if decision_margin < gate.min_decision_margin:
        return f'decision_margin {decision_margin:.1f} < {gate.min_decision_margin:.1f}'
    edge = corner_min_edge_px(corners)
    if edge < gate.min_edge_px:
        return f'shortest tag edge {edge:.1f} px < {gate.min_edge_px:.1f} (too far or too oblique)'
    return None


# --------------------------------------------------------------- aggregation

Aggregate = namedtuple(
    'Aggregate', 'position orientation position_spread_m orientation_spread_rad count')


def aggregate_samples(positions, orientations):
    """Combine repeated observations of one tag into a single pose estimate.

    Position is the component-wise median and orientation is the *medoid* --
    the observed orientation closest to all the others. Neither is an average
    on purpose: AprilTag's planar pose has a two-solution ambiguity that makes
    the orientation flip between frames, and averaging across a flip returns a
    rotation that was never observed and is not close to either solution. A
    medoid always returns something the camera actually saw.

    The returned spreads are what the caller gates on: they are the honest
    measure of whether the flip is happening right now.
    """
    positions = np.asarray(positions, dtype=float).reshape(-1, 3)
    orientations = [quat_normalize(q) for q in orientations]
    if len(positions) != len(orientations) or not len(positions):
        raise ValueError('need the same non-zero number of positions and orientations')

    median = np.median(positions, axis=0)
    position_spread = float(max(np.linalg.norm(p - median) for p in positions))

    total = [sum(quat_angle(a, b) for b in orientations) for a in orientations]
    medoid = orientations[int(np.argmin(total))]
    orientation_spread = float(max(quat_angle(medoid, q) for q in orientations))

    return Aggregate(median, medoid, position_spread, orientation_spread, len(positions))


# ------------------------------------------------------------------ composing

def compose(position, orientation, offset_position, offset_orientation):
    """Apply a tag-frame offset to a tag pose: T_base_grasp = T_base_tag * T_tag_grasp."""
    orientation = quat_normalize(orientation)
    moved = np.asarray(position, dtype=float) + quat_rotate(orientation, offset_position)
    return moved, quat_normalize(quat_multiply(orientation, offset_orientation))


def fold_yaw(yaw):
    """Fold *yaw* into (-pi/2, pi/2].

    A parallel gripper closing on an axis is unchanged by a half turn, so the
    two yaws are equivalent grasps -- but only one of them avoids driving the
    wrist most of the way round to reach it.
    """
    folded = (yaw + math.pi / 2.0) % math.pi - math.pi / 2.0
    return math.pi / 2.0 if math.isclose(folded, -math.pi / 2.0) else folded


def top_down_from_yaw_axis(orientation, yaw_axis=(1.0, 0.0, 0.0), min_planar_norm=0.2,
                           fold=True):
    """Keep only the tag's yaw and approach straight down, or return None.

    This exists because the tag's *position* is trustworthy and its *full
    orientation* is not (see aggregate_samples). Keeping the yaw and throwing
    the rest away turns a two-solution ambiguity into a single well-posed
    number, at the cost of assuming a top-down approach.

    None is returned when *yaw_axis* points too close to vertical for its
    projection on the base xy-plane to define a yaw at all -- a tag seen almost
    edge-on. The caller must treat that as a rejection, not as zero yaw.
    """
    axis_in_base = quat_rotate(quat_normalize(orientation), yaw_axis)
    planar = axis_in_base[:2]
    if float(np.linalg.norm(planar)) < min_planar_norm:
        return None
    yaw = math.atan2(float(planar[1]), float(planar[0]))
    if fold:
        yaw = fold_yaw(yaw)
    return quat_multiply(quat_from_yaw(yaw), TOP_DOWN)
