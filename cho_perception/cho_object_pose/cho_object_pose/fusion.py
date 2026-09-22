"""Combine several cameras' views of one tag into a single pose. No ROS here.

This is a SEPARATE step from :func:`geometry.aggregate_samples`, and the split
matters. That function combines repeated looks by ONE camera, where the samples
differ by noise; this one combines the per-camera results, where they differ by
each camera's own systematic range error. Averaging is right for the first and
is the whole question for the second.

WHY THE QUESTION IS INTERESTING. A planar marker's error is not isotropic. The
corners are located to a fixed fraction of a pixel, so the TRANSVERSE error is
range/focal metres; the RANGE error comes from how much the tag's apparent
shape changes with distance, which for a tag of side `s` seen at range `r` is
worse by about `r/s`. Measured on this cell: transverse 0.03-0.06 mm against a
total of 10-16 mm, i.e. the error is essentially 100% along the viewing ray.

    The bearing is right. Only the distance is wrong.

Two rules follow, and this module offers both:

``intersect`` (the default)
    Each camera contributes a RAY -- its optical centre and the direction it
    saw the tag in -- and the pose is the point closest to all of them in a
    least-squares sense. This throws each camera's bad number away and keeps
    its good one. Measured against the alternative below on this cell's
    geometry: 0.06 mm against 11.28 mm. The gain is a RATIO and survives any
    noise scale, because both sides are proportional to the same sigma.

``inverse_distance``
    The published method: weight each camera by the inverse square of its
    distance to the marker, average the translations and SLERP the rotations.
    Kept so the paper's own Perception Module can be reproduced and measured on
    this bench rather than argued about. See the note on its own function --
    it is not a worse implementation of the same idea, it is a different
    estimator, and on this geometry it came out BEHIND the better of the two
    single cameras (11.28 mm against 10.15 mm).

``median``
    What this package did before either existed: the component-wise median over
    every sample from every camera at once, with no per-camera step. Kept
    because a bench running it should be able to keep running it.

ORIENTATION IS NEVER INTERPOLATED except by ``inverse_distance``, whose
published definition says to. AprilTag's planar pose has a two-solution
ambiguity that flips between frames; interpolating across a flip returns a
rotation neither camera saw and which is close to neither solution. The other
modes take the MEDOID -- the observed orientation closest to all the others --
which always returns something a camera actually reported.
"""

from collections import namedtuple
import math

import numpy as np

from cho_object_pose.geometry import quat_angle, quat_normalize

#: One camera's finished word about one tag, in the robot's base frame.
#: ``origin`` is that camera's optical centre, and it is what separates a ray
#: from a point -- without it only ``median`` can run.
CameraEstimate = namedtuple('CameraEstimate', 'camera position orientation origin')
CameraEstimate.__new__.__defaults__ = (None,)

#: The fused pose, plus how it was reached. ``residual_m`` is the fit's own
#: error measure and differs by mode: for ``intersect`` it is how far the rays
#: missed each other, which is a direct check on the EXTRINSICS; for the others
#: it is how far the camera estimates lie from the result, which mixes the
#: extrinsics with each camera's range error and cannot separate them.
Fused = namedtuple('Fused', 'position orientation mode residual_m detail')

INTERSECT = 'intersect'
INVERSE_DISTANCE = 'inverse_distance'
MEDIAN = 'median'
MODES = (INTERSECT, INVERSE_DISTANCE, MEDIAN)
DEFAULT_MODE = INTERSECT

#: Below this angle between two cameras' lines of sight, intersecting them is
#: ill-conditioned: the crossing point slides a long way for a small angular
#: error, which is the one error a marker does NOT make but an extrinsic does.
#: The cell this was written for has 53.7 degrees between its standing views.
DEFAULT_MIN_RAY_ANGLE_RAD = math.radians(10.0)

#: The paper's regularisation constant, spelled the way it is spelled there.
EPSILON = 1e-6


def medoid_orientation(orientations):
    """Return the orientation closest to all the others, and never an average.

    See the module note: averaging across AprilTag's two-solution flip returns
    a rotation nobody saw.
    """
    quats = [quat_normalize(q) for q in orientations]
    total = [sum(quat_angle(a, b) for b in quats) for a in quats]
    return quats[int(np.argmin(total))]


def max_ray_angle(estimates):
    """Return the widest angle between any two cameras' lines of sight, in radians.

    THE NUMBER THAT SAYS WHETHER INTERSECTION IS WORTH DOING. Two cameras
    standing beside each other see a tag along nearly the same ray, and
    crossing those rays estimates the range from almost nothing.
    """
    bearings = [_bearing(estimate) for estimate in estimates]
    bearings = [b for b in bearings if b is not None]
    widest = 0.0
    for index, first in enumerate(bearings):
        for second in bearings[index + 1:]:
            cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
            widest = max(widest, math.acos(cosine))
    return widest


def _bearing(estimate):
    """Return the unit vector from the camera's optical centre to the tag, or None."""
    if estimate.origin is None:
        return None
    ray = np.asarray(estimate.position, dtype=float) - np.asarray(estimate.origin, dtype=float)
    length = float(np.linalg.norm(ray))
    if length < 1e-9:
        return None
    return ray / length


def intersect_rays(estimates):
    """Least-squares closest point to every camera's line of sight.

    Minimises the sum of squared PERPENDICULAR distances to the rays, which is
    the maximum-likelihood point when each camera's error is along its own ray
    and its transverse errors are comparable -- the measured situation here.

    Closed form: with ``P_i = I - d_i d_i^T`` projecting onto the plane across
    ray *i*, the point solves ``(sum P_i) p = sum P_i o_i``.

    Returns ``(point, residual_m)``, or ``(None, None)`` when fewer than two
    cameras carry an origin or the normal matrix is singular.
    """
    rays = [(np.asarray(e.origin, dtype=float), _bearing(e)) for e in estimates
            if e.origin is not None]
    rays = [(origin, bearing) for origin, bearing in rays if bearing is not None]
    if len(rays) < 2:
        return None, None

    normal = np.zeros((3, 3))
    target = np.zeros(3)
    for origin, bearing in rays:
        projector = np.eye(3) - np.outer(bearing, bearing)
        normal += projector
        target += projector @ origin
    # Rank-deficient exactly when every ray is parallel; the caller has already
    # gated on the angle, so this is belt and braces rather than the real check.
    if np.linalg.matrix_rank(normal, tol=1e-9) < 3:
        return None, None
    point = np.linalg.solve(normal, target)

    misses = [float(np.linalg.norm((np.eye(3) - np.outer(bearing, bearing)) @ (point - origin)))
              for origin, bearing in rays]
    return point, float(np.sqrt(np.mean(np.square(misses))))


def inverse_distance_weights(estimates):
    """Return the published weights ``W_c = d_c^-2 / (sum_j d_j^-2 + eps)``.

    ``d_c`` is the camera-to-marker distance, which is what the paper records
    as its reliability metric. Needs every camera's origin; without one the
    distance is not knowable here.
    """
    distances = [float(np.linalg.norm(np.asarray(e.position, dtype=float)
                                      - np.asarray(e.origin, dtype=float)))
                 if e.origin is not None else None for e in estimates]
    if any(d is None for d in distances):
        return None
    raw = np.array([1.0 / (d * d + EPSILON) for d in distances])
    return raw / (raw.sum() + EPSILON)


def _fuse_inverse_distance(estimates):
    """Apply the published inverse-distance rule: weighted mean, then SLERP.

    FAITHFUL TO THE PAPER FOR TWO CAMERAS, which is the case it defines
    (``c in {1, 2}``). With more, the same weights are applied to the positions
    and the orientation falls back to the medoid, because SLERP is a two-term
    operation and chaining it is not what was published; the detail string says
    so rather than letting an extension pass for the method.
    """
    weights = inverse_distance_weights(estimates)
    if weights is None:
        return None
    positions = np.array([np.asarray(e.position, dtype=float) for e in estimates])
    position = (positions * weights[:, None]).sum(axis=0)

    if len(estimates) == 2:
        orientation = _slerp(quat_normalize(estimates[0].orientation),
                             quat_normalize(estimates[1].orientation),
                             float(weights[1]))
        detail = (f'inverse-distance weights '
                  f'{estimates[0].camera} {weights[0]:.2f} / '
                  f'{estimates[1].camera} {weights[1]:.2f}, SLERP orientation')
    else:
        orientation = medoid_orientation([e.orientation for e in estimates])
        detail = (f'inverse-distance weights over {len(estimates)} cameras; '
                  'medoid orientation (SLERP is defined for two)')
    residual = float(max(np.linalg.norm(p - position) for p in positions))
    return Fused(position, orientation, INVERSE_DISTANCE, residual, detail)


def _slerp(first, second, fraction):
    """Interpolate between two quaternions along the shorter arc."""
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second, dot = -second, -dot
    dot = min(1.0, max(-1.0, dot))
    # Nearly parallel: the arc is shorter than the numerics, so lerp instead.
    if dot > 1.0 - 1e-9:
        return quat_normalize(first + fraction * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return quat_normalize((math.sin((1.0 - fraction) * angle) / sine) * first
                          + (math.sin(fraction * angle) / sine) * second)


def _fuse_median(estimates):
    """Take the component-wise median of the camera estimates, medoid orientation."""
    positions = np.array([np.asarray(e.position, dtype=float) for e in estimates])
    position = np.median(positions, axis=0)
    orientation = medoid_orientation([e.orientation for e in estimates])
    residual = float(max(np.linalg.norm(p - position) for p in positions))
    return Fused(position, orientation, MEDIAN, residual,
                 f'median over {len(estimates)} camera(s)')


def fuse(estimates, mode=DEFAULT_MODE, min_ray_angle_rad=DEFAULT_MIN_RAY_ANGLE_RAD):
    """Combine per-camera estimates into one pose by the named *mode*.

    ONE CAMERA IS NOT A FUSION and every mode returns its estimate unchanged --
    there is nothing to weight, cross or outvote, and the published method says
    the same ("its estimate is used directly without fusion").

    ``intersect`` falls back to ``inverse_distance`` when the geometry cannot
    support it -- no optical centres configured, or every line of sight within
    *min_ray_angle_rad* of every other. The fallback is named in ``detail``, so
    a bench that quietly stopped intersecting says so in its own status line
    instead of just getting worse.
    """
    if mode not in MODES:
        raise ValueError(f'unknown fusion mode {mode!r}; expected one of {", ".join(MODES)}')
    estimates = list(estimates)
    if not estimates:
        raise ValueError('need at least one camera estimate to fuse')
    if len(estimates) == 1:
        only = estimates[0]
        return Fused(np.asarray(only.position, dtype=float),
                     quat_normalize(only.orientation), mode, 0.0,
                     f'{only.camera} only; no fusion')

    if mode == MEDIAN:
        return _fuse_median(estimates)
    if mode == INVERSE_DISTANCE:
        fused = _fuse_inverse_distance(estimates)
        if fused is not None:
            return fused
        return _fuse_median(estimates)._replace(
            detail='no optical_frame configured, so distances are unknown; fell back to median')

    # WHY, before the angle. A camera with no optical_frame contributes no
    # bearing at all, and max_ray_angle would then return 0.0 -- which trips the
    # angle gate and reports "the cameras are in line" for a bench that simply
    # has not been told where its cameras are.
    rays = sum(1 for estimate in estimates if _bearing(estimate) is not None)
    angle = max_ray_angle(estimates)
    point, residual, why = None, None, None
    if rays < 2:
        why = (f'only {rays} of {len(estimates)} cameras declare an optical_frame, '
               'so there are no rays to cross')
    elif angle < min_ray_angle_rad:
        why = (f'lines of sight only {math.degrees(angle):.1f} deg apart '
               f'(want {math.degrees(min_ray_angle_rad):.1f})')
    else:
        point, residual = intersect_rays(estimates)
        if point is None:
            why = 'the lines of sight are degenerate'
    if point is None:
        fallback = _fuse_inverse_distance(estimates) or _fuse_median(estimates)
        return fallback._replace(detail=f'not intersecting: {why}; used {fallback.detail}')

    orientation = medoid_orientation([e.orientation for e in estimates])
    return Fused(point, orientation, INTERSECT, residual,
                 f'intersected {len(estimates)} lines of sight '
                 f'{math.degrees(angle):.1f} deg apart, missing by '
                 f'{residual * 1e3:.2f} mm')
