"""Waypoint files and arm-profile names for the OpenArm task-space tools.

No ROS here, so the parsing is unit-testable. A waypoint file is a YAML list::

    - name: top
      joints: [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0]      # FK'd at run time
    - name: side
      position: [0.37, -0.49, 0.44]                          # model-root frame, metres
      orientation: [0.7852, 0.1335, 0.4814, 0.3658]          # quaternion x y z w
    - name: tilted
      position: [0.40, -0.15, 0.48]
      rpy: [-0.785, -1.571, -2.356]                          # radians, alternative to orientation

Joint-defined waypoints are the safe way to write a far target: the pose is
the forward kinematics of a posture inside the joint limits, so it is
reachable by construction and its orientation is consistent with the arm.
"""

import math
import os

import yaml

# Fallback for a caller with no installed description (unit tests). These are
# the SINGLE-arm mount's limits; the bimanual torso's two arms do not share
# them, which is what profile_joint_limits() exists to resolve.
JOINT_LIMITS_LOWER = [-1.396263, -1.745329, -1.570796, 0.0, -1.570796, -0.785398, -1.570796]
JOINT_LIMITS_UPPER = [3.490659, 1.745329, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796]

DEFAULT_SAFETY_PROFILE = 'real_conservative_commissioning'


def profile_joint_limits(arm, profile=DEFAULT_SAFETY_PROFILE, path=None):
    """The joint window this arm is actually gated against, from the MIT profile.

    The bimanual torso rolls each arm's joint 2 frame and shifts the left arm's
    joint 1, so the two arms have different windows and the single-arm window
    fits neither. Reading them from the safety profile keeps this tool and the
    controller that will reject the goal quoting the same numbers.
    """
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory('cho_description_openarm'),
            'config', 'mit_safety_profiles_v1.yaml')
    with open(path) as handle:
        limits = yaml.safe_load(handle)['profiles'][profile]['joint_limits']
    prefix = '' if arm in ('single', '', None) else f'{arm}_'
    lower = limits.get(f'{prefix}position_lower')
    upper = limits.get(f'{prefix}position_upper')
    if lower is None or upper is None:
        raise ValueError(f"safety profile '{profile}' has no joint window for arm '{arm}'")
    return [float(v) for v in lower], [float(v) for v in upper]


def arm_names(arm):
    """Topic, service, action and model names for one arm profile."""
    if arm not in ('single', 'left', 'right'):
        raise ValueError(f"arm must be single, left or right (got '{arm}')")
    prefix = '' if arm == 'single' else f'{arm}_'
    joint_prefix = 'openarm_' if arm == 'single' else f'openarm_{arm}_'
    controller = f'{prefix}task_space_impedance_mit_controller'
    return {
        'controller': controller,
        'action': f'/controller_action_server/{controller}',
        'diagnostics': f'/{controller}/task_diagnostics',
        'protocol_status': f'/{controller}/protocol_status',
        'pose_topic': '/ee_state/pose' if arm == 'single' else f'/ee_state/{arm}/pose',
        'joints': [f'{joint_prefix}joint{i}' for i in range(1, 8)],
        'ee_frame': f'{joint_prefix}hand_tcp',
    }


def rpy_to_quaternion(roll, pitch, yaw):
    """ZYX Euler angles to a quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def normalized(quaternion):
    norm = math.sqrt(sum(v * v for v in quaternion))
    if norm < 1e-9:
        raise ValueError('quaternion has zero norm')
    return tuple(v / norm for v in quaternion)


def load_waypoints(path, limits=None):
    """Read and validate a waypoint file.

    Returns a list of dicts with a `name` and either `joints` (7 values inside
    this arm's window) or `position` + `orientation`. `limits` is the
    (lower, upper) pair from profile_joint_limits(); without it the single-arm
    window is assumed, which is wrong for either arm of the torso.
    """
    lower_limits, upper_limits = limits if limits else (JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    with open(path) as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, list) or not raw:
        raise ValueError(f'{path}: expected a non-empty YAML list of waypoints')
    out = []
    for index, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise ValueError(f'{path}: waypoint {index} is not a mapping')
        name = str(entry.get('name', f'wp{index}'))
        if 'joints' in entry:
            joints = [float(v) for v in entry['joints']]
            if len(joints) != 7:
                raise ValueError(f'{path}: waypoint {name} needs 7 joint values, got {len(joints)}')
            for i, (q, lo, hi) in enumerate(zip(joints, lower_limits, upper_limits)):
                if not lo <= q <= hi:
                    raise ValueError(
                        f'{path}: waypoint {name} joint {i + 1} = {q} is outside [{lo}, {hi}]')
            out.append({'name': name, 'joints': joints})
            continue
        if 'position' not in entry:
            raise ValueError(f'{path}: waypoint {name} needs either joints or position')
        position = [float(v) for v in entry['position']]
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError(f'{path}: waypoint {name} position needs 3 finite values')
        if 'orientation' in entry:
            orientation = [float(v) for v in entry['orientation']]
            if len(orientation) != 4:
                raise ValueError(f'{path}: waypoint {name} orientation needs 4 values (x y z w)')
            orientation = normalized(orientation)
        elif 'rpy' in entry:
            rpy = [float(v) for v in entry['rpy']]
            if len(rpy) != 3:
                raise ValueError(f'{path}: waypoint {name} rpy needs 3 values')
            orientation = rpy_to_quaternion(*rpy)
        else:
            raise ValueError(f'{path}: waypoint {name} needs orientation (x y z w) or rpy')
        out.append({'name': name, 'position': position, 'orientation': list(orientation)})
    return out


def resolve_waypoints(waypoints, forward_kinematics):
    """Turn every waypoint into (name, position, orientation).

    `forward_kinematics(joints) -> (position, orientation_xyzw)` is only called
    for joint-defined waypoints, so a pose-only file needs no robot model.
    """
    resolved = []
    for waypoint in waypoints:
        if 'joints' in waypoint:
            if forward_kinematics is None:
                raise ValueError(f"waypoint {waypoint['name']} is joint-defined but no model is available")
            position, orientation = forward_kinematics(waypoint['joints'])
            resolved.append((waypoint['name'], [float(v) for v in position],
                             [float(v) for v in normalized(orientation)]))
        else:
            resolved.append((waypoint['name'], waypoint['position'], waypoint['orientation']))
    return resolved


def pose_error(target_position, target_orientation, measured_position, measured_orientation):
    """Translation error vector (m) and rotation angle (rad) between two poses."""
    translation = [t - m for t, m in zip(target_position, measured_position)]
    tx, ty, tz, tw = normalized(target_orientation)
    mx, my, mz, mw = normalized(measured_orientation)
    # |dot| of unit quaternions gives half the angle between the rotations.
    dot = abs(tx * mx + ty * my + tz * mz + tw * mw)
    angle = 2.0 * math.acos(min(1.0, dot))
    return translation, angle
