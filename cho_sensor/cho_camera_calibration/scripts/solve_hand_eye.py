#!/usr/bin/env python3
"""Solve AX = ZB: a hand-eye transform AND the target's pose, together.

    A_i  base -> arm frame        the robot's forward kinematics
    B_i  camera -> board          PnP over every tag corner, per pose
    X    arm frame -> camera      the hand-eye, unknown
    Z    base -> board            the board's pose, ALSO unknown

Z being an unknown is the point. Where a target was placed is a human
placement: measured on this bench, the same board taped twice at the same
nominal spot moved the solved camera 52 mm, and when the robot finally
measured where it actually was, it sat 48 mm from where it had been put. A
procedure that takes the placement as input inherits all of that silently.

Park & Martin's closed form, over every pair of poses. No initial guess, which
matters when a mount has been changed and the recorded extrinsic describes the
old one.

    ros2 run cho_camera_calibration solve_hand_eye.py DATA.json
        --board <share>/cho_camera_calibration/config/tag_board_70mm.yaml
        --moving-info /wrist/wrist/infra1/camera_info
        --static-info side_1=/side_1/left/camera_info
                      side_2=/side_2/side_2/infra1/camera_info

(one command; the wrapped lines are its arguments)

The cameras must still be publishing camera_info; nothing else need be running.
"""
import argparse
import itertools
import json
import math

import cv2
import numpy as np
import rclpy
import yaml

ZERO_D = np.zeros(5)


def board_points(spec):
    """Place the tag centres in the board frame, from the config layout.

    Origin at the midpoint of the two middle tags, x along the columns, y up
    the rows. The board frame is a convention, not a measurement -- the solve
    finds where it sits -- but the ids and the geometry have to agree on it.
    """
    pitch, cols, rows = spec['pitch_m'], spec['columns'], spec['rows']
    ids, out = spec['ids'], {}
    for index, tag in enumerate(ids):
        col, row = index % cols, index // cols
        out[int(tag)] = (pitch * (col - (cols - 1) / 2.0),
                         pitch * ((rows - 1) / 2.0 - row))
    return out


def corner_orders(half):
    """Build the eight plausible corner orders: four rotations, two handedness.

    apriltag's ordering is not worth guessing at, so every candidate is tried
    and the one that reprojects best wins. The margin is printed: on this
    bench the winner sat at 0.67 px and the runner-up at 56, which is the
    difference between a settled convention and a hopeful one.
    """
    ccw = [(-half, -half), (+half, -half), (+half, +half), (-half, +half)]
    orders = []
    for flip in (False, True):
        base = [(x, -y) for x, y in ccw] if flip else ccw
        for shift in range(4):
            orders.append(base[shift:] + base[:shift])
    return orders


def board_pnp(corners, K, order, centres):
    """Solve camera -> board, and report the reprojection error in pixels."""
    obj, img = [], []
    for tag, quad in corners.items():
        if int(tag) not in centres:
            continue
        cx, cy = centres[int(tag)]
        for (ox, oy), (u, v) in zip(order, quad):
            obj.append([cx + ox, cy + oy, 0.0])
            img.append([u, v])
    if len(obj) < 8:
        return None, None
    obj = np.array(obj, dtype=np.float64)
    img = np.array(img, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, ZERO_D,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, ZERO_D)
    error = float(np.sqrt(np.mean(np.sum(
        (projected.reshape(-1, 2) - img) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, error


def R_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
             (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
             (R[0, 2] - R[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
             (R[1, 0] - R[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def se3(vec):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(np.array(vec[3:]))
    T[:3, 3] = vec[:3]
    return T


def log_so3(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    angle = math.acos(c)
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis * (angle / (2.0 * math.sin(angle)))


def intrinsics(node, topic):
    """Read the K a camera is publishing, as a matrix.

    Both streams here are RECTIFIED -- the D435's infra1 arrives that way and
    image_proc rectifies the OAK's wide mono ahead of the detector -- so the
    distortion is zero and K is the right matrix. Feeding a rational_polynomial
    D alongside an already-rectified image would undistort it twice.
    """
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    got = {}
    node.create_subscription(CameraInfo, topic,
                             lambda m: got.setdefault('m', m),
                             qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + 15e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < end and not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        raise SystemExit(f'no camera_info on {topic}')
    k = got['m'].k
    return np.array([[k[0], 0.0, k[2]], [0.0, k[4], k[5]], [0.0, 0.0, 1.0]])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('data', help='what record_board_views.py wrote')
    parser.add_argument('--board', required=True, help='the board config')
    parser.add_argument('--moving-info', required=True,
                        help="camera_info of the camera ON the arm")
    parser.add_argument('--static-info', nargs='*', default=[],
                        metavar='NAME=TOPIC',
                        help='camera_info of each fixed camera to solve as well, '
                             'named as in the recording')
    args = parser.parse_args()

    spec = yaml.safe_load(open(args.board, encoding='utf-8'))
    centres = board_points(spec)
    orders = corner_orders(spec['tag_size_m'] / 2.0)
    records = json.load(open(args.data, encoding='utf-8'))

    rclpy.init()
    node = rclpy.create_node('solve_hand_eye')
    K_moving = intrinsics(node, args.moving_info)
    K_static = {}
    for item in args.static_info:
        label, _, topic = item.partition('=')
        if not label or not topic:
            raise SystemExit(f'--static-info wants NAME=TOPIC, got {item!r}')
        K_static[label] = intrinsics(node, topic)
    node.destroy_node()
    rclpy.shutdown()

    # ---- settle the corner order on the data itself ---------------------
    scores = []
    for index, order in enumerate(orders):
        errs = [board_pnp(r['moving_corners'], K_moving, order, centres)[1]
                for r in records if r['moving_corners']]
        errs = [e for e in errs if e is not None]
        scores.append((float(np.mean(errs)) if errs else 1e9, index))
    scores.sort()
    order = orders[scores[0][1]]
    print(f'corner order {scores[0][1]} reprojects at {scores[0][0]:.3f} px, '
          f'next best {scores[1][0]:.3f} px -- chosen by '
          f'{scores[1][0] / max(scores[0][0], 1e-9):.0f}x\n')

    A, B, errs = [], [], []
    for rec in records:
        if rec['arm'] is None or not rec['moving_corners']:
            continue
        T, err = board_pnp(rec['moving_corners'], K_moving, order, centres)
        if T is None:
            continue
        A.append(se3(rec['arm']))
        B.append(T)
        errs.append(err)
        print(f'  {rec["name"]:<6s} {len(rec["moving_corners"])} tags, '
              f'reprojection {err:.3f} px')
    if len(A) < 3:
        raise SystemExit('need at least three poses that saw the board')
    print(f'\n{len(A)} poses, reprojection {min(errs):.3f}-{max(errs):.3f} px')

    # ---- Park & Martin --------------------------------------------------
    M = np.zeros((3, 3))
    pairs = []
    for i, j in itertools.combinations(range(len(A)), 2):
        A_bar = np.linalg.inv(A[j]) @ A[i]
        B_bar = B[j] @ np.linalg.inv(B[i])
        alpha, beta = log_so3(A_bar[:3, :3]), log_so3(B_bar[:3, :3])
        if np.linalg.norm(alpha) < 0.15 or np.linalg.norm(beta) < 0.15:
            continue          # too little rotation between them to say anything
        pairs.append((A_bar, B_bar))
        M += np.outer(beta, alpha)
    print(f'{len(pairs)} pose pairs carry a real rotation')
    if len(pairs) < 3:
        raise SystemExit('the poses barely rotate; see the README')

    w, v = np.linalg.eigh(M.T @ M)
    R_X = v @ np.diag(1.0 / np.sqrt(w)) @ v.T @ M.T
    C = np.vstack([a[:3, :3] - np.eye(3) for a, _b in pairs])
    d = np.concatenate([R_X @ b[:3, 3] - a[:3, 3] for a, b in pairs])
    t_X, *_ = np.linalg.lstsq(C, d, rcond=None)
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R_X, t_X
    q = R_to_quat(R_X)
    print('\nX  arm frame -> moving camera OPTICAL frame:')
    print(f'    xyz: [{t_X[0]:.5f}, {t_X[1]:.5f}, {t_X[2]:.5f}]')
    print(f'    quaternion: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]')

    Zs = [A[i] @ X @ B[i] for i in range(len(A))]
    pos = np.array([Z[:3, 3] for Z in Zs])
    Z_pos = pos.mean(axis=0)
    spread = np.linalg.norm(pos - Z_pos, axis=1)
    R_Z = Zs[int(np.argmin(spread))][:3, :3]
    print('\nZ  base -> board, an output rather than a measurement:')
    print(f'    xyz: [{Z_pos[0]:.5f}, {Z_pos[1]:.5f}, {Z_pos[2]:.5f}]')
    print(f'    yaw {math.degrees(math.atan2(R_Z[1, 0], R_Z[0, 0])):+.2f} deg, '
          f'plane {math.degrees(math.acos(min(1.0, abs(R_Z[2, 2])))):.2f} deg '
          f'off level')
    print(f'    agreement across poses: {1000 * spread.mean():.2f} mm mean, '
          f'{1000 * spread.max():.2f} mm worst')
    print('    -- if the board lies on a surface of known height, its solved z '
          'is a free check:\n       nothing here knows that height.')

    if not K_static:
        return
    # Each fixed camera is solved SEPARATELY against the one board pose above.
    # Nothing ties them to each other, which is what makes printing both worth
    # doing: two cameras solved off the same board that then disagree about
    # where anything is have an extrinsic problem between them -- and that is
    # exactly the error crossing their lines of sight is sensitive to.
    Z_full = np.eye(4)
    Z_full[:3, :3], Z_full[:3, 3] = R_Z, Z_pos
    for label, K in K_static.items():
        cams, oerrs = [], []
        for rec in records:
            corners = (rec.get('static_corners') or {}).get(label)
            if not corners:
                continue
            T, err = board_pnp(corners, K, order, centres)
            if T is None:
                continue
            cams.append(Z_full @ np.linalg.inv(T))
            oerrs.append(err)
        print(f'\nbase -> {label} OPTICAL frame:')
        if not cams:
            print(f'    NOTHING RECORDED under {label!r} -- check the name '
                  'matches the one given to --static-detections.')
            continue
        cpos = np.array([c[:3, 3] for c in cams])
        mean = cpos.mean(axis=0)
        best = int(np.argmin(np.linalg.norm(cpos - mean, axis=1)))
        q = R_to_quat(cams[best][:3, :3])
        print(f'    xyz: [{mean[0]:.5f}, {mean[1]:.5f}, {mean[2]:.5f}]')
        print(f'    quaternion: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]')
        print(f'    from {len(cams)} views, spread '
              f'{1000 * np.linalg.norm(cpos - mean, axis=1).max():.2f} mm, '
              f'reprojection {min(oerrs):.3f}-{max(oerrs):.3f} px')
    print('\nEVERY POSE ABOVE IS AN OPTICAL FRAME. camera_extrinsics.yaml names '
          "each driver's\nown root, so compose each with its driver's internal "
          'chain before writing it there.')


main()
