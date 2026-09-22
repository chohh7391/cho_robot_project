#!/usr/bin/env python3
"""Solve a bench's occlusion-recovery raster into joint waypoints.

``config/sweep/<bench>.yaml`` is a list of joint configurations, which is the
right thing for a tree to read and the wrong thing for a person to write: a
raster is twenty-odd of them and they all have to lie in one IK branch. This
script is how that file is produced, and it is committed beside the file it
produces so a bench that moves can be re-solved rather than re-typed.

    python3 scripts/solve_sweep_raster.py config/sweep/fr5_bench.raster.yaml \\
        --out config/sweep/fr5_bench.yaml

WHAT SHAPE, AND WHY
-------------------
A boustrophedon raster with the tool held vertical: rows at constant x swept
far to near, each row traversed in y, the direction alternating. That is the
classic coverage pattern (Choset's boustrophedon cellular decomposition), and
it is what a person sweeping a bench by hand does.

EVERY LATERAL MOVE HAPPENS AT THE TOP HEIGHT, and the only descent is
vertical, at the end. Shaped like a Gamma and never like an L. Descending
first and then translating drags the gripper across a bench with other
glassware standing on it; translating first and then dropping straight down
passes over everything and only approaches the one place it was sent to. That
is also why the lower pass is a SINGLE cell rather than a row: a row down
there is lateral motion at low clearance, which is the shape being avoided.

IT COVERS AN AREA, NOT A PRESUMED OBJECT POSITION. What is being looked for is
an object whose location is not known -- that is the whole situation, and a
raster centred on where the layout file thinks the tag is would be assuming the
answer. ``area`` is stated in the robot's base frame and the rows and columns
fall out of it.

THE SPACING IS TIGHTER THAN COVERAGE ALONE WOULD ASK FOR, and deliberately. The
textbook rule is ``stripe = footprint x (1 - overlap)``, and a D435's infra
stream sees about 0.6 m across at 0.55 m, so coverage alone would put the rows
0.4 m apart. The spacing here is smaller because the motion has a second job:
PARALLAX -- a different line of sight to the same tag -- since the failure being
recovered from is something standing in the way, and the active-vision result is
that translation only helps when it changes the viewing direction. It also buys
the viewing ANGLE, below. Whichever of the two asks for less spacing wins, and
it is the parallax.

THE LATERAL OFFSET IS ALSO WHAT BUYS THE VIEWING ANGLE. A tag viewed at normal
incidence is the worst case for AprilTag's planar pose, whose two-solution
ambiguity is strongest there -- this repo already knows it, which is why
``top_down_yaw`` exists. Incidence is set by WHERE THE CAMERA IS, not by where
it points, so a raster cell 0.15 m to the side at 0.55 m gives about 15 degrees
of it and the cell directly overhead gives none.

``tilt_deg`` tips the whole tool and defaults to 0 for that reason. Measured in
simulation: tilting 15 degrees left the incidence at the central cell at 1.3
degrees -- unchanged, because the camera was still directly above the tag --
while pushing the tag from 15.1 to 22.2 degrees off the optical axis at an
offset cell, against a guaranteed half-cone of 29. It spent field of view and
bought nothing. Leave it at 0 unless there is a reason the geometry does not
already provide.

Consecutive waypoints are checked ALONG the joint-space straight line between
them, not only at their endpoints. A pair of poses that each clear the bench
can still be joined by an interpolation that dips through it, and that dip is
the collision this pattern exists to avoid.

WHAT THIS DOES NOT DO
---------------------
Nothing adapts. A real next-best-view planner builds a map of the occluder
boundaries and computes a motion that clears them; this is a fixed pattern that
stops at the first viewpoint good enough. The fixed pattern is the right first
step because it needs no occluder model, no belief state and no solver at run
time -- but when the high pass finds the tag at one cell, the low pass still
goes to the middle of the bench rather than back to that cell, and that is the
obvious thing to make adaptive next.
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import yaml

try:
    import pinocchio as pin
except ImportError:  # pragma: no cover - a tool, not a runtime dependency
    sys.exit('pinocchio is needed to solve waypoints: source a ROS environment')


def share(package, *parts):
    """A file inside an installed package.

    Through the ament index rather than a path relative to this script, so the
    spec names packages the way every launch file does and the tool works from
    any directory.
    """
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), *parts)


def build_model(spec):
    """Expand the robot's xacro and add the camera frame the extrinsics name."""
    xacro_args = ' '.join(f'{key}:={value}'
                          for key, value in spec['robot']['xacro_args'].items())
    with tempfile.NamedTemporaryFile('w+', suffix='.urdf', delete=False) as handle:
        urdf_path = handle.name
    command = (f"xacro {share(spec['robot']['package'], spec['robot']['xacro'])} "
               f'{xacro_args}')
    with open(urdf_path, 'w', encoding='utf-8') as handle:
        subprocess.run(command, shell=True, check=True, stdout=handle)
    model = pin.buildModelFromUrdf(urdf_path)
    os.unlink(urdf_path)

    # The camera rides a link, and where it rides is the cell's measured
    # extrinsic -- the same file the static publisher reads, so the poses this
    # solves for and the transform the robot publishes cannot disagree.
    extrinsics = share(spec['camera']['package'], spec['camera']['extrinsics'])
    with open(extrinsics, encoding='utf-8') as handle:
        transforms = yaml.safe_load(handle)['transforms']
    entry = next(item for item in transforms if item['name'] == spec['camera']['name'])
    quaternion = entry['quaternion']
    placement = pin.SE3(
        pin.Quaternion(quaternion[3], quaternion[0], quaternion[1],
                       quaternion[2]).normalized().matrix(),
        np.array(entry['xyz'], dtype=float))

    parent_id = model.getFrameId(entry['parent_frame'])
    parent = model.frames[parent_id]
    camera_id = model.addFrame(pin.Frame(
        'sweep_camera', parent.parentJoint, parent_id,
        parent.placement * placement, pin.FrameType.OP_FRAME))
    return model, camera_id, entry


class Solver:
    """Position + optical-direction IK, with the image roll left free."""

    def __init__(self, model, camera_id, arm_dof):
        self.model = model
        self.data = model.createData()
        self.camera_id = camera_id
        self.dof = arm_dof
        self.lower = model.lowerPositionLimit[:arm_dof].copy()
        self.upper = model.upperPositionLimit[:arm_dof].copy()

    def forward(self, q):
        full = np.zeros(self.model.nq)
        full[:self.dof] = q
        pin.forwardKinematics(self.model, self.data, full)
        pin.updateFramePlacements(self.model, self.data)
        return full

    def solve(self, position, direction, seed, rotation=None, iterations=500):
        """Put the camera at *position* looking along *direction*.

        With *rotation* given the camera's whole attitude is commanded and the
        task is 6-DOF; without it only the optical axis is, and the roll about
        that axis is left free.

        FIXING THE ROLL IS WORTH THE HARDER IK. Left free, the solver picks
        whatever roll is nearest at each cell, so the camera slowly spins as it
        translates: the tool visibly rotates across the sweep, and -- worse --
        which way round the image lies is unknown, so the only field of view
        that can be counted on is the SHORT one. Fixed, the picture keeps one
        orientation, the tool keeps one attitude, and the full rectangle is
        usable.
        """
        q = np.clip(np.asarray(seed, dtype=float).copy(), self.lower, self.upper)
        direction = np.asarray(direction, dtype=float)
        direction = direction / np.linalg.norm(direction)
        error = np.ones(6)
        for _ in range(iterations):
            full = self.forward(q)
            placement = self.data.oMf[self.camera_id]
            if rotation is None:
                # Position, and the rotation that swings the optical axis onto
                # the wanted one. The cross product has no component along the
                # axis, so the roll about it is never commanded.
                error = np.concatenate([
                    position - placement.translation,
                    np.cross(placement.rotation[:, 0], direction)])
                frame = pin.LOCAL_WORLD_ALIGNED
            else:
                error = pin.log6(
                    placement.inverse() * pin.SE3(rotation, np.asarray(position))).vector
                frame = pin.LOCAL
            if np.linalg.norm(error) < 1e-10:
                break
            jacobian = pin.computeFrameJacobian(
                self.model, self.data, full, self.camera_id, frame)[:, :self.dof]
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-6 * np.eye(6), error)
            q = np.clip(q + np.clip(step, -0.12, 0.12), self.lower, self.upper)
        return q

    def path_clearance(self, start, end, bench_z, checks, samples=25):
        """Lowest tool and arm height along the straight line from start to end.

        THE ENDPOINTS ARE NOT THE PATH. Two poses that each clear the bench can
        be joined by an interpolation that dips through it -- the action server
        interpolates in joint space, and a joint-space straight line is a curve
        in Cartesian space with no reason to stay above anything.
        """
        tool, arm = float('inf'), float('inf')
        for step in range(samples + 1):
            q = start + (end - start) * (step / samples)
            self.forward(q)
            heights = {name: float(self.data.oMf[self.model.getFrameId(name)].translation[2])
                       for name in checks['tool_frames'] + checks['arm_frames']}
            tool = min(tool, min(heights[n] for n in checks['tool_frames']) - bench_z)
            arm = min(arm, min(heights[n] for n in checks['arm_frames']) - bench_z)
        return tool, arm

    def inspect(self, q, position, direction, bench_z, checks):
        self.forward(q)
        placement = self.data.oMf[self.camera_id]
        axis = placement.rotation[:, 0]
        jacobian = pin.computeFrameJacobian(
            self.model, self.data, self.forward(q), self.camera_id,
            pin.LOCAL_WORLD_ALIGNED)[:, :self.dof]
        heights = {name: float(self.data.oMf[self.model.getFrameId(name)].translation[2])
                   for name in checks['tool_frames'] + checks['arm_frames']}
        return {
            'position_error_m': float(np.linalg.norm(placement.translation - position)),
            'axis_error_deg': float(np.degrees(np.arccos(
                np.clip(np.dot(axis, direction), -1.0, 1.0)))),
            'tool_over_bench_m': min(heights[n] for n in checks['tool_frames']) - bench_z,
            'arm_over_bench_m': min(heights[n] for n in checks['arm_frames']) - bench_z,
            'joint_margin_rad': float(min(np.min(q - self.lower), np.min(self.upper - q))),
            'condition': float(np.linalg.cond(jacobian)),
        }


def tilted_axis(tilt_rad, heading_rad):
    """Optical axis *tilt_rad* off straight down, leaning along *heading_rad* in xy."""
    return np.array([math.sin(tilt_rad) * math.cos(heading_rad),
                     math.sin(tilt_rad) * math.sin(heading_rad),
                     -math.cos(tilt_rad)])


def camera_attitude(axis, up_heading_rad):
    """The camera's whole attitude: optical axis *axis*, picture up along a heading.

    Columns are the camera frame's own x, y, z in base coordinates, and the
    frame is the RealSense camera_link convention: +x is the optical axis, so
    the other two span the picture. Fixing them is what stops the camera
    rotating as it translates, and what makes the field of view a rectangle
    with a known orientation instead of a cone.
    """
    x = np.asarray(axis, dtype=float)
    x = x / np.linalg.norm(x)
    up = np.array([math.cos(up_heading_rad), math.sin(up_heading_rad), 0.0])
    z = up - np.dot(up, x) * x
    norm = np.linalg.norm(z)
    if norm < 1e-6:
        raise ValueError('camera_up_heading_deg is parallel to the optical axis')
    z = z / norm
    return np.column_stack([x, np.cross(z, x), z])


def _line(low, high, spacing):
    """Sample [low, high] at no more than *spacing* apart, ends included."""
    span = high - low
    if span <= 1e-9:
        return [low]
    count = max(2, int(math.ceil(span / spacing)) + 1)
    return [low + span * i / (count - 1) for i in range(count)]


def raster_cells(spec):
    """The boustrophedon over the search area, far to near, alternating in y.

    Every pass after the first is entered by a LATERAL move at the previous
    pass's height followed by a VERTICAL drop -- a Gamma, never an L. The
    approach waypoint that makes the corner is generated here rather than
    written out, because it is a consequence of the two heights and would
    otherwise be a number someone has to keep in step with both.
    """
    area = spec['raster']['area']
    # Two spacings, because they do different jobs. ROW spacing is how many
    # times the arm sweeps left and right -- each row is one traverse -- and
    # COLUMN spacing is how finely it samples within a traverse. The camera's
    # field of view is wide enough that a few stops cover a row, so the rows
    # are the ones worth having several of.
    row_spacing = float(spec['raster']['row_spacing_m'])
    column_spacing = float(spec['raster']['column_spacing_m'])
    centre = ((area['x'][0] + area['x'][1]) / 2.0,
              (area['y'][0] + area['y'][1]) / 2.0)
    cells = []
    previous_height = None
    for sweep in spec['raster']['passes']:
        height = float(sweep['height_m'])
        # A pass marked `at_centre` is the close look, and it is ONE cell. A row
        # down there would be lateral motion at low clearance, which is the
        # shape this whole pattern avoids. It cannot know where to descend --
        # that is what the search above it was for -- so it takes the middle of
        # the area and says so.
        at_centre = bool(sweep.get('at_centre', False))
        if previous_height is not None and height < previous_height:
            first_x = centre[0] if at_centre else area['x'][1]
            first_y = centre[1] if at_centre else area['y'][0]
            cells.append({
                'name': f"{sweep['name']}_approach",
                'position': np.array([first_x, first_y,
                                      spec['bench_z_m'] + previous_height]),
                'duration': float(sweep.get('duration_s', 3.0)),
                'first': False,
            })
        previous_height = height
        # Far to near: +x is away from the robot base, so x descends.
        xs = ([centre[0]] if at_centre
              else list(reversed(_line(area['x'][0], area['x'][1], row_spacing))))
        for row, x in enumerate(xs):
            ys = ([centre[1]] if at_centre
                  else _line(area['y'][0], area['y'][1], column_spacing))
            # The column index is SPATIAL and not the visiting order: c0 is the
            # lowest y in every row. A name has to say where the waypoint is,
            # and an index that counted visits would make r0c0 and r1c0 two
            # different places.
            cells_in_row = list(enumerate(ys))
            # The turn that makes it a boustrophedon rather than a set of
            # scan lines: every other row is traversed the other way, so the
            # arm never flies back across the bench to start the next one.
            if row % 2:
                cells_in_row = list(reversed(cells_in_row))
            for column, y in cells_in_row:
                cells.append({
                    'name': (sweep['name'] if at_centre
                             else f"{sweep['name']}_r{row}c{column}"),
                    'position': np.array([x, y, spec['bench_z_m'] + height]),
                    'duration': float(sweep.get('duration_s', 3.0)),
                    'first': not cells,
                })
    return cells


def solve_raster(solver, spec, seed):
    """Solve the whole search raster once. It is the same for every object."""
    tilt = math.radians(float(spec['raster'].get('tilt_deg', 0.0)))
    checks = spec['checks']
    tries = int(spec['solver']['tries'])
    # Seeded once and fixed, so re-solving an unchanged spec gives the same
    # file. A generated table that churned on every run would make every diff
    # unreadable.
    rng = np.random.default_rng(int(spec['solver'].get('seed', 0)))
    up_heading = spec['raster'].get('camera_up_heading_deg')
    waypoints, anchor, failures = [], seed, []
    # How many cells in a row have been dropped. The step limit is per CELL, so
    # after a drop the arm has two cells' worth of ground to cover and the
    # limit has to say so -- otherwise one unreachable cell takes the next one
    # with it, and the cascade ate the whole close pass when this was fixed.
    skipped = 0
    for cell in raster_cells(spec):
        direction = tilted_axis(tilt, float(spec['raster'].get('tilt_heading_rad', 0.0)))
        attitude = (None if up_heading is None
                    else camera_attitude(direction, math.radians(float(up_heading))))
        best = None
        if cell['first']:
            # WIDE seeding, and it is not optional: the first waypoint has no
            # anchor to stay near, and damped least squares started from the
            # home pose alone converges into whichever branch home happens to
            # sit in -- which for this arm is the wrong side of the bench. It
            # then fails the joint-margin check, and because every later cell
            # is anchored on it, the whole raster comes out empty.
            seeds = [anchor] + [rng.uniform(solver.lower + 0.4, solver.upper - 0.4)
                                for _ in range(tries)]
        else:
            # Narrow, around the previous waypoint: the raster has to stay in
            # one IK branch, so the search is for the nearest solution and not
            # the best one.
            seeds = [anchor] + [np.clip(anchor + rng.normal(0, 0.4, solver.dof),
                                        solver.lower, solver.upper)
                                for _ in range(tries)]
        for candidate in seeds:
            q = solver.solve(cell['position'], direction, candidate, rotation=attitude)
            report = solver.inspect(q, cell['position'], direction,
                                    spec['bench_z_m'], checks)
            if (report['position_error_m'] > checks['max_position_error_m']
                    or report['axis_error_deg'] > checks['max_axis_error_deg']
                    or report['joint_margin_rad'] < checks['min_joint_margin_rad']
                    or report['condition'] > checks['max_condition']
                    or report['tool_over_bench_m'] < checks['min_tool_over_bench_m']
                    or report['arm_over_bench_m'] < checks['min_arm_over_bench_m']):
                continue
            step = float(np.max(np.abs(q - anchor)))
            if not cell['first'] and step > checks['max_step_rad'] * (skipped + 1):
                continue
            if not cell['first']:
                path_tool, path_arm = solver.path_clearance(
                    anchor, q, spec['bench_z_m'], checks)
                if (path_tool < checks['min_path_tool_over_bench_m']
                        or path_arm < checks['min_path_arm_over_bench_m']):
                    continue
                report['path_tool_over_bench_m'] = path_tool
            # NEAREST, including for the first waypoint. Picking the
            # best-conditioned solution there instead cost 22 of 38 cells: the
            # first waypoint fixes the IK branch every later one has to stay
            # in, and how well conditioned it is says nothing about whether the
            # rest of the raster can be reached from it. Nearest-to-home keeps
            # the branch the one the arm is already in.
            if best is None or step < best[0]:
                best = (step, q, report)
        if best is None:
            failures.append(cell['name'])
            skipped += 1
            continue
        skipped = 0
        _key, q, report = best
        step = float(np.max(np.abs(q - anchor)))
        waypoints.append((cell, q, report, step))
        anchor = q
    return waypoints, failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('spec')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    with open(args.spec, encoding='utf-8') as handle:
        spec = yaml.safe_load(handle)

    model, camera_id, extrinsic = build_model(spec)
    solver = Solver(model, camera_id, int(spec['robot']['arm_dof']))
    home = np.array(spec['robot']['home'], dtype=float)

    lines = [
        '# GENERATED by scripts/solve_sweep_raster.py -- do not hand-edit.',
        f'# Source: {os.path.basename(args.spec)}',
        '#',
        '# Re-solve it rather than adjusting a number here, or the waypoints stop',
        '# agreeing with each other about which IK branch they are in. From the',
        '# cho_task_manager source directory:',
        '#',
        # Canonical form rather than whatever paths this run was given, so the
        # comment stays a command someone can paste.
        ('#   python3 scripts/solve_sweep_raster.py '
         f'config/sweep/{os.path.basename(args.spec)} '
         f'--out config/sweep/{os.path.basename(args.out)}'),
        '#',
    ]
    for line in spec['header'].strip().splitlines():
        lines.append(f'# {line}'.rstrip())
    lines.append('')
    lines.append('defaults:')
    for key, value in spec['defaults'].items():
        # Python spells its booleans with a capital, and while PyYAML reads
        # `True` back as a bool, YAML 1.2 does not. Emit what both accept.
        text = str(value).lower() if isinstance(value, bool) else value
        lines.append(f'  {key}: {text}')
    lines.append('')
    lines.append('sweeps:')

    # ONE raster, solved once and given to every object. The search does not
    # know where anything is -- that is the situation it exists for -- so there
    # is nothing to make it per-object. In a run it is also cheaper than it
    # looks: the first object's sweep passes over the whole area, so anything
    # else out there becomes visible during it and the next object's leaf skips
    # without moving.
    waypoints, failures = solve_raster(solver, spec, home)
    area = spec['raster']['area']
    print(f'raster: {len(waypoints)} waypoint(s)'
          + (f', {len(failures)} unreachable: {failures}' if failures else ''),
          file=sys.stderr)
    if not waypoints:
        sys.exit('no reachable viewpoint anywhere in the search area')

    for entry in spec['objects']:
        lines.append(f"  - object: {entry['object']}")
        lines.append(f"    # Searches x {area['x']} by y {area['y']}, rows "
                     f"{spec['raster']['row_spacing_m']} m apart and stops "
                     f"{spec['raster']['column_spacing_m']} m along each, bench "
                     f"plane z = {spec['bench_z_m']}. Those are CAMERA positions; "
                     'what it sees reaches well beyond them. NOT aimed at where '
                     'this object is thought to be: finding that out is the job.')
        if failures:
            lines.append('    # Dropped as unreachable in this IK branch: '
                         + ', '.join(failures))
        lines.append('    waypoints:')
        for cell, q, report, step in waypoints:
            lines.append(f"      # cam ({cell['position'][0]:+.3f}, "
                         f"{cell['position'][1]:+.3f}, {cell['position'][2]:+.3f}), "
                         f"tool {report['tool_over_bench_m']:.3f} m over the bench"
                         + (f" ({report['path_tool_over_bench_m']:.3f} lowest on the "
                            'way here)' if 'path_tool_over_bench_m' in report else '')
                         + f", cond(J) {report['condition']:.0f}, "
                         f'{step:.2f} rad from the last')
            lines.append(f"      - name: {cell['name']}")
            lines.append('        joints: [' + ', '.join(f'{v:.4f}' for v in q) + ']')
            if cell['first']:
                lines.append(f"        duration: {spec['raster']['approach_duration_s']}")
            elif cell['duration'] != float(spec['defaults']['waypoint_duration']):
                lines.append(f"        duration: {cell['duration']}")
        lines.append('')

    with open(args.out, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines).rstrip() + '\n')
    print(f'wrote {args.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
