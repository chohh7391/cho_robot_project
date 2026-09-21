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
A boustrophedon raster with the tool held at a fixed tilt: rows at constant x
swept far to near, each row traversed in y, the direction alternating. That is
the classic coverage pattern (Choset's boustrophedon cellular decomposition),
and it is what a person sweeping a bench by hand does.

BUT THE SPACING IS NOT THE COVERAGE ONE. The textbook rule is
``stripe = footprint x (1 - overlap)``, and here that gives a single cell: a
D435's infra stream sees about 1.04 x 0.59 m at 0.55 m, which already covers
the whole area of interest from one viewpoint. Coverage is not what the motion
buys. What it buys is PARALLAX -- a different line of sight to the same tag --
because the failure being recovered from is something standing in the way, and
the active-vision result is that translation alone only helps when it changes
the viewing direction. So the spacing is chosen as an angle: ``extent_m`` at
``heights_m[0]`` subtends the spread of viewing directions the sweep samples.

The tool tilt is the other half of that, and it is the cheaper half. Tilting
the optical axis off vertical costs nothing in reachability -- it is still one
fixed orientation per row, so one IK branch -- and it buys two things the
translation does not:

* a tag viewed at normal incidence is the WORST case for AprilTag's planar
  pose, whose two-solution ambiguity is strongest there. This repo already
  knows it: ``top_down_yaw`` exists because the tag normal is the part of the
  pose that cannot be trusted.
* reversing the tilt per row gives the raster two genuinely different viewing
  DIRECTIONS rather than one, for free.

The tilt follows the direction of travel: a row swept toward +y leans toward
+y. So it alternates with the boustrophedon and needs no separate rule.

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

    def solve(self, position, direction, seed, iterations=400):
        """Camera origin at *position* with its +x along *direction*."""
        q = np.clip(np.asarray(seed, dtype=float).copy(), self.lower, self.upper)
        direction = np.asarray(direction, dtype=float)
        direction = direction / np.linalg.norm(direction)
        error = np.ones(6)
        for _ in range(iterations):
            full = self.forward(q)
            placement = self.data.oMf[self.camera_id]
            # Position, and the rotation that swings the optical axis onto the
            # wanted one. The cross product has no component along the axis, so
            # the roll about it is never commanded -- which is what leaves the
            # task 5-DOF and the IK far better conditioned than a full pose.
            error = np.concatenate([
                position - placement.translation,
                np.cross(placement.rotation[:, 0], direction)])
            if np.linalg.norm(error) < 1e-10:
                break
            jacobian = pin.computeFrameJacobian(
                self.model, self.data, full, self.camera_id,
                pin.LOCAL_WORLD_ALIGNED)[:, :self.dof]
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-6 * np.eye(6), error)
            q = np.clip(q + np.clip(step, -0.12, 0.12), self.lower, self.upper)
        return q

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


def raster_cells(spec, tag_xy):
    """The boustrophedon, far to near, alternating along y, per height pass."""
    extent = spec['raster']['extent_m']
    columns = int(spec['raster']['columns'])
    cells = []
    for pass_index, sweep in enumerate(spec['raster']['passes']):
        height = float(sweep['height_m'])
        rows = int(sweep['rows'])
        # Far to near: +x is away from the robot base, so x descends.
        xs = ([tag_xy[0]] if rows == 1
              else [tag_xy[0] + extent - 2 * extent * i / (rows - 1) for i in range(rows)])
        for row, x in enumerate(xs):
            ys = ([tag_xy[1]] if columns == 1
                  else [tag_xy[1] - extent + 2 * extent * i / (columns - 1)
                        for i in range(columns)])
            # The column index is SPATIAL and not the visiting order: c0 is the
            # lowest y in every row. A name has to say where the waypoint is,
            # and an index that counted visits would make r0c0 and r1c0 two
            # different places.
            cells_in_row = list(enumerate(ys))
            # The turn that makes it a boustrophedon rather than a set of
            # scan lines: every other row is traversed the other way, so the
            # arm never flies back across the bench to start the next one.
            heading = math.pi / 2 if row % 2 == 0 else -math.pi / 2
            if row % 2:
                cells_in_row = list(reversed(cells_in_row))
            for column, y in cells_in_row:
                cells.append({
                    'name': f"{sweep['name']}_r{row}c{column}",
                    'position': np.array([x, y, spec['bench_z_m'] + height]),
                    # The tilt follows the direction of travel, so it reverses
                    # with the boustrophedon and needs no rule of its own.
                    'heading_rad': heading,
                    'duration': float(sweep.get('duration_s', 3.0)),
                    'first': pass_index == 0 and row == 0 and column == 0,
                })
    return cells


def solve_object(solver, spec, entry, seed):
    tag_xy = np.array(entry['tag_xy'], dtype=float)
    tilt = math.radians(float(spec['raster']['tilt_deg']))
    checks = spec['checks']
    tries = int(spec['solver']['tries'])
    # Seeded once and fixed, so re-solving an unchanged spec gives the same
    # file. A generated table that churned on every run would make every diff
    # unreadable.
    rng = np.random.default_rng(int(spec['solver'].get('seed', 0)))
    waypoints, anchor, failures = [], seed, []
    for cell in raster_cells(spec, tag_xy):
        direction = tilted_axis(tilt, cell['heading_rad'])
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
            seeds = [anchor] + [np.clip(anchor + rng.normal(0, 0.3, solver.dof),
                                        solver.lower, solver.upper)
                                for _ in range(tries)]
        for candidate in seeds:
            q = solver.solve(cell['position'], direction, candidate)
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
            if not cell['first'] and step > checks['max_step_rad']:
                continue
            # Nearest first for a raster step; best conditioned for the very
            # first waypoint, which sets the branch every later one inherits.
            key = -report['condition'] if cell['first'] else step
            if best is None or key < best[0]:
                best = (key, q, report)
        if best is None:
            failures.append(cell['name'])
            continue
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
        lines.append(f'  {key}: {value}')
    lines.append('')
    lines.append('sweeps:')

    for entry in spec['objects']:
        waypoints, failures = solve_object(solver, spec, entry, home)
        print(f"{entry['object']}: {len(waypoints)} waypoint(s)"
              + (f', {len(failures)} unreachable: {failures}' if failures else ''),
              file=sys.stderr)
        if not waypoints:
            sys.exit(f"no reachable viewpoint for {entry['object']}")
        lines.append(f"  - object: {entry['object']}")
        lines.append(f"    # Tag at ({entry['tag_xy'][0]:.4f}, {entry['tag_xy'][1]:.4f}), "
                     f'bench plane z = {spec["bench_z_m"]}.')
        if failures:
            lines.append('    # Dropped as unreachable in this IK branch: '
                         + ', '.join(failures))
        lines.append('    waypoints:')
        for cell, q, report, step in waypoints:
            lines.append(f"      # cam ({cell['position'][0]:+.3f}, "
                         f"{cell['position'][1]:+.3f}, {cell['position'][2]:+.3f}), "
                         f"tool {report['tool_over_bench_m']:.3f} m over the bench, "
                         f"cond(J) {report['condition']:.0f}, "
                         f"{step:.2f} rad from the last")
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
