#!/usr/bin/env python3
# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Rewrite the MuJoCo model's inertial parameters from the URDF.

Why this exists
---------------
cho_description_openarm vendors two upstream assets that describe the same arm:
the URDF (enactic/openarm_description) and the MJCF (enactic/openarm_mujoco).
Their inertial parameters have drifted apart - most visibly openarm_link4, which
is 0.635 kg in the URDF and 1.370 kg in the MJCF.

That matters because every cho controller builds its Pinocchio model from the
URDF and feeds gravity/Coriolis compensation forward. If MuJoCo simulates a
heavier arm than the controller models, the feed-forward is wrong by
construction: with the raw upstream MJCF the arm collapses onto its joint limits
under gravity compensation alone, and no amount of gain tuning fixes it because
the error is a modelling error, not a control one.

So the URDF wins, and this script pushes its values into the MJCF. The URDF is
the model the controller uses in *every* environment, real hardware included, so
aligning the simulator to it is what makes a MuJoCo result mean anything about
the real robot.

Pinocchio does the hard part: it lumps fixed-joint children (openarm_hand into
openarm_link7) into the parent body exactly the way the MJCF already does, and
reports the result in the joint frame - which is also the MJCF body frame here,
because every MJCF joint sits at pos="0 0 0" of its body.

Run it after changing config/arm/inertials.yaml, or after re-vendoring either
upstream asset:

    python3 scripts/sync_mjcf_inertials.py
"""

import argparse
import os
import pathlib
import re
import xml.etree.ElementTree as ET

import numpy as np

import pinocchio as pin

import xacro

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_XACRO = os.path.join(HERE, 'robots', 'openarm_v10', 'openarm_v10.urdf.xacro')
DEFAULT_MJCF = os.path.join(HERE, 'xml', 'openarm_v10', 'openarm.xml')


def fmt(values):
    return ' '.join(f'{float(v):.9g}' for v in values)


def is_physical(matrix):
    """MuJoCo's admissibility test for an inertia tensor.

    Requires positive eigenvalues and the triangle inequality A + B >= C on
    them. Pinocchio and the URDF parser do not check either, so an unphysical
    tensor travels all the way from the YAML to the MJCF compiler before anyone
    notices.
    """
    eigenvalues = np.sort(np.linalg.eigvalsh(matrix))
    if eigenvalues[0] <= 0.0:
        return False
    return eigenvalues[0] + eigenvalues[1] >= eigenvalues[2]


def inertia_attribute(matrix, body_name):
    """MJCF inertia attribute for `matrix`, falling back to the diagonal.

    openarm_description fills the products of inertia with a 1e-06 placeholder
    for every link. On the big links that is negligible, but on the 36 g gripper
    fingers it is the same order as the diagonal itself and makes the tensor
    unphysical, which MuJoCo rejects outright:

        Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix

    Dropping the placeholder products rather than balancing the tensor keeps the
    principal moments exactly as specified upstream.
    """
    if is_physical(matrix):
        full = [matrix[0, 0], matrix[1, 1], matrix[2, 2],
                matrix[0, 1], matrix[0, 2], matrix[1, 2]]
        return f'fullinertia="{fmt(full)}"'

    diagonal = np.diag(np.diag(matrix))
    if not is_physical(diagonal):
        raise SystemExit(
            f'{body_name}: inertia is unphysical even after dropping the products '
            f'of inertia; fix it in config/*/inertials.yaml')
    print(f'  {body_name:<24} products of inertia dropped (unphysical tensor upstream)')
    return f'diaginertia="{fmt(np.diag(matrix))}"'


def mjcf_bodies_in_lump(body):
    """The MJCF bodies Pinocchio would fold into one rigid body.

    A jointless child is rigidly attached to its parent, so Pinocchio merges it;
    a child carrying a joint is its own body and stops the walk. This is what
    lets one script serve both MJCF layouts: the single-arm model folds the hand
    into openarm_link7 the way the URDF's fixed joint does, while the bimanual
    model keeps openarm_*_link8 / _hand / _hand_tcp as separate bodies.
    """
    found = [body]
    for child in body.findall('body'):
        if child.findall('joint'):
            continue
        found.extend(mjcf_bodies_in_lump(child))
    return found


def joint_to_mjcf_body(urdf_root):
    """Map each URDF movable joint to the MJCF body that carries it.

    Derived from the URDF's own joint->child_link edges rather than assumed, so a
    renamed link shows up as a KeyError here instead of as silently mismatched
    dynamics.
    """
    mapping = {}
    for joint in urdf_root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        mapping[joint.get('name')] = joint.find('child').get('link')
    return mapping


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xacro', default=DEFAULT_XACRO)
    parser.add_argument('--mjcf', default=DEFAULT_MJCF)
    parser.add_argument('--bimanual', action='store_true',
                        help='Sync the two-arm model instead of the single-arm one')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    if args.bimanual and args.mjcf == DEFAULT_MJCF:
        args.mjcf = os.path.join(HERE, 'xml', 'openarm_v10_bimanual', 'openarm_bimanual.xml')

    urdf = xacro.process_file(args.xacro, mappings={
        'hardware': 'mujoco',
        'control_mode': 'torque',
        'bimanual': 'true' if args.bimanual else 'false',
    }).toxml()
    urdf_root = ET.fromstring(urdf)
    model = pin.buildModelFromXML(urdf)
    body_of = joint_to_mjcf_body(urdf_root)

    # Rewrite the MJCF as text rather than through ElementTree: the file carries
    # the upstream Apache header as a comment outside the root element, which the
    # ElementTree round-trip silently drops. Everything except the <inertial>
    # attributes stays byte-for-byte identical this way.
    source = pathlib.Path(args.mjcf).read_text()

    # Joint axes are checked, not copied: a mismatch means the two vendored
    # assets disagree about the robot's kinematics, which is worth seeing rather
    # than silently papering over. openarm_joint7 is the known case - the URDF's
    # single-arm build (arm_prefix="") is the mirrored variant and gives 0 -1 0
    # where the MJCF has 0 1 0, so a commanded wrist rotation runs backwards in
    # simulation.
    urdf_axes = {
        j.get('name'): [float(v) for v in j.find('axis').get('xyz').split()]
        for j in urdf_root.findall('joint')
        if j.get('type') != 'fixed' and j.find('axis') is not None
    }

    mjcf_tree = ET.parse(args.mjcf)
    mjcf_bodies = {b.get('name'): b for b in mjcf_tree.getroot().iter('body')}
    urdf_links = {}
    for link in urdf_root.findall('link'):
        element = link.find('inertial')
        if element is not None:
            urdf_links[link.get('name')] = element

    updated = source
    changed = 0
    per_link = []
    for i in range(1, model.njoints):
        joint_name = model.names[i]
        body_name = body_of[joint_name]

        body_open = re.search(rf'<body name="{re.escape(body_name)}"', updated)
        if body_open is None:
            print(f'  skip {joint_name}: no MJCF body named {body_name!r}')
            continue

        # <inertial> must be the first child of <body>, so the next one in the
        # text after the body tag is this body's.
        inertial = re.search(r'<inertial\b[^>]*/>', updated[body_open.end():])
        if inertial is None:
            print(f'  skip {joint_name}: {body_name} has no <inertial>')
            continue

        mjcf_joint = re.search(
            rf'<joint name="{re.escape(joint_name)}"[^>]*axis="([^"]+)"', updated)
        if mjcf_joint is not None and joint_name in urdf_axes:
            mjcf_axis = [float(v) for v in mjcf_joint.group(1).split()]
            if mjcf_axis != urdf_axes[joint_name]:
                urdf_axis_text = fmt(urdf_axes[joint_name])
                print(f'  {joint_name:<24} axis {mjcf_joint.group(1)} -> {urdf_axis_text}')
                span = mjcf_joint.span(1)
                updated = updated[:span[0]] + urdf_axis_text + updated[span[1]:]

        # Only write Pinocchio's lumped inertia when the MJCF folds the same
        # bodies together. Where it does not - the bimanual model keeps the hand
        # as its own body - writing the lump here would double-count it against
        # the sibling bodies that are still carrying their own mass, so those are
        # synced from their matching URDF link instead (see per_link below).
        lump = mjcf_bodies_in_lump(mjcf_bodies[body_name])
        if len(lump) > 1:
            per_link.extend(b.get('name') for b in lump)
            continue

        inertia = model.inertias[i]
        # inertia.inertia is about the COM, expressed in the joint (= body)
        # frame. MuJoCo reads fullinertia as (ixx iyy izz ixy ixz iyz), the same
        # order and sign convention the URDF uses. diaginertia and fullinertia
        # are mutually exclusive, so the replacement carries exactly one.
        replacement = (
            f'<inertial pos="{fmt(inertia.lever)}" mass="{inertia.mass:.9g}"'
            f' {inertia_attribute(inertia.inertia, body_name)}/>'
        )
        start = body_open.end() + inertial.start()
        end = body_open.end() + inertial.end()
        old_mass = re.search(r'mass="([^"]+)"', inertial.group(0))
        print(f'  {body_name:<24} mass {old_mass.group(1)} -> {inertia.mass:.9g}')
        updated = updated[:start] + replacement + updated[end:]
        changed += 1

    # Bodies the MJCF keeps separate: sync each from its own URDF link.
    for body_name in per_link:
        element = urdf_links.get(body_name)
        if element is None:
            continue          # MJCF-only frame (openarm_*_link8), carries no mass
        mass = float(element.find('mass').get('value'))
        origin = element.find('origin')
        com = [float(v) for v in (origin.get('xyz') if origin is not None else '0 0 0').split()]
        inertia_element = element.find('inertia')
        matrix = np.array([
            [float(inertia_element.get('ixx')), float(inertia_element.get('ixy')),
             float(inertia_element.get('ixz'))],
            [float(inertia_element.get('ixy')), float(inertia_element.get('iyy')),
             float(inertia_element.get('iyz'))],
            [float(inertia_element.get('ixz')), float(inertia_element.get('iyz')),
             float(inertia_element.get('izz'))],
        ])
        body_open = re.search(rf'<body name="{re.escape(body_name)}"', updated)
        inertial = re.search(r'<inertial\b[^>]*/>', updated[body_open.end():])
        if body_open is None or inertial is None:
            continue
        old_mass = re.search(r'mass="([^"]+)"', inertial.group(0)).group(1)
        replacement = (f'<inertial pos="{fmt(com)}" mass="{mass:.9g}"'
                       f' {inertia_attribute(matrix, body_name)}/>')
        start = body_open.end() + inertial.start()
        end = body_open.end() + inertial.end()
        if float(old_mass) != mass:
            print(f'  {body_name:<28} mass {old_mass} -> {mass:.9g}')
        updated = updated[:start] + replacement + updated[end:]
        changed += 1

    total = sum(model.inertias[i].mass for i in range(1, model.njoints))
    print(f'\n  {changed} bodies updated, total moving mass {total:.4f} kg')

    if args.dry_run:
        print('  (dry run, nothing written)')
        return

    pathlib.Path(args.mjcf).write_text(updated)
    print(f'  wrote {args.mjcf}')


if __name__ == '__main__':
    main()
