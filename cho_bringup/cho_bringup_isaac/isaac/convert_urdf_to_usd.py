#!/usr/bin/env python3
# Copyright (c) 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert a cho_description URDF or xacro into a USD asset for Isaac Sim.

Run under Isaac Sim's interpreter, once, whenever the URDF changes:

    ~/isaacsim/python.sh convert_urdf_to_usd.py \\
        --urdf <description-share>/urdf/<robot>.urdf.xacro \\
        --usd-path <description-share>/usd \\
        --ros-package <description-package>:<description-share> \\
        --require-link <controller-ee-frame>

This is a thin wrapper around Isaac's own URDF importer that pins the settings
this project needs and then checks the result. The defaults are the point:

  merge_fixed_joints = False
      Controller EE and sensor frames are often attached by fixed joints.
      Merging would delete them; use --require-link to verify critical frames.

  fix_base = True
      The robot is treated as bolted down for these controller bringups.

  joint_target_type = "none", stiffness = damping = 0
      Drive gains belong to the control mode, and run_isaac_sim.py sets them at
      runtime. Baking them in would need one USD per mode.

The importer does not look at <ros2_control>, so the USD is control-mode
independent -- one asset serves position, velocity and torque.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

def movable_joints(urdf_path):
    """Every actuated joint in the URDF - these must all survive the import."""
    import xml.etree.ElementTree as ET

    names = []
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        if joint.get("type") in ("revolute", "continuous", "prismatic"):
            names.append(joint.get("name"))
    return names


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--urdf", required=True, help="URDF or xacro to convert")
    p.add_argument("--usd-path", required=True, help="Output directory for the USD asset")
    p.add_argument("--ros-package", action="append", default=[], metavar="NAME:PATH",
                   help="Resolve package:// URIs. Repeatable.")
    p.add_argument("--xacro-arg", action="append", default=[], metavar="NAME:=VALUE",
                   help="Passed through to xacro when --urdf is a xacro file")
    p.add_argument("--merge-fixed-joints", action="store_true",
                   help="Override the default. Breaks fr3_hand_tcp and the FT frame; "
                        "only useful for a visual-only asset.")
    p.add_argument("--collision-from-visuals", action="store_true",
                   help="Derive colliders from visual meshes instead of the URDF <collision> meshes")
    p.add_argument("--strip-links", default=r".*_sc$", metavar="REGEX",
                   help="Drop links whose name matches, and the joints attaching them, before "
                        "importing. Defaults to the URDF's *_sc self-collision helper links: they "
                        "carry no <inertial>, so the importer turns each into a massless rigid "
                        "body on a fixed joint and the articulation's mass matrix becomes so "
                        "ill-conditioned that any joint drive stiffness diverges on the first "
                        "step. Pass an empty string to keep everything.")
    p.add_argument("--require-link", action="append", default=[], metavar="NAME",
                   help="Also assert this link/frame survived the import. Repeatable. Use it for "
                        "frames that carry no mass and would otherwise be merged away silently, "
                        "e.g. a TCP frame or an FT measurement frame.")
    p.add_argument("--skip-checks", action="store_true", help="Do not verify the imported asset")
    return p.parse_args()


def strip_links(urdf_path, pattern):
    """Remove matching links and the joints that attach them; return a new path."""
    import xml.etree.ElementTree as ET

    regex = re.compile(pattern)
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    dropped_links = {link.get("name") for link in root.findall("link")
                     if regex.match(link.get("name") or "")}
    if not dropped_links:
        return urdf_path, set(), set()

    dropped_joints = set()
    for joint in list(root.findall("joint")):
        child = joint.find("child")
        parent = joint.find("parent")
        child_name = child.get("link") if child is not None else None
        parent_name = parent.get("link") if parent is not None else None
        if child_name in dropped_links or parent_name in dropped_links:
            dropped_joints.add(joint.get("name"))
            root.remove(joint)
    for link in list(root.findall("link")):
        if link.get("name") in dropped_links:
            root.remove(link)

    out_dir = tempfile.mkdtemp(prefix="cho_isaac_stripped_")
    out = os.path.join(out_dir, os.path.basename(urdf_path))
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return out, dropped_links, dropped_joints


def expand_xacro(path, xacro_args):
    """Expand a xacro file with the system ROS 2 toolchain.

    Isaac's interpreter is Python 3.12 and has no xacro, so this shells out. The
    child gets a scrubbed environment: Kit puts its own 3.12 site-packages on
    PYTHONPATH, which would shadow the ROS 2 Humble (3.10) modules that xacro
    imports.
    """
    env = {k: v for k, v in os.environ.items()
           if k not in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD")}
    setup = "/opt/ros/%s/setup.bash" % os.environ.get("ROS_DISTRO", "humble")
    sources = [setup] if os.path.exists(setup) else []
    ws_setup = os.environ.get("CHO_WS_SETUP")
    if ws_setup and os.path.exists(ws_setup):
        sources.append(ws_setup)
    prefix = "".join("source %s >/dev/null 2>&1; " % s for s in sources)
    cmd = "%sxacro %s %s" % (prefix, path, " ".join(xacro_args))

    result = subprocess.run(["bash", "-c", cmd], env=env, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(
            "xacro failed on %s\n%s\n\n"
            "Expand it yourself and pass the plain URDF instead:\n"
            "    xacro %s %s > /tmp/robot.urdf\n"
            "    ... convert_urdf_to_usd.py --urdf /tmp/robot.urdf ...\n"
            "(set CHO_WS_SETUP=<ws>/install/setup.bash if the URDF uses $(find ...))"
            % (path, result.stderr.strip(), path, " ".join(xacro_args))
        )
    # Keep the source stem so the importer produces a predictable asset name,
    # but always hand it a .urdf file. Isaac Sim 6's importer rejects expanded
    # XML whose filename still ends in .xacro ("Expected file with extension
    # '.urdf'"). For foo.urdf.xacro this intentionally becomes foo.urdf.
    tmp_dir = tempfile.mkdtemp(prefix="cho_isaac_")
    basename = os.path.basename(path)
    if basename.endswith(".xacro"):
        basename = basename[:-len(".xacro")]
    if not basename.endswith(".urdf"):
        basename += ".urdf"
    out = os.path.join(tmp_dir, basename)
    with open(out, "w", encoding="utf-8") as f:
        f.write(result.stdout)
    return out


args = parse_args()

urdf_path = os.path.abspath(args.urdf)
if not os.path.exists(urdf_path):
    raise SystemExit("URDF not found: %s" % urdf_path)

temp_urdf = None
if "xacro" in open(urdf_path, encoding="utf-8").read(4096):
    print("[convert] expanding xacro: %s" % urdf_path)
    temp_urdf = expand_xacro(urdf_path, args.xacro_arg)
    urdf_path = temp_urdf

stripped_urdf = None
if args.strip_links:
    stripped_urdf, dropped_links, dropped_joints = strip_links(urdf_path, args.strip_links)
    if stripped_urdf != urdf_path:
        print("[convert] stripped %d links matching %r (and %d joints): %s"
              % (len(dropped_links), args.strip_links, len(dropped_joints),
                 ", ".join(sorted(dropped_links))))
        urdf_path = stripped_urdf

ros_packages = []
for spec in args.ros_package:
    if ":" not in spec:
        raise SystemExit("--ros-package expects NAME:PATH, got %r" % spec)
    name, path = spec.split(":", 1)
    ros_packages.append({"name": name.strip(), "path": path.strip()})

# --------------------------------------------------------------------------- #
from isaacsim import SimulationApp  # noqa: E402  isort:skip

simulation_app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402


def _enable(name):
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(name, True)


# Both are prerequisites of the importer, per Isaac's own urdf_import.py sample.
_enable("omni.scene.optimizer.core")
_enable("isaacsim.robot.schema")

from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig  # noqa: E402

exit_code = 0
output_usd = None
try:
    config = URDFImporterConfig()
    config.urdf_path = urdf_path
    config.usd_path = os.path.abspath(args.usd_path)
    config.fix_base = True
    config.merge_fixed_joints = bool(args.merge_fixed_joints)
    config.collision_from_visuals = bool(args.collision_from_visuals)
    config.joint_target_type = "none"
    config.joint_drive_type = "force"
    config.override_joint_stiffness = 0.0
    config.override_joint_damping = 0.0
    if ros_packages:
        config.ros_package_paths = ros_packages

    print("[convert] importing %s -> %s" % (urdf_path, config.usd_path))
    output_usd = URDFImporter(config).import_urdf()
    if not output_usd:
        raise RuntimeError("importer returned no output path")
    print("[convert] wrote %s" % output_usd)
    print("")
    print("    robot_usd:=%s" % output_usd)
    print("")

    if not args.skip_checks:
        from pxr import Usd

        stage = Usd.Stage.Open(output_usd)
        names = {p.GetName() for p in stage.Traverse()}
        missing_links = [n for n in args.require_link if n not in names]
        missing_joints = [n for n in movable_joints(urdf_path) if n not in names]
        if missing_links or missing_joints:
            print("[convert] FAILED verification")
            if missing_links:
                print("  missing links : %s" % missing_links)
            if missing_joints:
                print("  missing joints: %s" % missing_joints)
            if args.merge_fixed_joints:
                print("  --merge-fixed-joints removes fixed-joint frames; drop it.")
            exit_code = 1
        else:
            print("[convert] verified: %d actuated joints and %d required links present"
                  % (len(movable_joints(urdf_path)), len(args.require_link)))
except Exception as exc:  # noqa: BLE001 - report and exit non-zero
    print("[convert] ERROR: %r" % (exc,))
    exit_code = 1
finally:
    if temp_urdf:
        shutil.rmtree(os.path.dirname(temp_urdf), ignore_errors=True)
    if stripped_urdf:
        shutil.rmtree(os.path.dirname(stripped_urdf), ignore_errors=True)
    simulation_app.close()

sys.exit(exit_code)
