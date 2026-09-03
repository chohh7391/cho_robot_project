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

"""Isaac Sim standalone runner for the cho ros2_control bringups.

This is NOT a ROS 2 node and must not be launched with `ros2 run`. It runs under
Isaac Sim's own interpreter, normally via bringup_isaac_robot.launch.py:

    ~/isaacsim/python.sh run_isaac_sim.py --robot-usd <robot.usda> \
        --robot-profile robots/fr3.json --control-mode torque

The arm-specific numbers (joint names, home pose, rotor inertia, drive gains, FT
link) come from the JSON profile, so the same runner serves the Franka and the UR.

It stands up the simulation and the OmniGraph ROS 2 bridge; the controllers live
on the ROS side in a normal controller_manager whose hardware plugin is
`topic_based_ros2_control/TopicBasedSystem` (see the IsaacArmSystem /
IsaacHandSystem blocks in the robot's URDF).

    ROS -> Isaac : /isaac_joint_commands   (arm,  sensor_msgs/JointState)
                   /isaac_gripper_commands (hand, sensor_msgs/JointState)
                   /isaac/commands_enabled (startup gate, sensor_msgs/JointState)
    Isaac -> ROS : /isaac_joint_states     (all DOFs)
                   /clock
                   /isaac/ft_raw          (geometry_msgs/Wrench, with --publish-ft)

Startup gate
------------
TopicBasedSystem zero-initialises its command buffers and starts publishing them
the moment the hardware component activates, which is before any controller
exists. Acting on those zeros would drop the arm (torque mode) or yank it to q=0
(position mode). So both articulation controllers start disabled and the arm is
held at its home pose by a stiff position drive. cho_bringup_franka's
isaac_command_gate.py starts publishing on the gate topic once the controller
spawner has finished, and only then do the real drive gains and the command path
come alive.

No rclpy in this process - deliberately
---------------------------------------
Every ROS interaction here goes through the OmniGraph bridge nodes, which are
C++. Python-side rclpy is NOT usable in this configuration:

  * The bridge needs /opt/ros/<distro>/setup.bash sourced so that it binds the
    system ROS 2 libraries and interoperates natively with the workspace.
  * Importing the system rclpy then fails, because it is built for Python 3.10
    while Kit runs 3.12:
        ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
  * Importing Isaac's bundled rclpy instead gets further - rclpy.init() even
    succeeds - and then segfaults inside rclpy.node.Node(), because its
    _rclpy_pybind11 was built against Isaac's own copies of the ROS libraries,
    not the system ones now loaded.

So the gate is read from a ROS2SubscribeJointState node's output attribute, and
the emulated force/torque reading is published by a generic ROS2Publisher node.
Anything needing real ROS Python - stamping the wrench into a WrenchStamped,
serving the tare service - lives in a normal node on the ROS side
(scripts/isaac_ft_sensor.py).
"""

import argparse
import json
import math
import os
import sys

# --------------------------------------------------------------------------- #
# Robot profile
#
# Everything robot-specific - joint names, home pose, rotor inertia, drive gains,
# the FT link - lives in a JSON profile under isaac/robots/ rather than here, so
# one runner serves every arm. See isaac/robots/fr3.json for a documented example;
# the joint names and home pose must match the robot's <ros2_control> block.
# --------------------------------------------------------------------------- #
PROFILE_DEFAULTS = {
    "prim_path": "/robot",
    "arm_joints": [],
    "hand_joints": [],
    "arm_home": [],
    "hand_home": [],
    "arm_armature": [],
    "hand_armature": [],
    "arm_position_stiffness": 0.0,
    "arm_position_damping": 0.0,
    "arm_velocity_damping": 0.0,
    "arm_torque_damping": 0.0,
    "hand_stiffness": 0.0,
    "hand_damping": 0.0,
    "joint_friction": 0.0,
    # PhysX articulation solver settings. The URDF importer writes none, so the
    # articulation would otherwise run on PhysX's (low) defaults and the solver
    # cannot resolve the joint torques these controllers command. 32/1/False are
    # the values NVIDIA authors in both its Franka and its UR5e assets.
    "solver_position_iterations": 32,
    "solver_velocity_iterations": 1,
    "enable_self_collisions": False,
    "ft_link": None,
}


def load_profile(path):
    with open(path, encoding="utf-8") as handle:
        raw = json.load(handle)
    profile = dict(PROFILE_DEFAULTS)
    for key, value in raw.items():
        if key.startswith("_"):          # "_comment" style documentation keys
            continue
        if key not in PROFILE_DEFAULTS:
            raise SystemExit(f"{path}: unknown profile key {key!r}")
        profile[key] = value

    if not profile["arm_joints"]:
        raise SystemExit(f"{path}: 'arm_joints' is required")
    for names, values, label in (("arm_joints", "arm_home", "arm_home"),
                                 ("arm_joints", "arm_armature", "arm_armature"),
                                 ("hand_joints", "hand_home", "hand_home"),
                                 ("hand_joints", "hand_armature", "hand_armature")):
        if profile[values] and len(profile[values]) != len(profile[names]):
            raise SystemExit(f"{path}: {label} has {len(profile[values])} entries "
                             f"but {names} has {len(profile[names])}")
    return profile


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--robot-usd", required=True, help="USD produced by convert_urdf_to_usd.py")
    p.add_argument("--robot-profile", required=True,
                   help="JSON physics profile for this arm (see isaac/robots/)")
    p.add_argument("--control-mode", default="torque", choices=["position", "velocity", "torque"],
                   help="Must match the control_mode the URDF was expanded with")
    p.add_argument("--physics-rate", type=float, default=250.0,
                   help="Physics steps per second. MUST equal controller_manager update_rate")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--device", default="cpu", choices=["cpu", "cuda"], help="Physics device")
    p.add_argument("--joint-states-topic", default="isaac_joint_states")
    p.add_argument("--arm-commands-topic", default="isaac_joint_commands")
    p.add_argument("--gripper-commands-topic", default="isaac_gripper_commands")
    p.add_argument("--clock-topic", default="clock")
    p.add_argument("--gate-topic", default="isaac/commands_enabled")
    p.add_argument("--no-gate", action="store_true",
                   help="Enable the command path immediately. Only for debugging the bridge "
                        "without a controller_manager; the arm WILL fall in torque mode.")
    p.add_argument("--arm-torque-damping", type=float, nargs="+", default=None, metavar="D",
                   help="Override the profile's per-joint torque-mode drive damping (Nms/rad)")
    p.add_argument("--joint-friction", type=float, default=None,
                   help="Override the profile's PhysX joint friction COEFFICIENT (not a torque)")
    p.add_argument("--publish-ft", action="store_true",
                   help="Publish the emulated FT reading as an unstamped geometry_msgs/Wrench; "
                        "scripts/isaac_ft_sensor.py turns it into the Bota topic and tare service")
    p.add_argument("--ft-raw-topic", default="isaac/ft_raw")
    p.add_argument("--ft-link", default=None,
                   help="Override the profile's FT measurement link")
    p.add_argument("--ft-sign", type=float, default=-1.0,
                   help="Sign applied to the measured joint reaction. -1 reports the load applied "
                        "to the tool, matching mujoco_ros2_control's FT convention. Validate "
                        "against the real Bota before trusting the sign.")
    p.add_argument("--physics-engine", default="physx", choices=["physx", "newton"],
                   help="Physics backend. 'newton' boots Kit with the "
                        "isaacsim.exp.full.newton experience, which registers Newton "
                        "through omni.physics and switches the stage to it on startup "
                        "(exts.'isaacsim.physics.newton'.auto_switch_on_startup). "
                        "Default keeps PhysX, so an existing bringup is unaffected.")
    return p.parse_args(argv)


args = parse_args()
PROFILE = load_profile(args.robot_profile)

ROBOT_PRIM_PATH = PROFILE["prim_path"]
ARM_DOF_NAMES = PROFILE["arm_joints"]
HAND_DOF_NAMES = PROFILE["hand_joints"]
ARM_HOME = PROFILE["arm_home"] or [0.0] * len(ARM_DOF_NAMES)
HAND_HOME = PROFILE["hand_home"] or [0.0] * len(HAND_DOF_NAMES)
ARM_ARMATURE = PROFILE["arm_armature"]
HAND_ARMATURE = PROFILE["hand_armature"]
ARM_POSITION_STIFFNESS = PROFILE["arm_position_stiffness"]
ARM_POSITION_DAMPING = PROFILE["arm_position_damping"]
ARM_VELOCITY_DAMPING = PROFILE["arm_velocity_damping"]
HAND_STIFFNESS = PROFILE["hand_stiffness"]
HAND_DAMPING = PROFILE["hand_damping"]
SOLVER_POSITION_ITERATIONS = PROFILE["solver_position_iterations"]
SOLVER_VELOCITY_ITERATIONS = PROFILE["solver_velocity_iterations"]
ENABLE_SELF_COLLISIONS = PROFILE["enable_self_collisions"]

ARM_TORQUE_DAMPING = (args.arm_torque_damping if args.arm_torque_damping is not None
                      else PROFILE["arm_torque_damping"])
JOINT_FRICTION = (args.joint_friction if args.joint_friction is not None
                  else PROFILE["joint_friction"])
FT_LINK = args.ft_link or PROFILE["ft_link"]

# --------------------------------------------------------------------------- #
# Isaac Sim must be booted before any omni.* / isaacsim.* import.
# --------------------------------------------------------------------------- #
from isaacsim import SimulationApp  # noqa: E402  isort:skip

# Newton needs a different Kit experience, not a runtime setting: the app file is
# what disables the PhysX extensions and pulls in isaacsim.physics.newton and its
# tensor backend. There is no python-flavoured Newton experience shipped, so the
# full one is used; it runs headless the same way.
_EXPERIENCE = ""
if args.physics_engine == "newton":
    _EXPERIENCE = os.path.join(
        os.environ.get("ISAAC_SIM_PATH", os.path.expanduser("~/isaacsim")),
        "apps", "isaacsim.exp.full.newton.kit")
    if not os.path.exists(_EXPERIENCE):
        raise SystemExit(
            f"--physics-engine newton needs {_EXPERIENCE}, which this Isaac Sim "
            f"install does not have. Newton ships from Isaac Sim 6.0.")

simulation_app = (SimulationApp({"headless": args.headless}, experience=_EXPERIENCE)
                  if _EXPERIENCE else SimulationApp({"headless": args.headless}))

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.objects import DistantLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import PhysxSchema, Usd, UsdPhysics  # noqa: E402


def log(msg):
    print(f"[isaac_sim] {msg}", flush=True)


app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# --------------------------------------------------------------------------- #
# Scene
# --------------------------------------------------------------------------- #
stage_utils.set_stage_units(meters_per_unit=1.0)

# Deliberately minimal, mirroring MuJoCo's scene.xml: a ground plane and one
# light, nothing else. Task props belong in a separate scene USD.
GroundPlane("/World/GroundPlane")
DistantLight("/World/DistantLight").set_intensities([3000.0])

stage_utils.add_reference_to_stage(usd_path=args.robot_usd, path=ROBOT_PRIM_PATH)
simulation_app.update()

robot = Articulation(ROBOT_PRIM_PATH)

# The URDF importer does not put the articulation root on the referenced prim: it
# applies PhysicsArticulationRootAPI to a descendant (/fr3/Geometry/base for this
# asset). Articulation() resolves that itself, but the OmniGraph nodes take a raw
# path string, and pointing them at /fr3 fails with
#   "Provided pattern list did not match any articulations"
#   "JointStateSensor: no DOFs found under articulation root '/fr3'"
# so resolve it once and hand the real path to the graph.
ARTICULATION_PATH = robot.paths[0]
if ARTICULATION_PATH != ROBOT_PRIM_PATH:
    log(f"articulation root resolved to {ARTICULATION_PATH} (reference is at {ROBOT_PRIM_PATH})")

NEWTON = args.physics_engine == "newton"

# The URDF importer writes a "Physics" variant set with one layer per backend:
# 'physics' is the engine-neutral UsdPhysics description, 'physx' adds the
# PhysxSchema overs on top of it, 'mujoco' adds MjcActuator prims instead, and
# 'none' strips physics entirely. Switching the ENGINE does not switch the
# VARIANT - Newton's auto_switch_on_startup only registers the solver - so the
# stage has to be pointed at the right layer explicitly.
#
# Newton gets 'physics', the engine-neutral UsdPhysics layer.
#
# The obvious choice is 'mujoco' - that is the mapping Newton's own test helper
# uses ({"physx": "physx", "mujoco": "newton"} in isaacsim.physics.newton's
# tests/test_rigid_body.py), and it is what Newton's auto-switch selects. It is
# wrong for anything but effort control. Under 'mujoco' the joints carry no
# UsdPhysics.DriveAPI, so Newton's importer sees has_drive=False and
# JointTargetMode.from_gains() returns NONE for every DOF: no position and no
# velocity actuator is installed at all (newton/_src/sim/enums.py). Torque still
# works, because effort is applied straight to joint_f and needs no actuator -
# which is exactly why this was not noticed until velocity mode was tried and
# the arm just drifted under gravity with joint_target_vel ignored.
#
# 'physics' carries the DriveAPI, so authoring the gains before play (the same
# _author_drive_gains() the PhysX path uses) makes the importer infer the right
# mode per DOF: POSITION when stiffness is set, VELOCITY when only damping is,
# EFFORT when neither. Verified by reading builder.joint_target_mode back.
#
# The per-degree scaling in _author_drive_gains() is correct for Newton too:
# authoring 25 * pi/180 comes back as joint_target_kd = 25.0 per radian.
NEWTON_PHYSICS_VARIANT = "physics"


def _select_physics_variant(name):
    """Point the asset's Physics variant set at the layer this engine consumes."""
    stage = stage_utils.get_current_stage(backend="usd")
    prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    variant_set = prim.GetVariantSet("Physics")
    available = list(variant_set.GetVariantNames()) if variant_set.IsValid() else []
    if not available:
        log(f"WARNING: {ROBOT_PRIM_PATH} has no 'Physics' variant set, so the stage is used "
            f"as built. If this asset was authored for PhysX only, Newton will fall back to "
            f"its own defaults for every joint.")
        return
    if name not in available:
        raise SystemExit(
            f"{args.robot_usd}: --physics-engine {args.physics_engine} needs the {name!r} "
            f"Physics variant, but the asset only has {available}. Rebuild the USD with "
            f"convert_urdf_to_usd.py from a current Isaac Sim.")
    previous = variant_set.GetVariantSelection()
    if previous != name:
        variant_set.SetVariantSelection(name)
        simulation_app.update()
    log(f"Physics variant: {previous!r} -> {name!r} (available: {available})")


def _fix_negative_collision_scales():
    """Make collision-mesh scales positive, which Newton requires and PhysX does not.

    The URDF importer expresses the mesh's own handedness as a MIRRORED scale:
    the OpenArm collision hulls come in at (0.001, -0.001, 0.001). PhysX accepts
    a negative determinant; Newton's MuJoCo solver asserts on it, because it
    decomposes the transform into an all-negative scale and then rejects any
    non-plane shape whose size has no positive component:

        newton/_src/solvers/mujoco/solver_mujoco.py
        assert stype == GeoType.PLANE, "Only plane shapes are allowed to have a size of zero"

    which surfaces only as "[Newton] Initialization failed" at play time.

    Taking the absolute value mirrors each collision hull about that axis. That
    is acceptable here and nowhere near as bad as it sounds: these are convex
    hulls of near-symmetric links, self-collision is off, and a fixed-base arm
    doing free-space motion has no contact pair but the ground plane. The VISUAL
    meshes are deliberately left alone, so the rendered robot is unchanged.

    Scoped by "has a descendant with UsdPhysics.CollisionAPI" rather than by
    prim name, because the collision mesh is inside an instance prototype while
    the scale sits on the instance root.
    """
    stage = stage_utils.get_current_stage(backend="usd")
    fixed = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH),
                              Usd.TraverseInstanceProxies()):
        attr = prim.GetAttribute("xformOp:scale")
        if not attr or not attr.HasAuthoredValue():
            continue
        scale = attr.Get()
        if scale is None or all(component >= 0.0 for component in scale):
            continue
        has_collider = any(
            descendant.HasAPI(UsdPhysics.CollisionAPI)
            for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
        if not has_collider:
            continue
        attr.Set(type(scale)(*[abs(component) for component in scale]))
        fixed.append(prim.GetName())

    if fixed:
        log(f"newton: flipped {len(fixed)} mirrored collision scale(s) positive "
            f"({', '.join(sorted(fixed))}). Their hulls are mirrored; visuals untouched.")


if NEWTON:
    _select_physics_variant(NEWTON_PHYSICS_VARIANT)
    _fix_negative_collision_scales()

# The importer already gives every DOF a "force" drive and the FR3 effort limits
# from the URDF ([87]*4 + [12]*3 Nm, 100 N fingers), so neither the drive type
# nor the max efforts are touched here.
#
# Drive gains are authored into USD *before* play instead of through the tensor
# API: Articulation.set_dof_gains() corrupts this articulation's state on Isaac
# Sim 6.0.1 - joint positions jump to ~1e3 rad within the call itself, with or
# without dof_indices - and the simulation then diverges immediately.


def _author_articulation_solver():
    """Apply the official Franka articulation solver settings to the root prim."""
    stage = stage_utils.get_current_stage(backend="usd")
    prim = stage.GetPrimAtPath(ARTICULATION_PATH)
    if not prim.IsValid():
        raise RuntimeError(f"articulation root prim {ARTICULATION_PATH} is not valid")
    api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    api.CreateSolverPositionIterationCountAttr().Set(SOLVER_POSITION_ITERATIONS)
    api.CreateSolverVelocityIterationCountAttr().Set(SOLVER_VELOCITY_ITERATIONS)
    api.CreateEnabledSelfCollisionsAttr().Set(ENABLE_SELF_COLLISIONS)
    log(f"articulation: solver position={SOLVER_POSITION_ITERATIONS} "
        f"velocity={SOLVER_VELOCITY_ITERATIONS} selfCollisions={ENABLE_SELF_COLLISIONS}")


def _author_armatures():
    """Write physxJoint:armature onto the USD joint prims (before play)."""
    stage = stage_utils.get_current_stage(backend="usd")
    wanted = dict(zip(ARM_DOF_NAMES, ARM_ARMATURE))
    wanted.update(dict(zip(HAND_DOF_NAMES, HAND_ARMATURE)))

    found = set()
    for prim in stage.Traverse():
        value = wanted.get(prim.GetName())
        if value is None or not prim.IsA(UsdPhysics.Joint):
            continue
        # Apply() is idempotent - it returns the existing schema when the API is
        # already present, which it is for the joints the importer wrote
        # physxJoint:jointFriction onto.
        api = PhysxSchema.PhysxJointAPI.Apply(prim)
        api.CreateArmatureAttr().Set(value)
        api.CreateJointFrictionAttr().Set(JOINT_FRICTION)
        found.add(prim.GetName())

    missing = set(wanted) - found
    if missing:
        raise RuntimeError(f"no USD joint found for {sorted(missing)} while setting armature")
    log(f"armature + jointFriction={JOINT_FRICTION:g} authored on {len(found)} joints")


def _author_drive_gains(stiffness_rad, damping_rad):
    """damping_rad may be a scalar or a per-arm-joint sequence."""
    """Write the arm and finger drive gains onto the USD joint prims.

    USD stores ANGULAR drive gains per degree while the physics view reports them
    per radian, so an angular value has to be scaled by pi/180 on the way in or
    it lands 180/pi = 57.3x too stiff (that alone is enough to diverge). Linear
    drives - the two fingers - are already in N/m and need no conversion.
    """
    stage = stage_utils.get_current_stage(backend="usd")
    dampings = ([damping_rad] * len(ARM_DOF_NAMES)
                if isinstance(damping_rad, (int, float)) else list(damping_rad))
    stiffnesses = ([stiffness_rad] * len(ARM_DOF_NAMES)
                   if isinstance(stiffness_rad, (int, float)) else list(stiffness_rad))
    wanted = {name: (k * math.pi / 180.0, d * math.pi / 180.0, "angular")
              for name, k, d in zip(ARM_DOF_NAMES, stiffnesses, dampings)}
    wanted.update({name: (HAND_STIFFNESS, HAND_DAMPING, "linear") for name in HAND_DOF_NAMES})

    found = set()
    for prim in stage.Traverse():
        spec = wanted.get(prim.GetName())
        if spec is None or not prim.HasAPI(UsdPhysics.DriveAPI):
            continue
        stiffness, damping, axis = spec
        drive = UsdPhysics.DriveAPI.Get(prim, axis) or UsdPhysics.DriveAPI.Apply(prim, axis)
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(damping)
        found.add(prim.GetName())

    missing = set(wanted) - found
    if missing:
        raise RuntimeError(f"no USD drive found for joints {sorted(missing)}; was the USD built "
                           f"by convert_urdf_to_usd.py?")


def _arm_drive_gains():
    """(stiffness, damping) for the arm in this control mode, per radian.

    Shared by the PhysX and Newton paths so the two backends cannot drift apart;
    only where the numbers are written differs.
    """
    if args.control_mode == "position":
        return ARM_POSITION_STIFFNESS, ARM_POSITION_DAMPING
    if args.control_mode == "velocity":
        return 0.0, ARM_VELOCITY_DAMPING
    return 0.0, ARM_TORQUE_DAMPING


ARM_STIFFNESS, ARM_DAMPING = _arm_drive_gains()

if not NEWTON:
    # PhysX-only: PhysxArticulationAPI solver counts and physxJoint:armature.
    _author_articulation_solver()
    _author_armatures()

# UsdPhysics.DriveAPI is engine-neutral, and both backends need it authored
# before play. For Newton it is not just the gain values: the importer decides
# from these whether a DOF gets a position actuator, a velocity actuator or
# neither, and that decision cannot be revised afterwards.
_author_drive_gains(ARM_STIFFNESS, ARM_DAMPING)
log(f"drive gains authored for control_mode={args.control_mode} "
    f"(stiffness={ARM_STIFFNESS}, damping={ARM_DAMPING} per rad)")


# --------------------------------------------------------------------------- #
# ROS 2 bridge action graph
# --------------------------------------------------------------------------- #
def _topic(name):
    """Isaac's ROS 2 nodes take relative topic names; strip a leading slash."""
    return name.lstrip("/")


keys = og.Controller.Keys
GRAPH_PATH = "/ActionGraph"

graph_nodes = [
    # Driven by OnPhysicsStep, not OnPlaybackTick: the bridge must run at the
    # physics rate because the controller_manager is paced by the /clock this
    # graph publishes. On a render tick it would emit clock at the frame rate.
    ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ("SubscribeArm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
    ("ArmController", "isaacsim.core.nodes.IsaacArticulationController"),
    # The gate. A JointState subscriber is reused rather than adding a generic
    # subscriber node: its outputs are statically typed, so the main loop can
    # simply poll outputs:jointNames and open the gate once it is non-empty.
    ("SubscribeGate", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
]
if HAND_DOF_NAMES:
    graph_nodes += [
        ("SubscribeHand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("HandController", "isaacsim.core.nodes.IsaacArticulationController"),
    ]

graph_connections = [
    ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

    ("OnPhysicsStep.outputs:step", "ReadJointState.inputs:execIn"),
    ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
    ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
    ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
    ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
    ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
    ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
    ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
    ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),

    ("OnPhysicsStep.outputs:step", "SubscribeArm.inputs:execIn"),
    ("OnPhysicsStep.outputs:step", "ArmController.inputs:execIn"),
    ("SubscribeArm.outputs:jointNames", "ArmController.inputs:jointNames"),
    ("SubscribeArm.outputs:positionCommand", "ArmController.inputs:positionCommand"),
    ("SubscribeArm.outputs:velocityCommand", "ArmController.inputs:velocityCommand"),
    ("SubscribeArm.outputs:effortCommand", "ArmController.inputs:effortCommand"),

    ("OnPhysicsStep.outputs:step", "SubscribeGate.inputs:execIn"),

    ("Context.outputs:context", "PublishClock.inputs:context"),
    ("Context.outputs:context", "PublishJointState.inputs:context"),
    ("Context.outputs:context", "SubscribeArm.inputs:context"),
    ("Context.outputs:context", "SubscribeGate.inputs:context"),
]
if HAND_DOF_NAMES:
    graph_connections += [
        ("OnPhysicsStep.outputs:step", "SubscribeHand.inputs:execIn"),
        ("OnPhysicsStep.outputs:step", "HandController.inputs:execIn"),
        ("SubscribeHand.outputs:jointNames", "HandController.inputs:jointNames"),
        ("SubscribeHand.outputs:positionCommand", "HandController.inputs:positionCommand"),
        ("SubscribeHand.outputs:velocityCommand", "HandController.inputs:velocityCommand"),
        ("SubscribeHand.outputs:effortCommand", "HandController.inputs:effortCommand"),
        ("Context.outputs:context", "SubscribeHand.inputs:context"),
    ]

graph_values = [
    # useDomainIDEnvVar defaults to true, so ROS_DOMAIN_ID (25 under
    # dds/dds_mode.sh) is inherited from the launching shell.
    ("PublishClock.inputs:topicName", _topic(args.clock_topic)),
    ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(ARTICULATION_PATH)]),
    ("PublishJointState.inputs:topicName", _topic(args.joint_states_topic)),
    ("SubscribeArm.inputs:topicName", _topic(args.arm_commands_topic)),
    ("ArmController.inputs:robotPath", ARTICULATION_PATH),
    ("SubscribeGate.inputs:topicName", _topic(args.gate_topic)),
]
if HAND_DOF_NAMES:
    graph_values += [
        ("SubscribeHand.inputs:topicName", _topic(args.gripper_commands_topic)),
        ("HandController.inputs:robotPath", ARTICULATION_PATH),
    ]

if args.publish_ft:
    # Generic publisher: an unstamped geometry_msgs/Wrench, so the only dynamic
    # attributes are force/torque xyz. Emitting a WrenchStamped from here would
    # mean filling a header (and its stamp) through dynamic attributes; instead
    # scripts/isaac_ft_sensor.py stamps it, applies the tare offset and
    # republishes on the Bota topic the real driver uses.
    graph_nodes.append(("PublishFT", "isaacsim.ros2.bridge.ROS2Publisher"))
    graph_connections.append(("OnPhysicsStep.outputs:step", "PublishFT.inputs:execIn"))
    graph_connections.append(("Context.outputs:context", "PublishFT.inputs:context"))
    graph_values.append(("PublishFT.inputs:topicName", _topic(args.ft_raw_topic)))
    graph_values.append(("PublishFT.inputs:messagePackage", "geometry_msgs"))
    graph_values.append(("PublishFT.inputs:messageSubfolder", "msg"))
    graph_values.append(("PublishFT.inputs:messageName", "Wrench"))

og.Controller.edit(
    {
        "graph_path": GRAPH_PATH,
        "evaluator_name": "execution",
        # OnPhysicsStep is driven by the physics step callback, not the render
        # pipeline, so the graph has to be on-demand. Without this the node logs
        # "Physics OnSimulationStep node detected in a non on-demand Graph" and
        # never fires - no clock, no joint states, no commands. Same setup as
        # Isaac's own isaacsim.ros2.nodes test_clock.test_sim_clock_physics_step.
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: graph_nodes,
        keys.CONNECT: graph_connections,
        keys.SET_VALUES: graph_values,
    },
)
simulation_app.update()

arm_controller_node = og.Controller.node(f"{GRAPH_PATH}/ArmController")
hand_controller_node = (og.Controller.node(f"{GRAPH_PATH}/HandController")
                       if HAND_DOF_NAMES else None)
gate_node = og.Controller.node(f"{GRAPH_PATH}/SubscribeGate")

ft_node = None
if args.publish_ft:
    ft_node = og.Controller.node(f"{GRAPH_PATH}/PublishFT")
    # The message identity is set during graph creation (above) so the bridge has
    # already created the per-field input attributes by now.
    simulation_app.update()
    ft_attr_names = [a.get_name() for a in ft_node.get_attributes()]
    log(f"FT publisher attributes: {sorted(n for n in ft_attr_names if n.startswith('inputs:'))}")


def set_command_path_enabled(enabled):
    arm_controller_node.set_disabled(not enabled)
    if hand_controller_node is not None:
        hand_controller_node.set_disabled(not enabled)


set_command_path_enabled(False)

# --------------------------------------------------------------------------- #
# Physics
# --------------------------------------------------------------------------- #
physics_dt = 1.0 / args.physics_rate
SimulationManager.setup_simulation(dt=physics_dt, device=args.device)
# The application loop dt is deliberately the SAME as the physics dt: exactly one
# physics step per app update, hence one evenly spaced /clock message per step.
# A coarser app dt would make PhysX substep, so the bridge would emit a burst of
# clock and joint-state messages per frame - the controller_manager would run
# those cycles back to back and TopicBasedSystem's spin_some() would keep only
# the last state of each burst. Rendering therefore runs at the physics rate too;
# with a viewport open at a high physics rate, simulated time advances slower
# than wall-clock time (harmless, everything downstream is on sim time).
RenderingManager.set_dt(physics_dt)

app_utils.play()
# The tensor backend (set_dof_gains / set_dof_positions / ...) only becomes valid
# once physics is running and the articulation view has been created.
for _ in range(4):
    simulation_app.update()

dof_names = list(robot.dof_names)
arm_dofs = [dof_names.index(name) for name in ARM_DOF_NAMES]


def _apply_newton_runtime_properties():
    """Set the armature, which is the one thing Newton cannot take from USD.

    The drive gains are NOT set here. They are authored into UsdPhysics.DriveAPI
    before play, because for Newton those values do double duty: the importer
    reads them to decide whether each DOF gets a position actuator, a velocity
    actuator or neither, and no runtime call revises that. Setting them again
    here would put a second source of truth behind a decision already made.

    Armature has to be runtime. USD-authored 'newton:armature' is deprecated in
    this release, and physxJoint:armature is ignored. It is not optional either:
    Newton's default is a flat 0.1 kg m^2 on every joint (NewtonConfig.armature),
    which is twelve times the DM8009 rotor this arm actually has on joints 1-2
    and twenty times the DM3507 on joint 7. Leaving it would make the simulated
    arm a different robot from the one the controller's M(q) models.

    No degree-to-radian scaling here, unlike _author_drive_gains: that conversion
    exists only because USD stores ANGULAR drive gains per degree. The tensor API
    is SI throughout.
    """
    armatures = _full_dof_vector(ARM_ARMATURE, HAND_ARMATURE) if ARM_ARMATURE else None
    if armatures is not None:
        robot.set_dof_armatures(armatures)

    got_armature = robot.get_dof_armatures().numpy().reshape(-1)
    got_stiffness = robot.get_dof_gains()[0].numpy().reshape(-1)
    got_damping = robot.get_dof_gains()[1].numpy().reshape(-1)
    # Effort limits come from the URDF via the asset, not from the profile, and a
    # position drive that cannot reach the gravity torque sags no matter how
    # stiff it is - so they are logged next to the gains rather than assumed.
    # Only the underlying physics view exposes this; the Articulation wrapper
    # does not wrap it in this release.
    try:
        view = robot._physics_articulation_view                    # noqa: SLF001
        got_max_force = view.get_dof_max_forces().numpy().reshape(-1)
        log(f"newton max force  : {[round(float(v), 2) for v in got_max_force]}")
    except Exception as exc:                                       # noqa: BLE001
        log(f"newton max force  : unavailable ({type(exc).__name__}: {exc})")
    log(f"newton armature   : {[round(float(v), 4) for v in got_armature]}")
    log(f"newton stiffness  : {[round(float(v), 3) for v in got_stiffness]}")
    log(f"newton damping    : {[round(float(v), 3) for v in got_damping]}")
    # Which actuator Newton's importer installed for the arm, derived from the
    # gains authored into UsdPhysics.DriveAPI before play - the same rule
    # JointTargetMode.from_gains() applies (newton/_src/sim/enums.py). Reported
    # because a DOF that ends up with no actuator silently ignores every
    # position and velocity target, which is how the 'mujoco' variant wasted a
    # debugging session: torque still worked, so nothing looked wrong until
    # velocity mode drifted under gravity.
    def _expected_mode(k, d):
        k = max(abs(v) for v in ([k] if isinstance(k, (int, float)) else k))
        d = max(abs(v) for v in ([d] if isinstance(d, (int, float)) else d))
        if k:
            return "POSITION"
        return "VELOCITY" if d else "EFFORT (no actuator; targets ignored)"

    log(f"newton arm drive  : {_expected_mode(ARM_STIFFNESS, ARM_DAMPING)} "
        f"(from the authored DriveAPI gains)")
    if args.control_mode != "torque" and _expected_mode(ARM_STIFFNESS, ARM_DAMPING).startswith("EFFORT"):
        log(f"WARNING: control_mode={args.control_mode} but both drive gains are zero, so no "
            f"actuator is installed and the targets will be ignored. Check the profile.")

    if armatures is not None:
        worst = max(abs(float(g) - w) for g, w in zip(got_armature, armatures[0]))
        if worst > 1e-4:
            log(f"WARNING: armature read-back differs from the profile by up to {worst:.4g}; "
                f"the controller's rotor_inertia no longer matches the simulator.")

    if JOINT_FRICTION:
        # set_dof_friction_properties is a logged no-op on Newton's articulation
        # view in this release, so the profile's joint_friction cannot be applied.
        # Said plainly rather than silently dropped: it is what holds the arm
        # still at rest under PhysX, and its absence changes how the arm settles.
        log(f"newton: profile joint_friction={JOINT_FRICTION:g} NOT applied - "
            f"set_dof_friction_properties is unimplemented for Newton in this "
            f"Isaac Sim release.")


if not NEWTON:
    log(f"armatures in effect: "
        f"{[round(v, 4) for v in robot.get_dof_armatures().numpy().reshape(-1)]}")


def _full_dof_vector(arm_values, hand_values):
    """Scatter arm/hand values into a full-width (1, num_dofs) array."""
    values = [0.0] * len(dof_names)
    for name, value in zip(ARM_DOF_NAMES, arm_values):
        values[dof_names.index(name)] = value
    for name, value in zip(HAND_DOF_NAMES, hand_values):
        values[dof_names.index(name)] = value
    return [values]


HOME_DOF_VECTOR = _full_dof_vector(ARM_HOME, HAND_HOME)
ZERO_DOF_VECTOR = [[0.0] * len(dof_names)]

# Deferred to here only because it needs _full_dof_vector, which needs dof_names.
if NEWTON:
    _apply_newton_runtime_properties()


def hold_home_pose():
    """Pin the articulation at its home pose.

    Used while the gate is closed. This is a kinematic freeze rather than a stiff
    position drive because the drive gains are fixed at startup (see
    _author_drive_gains) - in torque mode they are zero, so there is nothing to
    hold with and the arm would simply fall.
    """
    robot.set_dof_positions(HOME_DOF_VECTOR)
    robot.set_dof_velocities(ZERO_DOF_VECTOR)
    robot.set_dof_position_targets(HOME_DOF_VECTOR)


hold_home_pose()

# --------------------------------------------------------------------------- #
# FT emulation
# --------------------------------------------------------------------------- #
ft_link_index = None
if args.publish_ft and not FT_LINK:
    log("WARNING: --publish-ft given but this robot profile has no 'ft_link'; FT emulation off")
    ft_node = None
elif args.publish_ft:
    link_names = list(robot.link_names)
    if FT_LINK in link_names:
        ft_link_index = link_names.index(FT_LINK)
        log(f"FT emulation on: link='{FT_LINK}' -> /{_topic(args.ft_raw_topic)}")
    else:
        log(f"WARNING: FT link '{FT_LINK}' is not in the articulation ({link_names}); "
            f"FT emulation disabled. Was the USD imported with --no-merge-fixed-joints?")
        ft_node = None


def publish_ft():
    """Write this step's joint reaction into the generic publisher's inputs.

    get_link_incoming_joint_force gives the 6D load carried by a link's incoming
    joint. bota_ft_sensor_wrench is the link the real sensor measures at, so its
    incoming joint force is the sensor reading, already in the sensor frame.
    """
    forces, torques = robot.get_link_incoming_joint_force(link_indices=[ft_link_index])
    wrench = args.ft_sign * np.concatenate(
        [forces.numpy().reshape(3), torques.numpy().reshape(3)]
    )
    og.Controller.attribute("inputs:force:x", ft_node).set(float(wrench[0]))
    og.Controller.attribute("inputs:force:y", ft_node).set(float(wrench[1]))
    og.Controller.attribute("inputs:force:z", ft_node).set(float(wrench[2]))
    og.Controller.attribute("inputs:torque:x", ft_node).set(float(wrench[3]))
    og.Controller.attribute("inputs:torque:y", ft_node).set(float(wrench[4]))
    og.Controller.attribute("inputs:torque:z", ft_node).set(float(wrench[5]))


# --------------------------------------------------------------------------- #
# Startup gate
# --------------------------------------------------------------------------- #
_gate_open = False
_gate_names_attr = og.Controller.attribute("outputs:jointNames", gate_node)


def open_gate():
    global _gate_open
    if _gate_open:
        return
    _gate_open = True
    set_command_path_enabled(True)
    log(f"command path ENABLED (control_mode={args.control_mode})")


def gate_signalled():
    """True once anything has been received on the gate topic.

    The subscriber leaves outputs:jointNames empty until a message arrives and
    keeps the last value afterwards, so a non-empty list is a latch.
    """
    names = _gate_names_attr.get()
    return names is not None and len(names) > 0


if args.no_gate:
    log("--no-gate: enabling the command path immediately")
    open_gate()
else:
    log(f"waiting for the startup gate on '/{_topic(args.gate_topic)}' before accepting commands")

log(f"running: physics={args.physics_rate:g} Hz, device={args.device}, "
    f"mode={args.control_mode}, headless={args.headless}")

# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
try:
    while simulation_app.is_running():
        simulation_app.update()
        if not _gate_open:
            if gate_signalled():
                open_gate()
            else:
                hold_home_pose()
        if ft_node is not None:
            publish_ft()
except KeyboardInterrupt:
    pass
finally:
    app_utils.stop()
    simulation_app.close()

sys.exit(0)
