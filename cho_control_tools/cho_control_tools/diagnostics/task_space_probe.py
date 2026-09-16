#!/usr/bin/env python3
"""Build and check a small task-space probe around wherever the arm is NOW.

`task_space_ik_controller` takes an absolute wrist3_link pose and walks a
straight Cartesian line to it. That makes it easy to command and easy to get
wrong, because the two things it checks for you are not the two things that hurt:

* its ``enforce_workspace_floor`` guard tests wrist3_link's height against
  ``minimum_ee_height`` (0.15 m on the real bringup). With the AG-95 bolted on,
  the jaw envelope hangs 0.30 m BELOW wrist3_link -- 0.100 m of tool_tcp offset
  plus the 0.2008 m the gripper reaches past the flange. wrist3_link sitting
  legally at 0.15 m therefore puts the jaws 0.15 m UNDER the table. The guard is
  a workspace plane for a bare flange; it is not table clearance for this arm.
* it clamps each cycle to ``max_delta_q`` and stops at the joint limits, but it
  does no mesh or self-collision checking at all. The gripper was driven into
  the arm before its volume was modelled, and nothing in the controller would
  catch that today.

So this tool does the checking the controller does not, against the same
geometry MoveIt plans with, and only then prints poses worth sending.

    ros2 run cho_control_tools task_space_probe
    ros2 run cho_control_tools task_space_probe --offset 0.05 --scene <mjcf>
    ros2 run cho_control_tools task_space_probe --execute

Everything is measured from the CURRENT joint state, not from a stored home
pose. `cho_robot_config`'s ``motions.reach`` presets are absolute poses anchored
at home1; sending one from somewhere else is a long Cartesian move, not the
10 cm nudge the name suggests.
"""

import argparse
import sys
import xml.etree.ElementTree as ElementTree

import numpy as np

#: wrist3_link is the controllers' end-effector frame (`ee_name` in
#: controllers.yaml). Every pose printed here is that frame, so it can be pasted
#: into a TaskSpace goal unchanged.
EE_FRAME = 'wrist3_link'

#: MoveIt's own floor, from cho_moveit_fr5/config/planning_scene.yaml: a
#: 4 x 4 x 0.10 m box centred at z = -0.05, so its top face is exactly z = 0.
#: Duplicated rather than read because this tool must work against a bringup
#: that has no move_group -- but keep the two in step.
FLOOR_SIZE = (4.0, 4.0, 0.10)
FLOOR_ORIGIN = (0.0, 0.0, -0.05)

#: Pairs MoveIt is told to ignore (cho_moveit_fr5/config/fr5.srdf). Without
#: these every adjacent link reads as a collision and the report is noise.
DISABLED_PAIRS = frozenset({
    ('base_link', 'shoulder_link'), ('base_link', 'upperarm_link'),
    ('forearm_link', 'shoulder_link'), ('forearm_link', 'upperarm_link'),
    ('forearm_link', 'wrist1_link'), ('shoulder_link', 'upperarm_link'),
    ('shoulder_link', 'wrist1_link'), ('upperarm_link', 'wrist1_link'),
    ('wrist1_link', 'wrist2_link'), ('wrist1_link', 'wrist3_link'),
    ('wrist2_link', 'wrist3_link'), ('gripper_base_link', 'wrist3_link'),
})

#: Links that cannot approach the table or a vessel no matter what the arm does:
#: base_link is bolted to it, and shoulder_link only turns about the vertical j1
#: axis so its 74 mm floor clearance is a constant of the machine. Reporting
#: either as "the nearest thing to the floor" buries the clearance that moves.
GROUNDED_LINKS = frozenset({'base_link', 'shoulder_link'})

#: Per-cycle joint step the real controller allows (`max_delta_q`), at its
#: 125 Hz update rate. A leg whose IK needs more than this is not refused by the
#: controller -- it silently falls behind the Cartesian line it was asked for.
MAX_DELTA_Q = 0.005
CONTROL_HZ = 125.0

#: Damping for the DLS solve. The controller's own `lambda`, so the joint path
#: checked here is the joint path it will take.
IK_LAMBDA = 0.02

#: Clearances below these are reported as a refusal rather than a warning.
#: Not derived from anything: a hand-set margin for a first commissioning probe.
#: The self-collision figure is deliberately small because the FR5's own folded
#: wrist stands at 17 mm with no gripper involved -- a larger number would
#: refuse the arm's resting pose.
MIN_SELF_CLEARANCE = 0.010
MIN_CELL_CLEARANCE = 0.030


def _link_of(name, obstacles):
    """Link name behind a geometry name ('wrist2_link_0' -> 'wrist2_link')."""
    return name if name in obstacles else name.rsplit('_', 1)[0]


def _yaw_matrix(w, z):
    """Rotation from a (w, 0, 0, z) quaternion -- the only kind MJCF cell objects use."""
    angle = 2.0 * np.arctan2(z, w)
    cos, sin = np.cos(angle), np.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def parse_scene_obstacles(path):
    """Read the cell's worldbody geoms out of an MJCF scene.

    The MuJoCo scene is the source of truth for what is physically on the bench:
    the replay layout YAML carries only xy and yaw, and a vessel's HEIGHT is the
    number that decides whether the jaws clear it. Sizes are MuJoCo half-extents.
    The bench itself is skipped -- the floor box already covers z = 0.
    """
    import coal

    obstacles = []
    root = ElementTree.parse(path).getroot()
    world = root.find('worldbody')
    if world is None:
        return obstacles
    for geom in world.findall('geom'):
        name = geom.get('name')
        kind = geom.get('type')
        if not name or name == 'bench':
            continue
        size = [float(v) for v in (geom.get('size') or '').split()]
        pos = [float(v) for v in (geom.get('pos') or '0 0 0').split()]
        quat = [float(v) for v in (geom.get('quat') or '1 0 0 0').split()]
        if kind == 'cylinder' and len(size) >= 2:
            shape = coal.Cylinder(size[0], 2.0 * size[1])
        elif kind == 'box' and len(size) >= 3:
            shape = coal.Box(2.0 * size[0], 2.0 * size[1], 2.0 * size[2])
        elif kind == 'sphere' and size:
            shape = coal.Sphere(size[0])
        else:
            continue
        obstacles.append((name, shape, _yaw_matrix(quat[0], quat[3]), pos))
    return obstacles


class Cell:
    """The arm, its gripper, the table and whatever is standing on it."""

    def __init__(self, urdf_xml, package_dirs, obstacles=()):
        import coal
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromXML(urdf_xml)
        self.geom = pin.buildGeomFromUrdfString(
            self.model, urdf_xml, pin.GeometryType.COLLISION, None, list(package_dirs))

        self.obstacles = {}
        floor = pin.GeometryObject(
            'floor', 0, 0, pin.SE3(np.eye(3), np.array(FLOOR_ORIGIN)), coal.Box(*FLOOR_SIZE))
        self.obstacles['floor'] = self.geom.addGeometryObject(floor)
        for name, shape, rotation, position in obstacles:
            self.obstacles[name] = self.geom.addGeometryObject(pin.GeometryObject(
                name, 0, 0, pin.SE3(rotation, np.array(position, dtype=float)), shape))

        self._select_pairs()
        self.data = self.model.createData()
        self.gdata = self.geom.createData()
        self.ee = self.model.getFrameId(EE_FRAME)
        self.arm_dof = min(6, self.model.nq)
        # The jaws are a state, not a variable, for this check: hold them where
        # they are so the envelope box is placed exactly as the arm carries it.
        self.jaw = 0.0

    def _select_pairs(self):
        """Keep the pairs that can actually tell us something."""
        self.geom.addAllCollisionPairs()
        keep = []
        for pair in self.geom.collisionPairs:
            first = _link_of(self.geom.geometryObjects[pair.first].name, self.obstacles)
            second = _link_of(self.geom.geometryObjects[pair.second].name, self.obstacles)
            if first == second or tuple(sorted((first, second))) in DISABLED_PAIRS:
                continue
            in_cell = (first in self.obstacles, second in self.obstacles)
            if all(in_cell):                       # the bench against itself
                continue
            if any(in_cell) and (first in GROUNDED_LINKS or second in GROUNDED_LINKS):
                continue
            keep.append((pair.first, pair.second))
        self.geom.removeAllCollisionPairs()
        for first, second in keep:
            self.geom.addCollisionPair(self.pin.CollisionPair(first, second))

    def _full_q(self, arm_q):
        q = np.zeros(self.model.nq)
        q[:self.arm_dof] = arm_q
        if self.model.nq > self.arm_dof:
            q[self.arm_dof:] = self.jaw
        return q

    def place(self, arm_q):
        """Pose the whole model at *arm_q* and return the wrist3_link placement."""
        q = self._full_q(arm_q)
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        self.pin.updateGeometryPlacements(self.model, self.data, self.geom, self.gdata)
        return self.data.oMf[self.ee].copy()

    def clearances(self, arm_q):
        """Nearest self pair and nearest cell pair, as (label, metres) each."""
        self.place(arm_q)
        self.pin.computeDistances(self.geom, self.gdata)
        nearest = {'self': ('none', np.inf), 'cell': ('none', np.inf)}
        for index, pair in enumerate(self.geom.collisionPairs):
            first = _link_of(self.geom.geometryObjects[pair.first].name, self.obstacles)
            second = _link_of(self.geom.geometryObjects[pair.second].name, self.obstacles)
            kind = 'cell' if (first in self.obstacles or second in self.obstacles) else 'self'
            distance = self.gdata.distanceResults[index].min_distance
            if distance < nearest[kind][1]:
                nearest[kind] = ('%s <-> %s' % (first, second), distance)
        return nearest

    def jacobian(self, arm_q):
        q = self._full_q(arm_q)
        self.pin.computeJointJacobians(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.pin.getFrameJacobian(
            self.model, self.data, self.ee, self.pin.LOCAL)[:, :self.arm_dof]

    def solve_ik(self, target, seed, iterations=600):
        """Run the controller's own damped-least-squares step to convergence.

        The controller takes ONE such step per cycle from the previous
        reference; iterating the same step from the same seed lands on the same
        branch, so the joint path checked here is the joint path it will take.
        """
        arm_q = np.array(seed, dtype=float)
        lower = self.model.lowerPositionLimit[:self.arm_dof]
        upper = self.model.upperPositionLimit[:self.arm_dof]
        error = np.zeros(6)
        for _ in range(iterations):
            placement = self.place(arm_q)
            error[:3] = placement.rotation.T @ (target.translation - placement.translation)
            error[3:] = self.pin.log3(placement.rotation.T @ target.rotation)
            if np.linalg.norm(error) < 1e-11:
                break
            jac = self.jacobian(arm_q)
            damped = jac @ jac.T + (IK_LAMBDA ** 2) * np.eye(6)
            arm_q = np.clip(arm_q + jac.T @ np.linalg.solve(damped, error), lower, upper)
        return arm_q, float(np.linalg.norm(error))


class Leg:
    """One straight Cartesian move, and what the check found along it."""

    def __init__(self, name, target, arm_q):
        self.name = name
        self.target = target
        self.arm_q = arm_q
        self.ik_residual = 0.0
        self.min_self = np.inf
        self.min_cell = np.inf
        self.self_at = ''
        self.cell_at = ''
        self.min_ee_height = np.inf
        self.peak_rate = 0.0
        self.condition = 0.0
        self.refusals = []

    @property
    def safe(self):
        return not self.refusals


def check_leg(cell, name, start_pose, delta, seed, duration, floor_guard, samples=64):
    """Walk the straight line the controller will walk, checking every sample.

    Checking only the endpoint is not enough: the controller interpolates in
    CARTESIAN space, so the joint path bulges between two poses that are both
    fine on their own, and that bulge is where a self-collision lives.
    """
    leg = Leg(name, start_pose.translation + np.asarray(delta, dtype=float), np.asarray(seed))
    arm_q = np.array(seed, dtype=float)
    previous = arm_q.copy()
    step_seconds = duration / samples
    for fraction in np.linspace(0.0, 1.0, samples + 1)[1:]:
        target = start_pose.copy()
        target.translation = start_pose.translation + fraction * np.asarray(delta, dtype=float)
        arm_q, residual = cell.solve_ik(target, arm_q)
        leg.ik_residual = max(leg.ik_residual, residual)

        near = cell.clearances(arm_q)
        if near['self'][1] < leg.min_self:
            leg.min_self, leg.self_at = near['self'][1], near['self'][0]
        if near['cell'][1] < leg.min_cell:
            leg.min_cell, leg.cell_at = near['cell'][1], near['cell'][0]
        leg.min_ee_height = min(leg.min_ee_height, cell.place(arm_q).translation[2])
        leg.peak_rate = max(leg.peak_rate, float(np.abs(arm_q - previous).max()) / step_seconds)
        previous = arm_q.copy()

    leg.arm_q = arm_q
    leg.condition = float(np.linalg.cond(cell.jacobian(arm_q)))

    if leg.ik_residual > 1e-6:
        leg.refusals.append('IK did not converge (residual %.1e)' % leg.ik_residual)
    if leg.min_self < MIN_SELF_CLEARANCE:
        leg.refusals.append('self-collision clearance %.4f m at %s'
                            % (leg.min_self, leg.self_at))
    if leg.min_cell < MIN_CELL_CLEARANCE:
        leg.refusals.append('cell clearance %.4f m at %s' % (leg.min_cell, leg.cell_at))
    if leg.min_ee_height < floor_guard:
        leg.refusals.append('wrist3_link drops to %.4f m, under the controller floor '
                            'guard at %.4f m' % (leg.min_ee_height, floor_guard))
    if leg.peak_rate > MAX_DELTA_Q * CONTROL_HZ:
        leg.refusals.append('needs %.3f rad/s, over the controller cap of %.3f rad/s '
                            '(max_delta_q x %.0f Hz) -- raise --duration'
                            % (leg.peak_rate, MAX_DELTA_Q * CONTROL_HZ, CONTROL_HZ))
    return leg


def square_probe(offset):
    """Up, round a square at the raised height, back down.

    A square rather than six independent nudges: it exercises each axis, it
    closes on its own start so a drift shows up as a gap, and it is legible from
    across the room, which matters when the person holding the E-Stop has to
    decide whether what they are watching is what was asked for.
    """
    return [
        ('1 up    +z', (0.0, 0.0, offset)),
        ('2 east  +x', (offset, 0.0, 0.0)),
        ('3 south -y', (0.0, -offset, 0.0)),
        ('4 west  -x', (-offset, 0.0, 0.0)),
        ('5 north +y', (0.0, offset, 0.0)),
        ('6 down  -z', (0.0, 0.0, -offset)),
    ]


class Probe:
    """Reads the live description and joint state, then reports or sends."""

    def __init__(self, node):
        self.node = node

    # -- live state -------------------------------------------------------

    def robot_description(self, timeout_sec):
        """Return the description the CONTROLLER is running, not a re-expanded one.

        Re-running xacro would silently check a different robot whenever an
        argument differed -- `gripper:=ag95` above all, which is the whole
        reason this tool exists. robot_state_publisher latches the real one.
        """
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        from std_msgs.msg import String

        holder = {}
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = self.node.create_subscription(
            String, '/robot_description', lambda msg: holder.setdefault('xml', msg.data), qos)
        self._spin_until(lambda: 'xml' in holder, timeout_sec)
        self.node.destroy_subscription(subscription)
        return holder.get('xml')

    def joint_state(self, joint_names, timeout_sec):
        """Return the current position of *joint_names*, in that order."""
        from sensor_msgs.msg import JointState

        holder = {}

        def remember(msg):
            index = {name: i for i, name in enumerate(msg.name)}
            if all(name in index for name in joint_names):
                holder['q'] = np.array([msg.position[index[name]] for name in joint_names])

        subscription = self.node.create_subscription(JointState, '/joint_states', remember, 10)
        self._spin_until(lambda: 'q' in holder, timeout_sec)
        self.node.destroy_subscription(subscription)
        return holder.get('q')

    def controller_states(self, timeout_sec):
        """Every controller with its state and claimed interfaces, or None if unreachable."""
        from controller_manager_msgs.srv import ListControllers

        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        import rclpy
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            return None
        return list(future.result().controller)

    def _spin_until(self, done, timeout_sec):
        import rclpy
        deadline = self.node.get_clock().now().nanoseconds + timeout_sec * 1e9
        while rclpy.ok() and not done() and self.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    # -- sending ----------------------------------------------------------

    def send(self, action_name, legs, start_pose, duration):
        """Send each safe leg as an absolute TaskSpace goal, stopping on the first failure."""
        import rclpy
        from cho_interfaces.action import TaskSpace
        from geometry_msgs.msg import Pose
        from rclpy.action import ActionClient

        client = ActionClient(self.node, TaskSpace, action_name)
        if not client.wait_for_server(timeout_sec=10.0):
            print('no TaskSpace action server at %s' % action_name, file=sys.stderr)
            return False

        quaternion = _quaternion_xyzw(start_pose.rotation)
        for leg in legs:
            goal = TaskSpace.Goal()
            goal.target_pose = Pose()
            goal.duration = float(duration)
            # Absolute, because the controller resolves a RELATIVE goal against
            # its own reference, which is not where the previous leg was asked
            # to finish once any of them has been clamped.
            goal.relative = False
            (goal.target_pose.position.x,
             goal.target_pose.position.y,
             goal.target_pose.position.z) = (float(v) for v in leg.target)
            (goal.target_pose.orientation.x, goal.target_pose.orientation.y,
             goal.target_pose.orientation.z, goal.target_pose.orientation.w) = quaternion

            print('  %s -> [%+.4f %+.4f %+.4f] ... ' % (leg.name, *leg.target), end='', flush=True)
            send = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            handle = send.result()
            if handle is None or not handle.accepted:
                print('REJECTED')
                return False
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result, timeout_sec=duration + 30.0)
            outcome = result.result()
            if outcome is None:
                print('no result in %.0f s' % (duration + 30.0))
                return False
            if not outcome.result.is_completed:
                print('FAILED: %s' % (outcome.result.message or 'no reason given'))
                return False
            print('ok')
        return True


def conflicting_controllers(states, target, arm_joints):
    """Active controllers that already hold the arm's position command interfaces.

    Derived rather than listed. `claimed_interfaces` IS the exclusivity rule --
    two controllers cannot hold `j1/position` at once -- so asking the
    controller manager what is claimed beats keeping a roster of names here that
    goes stale the first time a bringup gains a controller.
    """
    wanted = {'%s/position' % joint for joint in arm_joints}
    return sorted(
        state.name for state in states
        if state.state == 'active' and state.name != target
        and wanted.intersection(state.claimed_interfaces))


def _quaternion_xyzw(rotation):
    """(x, y, z, w) for a rotation matrix, via Pinocchio so the convention matches."""
    import pinocchio as pin
    quaternion = pin.Quaternion(rotation)
    return (float(quaternion.x), float(quaternion.y),
            float(quaternion.z), float(quaternion.w))


def report(cell, start_pose, start_q, legs, floor_guard):
    """Print the anchor, then one line per leg, then the pasteable poses."""
    quaternion = _quaternion_xyzw(start_pose.rotation)
    near = cell.clearances(start_q)
    lowest = _lowest_gripper_point(cell, start_q)

    print()
    print('anchor (current %s pose)' % EE_FRAME)
    print('  position    [%+.6f, %+.6f, %+.6f]' % tuple(start_pose.translation))
    print('  orientation [%+.6f, %+.6f, %+.6f, %+.6f]  (xyzw)' % quaternion)
    print('  joints      [%s]' % ', '.join('%+.6f' % v for v in start_q))
    print('  nearest self pair  %-28s %.4f m' % (near['self'][0], near['self'][1]))
    print('  nearest cell pair  %-28s %.4f m' % (near['cell'][0], near['cell'][1]))
    if lowest is not None:
        print('  gripper lowest point at z = %+.4f m, i.e. %.4f m below %s'
              % (lowest, start_pose.translation[2] - lowest, EE_FRAME))
        print('  (the controller floor guard is %.2f m on %s, which does NOT see this)'
              % (floor_guard, EE_FRAME))

    print()
    print('%-11s %-28s %9s %9s %9s %9s %7s' %
          ('leg', 'target xyz', 'ik res', 'min self', 'min cell', 'rad/s', 'cond'))
    for leg in legs:
        print('%-11s [%+.4f %+.4f %+.4f] %9.1e %9.4f %9.4f %9.3f %7.1f %s'
              % (leg.name, *leg.target, leg.ik_residual, leg.min_self, leg.min_cell,
                 leg.peak_rate, leg.condition, 'ok' if leg.safe else 'REFUSED'))
        for reason in leg.refusals:
            print('%-11s   %s' % ('', reason))

    safe = [leg for leg in legs if leg.safe]
    print()
    print('%d of %d legs passed' % (len(safe), len(legs)))
    if safe:
        print()
        print('poses (absolute %s, same orientation throughout):' % EE_FRAME)
        for leg in safe:
            print('  %-11s position: [%+.9f, %+.9f, %+.9f]' % (leg.name, *leg.target))
        print('  %-11s orientation: [%+.9f, %+.9f, %+.9f, %+.9f]' % ('', *quaternion))
    return len(safe) == len(legs)


def _lowest_gripper_point(cell, arm_q):
    """Lowest world z of the gripper's collision geometry, or None without one.

    Reported because it is the number the controller's floor guard leaves out,
    and the one an operator actually wants before commanding a downward move.
    """
    cell.place(arm_q)
    lowest = None
    for index, obj in enumerate(cell.geom.geometryObjects):
        if not obj.name.startswith('gripper_'):
            continue
        obj.geometry.computeLocalAABB()
        placement = cell.gdata.oMg[index]
        low, high = obj.geometry.aabb_local.min_, obj.geometry.aabb_local.max_
        corners = np.array([[x, y, z] for x in (low[0], high[0])
                            for y in (low[1], high[1]) for z in (low[2], high[2])])
        world_z = ((placement.rotation @ corners.T).T + placement.translation)[:, 2]
        lowest = world_z.min() if lowest is None else min(lowest, world_z.min())
    return lowest


def _mesh_roots(override):
    """Where `package://` mesh URIs are resolved from.

    AMENT_PREFIX_PATH by default: the description is read from the running
    bringup, so the meshes it names are the ones that bringup has installed.
    Each prefix is offered both as itself and with `/share` appended, because
    Pinocchio wants the directory that CONTAINS the package folder and the two
    conventions differ between an installed and a source tree.
    """
    import os

    if override:
        return [item for item in override.split(':') if item]
    roots = []
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if not prefix:
            continue
        roots.extend((prefix, os.path.join(prefix, 'share')))
    return roots


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--offset', type=float, default=0.08,
                        help='square edge in metres (default 0.08)')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='seconds per leg, as sent in the goal (default 5.0)')
    parser.add_argument('--floor-guard', type=float, default=0.15,
                        help="the controller's minimum_ee_height (default 0.15)")
    parser.add_argument('--scene', default=None,
                        help='MJCF scene whose worldbody geoms are on the bench')
    parser.add_argument('--package-dirs', default=None,
                        help='colon-separated roots for package:// mesh lookup '
                             '(default: AMENT_PREFIX_PATH)')
    parser.add_argument('--action',
                        default='/controller_action_server/task_space_ik_controller')
    parser.add_argument('--controller', default='task_space_ik_controller')
    parser.add_argument('--execute', action='store_true',
                        help='send the safe legs instead of only printing them')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('task_space_probe')
    probe = Probe(node)
    try:
        urdf_xml = probe.robot_description(args.timeout)
        if not urdf_xml:
            print('no /robot_description in %.0f s -- is a bringup running?' % args.timeout,
                  file=sys.stderr)
            return 1

        obstacles = parse_scene_obstacles(args.scene) if args.scene else ()
        cell = Cell(urdf_xml, _mesh_roots(args.package_dirs), obstacles)
        joint_names = [cell.model.names[i] for i in range(1, cell.model.njoints)]
        arm_joints = joint_names[:cell.arm_dof]

        start_q = probe.joint_state(arm_joints, args.timeout)
        if start_q is None:
            print('no /joint_states carrying %s in %.0f s' % (arm_joints, args.timeout),
                  file=sys.stderr)
            return 1
        # Pose the jaws where they are, so the swept envelope is placed as carried.
        gripper = probe.joint_state(joint_names[cell.arm_dof:], 2.0) \
            if cell.model.nq > cell.arm_dof else None
        if gripper is not None and len(gripper):
            cell.jaw = float(gripper[0])

        print('model: %d joints%s, %d collision pairs, %d cell obstacle(s)'
              % (cell.arm_dof,
                 ', gripper present' if cell.model.nq > cell.arm_dof else ', no gripper',
                 len(cell.geom.collisionPairs), len(obstacles)))

        start_pose = cell.place(start_q)
        legs = []
        pose = start_pose.copy()
        seed = start_q
        for name, delta in square_probe(args.offset):
            leg = check_leg(cell, name, pose, delta, seed, args.duration, args.floor_guard)
            legs.append(leg)
            seed = leg.arm_q
            pose = pose.copy()
            pose.translation = leg.target
        all_safe = report(cell, start_pose, start_q, legs, args.floor_guard)

        if not args.execute:
            return 0 if all_safe else 2

        if not all_safe:
            print('\nrefusing to execute: not every leg passed', file=sys.stderr)
            return 2
        states = probe.controller_states(args.timeout)
        if states is not None and args.controller not in {
                s.name for s in states if s.state == 'active'}:
            blocking = conflicting_controllers(states, args.controller, arm_joints)
            print('\n%s is not active.\nSwitch first:\n'
                  '  ros2 control switch_controllers --activate %s%s'
                  % (args.controller, args.controller,
                     ''.join(' --deactivate %s' % name for name in blocking)),
                  file=sys.stderr)
            return 3
        print('\nsending %d legs to %s' % (len(legs), args.action))
        return 0 if probe.send(args.action, legs, start_pose, args.duration) else 4
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
