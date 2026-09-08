#!/usr/bin/env python3
"""Measure MoveIt planning time per pipeline, for the cuRobo/cuMotion evaluation.

Sends plan-only MoveGroup goals that mirror what
`cho_moveit_common/scripts/moveit_action_bridge.py` builds, so the numbers are
comparable to what the Cho actions actually experience. Nothing is executed:
`planning_options.plan_only` is True, so the robot never moves.

Run against a live MoveIt stack, e.g.

    ros2 launch cho_bringup_fr5 bringup_gz_moveit.launch.py \
        gazebo_gui:=false launch_rviz:=false

    python3 todo/curobo_bench/bench_planning.py --robot fr5 --pipeline ompl \
        --out todo/curobo_bench/ompl_baseline.csv

Later, the same command with `--pipeline isaac_ros_cumotion` produces the row set
that Step 3 of `todo/CUROBO_MOVEIT_TODO.md` judges against.
"""

import argparse
import csv
import math
import os
import random
import statistics
import sys
import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from cho_robot_config import load_robot_config


# The bridge's own request shape (moveit_action_bridge.py). Keep in sync.
POSITION_TOLERANCE = 0.005
ORIENTATION_TOLERANCE = 0.01
JOINT_TOLERANCE = 0.001
NUM_PLANNING_ATTEMPTS = 5

OBSTACLE_ID = 'bench_obstacle'
FLOOR_ID = 'floor'


class PlanningBench(Node):

    def __init__(self, robot_type, pipeline, planning_time, scaling=None):
        super().__init__('planning_bench')
        config = load_robot_config(robot_type)
        self._config = config
        self._group = config['moveit']['planning_group']
        self._ee_link = config['model']['ee_link']
        self._world_frame = config['model']['base_frame']
        self._joint_names = list(config['model']['joints'])
        # The robot config's value is the default; --scaling overrides both.
        # Note the ceiling that actually binds the MoveIt path is
        # cho_moveit_<robot>/config/joint_limits.yaml, not this factor.
        if scaling is None:
            self._velocity_scaling = \
                config['moveit']['execution']['max_velocity_scaling_factor']
            self._acceleration_scaling = \
                config['moveit']['execution']['max_acceleration_scaling_factor']
        else:
            self._velocity_scaling = scaling
            self._acceleration_scaling = scaling
        self.scaling = self._velocity_scaling
        self._pipeline = pipeline
        self._planning_time = planning_time

        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._validity_client = self.create_client(
            GetStateValidity, '/check_state_validity')

    # ---------------------------------------------------------------- setup

    def wait_for_stack(self, timeout):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('/move_action did not appear')
        if not self._scene_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('/apply_planning_scene did not appear')
        if not self._validity_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('/check_state_validity did not appear')

    # ---------------------------------------------------------------- goals

    def joint_goals(self):
        """Home presets, minus the ones the robot config marks unusable."""
        goals = []
        for key, positions in sorted(self._config['poses']['home'].items()):
            safety = self._config['poses'].get('home_safety', {}).get(key)
            if safety is not None and safety.get('enabled') is False:
                continue
            goals.append((f'home{key}', list(positions)))
        return goals

    def pose_goals(self, random_count, seed, anchor_override=None):
        goals = []
        reach = self._config['motions']['reach']
        for key, motion in sorted(reach.items()):
            if motion.get('relative', False):
                continue
            goals.append((f'reach{key}',
                          list(motion['position']), list(motion['orientation'])))

        # Random poses share the reach presets' orientation and are drawn from a
        # box around them, so they stay in the same part of the workspace rather
        # than sampling mostly-unreachable space.
        orientation = list(reach[sorted(reach)[0]]['orientation'])
        if anchor_override is not None:
            # A hard scene is built around a specific spot, so the goal cloud has
            # to move with it. The reach presets stay in the list but the random
            # goals are drawn around the override instead.
            anchor = list(anchor_override)
            goals = [('anchor', list(anchor_override), orientation)]
        else:
            anchor = list(reach[sorted(reach)[0]]['position'])
        rng = random.Random(seed)
        for index in range(random_count):
            position = [
                anchor[0] + rng.uniform(-0.15, 0.15),
                anchor[1] + rng.uniform(-0.15, 0.15),
                anchor[2] + rng.uniform(-0.20, 0.10),
            ]
            goals.append((f'rand{index:02d}', position, orientation))
        return goals

    # ----------------------------------------------------------- constraints

    def joint_constraints(self, positions):
        constraints = Constraints()
        for name, value in zip(self._joint_names, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = JOINT_TOLERANCE
            joint.tolerance_below = JOINT_TOLERANCE
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def pose_constraints(self, position, orientation):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in position]
        # cho_robot_config stores orientation as [x, y, z, w].
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
            [float(v) for v in orientation]

        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self._world_frame
        position_constraint.link_name = self._ee_link
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[POSITION_TOLERANCE]))
        position_constraint.constraint_region.primitive_poses.append(pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self._world_frame
        orientation_constraint.link_name = self._ee_link
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    # ---------------------------------------------------------------- scene

    def apply_scene_diff(self, scene):
        scene.is_diff = True
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None or not future.result().success:
            raise RuntimeError('apply_planning_scene failed')

    def add_obstacle(self, boxes):
        """Add world-frame boxes, each (x, y, z, dx, dy, dz), as one object."""
        obstacle = CollisionObject()
        obstacle.header.frame_id = self._world_frame
        obstacle.id = OBSTACLE_ID
        obstacle.operation = CollisionObject.ADD
        for x, y, z, dx, dy, dz in boxes:
            obstacle.primitives.append(
                SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[dx, dy, dz]))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            obstacle.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(obstacle)
        self.apply_scene_diff(scene)

    def start_state_valid(self, positions):
        """True when the benchmark's fixed start state is collision-free.

        An obstacle overlapping the start state makes every plan fail before the
        planner runs, which measures the obstacle placement rather than the
        planner. The benchmark refuses to report such a scene.
        """
        request = GetStateValidity.Request()
        request.group_name = self._group
        request.robot_state.joint_state.name = list(self._joint_names)
        request.robot_state.joint_state.position = [float(v) for v in positions]
        request.robot_state.is_diff = False
        future = self._validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            raise RuntimeError('check_state_validity did not answer')
        return bool(response.valid)

    def remove_object(self, object_id):
        removal = CollisionObject()
        removal.header.frame_id = self._world_frame
        removal.id = object_id
        removal.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects.append(removal)
        self.apply_scene_diff(scene)

    # ----------------------------------------------------------------- plan

    def _goal_msg(self, constraints, start_positions):
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.pipeline_id = self._pipeline
        goal.request.num_planning_attempts = NUM_PLANNING_ATTEMPTS
        goal.request.allowed_planning_time = self._planning_time
        goal.request.max_velocity_scaling_factor = self._velocity_scaling
        goal.request.max_acceleration_scaling_factor = self._acceleration_scaling
        goal.request.goal_constraints = [constraints]
        # Fixed start state: every trial plans the same problem regardless of
        # where the simulator happens to be sitting.
        goal.request.start_state.joint_state.name = list(self._joint_names)
        goal.request.start_state.joint_state.position = [float(v) for v in start_positions]
        goal.request.start_state.is_diff = False
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        return goal

    def plan_once(self, constraints, start_positions):
        """Return (success, planning_time_s, wall_time_s, traj_duration_s, error_code)."""
        goal = self._goal_msg(constraints, start_positions)
        started = time.perf_counter()

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=60.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, float('nan'), time.perf_counter() - started, float('nan'), -1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        wall = time.perf_counter() - started
        wrapper = result_future.result()
        if wrapper is None:
            return False, float('nan'), wall, float('nan'), -1

        result = wrapper.result
        code = int(result.error_code.val)
        # Trajectory duration turns planning time into a fraction of the whole
        # motion cycle, which is what decides whether planning is the bottleneck.
        points = result.planned_trajectory.joint_trajectory.points
        if points:
            stamp = points[-1].time_from_start
            duration = stamp.sec + stamp.nanosec * 1e-9
        else:
            duration = float('nan')
        return code == 1, float(result.planning_time), wall, duration, code


def summarize(rows):
    keys = sorted({(row['scene'], row['goal_type']) for row in rows})
    lines = []
    for scene, goal_type in keys:
        subset = [r for r in rows if r['scene'] == scene and r['goal_type'] == goal_type]
        ok = [r for r in subset if r['success']]
        times = sorted(r['planning_time_s'] for r in ok
                       if not math.isnan(r['planning_time_s']))
        if times:
            median = statistics.median(times)
            p95 = times[min(len(times) - 1, int(round(0.95 * (len(times) - 1))))]
            mean = statistics.fmean(times)
        else:
            median = p95 = mean = float('nan')
        durations = [r['traj_duration_s'] for r in ok
                     if not math.isnan(r['traj_duration_s'])]
        traj_median = statistics.median(durations) if durations else float('nan')
        share = median / (median + traj_median) * 100 if durations and times else float('nan')
        lines.append(
            f'{scene:<16} {goal_type:<6} n={len(subset):<4} '
            f'success={len(ok)}/{len(subset)} '
            f'median={median * 1000:8.1f} ms  mean={mean * 1000:8.1f} ms  '
            f'p95={p95 * 1000:8.1f} ms  traj={traj_median:5.2f} s  '
            f'plan_share={share:4.1f}%')
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='fr5')
    parser.add_argument('--pipeline', default='ompl')
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--random-goals', type=int, default=12)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--planning-time', type=float, default=5.0)
    parser.add_argument('--scenes', default='floor,floor_obstacle',
                        help='comma separated subset of: empty, floor, floor_obstacle')
    parser.add_argument('--warmup', type=int, default=2,
                        help='discarded plans before measuring (GPU planners need this)')
    parser.add_argument(
        '--obstacle',
        default=('-0.30,0.18,0.55,0.10,0.10,0.50;'
                 '-0.30,-0.32,0.55,0.10,0.10,0.50;'
                 '-0.40,-0.10,0.60,0.04,0.50,0.50;'
                 '-0.15,-0.10,1.02,0.60,0.60,0.06'),
        help='floor_obstacle boxes, "x,y,z,dx,dy,dz" separated by ";"')
    parser.add_argument(
        '--start-joints', default=None,
        help='comma separated start configuration. Default: the first usable home '
             'preset. A hard scene usually needs a start pose that leaves the '
             'workspace free - see todo/curobo_bench/README.md.')
    parser.add_argument(
        '--goal-anchor', default=None,
        help='comma separated x,y,z the random goals are drawn around, replacing '
             'the reach preset anchor. Use with a scene built around that spot.')
    parser.add_argument('--scaling', type=float, default=None,
                        help='override velocity AND acceleration scaling '
                             '(default: the robot config value)')
    parser.add_argument('--out', default='todo/curobo_bench/ompl_baseline.csv')
    args = parser.parse_args()
    obstacle_box = []
    for chunk in args.obstacle.split(';'):
        values = [float(v) for v in chunk.split(',')]
        if len(values) != 6:
            parser.error('each --obstacle box needs six comma separated values')
        obstacle_box.append(values)

    rclpy.init()
    node = PlanningBench(args.robot, args.pipeline, args.planning_time, args.scaling)
    try:
        node.wait_for_stack(timeout=60.0)

        if args.start_joints:
            start_state = [float(v) for v in args.start_joints.split(',')]
        else:
            start_state = node.joint_goals()[0][1]  # first usable home preset
        anchor = ([float(v) for v in args.goal_anchor.split(',')]
                  if args.goal_anchor else None)
        joint_goals = node.joint_goals()
        pose_goals = node.pose_goals(args.random_goals, args.seed, anchor)

        node.get_logger().info(
            f'pipeline={args.pipeline} scaling={node.scaling} '
            f'joint_goals={len(joint_goals)} '
            f'pose_goals={len(pose_goals)} repeats={args.repeats}')

        # Warm up: the first plans of any pipeline are not representative
        # (allocation, and for GPU planners kernel load / graph build).
        warm_constraints = node.pose_constraints(pose_goals[0][1], pose_goals[0][2])
        for _ in range(args.warmup):
            node.plan_once(warm_constraints, start_state)

        rows = []
        for scene in [s.strip() for s in args.scenes.split(',') if s.strip()]:
            if scene == 'empty':
                node.remove_object(FLOOR_ID)
            elif scene == 'floor_obstacle':
                node.add_obstacle(obstacle_box)
            if not node.start_state_valid(start_state):
                if scene == 'floor_obstacle':
                    node.remove_object(OBSTACLE_ID)
                raise RuntimeError(
                    f'start state is in collision in scene {scene!r}; every plan '
                    'would fail before the planner runs. NOTE this only checks '
                    'MoveIt\'s mesh model - cuRobo\'s spheres are more '
                    'conservative, so check that side too '
                    '(todo/curobo_bench/README.md).')
            node.get_logger().info(f'--- scene: {scene}')

            for trial in range(args.repeats):
                for name, positions in joint_goals:
                    constraints = node.joint_constraints(positions)
                    success, plan_t, wall_t, traj_t, code = node.plan_once(
                        constraints, start_state)
                    rows.append(dict(pipeline=args.pipeline, scaling=node.scaling,
                                     scene=scene, goal_type='joint',
                                     goal_id=name, trial=trial, success=success,
                                     planning_time_s=plan_t, wall_time_s=wall_t,
                                     traj_duration_s=traj_t, error_code=code))
                for name, position, orientation in pose_goals:
                    constraints = node.pose_constraints(position, orientation)
                    success, plan_t, wall_t, traj_t, code = node.plan_once(
                        constraints, start_state)
                    rows.append(dict(pipeline=args.pipeline, scaling=node.scaling,
                                     scene=scene, goal_type='pose',
                                     goal_id=name, trial=trial, success=success,
                                     planning_time_s=plan_t, wall_time_s=wall_t,
                                     traj_duration_s=traj_t, error_code=code))
                node.get_logger().info(f'    trial {trial + 1}/{args.repeats} done')

            if scene == 'floor_obstacle':
                node.remove_object(OBSTACLE_ID)

        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        print(f'\nwrote {len(rows)} rows to {args.out}\n')
        for line in summarize(rows):
            print(line)

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
