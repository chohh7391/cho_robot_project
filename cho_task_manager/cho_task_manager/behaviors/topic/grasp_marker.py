"""Measure where a just-grasped vessel sits in the jaws, for a pour later on.

A pour under `pour_geometry: measured` needs the held vessel's marker relative
to the EE. The pouring controller can measure it itself when its goal starts,
but by then the vessel may hang 0.95 m from the only camera that sees it. Right
after the jaws close it is still on the bench, 0.4 m from side_2, and the arm is
standing still for the gripper settle. So a replay measures it THERE: the next
marker pose that arrives, and the joint positions the arm is in, latched as a
pair. The pour goal carries both, and the controller carries the marker to its
present EE pose with its own kinematics -- the grasp is one rigid offset.

Both are taken from messages that arrive AFTER this behaviour starts, and it is
placed after the gripper settle, so neither describes the jaws mid-stroke or an
arm still arriving. The joints are reordered from /joint_states into the
registry's joint order, which is the controller's.
"""

import math

import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from cho_task_manager.utils.blackboard import TASK_NAMESPACE, write_client

#: Blackboard keys (TASK_NAMESPACE) the pour goal reads the grasp from.
GRASP_JOINTS_KEY = 'pour_grasp_joints'
GRASP_MARKER_KEY = 'pour_grasp_marker'


class GraspMarkerSampleBehavior(py_trees.behaviour.Behaviour):
    """Latch the next marker pose and the arm's joints with it."""

    def __init__(
        self,
        name: str,
        marker_topic: str,
        required_frame: str,
        joint_names: list,
        joint_states_topic: str = '/joint_states',
        timeout_sec: float = 5.0,
        namespace: str = TASK_NAMESPACE,
    ):
        super().__init__(name)
        if not marker_topic:
            raise ValueError(f'[{name}] marker_topic is required')
        if not required_frame:
            raise ValueError(
                f'[{name}] required_frame is required: the controller takes the marker in the '
                "arm's base frame and nothing here transforms it")
        if not joint_names:
            raise ValueError(f'[{name}] joint_names is required')
        self.marker_topic = marker_topic
        self.required_frame = required_frame
        self.joint_names = list(joint_names)
        self.joint_states_topic = joint_states_topic
        self.timeout_sec = timeout_sec
        self.namespace = namespace
        self.node = None
        self._marker = None
        self._joints = None
        self._deadline = None
        self.board = write_client(name, [GRASP_JOINTS_KEY, GRASP_MARKER_KEY], namespace)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        group = ReentrantCallbackGroup()
        # cho_object_pose publishes with the default, reliable QoS.
        self.node.create_subscription(
            PoseStamped, self.marker_topic, self._on_marker,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), callback_group=group)
        self.node.create_subscription(
            JointState, self.joint_states_topic, self._on_joints, qos_profile_sensor_data,
            callback_group=group)
        return True

    def _on_marker(self, msg):
        self._marker = msg

    def _on_joints(self, msg):
        self._joints = msg

    def initialise(self):
        self._marker = None
        self._joints = None
        self._deadline = self.node.get_clock().now() + Duration(seconds=self.timeout_sec)

    def _joint_positions(self):
        """The latest joint state in registry order, or None if it is not all there."""
        if self._joints is None:
            return None
        by_name = dict(zip(self._joints.name, self._joints.position))
        try:
            positions = [float(by_name[name]) for name in self.joint_names]
        except KeyError:
            return None
        return positions if all(math.isfinite(q) for q in positions) else None

    def update(self):
        marker, joints = self._marker, self._joint_positions()
        if marker is None or joints is None:
            if self.node.get_clock().now() > self._deadline:
                missing = []
                if marker is None:
                    missing.append(
                        f'no pose on {self.marker_topic} -- is the pose node running with the '
                        "held vessel's table, and can a side camera see the marker from the grasp?")
                if joints is None:
                    missing.append(
                        f'no complete {self.joint_states_topic} for {self.joint_names}')
                self.node.get_logger().error(
                    f'[{self.name}] grasp not measured within {self.timeout_sec:.0f} s: '
                    + '; '.join(missing))
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        frame = marker.header.frame_id
        if frame != self.required_frame:
            self.node.get_logger().error(
                f"[{self.name}] {self.marker_topic} is in '{frame}', expected "
                f"'{self.required_frame}'. Nothing here transforms frames.")
            return py_trees.common.Status.FAILURE
        p = marker.pose.position
        position = [float(p.x), float(p.y), float(p.z)]
        if not all(math.isfinite(v) for v in position):
            self.node.get_logger().error(f'[{self.name}] the marker pose is not finite')
            return py_trees.common.Status.FAILURE

        setattr(self.board, GRASP_JOINTS_KEY, joints)
        setattr(self.board, GRASP_MARKER_KEY, position)
        self.node.get_logger().info(
            f'[{self.name}] grasp measured: marker at [{p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f}] m '
            f"in '{frame}' with the arm at [{', '.join('%+.4f' % q for q in joints)}] rad")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._deadline = None
