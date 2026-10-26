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

"""Wait until cho_object_pose says an object is seen well enough.

``/perception/object_visibility`` (``cho_interfaces/ObjectVisibilityArray``)
is cho_object_pose's account, camera by camera, of every object in its table:
whether the tag is in frame, whether it passed the quality gate, whether a
higher-priority camera outranks it, and how well it decoded
(``decision_margin``). A pose being published only says a detection cleared
that node's gate, which is set for "good enough to publish". A task that moves
a camera to get a better view wants more than that, and states how much here.

SUCCESS on a snapshot in which *camera* (any camera when it is empty) reports
the object OK with a ``decision_margin`` of at least *min_decision_margin*.
FAILURE when *timeout_sec* passes first, with what each camera said last in
the log. That FAILURE is an answer, not a fault: under a Selector it is how a
search moves on to its next viewpoint (tasks/fr5/tag_search.py).
"""

import py_trees
from cho_interfaces.msg import CameraVisibility, ObjectVisibilityArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

from cho_task_manager.utils.clock import arm, deadline_after

#: cho_object_pose's default visibility_topic.
VISIBILITY_TOPIC = '/perception/object_visibility'

STATE_NAMES = {
    CameraVisibility.STATE_UNKNOWN: 'UNKNOWN',
    CameraVisibility.STATE_OK: 'OK',
    CameraVisibility.STATE_NOT_IN_FRAME: 'NOT_IN_FRAME',
    CameraVisibility.STATE_REJECTED: 'REJECTED',
    CameraVisibility.STATE_NO_TF: 'NO_TF',
    CameraVisibility.STATE_SUPPRESSED: 'SUPPRESSED',
    CameraVisibility.STATE_STALE: 'STALE',
}


def seen_well_enough(entry, camera, min_decision_margin):
    """The ``CameraVisibility`` of *entry* that satisfies the wait, or None."""
    for view in entry.cameras:
        if camera and view.camera != camera:
            continue
        if view.state == CameraVisibility.STATE_OK and view.decision_margin >= min_decision_margin:
            return view
    return None


def describe(entry):
    """Each camera's state, and its decode when it scored one: what a log line needs."""
    parts = []
    for view in entry.cameras:
        text = f'{view.camera} {STATE_NAMES.get(view.state, view.state)}'
        if view.decision_margin >= 0.0:
            text += f' (margin {view.decision_margin:.1f}, edge {view.edge_px:.1f} px)'
        parts.append(text)
    return ', '.join(parts) or 'no cameras'


class ObjectVisibilityBehavior(py_trees.behaviour.Behaviour):
    """SUCCESS once *camera* sees *object_name* with a margin of at least *min_decision_margin*."""

    def __init__(self, name: str, object_name: str, min_decision_margin: float,
                 timeout_sec: float, camera: str = '', topic: str = VISIBILITY_TOPIC):
        super().__init__(name)
        if not object_name:
            raise ValueError(f'[{name}] object_name is required')
        if not timeout_sec > 0.0:
            raise ValueError(f'[{name}] timeout_sec must be positive, got {timeout_sec}')
        self.object_name = object_name
        self.min_decision_margin = float(min_decision_margin)
        self.timeout_sec = timeout_sec
        self.camera = camera
        self.topic = topic
        self.node = None
        self._latest = None
        self._entry = None
        self._deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        # RELIABLE depth 1, as cho_object_pose publishes it: the newest
        # snapshot, never a queue of old ones.
        self.node.create_subscription(
            ObjectVisibilityArray, self.topic, self._on_visibility, QoSProfile(depth=1),
            callback_group=ReentrantCallbackGroup())
        return True

    def _on_visibility(self, msg):
        self._latest = msg

    def initialise(self):
        # A snapshot from before this tick says where the camera was, not
        # where it is now.
        self._latest = None
        self._entry = None
        # None while the node clock reads 0 (utils/clock.py); update() takes it then.
        self._deadline = deadline_after(self.node.get_clock(), self.timeout_sec)

    def update(self):
        msg = self._latest
        if msg is not None:
            entry = next((item for item in msg.objects if item.name == self.object_name), None)
            if entry is None:
                self.node.get_logger().error(
                    f"[{self.name}] cho_object_pose reports no object '{self.object_name}' "
                    f'(its table has {sorted(item.name for item in msg.objects)}); '
                    'is object_pose_config the table this task expects?')
                return py_trees.common.Status.FAILURE
            self._entry = entry
            view = seen_well_enough(entry, self.camera, self.min_decision_margin)
            if view is not None:
                self.node.get_logger().info(
                    f'[{self.name}] {self.object_name}: {view.camera} sees it, '
                    f'decision_margin {view.decision_margin:.1f} >= {self.min_decision_margin:.1f} '
                    f'(edge {view.edge_px:.1f} px)')
                return py_trees.common.Status.SUCCESS
        clock = self.node.get_clock()
        self._deadline = arm(self._deadline, clock, self.timeout_sec)
        if self._deadline is not None and clock.now() > self._deadline:
            who = self.camera or 'any camera'
            if self._entry is None:
                self.node.get_logger().warn(
                    f'[{self.name}] nothing on {self.topic} within {self.timeout_sec}s; '
                    'is cho_object_pose running (object_pose_config:=)?')
            else:
                self.node.get_logger().info(
                    f'[{self.name}] {self.object_name} not seen by {who} with '
                    f'decision_margin >= {self.min_decision_margin:.1f} within '
                    f'{self.timeout_sec}s: {describe(self._entry)}')
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._deadline = None
