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

"""Every TaskSpace goal the commissioning tools send names its frame.

cho_interfaces/CONTRACT.md (Frames): an absolute goal is stamped with the
profile's absolute goal frame and a relative one with its EE frame, '' where
the registry declares none. These tools left frame_id '' throughout.
"""

import numpy as np
import pytest

from cho_control_tools.diagnostics import task_space_probe
from cho_control_tools.openarm_task import goal as openarm_goal
from cho_control_tools.openarm_task.waypoints import arm_names


def _registry_frame(robot_type, profile, relative):
    """The registry's frame for this profile, or None where it is not installed."""
    try:
        from cho_robot_config import load_robot_config, task_goal_frame
    except ImportError:
        return None
    return task_goal_frame(load_robot_config(robot_type, profile), relative)


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _RecordingActionClient:
    """Takes the goal and answers with no handle, which the tools read as a rejection."""

    def __init__(self, *_args, **_kwargs):
        self.goals = []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, **_kwargs):
        self.goals.append(goal)
        return _Future(None)


# arm, relative, expected frame
OPENARM_FRAMES = [
    ('single', False, 'world'),
    ('single', True, ''),
    ('left', False, 'world'),
    ('left', True, 'openarm_left_hand_tcp'),
    ('right', False, 'world'),
    ('right', True, 'openarm_right_hand_tcp'),
]


@pytest.mark.parametrize('arm,relative,expected', OPENARM_FRAMES)
def test_openarm_task_goal_is_stamped_with_the_profile_frame(monkeypatch, arm, relative, expected):
    monkeypatch.setattr(openarm_goal.rclpy, 'spin_until_future_complete',
                        lambda *_args, **_kwargs: None)
    client = object.__new__(openarm_goal.TaskGoalClient)
    client.names = arm_names(arm)
    client.action = _RecordingActionClient()

    status, _text = client.send([0.3, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0], 5.0, relative)

    assert status is None                       # "rejected" by the fake
    (goal,) = client.action.goals
    assert goal.relative is relative
    assert goal.target_pose.header.frame_id == expected
    registry = _registry_frame('openarm', arm, relative)
    if registry is not None:
        assert expected == registry


def test_task_space_probe_stamps_every_leg_with_the_fr5_absolute_goal_frame(monkeypatch):
    import rclpy
    import rclpy.action

    recorder = _RecordingActionClient()
    monkeypatch.setattr(rclpy.action, 'ActionClient', lambda *_args, **_kwargs: recorder)
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *_args, **_kwargs: None)
    start = type('Pose', (), {'rotation': np.eye(3)})()
    leg = task_space_probe.Leg('1 up    +z', np.array([0.1, 0.2, 0.3]), np.zeros(6))

    assert task_space_probe.Probe(node=None).send(
        '/task_space_ik_controller/task_space', [leg], start, 5.0) is False

    (goal,) = recorder.goals
    assert goal.relative is False
    assert goal.target_pose.header.frame_id == 'base_link'
    registry = _registry_frame('fr5', 'single', False)
    if registry is not None:
        assert registry == 'base_link'
