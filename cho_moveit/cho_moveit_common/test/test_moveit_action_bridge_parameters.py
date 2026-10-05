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

"""Real-node construction guards: optional array parameters and the node name."""

import importlib.util
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import pytest
import rclpy


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'moveit_action_bridge.py'
SPEC = importlib.util.spec_from_file_location('moveit_action_bridge_under_test', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_single_controller_launches_initialize_the_optional_array():
    # Only the OpenArm launch passes 'trajectory_controllers'; every other robot
    # brings the bridge up with the singular name alone.
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5'])
    node = None
    try:
        node = MODULE.MoveItActionBridge()
        assert node.get_parameter('trajectory_controllers').value == []
        assert node._trajectory_controllers == ['joint_trajectory_controller']
        # ~/joint_space and ~/task_space resolve to what the registry lists.
        assert node._joint_action == '/fr5_moveit_action_bridge/joint_space'
        assert node._task_action == '/fr5_moveit_action_bridge/task_space'
        # The planning budget is a parameter of its own; duration_sec is the
        # motion's minimum length, not a planning budget.
        assert node._planning_time == 5.0
        assert node._execute_action == '/execute_trajectory'
        # An absolute goal may also name the registry's arm_base_link, where TF
        # puts it at the planning frame.
        assert node._arm_base_link == 'base_link'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('remap', [
    [],  # the executable's own default node name
    ['-r', '__node:=ur5e_moveit_action_bridge'],  # another robot's bridge
    ['-r', '__ns:=/elsewhere', '-r', '__node:=fr5_moveit_action_bridge'],
])
def test_a_bridge_under_a_name_no_client_looks_for_refuses_to_start(remap):
    rclpy.init(args=['--ros-args', *remap, '-p', 'robot_type:=fr5'])
    try:
        with pytest.raises(ValueError, match='/fr5_moveit_action_bridge/joint_space'):
            MODULE.MoveItActionBridge()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('value', ['0.0', '-1.0'])
def test_a_planning_budget_that_cannot_plan_is_refused(value):
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5', '-p', f'planning_time_sec:={value}'])
    try:
        with pytest.raises(ValueError, match='planning_time_sec'):
            MODULE.MoveItActionBridge()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('extra,topic', [
    ([], '/trajectory_execution_event'),
    (['-p', 'execute_trajectory_action:=/arm/execute_trajectory'],
     '/arm/trajectory_execution_event'),
    (['-p', 'trajectory_execution_event_topic:=/other/event'], '/other/event'),
])
def test_the_stop_is_published_where_move_group_listens_for_it(extra, topic):
    # TrajectoryExecutionManager subscribes on move_group's node, beside its
    # execute_trajectory action; Humble ignores a cancel of that action.
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5', *extra])
    node = None
    try:
        node = MODULE.MoveItActionBridge()
        assert node._stop_publisher.topic_name == topic
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_the_bridge_shuts_down_cleanly_on_sigint_and_sigterm(signum):
    # It handles both itself, so that it can still publish "stop" for an
    # execution in flight before its context goes away (rclpy's own SIGINT
    # handler invalidates the context first). Started with SIGINT ignored, as
    # a background job of a script is, which rclpy's handler used to override
    # and this bridge must too.
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), '--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
         '-p', 'robot_type:=fr5'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env={**os.environ, 'RCUTILS_LOGGING_BUFFERED_STREAM': '0', 'PYTHONUNBUFFERED': '1'},
        preexec_fn=_ignore_sigint)
    try:
        deadline = time.monotonic() + 20.0
        started = False
        while not started and time.monotonic() < deadline:
            if select.select([process.stdout], [], [], 0.5)[0]:
                line = process.stdout.readline()
                started = 'waiting for its floor/JTC identity gate' in line
            assert process.poll() is None, 'the bridge exited before it started'
        assert started
        process.send_signal(signum)
        assert process.wait(timeout=10.0) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
