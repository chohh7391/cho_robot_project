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

"""Real-node guards: construction (optional array parameters, node name) and the readiness query."""

import importlib.util
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


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
        # A lost readiness response is given up on after this long [s].
        assert node._ready_query_timeout == 3.0
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


@pytest.mark.parametrize('value', ['0.0', '-1.0', '.nan', '.inf'])
def test_a_ready_query_timeout_that_cannot_expire_is_refused(value):
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5', '-p', f'ready_query_timeout_sec:={value}'])
    try:
        with pytest.raises(ValueError, match='ready_query_timeout_sec'):
            MODULE.MoveItActionBridge()
    finally:
        rclpy.shutdown()


def _wait_until(condition, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


def test_a_lost_readiness_response_does_not_keep_the_bridge_from_becoming_ready():
    # A response that never arrives used to leave the query pending for good:
    # no second request, so no READY and no actions even once the server
    # answers. The Trigger server here takes each request and holds its
    # response until released.
    rclpy.init(args=['--ros-args', '-r', '__node:=fr5_moveit_action_bridge',
                     '-p', 'robot_type:=fr5', '-p', 'ready_query_timeout_sec:=0.6'])
    release = threading.Event()
    asked = []
    bridge = servers = None
    executors = []
    threads = []
    try:
        servers = rclpy.create_node('fake_readiness_servers')
        group = ReentrantCallbackGroup()

        def trigger(_request, response):
            asked.append(time.monotonic())
            release.wait(timeout=30.0)
            response.success = True
            return response

        def controllers(_request, response):
            response.controller = [
                ControllerState(name='joint_trajectory_controller', state='active')]
            return response

        def scene(_request, response):
            response.scene.world.collision_objects = [CollisionObject(id='floor')]
            return response

        servers.create_service(Trigger, '/static_scene_ready', trigger, callback_group=group)
        servers.create_service(
            ListControllers, '/controller_manager/list_controllers', controllers,
            callback_group=group)
        servers.create_service(GetPlanningScene, '/get_planning_scene', scene, callback_group=group)
        bridge = MODULE.MoveItActionBridge()
        for node in (servers, bridge):
            executor = MultiThreadedExecutor(num_threads=6)
            executor.add_node(node)
            executors.append(executor)
            threads.append(threading.Thread(target=executor.spin, daemon=True))
            threads[-1].start()

        # Asked again and again while nothing is answered, and what it gave up
        # on does not pile up in the client.
        assert _wait_until(lambda: len(asked) >= 3, 10.0)
        assert not bridge._ready
        assert len(bridge._ready_client._pending_requests) <= 1

        release.set()
        assert _wait_until(lambda: bridge._ready, 10.0)
        assert bridge._joint_server is not None
    finally:
        release.set()
        for executor in executors:
            executor.shutdown(timeout_sec=2.0)
        for thread in threads:
            thread.join(timeout=5.0)
        for node in (bridge, servers):
            if node is not None:
                node.destroy_node()
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
