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

"""The Isaac start-up order: spawners after Isaac, the gate after the controller."""

import os

from cho_bringup_common import (
    check_isaac_install,
    ISAAC_READY_MARKER,
    isaac_command_gate,
    isaac_controller_startup,
    isaac_sim_command,
    isaac_sim_process,
    make_spawner_node,
    start_on_output,
)
import cho_bringup_common.isaac as isaac_module
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.events.process import ProcessExited, ProcessStderr, ProcessStdout
import pytest


@pytest.fixture(autouse=True)
def isaac_share(monkeypatch, tmp_path):
    # cho_simulation_isaac need not be built for these tests.
    share = tmp_path / 'cho_simulation_isaac'
    monkeypatch.setattr(isaac_module, 'isaac_share', lambda: str(share))
    return share


def process():
    return ExecuteProcess(cmd=['true'])


def event(cls, action, **kwargs):
    return cls(action=action, name='x', cmd=[], cwd=None, env=None, pid=1, **kwargs)


def test_command_line_order(isaac_share):
    cmd = isaac_sim_command('/isaac/python.sh', 'r.usda', 'p.json', 'torque', '250', 'cpu',
                            headless=True, extra_args=['--physics-engine', 'newton'])
    assert cmd == [
        '/isaac/python.sh', os.path.join(str(isaac_share), 'isaac', 'run_isaac_sim.py'),
        '--robot-usd', 'r.usda', '--robot-profile', 'p.json', '--control-mode', 'torque',
        '--physics-rate', '250', '--device', 'cpu', '--physics-engine', 'newton', '--headless']
    assert '--headless' not in isaac_sim_command('p', 'r', 'j', 'position', '1', 'cpu')


def test_missing_interpreter_says_which_argument_to_pass(tmp_path):
    with pytest.raises(RuntimeError, match='isaac_sim_path:='):
        check_isaac_install(str(tmp_path / 'nowhere'), 'r.usda', [])


def test_missing_usd_prints_a_runnable_shell_quoted_build_command(tmp_path, isaac_share):
    (tmp_path / 'python.sh').write_text('')
    with pytest.raises(RuntimeError) as error:
        check_isaac_install(str(tmp_path), str(tmp_path / 'r.usda'),
                            ['--urdf', '/a b/r.xacro', '--strip-links', '^world$'])
    message = str(error.value)
    convert = os.path.join(str(isaac_share), 'isaac', 'convert_urdf_to_usd.py')
    assert "--urdf '/a b/r.xacro' --strip-links '^world$'" in message
    assert f'{tmp_path}/python.sh {convert}' in message


def test_install_check_returns_the_interpreter(tmp_path):
    (tmp_path / 'python.sh').write_text('')
    (tmp_path / 'r.usda').write_text('')
    assert check_isaac_install(str(tmp_path), str(tmp_path / 'r.usda'), []) == \
        str(tmp_path / 'python.sh')


def test_isaac_process_shuts_the_launch_down_and_takes_extra_env():
    proc = isaac_sim_process(['a'], additional_env={'ISAAC_SIM_PATH': '/i'})
    assert proc._ExecuteLocal__on_exit is not None
    assert proc.process_description.additional_env


def test_start_on_output_fires_once_and_only_on_the_marker():
    target = process()
    actions = [make_spawner_node(['a'])]
    handler = start_on_output(target, 'READY', actions, stdout=True, stderr=True).event_handler
    context = LaunchContext()
    assert handler.handle(event(ProcessStdout, target, text=b'loading'), context) is None
    assert handler.handle(event(ProcessStderr, target, text=b'.. READY ..'), context) == actions
    assert handler.handle(event(ProcessStdout, target, text=b'READY'), context) is None


def test_start_on_output_ignores_a_stream_it_does_not_watch():
    target = process()
    handler = start_on_output(target, 'READY', [], stdout=True, stderr=False).event_handler
    assert handler.handle(event(ProcessStderr, target, text=b'READY'), LaunchContext()) is None


def test_spawners_wait_for_isaac_and_the_gate_for_a_successful_spawner():
    isaac = process()
    active = make_spawner_node(['jsb', 'arm'])
    spawners = [active]
    gate = isaac_command_gate({'use_sim_time': True})
    on_marker, on_exit = (h.event_handler for h in isaac_controller_startup(isaac, spawners, active, gate))
    context = LaunchContext()

    assert on_marker.handle(event(ProcessStdout, isaac, text=b'booting'), context) is None
    marker = f'{ISAAC_READY_MARKER} 250 Hz'.encode()
    assert on_marker.handle(event(ProcessStdout, isaac, text=marker), context) == spawners

    assert on_exit.handle(event(ProcessExited, active, returncode=0), context) == [gate]
    # A failed spawner leaves the controller inactive; opening the gate then
    # would hand Isaac the zero commands the gate exists to keep from it.
    assert on_exit.handle(event(ProcessExited, active, returncode=1), context) is None


def test_gate_node():
    gate = isaac_command_gate({'use_sim_time': True})
    assert gate._Node__package == 'cho_simulation_isaac'
    assert gate._Node__node_executable == 'isaac_command_gate.py'
