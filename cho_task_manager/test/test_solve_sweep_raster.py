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

"""scripts/solve_sweep_raster.py expands the robot description without a shell.

It used to build a ``xacro <file> key:=value ...`` command line and run it with
``shell=True``, so an argument value in the raster spec was shell syntax: a
``;`` or ``|`` in it ran something else. It now calls xacro's Python API, the
call every launch file here makes, and these tests pin that.
"""

import importlib.util
from pathlib import Path
import subprocess

import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'solve_sweep_raster.py'
SWEEP_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'sweep'


@pytest.fixture(scope='module')
def solver():
    spec = importlib.util.spec_from_file_location('solve_sweep_raster', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def no_subprocess(monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError(f'a subprocess was started: {args}')
    for name in ('run', 'Popen', 'call', 'check_call', 'check_output'):
        monkeypatch.setattr(subprocess, name, refuse)


def test_an_argument_value_reaches_xacro_literally(solver, no_subprocess, monkeypatch, tmp_path):
    # Shell syntax in a value is just text to xacro now.
    description = tmp_path / 'probe.urdf.xacro'
    description.write_text(
        '<robot name="probe" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:arg name="label" default=""/>'
        '<link name="$(arg label)"/></robot>')
    monkeypatch.setattr(solver, 'share', lambda package, *parts: str(description))
    label = 'a;b && c | d > e'
    urdf = solver.expand_description(
        {'robot': {'package': 'unused', 'xacro': 'unused', 'xacro_args': {'label': label}}})
    assert f'<link name="{label.replace("&", "&amp;").replace(">", "&gt;")}"/>' in urdf


@pytest.mark.parametrize('raster', sorted(path.name for path in SWEEP_CONFIG.glob('*.raster.yaml')))
def test_the_shipped_specs_expand_to_the_arm_they_solve_for(solver, no_subprocess, raster):
    spec = yaml.safe_load((SWEEP_CONFIG / raster).read_text())
    model = solver.pin.buildModelFromXML(solver.expand_description(spec))
    assert model.nq >= spec['robot']['arm_dof']


def test_the_script_starts_no_shell():
    text = SCRIPT.read_text()
    assert 'shell=True' not in text and 'import subprocess' not in text
