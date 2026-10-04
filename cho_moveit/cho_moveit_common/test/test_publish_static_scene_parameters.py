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

import importlib.util
from pathlib import Path

import rclpy


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'publish_static_scene.py'
SPEC = importlib.util.spec_from_file_location('publish_static_scene_under_test', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_optional_controller_arrays_are_initialized_when_overrides_are_omitted():
    rclpy.init()
    node = None
    try:
        node = MODULE.StaticSceneGate()
        assert node.get_parameter('activate_controllers').value == []
        assert node.get_parameter('deactivate_controllers').value == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
