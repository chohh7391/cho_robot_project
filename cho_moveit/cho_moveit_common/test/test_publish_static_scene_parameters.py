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
