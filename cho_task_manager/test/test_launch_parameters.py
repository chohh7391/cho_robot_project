"""The launch file and the node agree on defaults and on parameter types.

A launch argument arrives as a string and is YAML-typed on the way in, so
``probe_duration:=5`` used to reach the node as an INTEGER and be refused by a
parameter the node declares as a double. And the two declared different
default tasks, so starting the node directly ran something the launch never
would.
"""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cho_task_manager import task_manager_node

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'run_task_manager.launch.py'


def _description():
    spec = importlib.util.spec_from_file_location('run_task_manager_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _arguments():
    return {entity.name: entity for entity in _description().entities
            if isinstance(entity, DeclareLaunchArgument)}


def _node_parameters():
    nodes = [entity for entity in _description().entities if isinstance(entity, Node)]
    assert len(nodes) == 1
    # Node keeps the parameters it was given; the first entry is our dict.
    return nodes[0]._Node__parameters[0]


def test_the_node_and_the_launch_default_to_the_same_task():
    default = ''.join(text.text for text in _arguments()['task'].default_value)
    assert default == task_manager_node.DEFAULT_TASK == 'pick_place'


def test_double_parameters_are_typed_on_the_way_in():
    parameters = _node_parameters()
    for name in ('probe_duration', 'replay_speed_scale', 'replay_pour_grams',
                 'replay_pour_flow_index', 'replay_pour_timeout'):
        value = next(v for k, v in parameters.items()
                     if ''.join(getattr(part, 'text', '') for part in k) == name)
        assert isinstance(value, ParameterValue), name
        assert value.value_type is float, name


def test_the_home_via_description_names_the_real_default():
    from cho_task_manager.tasks.fr5.trajectory_replay import DEFAULT_HOME_VIA
    description = _arguments()['home_via'].description
    assert f'task default, {DEFAULT_HOME_VIA}' in description
