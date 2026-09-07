"""
Tests for the launch parameter table.

The table in scale.launch.py is the single source of truth for both the
launch arguments and the node parameter overrides, so the things worth
checking are that it stays internally consistent and that a launch argument
(always a string) survives the trip back to the type the node declared.
"""
import importlib.util
import os

import pytest

LAUNCH_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'launch', 'scale.launch.py')


def load_launch_module():
    spec = importlib.util.spec_from_file_location('scale_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def launch_module():
    return load_launch_module()


def test_parameter_names_are_unique(launch_module):
    names = [param['name'] for param in launch_module.configurable_parameters]
    assert len(names) == len(set(names))


def test_every_entry_is_fully_specified(launch_module):
    for param in launch_module.configurable_parameters:
        assert set(param) == {'name', 'default', 'type', 'description'}, param['name']
        assert isinstance(param['default'], str), param['name']
        assert param['description'], param['name']


def test_every_default_coerces_to_its_declared_type(launch_module):
    for param in launch_module.configurable_parameters:
        if param['default'] == '':
            continue
        value = launch_module.coerce(param['default'], param['type'])
        assert isinstance(value, param['type']), param['name']


def test_launch_only_arguments_all_exist_in_the_table(launch_module):
    names = {param['name'] for param in launch_module.configurable_parameters}
    assert launch_module.LAUNCH_ONLY <= names


def test_declared_arguments_cover_the_whole_table(launch_module):
    declared = launch_module.declare_configurable_parameters(
        launch_module.configurable_parameters)
    assert len(declared) == len(launch_module.configurable_parameters)


@pytest.mark.parametrize('text,expected', [
    ('true', True), ('True', True), ('1', True), ('yes', True), ('on', True),
    ('false', False), ('FALSE', False), ('0', False), ('no', False), ('off', False),
])
def test_boolean_coercion(launch_module, text, expected):
    assert launch_module.coerce(text, bool) is expected


def test_non_boolean_text_for_a_boolean_is_an_error(launch_module):
    with pytest.raises(RuntimeError):
        launch_module.coerce('maybe', bool)


def test_numeric_coercion(launch_module):
    assert launch_module.coerce('2400', int) == 2400
    assert launch_module.coerce('-1.0', float) == -1.0


def test_baudrate_default_matches_the_confirmed_hs_aa_rate(launch_module):
    entry = next(param for param in launch_module.configurable_parameters
                 if param['name'] == 'baudrate')
    assert launch_module.coerce(entry['default'], entry['type']) == 2400
