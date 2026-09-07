"""
Tests for the node's declared interface.

These construct the node but never transition it, so no serial port is
needed. What they pin down is the wiring that is easy to break silently:
which parameters exist, and whether the standard parameter services survive
the node's own attribute assignments.
"""
import os

import pytest

rclpy = pytest.importorskip('rclpy')
scale_node = pytest.importorskip('hansung_scale_driver.scale_node')

DYNAMIC_PARAMS = scale_node.DYNAMIC_PARAMS
TOPICS = scale_node.TOPICS
ScaleNode = scale_node.ScaleNode

RCLPY_BUILTIN_PARAMS = {'use_sim_time'}

PARAMETER_SERVICES = {
    'describe_parameters', 'get_parameter_types', 'get_parameters',
    'list_parameters', 'set_parameters', 'set_parameters_atomically',
}


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = ScaleNode()
    yield node
    node.destroy_node()


def settled(node, query, attempts=50, delay=0.02):
    """
    Poll a graph query until it returns something.

    The graph is eventually consistent: a query issued right after a node is
    created can come back empty, which would fail these assertions for a
    reason that has nothing to do with what they are checking.
    """
    for _ in range(attempts):
        result = query()
        if result:
            return result
        rclpy.spin_once(node, timeout_sec=delay)
    return query()


def basenames(pairs):
    return {name.rsplit('/', 1)[-1] for name, _ in pairs}


def service_basenames(node):
    return basenames(settled(
        node, lambda: node.get_service_names_and_types_by_node(node.get_name(), '/')))


def test_parameter_services_are_advertised(node):
    # Regression: assigning `self._services = []` in __init__ shadowed
    # rclpy.Node's own service list, dropping its references to these six.
    # They vanished off the graph and `ros2 param set` timed out with nothing
    # logged.
    assert PARAMETER_SERVICES <= service_basenames(node)


def test_lifecycle_services_are_advertised(node):
    assert {'change_state', 'get_state'} <= service_basenames(node)


def test_node_starts_unconfigured_with_no_port_open(node):
    assert node.conn is None


@pytest.mark.parametrize('name', sorted(DYNAMIC_PARAMS))
def test_every_dynamic_parameter_is_actually_declared(node, name):
    assert node.has_parameter(name)


@pytest.mark.parametrize('name', ['port', 'serial_no', 'usb_port_id', 'device_type',
                                  'wait_for_device_timeout', 'reconnect_timeout',
                                  'initial_reset', 'baudrate', 'bytesize', 'parity',
                                  'stopbits', 'timeout', 'line_ending', 'poll_mode',
                                  'enable_commands', 'diagnostics_period', 'autostart'])
def test_documented_parameters_are_declared(node, name):
    assert node.has_parameter(name)


def test_topic_table_parameters_are_all_declared(node):
    for _, _, enable_param, qos_param in TOPICS.values():
        assert node.has_parameter(enable_param)
        assert node.has_parameter(qos_param)


def test_hs_aa_defaults(node):
    assert node.get_parameter('baudrate').value == 2400
    assert node.get_parameter('bytesize').value == 8
    assert node.get_parameter('parity').value == 'NONE'
    assert node.get_parameter('stopbits').value == 1.0
    # Measured on the real unit: a frame every 192-209 ms. It was guessed at
    # 10.0 before anyone timed it.
    assert node.get_parameter('expected_frame_rate').value == 5.0


def test_parameters_carry_descriptions_for_ros2_param_describe(node):
    # `use_sim_time` is declared by rclpy itself and has no description.
    missing = [name for name in node._parameters
               if name not in RCLPY_BUILTIN_PARAMS
               and not node.describe_parameter(name).description]
    assert missing == []


def test_dynamic_parameters_are_settable_while_unconfigured(node):
    from rclpy.parameter import Parameter
    result = node.set_parameters([Parameter('frame_id', Parameter.Type.STRING, 'cell')])
    assert result[0].successful
    assert node.get_parameter('frame_id').value == 'cell'


def test_an_uncompilable_value_regex_is_refused(node):
    from rclpy.parameter import Parameter
    result = node.set_parameters([Parameter('value_regex', Parameter.Type.STRING, '[oops')])
    assert not result[0].successful
    assert 'does not compile' in result[0].reason


def test_a_non_positive_expected_frame_rate_is_refused(node):
    from rclpy.parameter import Parameter
    result = node.set_parameters([Parameter('expected_frame_rate', Parameter.Type.DOUBLE, 0.0)])
    assert not result[0].successful


COMMAND_SERVICES = {'tare', 'zero', 'send_command'}
ALWAYS_ON_SERVICES = {'hw_reset', 'device_info'}


def configure_on_pty(node, **params):
    """Point `node` at a fresh pty and configure it. Returns the master fd."""
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.parameter import Parameter
    master, slave = os.openpty()
    settings = {'port': os.ttyname(slave), 'autostart': False}
    settings.update(params)
    node.set_parameters([
        Parameter(name, value=value) for name, value in settings.items()])
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    return master


def test_command_interface_is_not_advertised_by_default(node):
    # The HS-AA RS232 port is output only, so a ~/tare that can never work
    # must not appear in `ros2 service list` at all.
    configure_on_pty(node)
    names = service_basenames(node)
    assert not (COMMAND_SERVICES & names)
    assert ALWAYS_ON_SERVICES <= names


def test_cmd_topic_is_not_subscribed_by_default(node):
    configure_on_pty(node)
    # Subscriptions are asserted against a settled parameter-events baseline:
    # every node has that one, so an empty result means "not settled yet".
    names = settled(node, lambda: node.get_subscriber_names_and_types_by_node(
        node.get_name(), '/'))
    assert not any(name.endswith('/cmd') for name, _ in names)


def test_command_interface_appears_when_enabled(node):
    configure_on_pty(node, enable_commands=True)
    assert COMMAND_SERVICES <= service_basenames(node)


def test_weight_publishers_exist_in_read_only_mode(node):
    configure_on_pty(node)
    topics = basenames(settled(node, lambda: node.get_publisher_names_and_types_by_node(
        node.get_name(), '/')))
    assert {'weight_stamped', 'weight', 'stable', 'unit', 'raw'} <= topics
