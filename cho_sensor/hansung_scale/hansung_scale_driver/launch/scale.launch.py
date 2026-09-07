"""
Launch the HS-AA scale driver.

Built the way realsense2_camera's `rs_launch.py` is built: one
`configurable_parameters` table drives both the `DeclareLaunchArgument` list
and the parameter dict handed to the node, so every node parameter is
overridable from the command line without the table being written out twice::

    ros2 launch hansung_scale_driver scale.launch.py serial_no:=FTEFY2BT
    ros2 launch hansung_scale_driver scale.launch.py params_file:=/path/to/my_scale.yaml
    ros2 launch hansung_scale_driver scale.launch.py scale_namespace:=/cell1 \
        scale_name:=weigh_station

One deliberate difference from `rs_launch.py`: launch arguments left at their
declared default are *dropped* rather than passed through, so `params_file`
actually decides those values. Passing all of them unconditionally makes the
launch defaults silently outrank every entry in the YAML — which is the
behaviour that makes people think their params file is being ignored.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

PACKAGE = 'hansung_scale_driver'


def default_params_file() -> str:
    """
    Return the path to the installed config/scale_params.yaml.

    Resolved lazily rather than at import time so this module can be imported
    (and its parameter table unit-tested) without the package being installed.
    """
    return os.path.join(get_package_share_directory(PACKAGE), 'config', 'scale_params.yaml')


#: name / default (as a string, the way launch arguments arrive) / type used to
#: coerce it back / description. `type` matters: a node parameter declared as an
#: int rejects the string '2400'.
configurable_parameters = [
    {'name': 'scale_namespace', 'default': '/', 'type': str,
     'description': 'Namespace for the driver node'},
    {'name': 'scale_name', 'default': 'scale_node', 'type': str,
     'description': 'Node name, and therefore the ~/ topic prefix'},
    {'name': 'params_file', 'default': '', 'type': str,
     'description': 'YAML supplying every parameter not overridden on the command line. '
                    'Empty means the packaged config/scale_params.yaml.'},
    {'name': 'log_level', 'default': 'info', 'type': str,
     'description': 'debug, info, warn, error or fatal'},

    {'name': 'port', 'default': '/dev/ttyUSB0', 'type': str,
     'description': 'Device path or /dev/serial/by-id symlink; ignored if a selector is set'},
    {'name': 'serial_no', 'default': '', 'type': str,
     'description': 'USB serial number of the RS232 adapter, e.g. FTEFY2BT'},
    {'name': 'usb_port_id', 'default': '', 'type': str,
     'description': 'Physical USB location prefix, e.g. 8-1'},
    {'name': 'device_type', 'default': '', 'type': str,
     'description': 'Regex matched against the adapter description/manufacturer/product'},
    {'name': 'wait_for_device_timeout', 'default': '-1.0', 'type': float,
     'description': 'Seconds to wait for a matching port during configure'},
    {'name': 'reconnect_timeout', 'default': '6.0', 'type': float,
     'description': 'Seconds to keep retrying after the link drops'},
    {'name': 'initial_reset', 'default': 'false', 'type': bool,
     'description': 'Pulse DTR and flush the input buffer on connect'},

    {'name': 'baudrate', 'default': '2400', 'type': int,
     'description': '2400 8N1 confirmed by sniffing the real HS-AA unit'},
    {'name': 'bytesize', 'default': '8', 'type': int, 'description': 'Bits per character'},
    {'name': 'parity', 'default': 'NONE', 'type': str,
     'description': 'NONE, EVEN, ODD, MARK or SPACE'},
    {'name': 'stopbits', 'default': '1.0', 'type': float, 'description': '1, 1.5 or 2'},
    {'name': 'timeout', 'default': '1.0', 'type': float,
     'description': 'pyserial read timeout in seconds; keep it under poll_interval in '
                    'poll mode'},

    {'name': 'frame_id', 'default': 'scale_link', 'type': str,
     'description': 'frame_id stamped into ~/weight_stamped'},
    {'name': 'enable_commands', 'default': 'false', 'type': bool,
     'description': 'Advertise the write-side interface. Off by default: the HS-AA RS232 '
                    'port is output only'},
    {'name': 'poll_mode', 'default': 'false', 'type': bool,
     'description': 'Request/response mode; the HS-AA streams on its own, so leave it off'},
    {'name': 'poll_command', 'default': '', 'type': str,
     'description': "Bytes sent each poll cycle, e.g. 'hex:05' for ENQ"},
    {'name': 'poll_interval', 'default': '0.5', 'type': float,
     'description': 'Seconds between polls'},

    {'name': 'weight_qos', 'default': 'SYSTEM_DEFAULT', 'type': str,
     'description': 'QoS preset for the weight topics; SENSOR_DATA for best-effort'},
    {'name': 'raw_qos', 'default': 'SYSTEM_DEFAULT', 'type': str,
     'description': 'QoS preset for ~/raw'},
    {'name': 'diagnostics_period', 'default': '1.0', 'type': float,
     'description': 'Seconds between /diagnostics updates; 0 disables them'},
    {'name': 'expected_frame_rate', 'default': '5.0', 'type': float,
     'description': 'Frames/s the indicator should produce; drives the staleness check'},

    {'name': 'autostart', 'default': 'true', 'type': bool,
     'description': 'Auto-configure and auto-activate instead of waiting for a lifecycle '
                    'manager'},
]

#: Launch-only arguments: they configure the launch action itself, so they must
#: not end up in the node's parameter dict.
LAUNCH_ONLY = {'scale_namespace', 'scale_name', 'params_file', 'log_level'}


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'],
                                  description=param['description'])
            for param in parameters]


def coerce(text: str, type_):
    """
    Turn a launch argument back into the type the node declared.

    Launch arguments always arrive as strings, and a node parameter declared
    as an int rejects the string '2400'.
    """
    if type_ is bool:
        lowered = text.strip().lower()
        if lowered in ('1', 'true', 'yes', 'on'):
            return True
        if lowered in ('0', 'false', 'no', 'off'):
            return False
        raise RuntimeError(f'expected a boolean, got {text!r}')
    return type_(text)


def overridden_parameters(context, parameters) -> dict:
    """Collect the node parameters the user actually set on the command line."""
    overrides = {}
    for param in parameters:
        if param['name'] in LAUNCH_ONLY:
            continue
        text = LaunchConfiguration(param['name']).perform(context)
        if text != param['default']:
            overrides[param['name']] = coerce(text, param['type'])
    return overrides


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context) or default_params_file()
    parameters = [params_file, overridden_parameters(context, configurable_parameters)]

    return [LifecycleNode(
        package=PACKAGE,
        executable='scale_node',
        namespace=LaunchConfiguration('scale_namespace'),
        name=LaunchConfiguration('scale_name'),
        output='screen',
        parameters=parameters,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters)
        + [OpaqueFunction(function=launch_setup)])
