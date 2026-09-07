"""
String-to-QoSProfile mapping, so publisher QoS is a ROS parameter.

realsense-ros exposes a `<stream>_qos` parameter per stream and accepts the
rmw preset names; the same knob matters here because a weight stream feeding
a control loop and one feeding a logger want opposite reliability settings.
"""
from rclpy.qos import (
    qos_profile_parameter_events,
    qos_profile_parameters,
    qos_profile_sensor_data,
    qos_profile_services_default,
    qos_profile_system_default,
)

QOS_PROFILES = {
    'SYSTEM_DEFAULT': qos_profile_system_default,
    'DEFAULT': qos_profile_system_default,
    'SENSOR_DATA': qos_profile_sensor_data,
    'SERVICES_DEFAULT': qos_profile_services_default,
    'PARAMETERS': qos_profile_parameters,
    'PARAMETER_EVENTS': qos_profile_parameter_events,
}

#: Accepted values, for parameter descriptors and error messages.
QOS_NAMES = tuple(QOS_PROFILES)


def qos_profile_from_string(name: str):
    """
    Look up a preset by name.

    Raises ValueError on an unknown name rather than silently falling back,
    which would hide a typo in a params file as a reliability change nobody
    notices until a message goes missing.
    """
    try:
        return QOS_PROFILES[str(name).strip().upper()]
    except KeyError:
        raise ValueError(f'unknown QoS profile {name!r}; expected one of {", ".join(QOS_NAMES)}')
