# hansung_scale

ROS 2 stack for the Hansung HS-AA series RS232 electronic scale: a driver node
and the interfaces it publishes.

**Self-contained.** This directory is not a package - it just holds the two
that make up the stack - and nothing in it depends on the repository around it.
Between them the two packages declare only stock ROS 2 dependencies plus
`python3-serial`:

```
hansung_scale_driver : rclpy, std_msgs, std_srvs, diagnostic_msgs,
                       rcl_interfaces, lifecycle_msgs, launch, launch_ros,
                       python3-serial, hansung_scale_msgs
hansung_scale_msgs   : rosidl_default_generators/runtime, std_msgs
```

No package here depends on anything outside this directory except stock ROS 2,
and the interfaces deliberately stay in `hansung_scale_msgs` rather than being
folded into any project-wide interface package. Copy the directory into another
ROS 2 workspace and it builds as-is.

| Package | Build type | Holds |
|---|---|---|
| [`hansung_scale_driver`](hansung_scale_driver/) | `ament_python` | the lifecycle driver node (`scale_node`), the serial sniffer (`scale_sniffer`), parameters, launch, tests |
| [`hansung_scale_msgs`](hansung_scale_msgs/) | `ament_cmake` | `WeightStamped.msg`, `DeviceInfo.srv`, `SendCommand.srv` |

The driver's own documentation - the protocol as actually measured, the full
parameter table, topics and services, troubleshooting - is in
[`hansung_scale_driver/README.md`](hansung_scale_driver/README.md).

## Why two packages

They cannot be merged into one, and this is the constraint to know before
trying. When `rosidl` generates interfaces it **owns that package's Python
namespace**: an interface package installs `__init__.py`, `msg/`, `srv/`,
`lib<package>__rosidl_generator_py.so`, the typesupport `.so` files and the
generated C sources all into `dist-packages/<package>/`.

With a single package name, the hand-written `scale_node.py`, `protocol.py` and
`device.py` would have to live in that same directory.
`ament_python_install_package` and the `rosidl` generator would both write
there and their `__init__.py` files would collide outright, leaving install
order to decide which one wins. An `ament_python` package cannot generate ROS
interfaces at all, either. This is the same reason realsense-ros keeps
`realsense2_camera_msgs` separate from `realsense2_camera`.

Dropping the custom interfaces is not an option either. `WeightStamped` carries
the timestamp, weight, unit, gram conversion, stability flag and raw status as
one sample, so a consumer never has to time-correlate several primitive topics.
`DeviceInfo` asks the running driver what it is actually talking to instead of
inferring it from parameters. `SendCommand` is the discovery tool for command
bytes the manufacturer does not document. None of them has a stock equivalent.

## Build and run

`hansung_scale_msgs` has to build first; the driver depends on it at runtime.

```bash
colcon build --symlink-install --packages-select hansung_scale_msgs hansung_scale_driver
```

```bash
ros2 launch hansung_scale_driver scale.launch.py serial_no:=<adapter serial>
```

When the port is unknown:

```bash
ros2 run hansung_scale_driver scale_sniffer --ros-args -p list_ports:=true
```

```bash
colcon test --packages-select hansung_scale_driver
```

## Worth knowing

- **The device is read-only.** The HS-AA's RS232 port is output-only, confirmed
  both by the manufacturer and on the hardware. Zero and tare are physical
  buttons on the scale; the driver reads and publishes, nothing else.
  `SendCommand` exists to verify that and to probe for undocumented commands.
- The protocol is not published, so it was reverse-confirmed by sniffing the
  serial line on real hardware. Every figure in the driver's protocol table is
  measured, not assumed.
- Frame parsing (`protocol.py`) and port selection (`device.py`) know nothing
  about ROS or pyserial hardware. That is what makes them testable without a
  scale or a ROS graph, and it narrows the files to touch when a device with a
  different protocol is added.

## Provenance

First-party code by `chohh7391@gmail.com`, Apache-2.0, with its own
`CHANGELOG.rst` and version (0.2.0). It was developed in a separate workspace
that was not a git repository, so there is no history to preserve.

The node package is named `hansung_scale_driver` rather than `hansung_scale`
because this directory takes that name; without the rename the path would
repeat it three times as `hansung_scale/hansung_scale/hansung_scale/`.
