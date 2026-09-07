# hansung_scale_driver

ROS 2 (Humble) driver that reads a Hansung **HS-AA series** electronic scale
over RS232.

**Read-only.** The manufacturer confirmed this indicator's RS232 port is
output-only, and testing on the hardware agrees ([log below](#rs232-one-way-verification)).
Zero and tare are physical buttons on the scale; the driver reads what the scale
emits and publishes it, nothing more.

The protocol is not published, so it was reverse-confirmed by sniffing the
serial line on the real device. Everything below is **measured**.

The package layout and interfaces follow the `realsense2_camera` wrapper from
[realsense-ros](https://github.com/realsenseai/realsense-ros).

This driver and `hansung_scale_msgs` are a self-contained pair: between them
they depend only on stock ROS 2 and `python3-serial`, so they drop into any
ROS 2 workspace as-is.

## Confirmed protocol

| Item | Value |
|---|---|
| Baud rate | **2400 bps** |
| Data / parity / stop bits | **8 / None / 1** |
| Mode | continuous output, streams unprompted |
| Frame rate | **5.000 Hz** (192-209 ms between frames) |
| Direction | **one way, scale to PC** |

Frame format:

```
WT<status:2><sign:1>   <value>   <unit>\r\n

for example: WTST+  12.70   g\r\n
```

| Field | Meaning |
|---|---|
| `WT` | fixed header |
| `status` | `ST` = stable. `ST` is the only value observed on the hardware |
| `sign` | `+` or `-` |
| `value` | space-padded on the left, two decimals |
| `unit` | space-padded on the left (`g` observed) |

`US` (unstable) and `OL` (overload) are **assumed** from the convention for this
frame style and have not been seen on the hardware. That is why `stable` is true
only when **`status == "ST"`**, rather than whenever `status != "US"`: an unknown
code must not be read as settled.

---

# Quick start

## 1. Serial port permissions

Only the `dialout` group can reach a USB-RS232 adapter. Needed **once**:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for it to take effect, or `newgrp dialout` in a new shell.
For a single session right now:

```bash
sudo chmod a+rw /dev/ttyUSB0     # reset when the adapter is replugged
```

Without permission the node says so:

```
[ERROR] Failed to open /dev/ttyUSB0: [Errno 13] Permission denied
        The user is not in the dialout group: `sudo usermod -aG dialout $USER`,
        then log out and back in.
```

## 2. Build

From the root of the workspace holding both packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hansung_scale_msgs hansung_scale_driver --symlink-install
source install/setup.bash
```

`hansung_scale_msgs` builds first. An `ament_python` package cannot generate ROS
interfaces, so the messages live in a separate `ament_cmake` package for the same
reason realsense-ros splits out `realsense2_camera_msgs`.

## 3. Find the adapter

Prints what is on which port, and **selectors you can paste straight in**:

```bash
ros2 run hansung_scale_driver scale_sniffer --ros-args -p list_ports:=true
```

```
DEVICE               VID:PID    SERIAL             USB PORT     DESCRIPTION
---------------------------------------------------------------------------
/dev/ttyUSB0         0403:6001  FTEFY2BT           8-1          USB Serial Converter
/dev/ttyS0           -          -                  -            n/a
... (legacy ports the kernel always creates)

# Stable selectors for scale_node (any one of these):
#   -p serial_no:=FTEFY2BT      (/dev/ttyUSB0)
#   -p usb_port_id:=8-1         (/dev/ttyUSB0)
#   -p port:=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTEFY2BT-if00-port0
```

USB adapters are sorted to the top. A number like `/dev/ttyUSB0` moves when USB
enumeration order changes, so **prefer `serial_no`**.

## 4. Run

```bash
ros2 launch hansung_scale_driver scale.launch.py serial_no:=FTEFY2BT
```

```
[INFO] Read-only mode: ~/cmd, ~/tare, ~/zero and ~/send_command are not advertised.
[INFO] Configured /dev/ttyUSB0 0403:6001 sn=FTEFY2BT usb_port_id=8-1 @ 2400 8N1
[INFO] Activated: streaming weight data
```

## 5. Read a value

```bash
ros2 topic echo /scale_node/weight_stamped
```

```yaml
header:
  stamp: {sec: 1788770099, nanosec: 279449507}
  frame_id: scale_link
weight: 12.7          # exactly what the scale reported
unit: g
weight_grams: 12.7    # converted to grams, NaN for an unknown unit
stable: true          # true only when status == ST
status: ST
```

---

# Interfaces

## Published topics

| Topic | Type | Description |
|---|---|---|
| `~/weight_stamped` | `hansung_scale_msgs/WeightStamped` | **Preferred.** Timestamp, weight, unit, gram conversion, stability and status in one message |
| `~/weight` | `std_msgs/Float32` | weight with the frame sign applied |
| `~/stable` | `std_msgs/Bool` | settled or not |
| `~/unit` | `std_msgs/String` | unit string |
| `~/raw` | `std_msgs/String` | the unparsed line, for debugging |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | when `diagnostics_period > 0` |

`~/weight`, `~/stable` and `~/unit` are three separate messages, leaving the
consumer to time-correlate them. **New code should use `~/weight_stamped`.**

`~/stable` and `~/unit` are published **only when a complete HS-AA frame was
parsed**. Emitting defaults (unstable, no unit) for a line where only the number
came out of the `value_regex` fallback would be indistinguishable from a real
measurement.

## Services

| Service | Type | Description |
|---|---|---|
| `~/device_info` | `hansung_scale_msgs/DeviceInfo` | which port is actually open, adapter serial, line settings, frame statistics |
| `~/hw_reset` | `std_srvs/Trigger` | close and reopen the port with a DTR pulse, for a wedged adapter |

`~/cmd`, `~/tare`, `~/zero` and `~/send_command` **do not exist by default**.
This scale accepts no input, and a service that cannot work has no business
appearing in `ros2 service list`. Attach an indicator that does accept input and
`enable_commands:=true` brings them back.

## Parameters

All of them are in `config/scale_params.yaml` with comments, and
`ros2 param describe /scale_node <name>` prints the same description.

### Device selection

Setting any of `serial_no`, `usb_port_id` or `device_type` makes `port`
**ignored**, so a `port` left at its default cannot beat a selector that was set
deliberately.

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | device path, or a `/dev/serial/by-id/...` symlink |
| `serial_no` | `''` | **Preferred.** Adapter USB serial number; survives reboots and replugging |
| `usb_port_id` | `''` | physical USB location prefix, e.g. `8-1`, for adapters with no serial |
| `device_type` | `''` | regex over the adapter's description/manufacturer/product, e.g. `ftdi` |
| `wait_for_device_timeout` | `-1.0` | seconds to wait at configure for the port to appear; negative or zero tries once |
| `reconnect_timeout` | `6.0` | seconds to retry after the link drops; negative stops the reader at the first read error |
| `initial_reset` | `false` | DTR pulse and input-buffer flush right after connecting |

### Line settings

| Parameter | Default | Description |
|---|---|---|
| `baudrate` | `2400` | measured |
| `bytesize` | `8` | 5, 6, 7, 8 |
| `parity` | `'NONE'` | `NONE`/`EVEN`/`ODD`/`MARK`/`SPACE` |
| `stopbits` | `1.0` | 1, 1.5, 2 |
| `timeout` | `1.0` | pyserial read timeout, seconds |

> `parity` is spelled `'NONE'` rather than a single `'N'` because in YAML 1.1 a
> bare `N` or `Y` is a **boolean literal**. Passed through a params file or a
> launch substitution it silently becomes `False`. The single-letter spellings
> are still accepted.

### Protocol

| Parameter | Default | Description |
|---|---|---|
| `line_ending` | `'\r\n'` | frame delimiter; escapes or `hex:0D0A` |
| `value_regex` | `'[-+]?\d+\.?\d*'` | fallback parser for lines the `WT...` frame does not match |
| `frame_id` | `'scale_link'` | frame_id on `~/weight_stamped` |

### Topics and QoS

| Parameter | Default | Description |
|---|---|---|
| `enable_weight_stamped`, `enable_weight`, `enable_stable`, `enable_unit`, `enable_raw` | `true` | whether each publisher is created |
| `weight_qos` | `'SYSTEM_DEFAULT'` | QoS preset for the weight topics |
| `raw_qos` | `'SYSTEM_DEFAULT'` | QoS preset for `~/raw` |

Presets: `SYSTEM_DEFAULT`, `DEFAULT`, `SENSOR_DATA`, `SERVICES_DEFAULT`,
`PARAMETERS`, `PARAMETER_EVENTS`. A consumer that only wants the latest value
wants `SENSOR_DATA` (best effort, small queue). A typo does not fall back
quietly; it fails configure.

### Diagnostics

| Parameter | Default | Description |
|---|---|---|
| `diagnostics_period` | `1.0` | `/diagnostics` period in seconds; 0 disables |
| `expected_frame_rate` | `5.0` | expected frame rate, **measured at 5.000 Hz**; the staleness threshold |

### Other

| Parameter | Default | Description |
|---|---|---|
| `enable_commands` | `false` | expose the write interfaces; this scale accepts no input |
| `poll_mode` / `poll_command` / `poll_interval` | `false` / `''` / `0.5` | for request/response indicators; not applicable here |
| `tare_command` / `zero_command` / `command_response_timeout` | `''` / `''` / `0.0` | only meaningful with `enable_commands` |
| `autostart` | `true` | configure and activate on startup |

### Parameters settable at runtime

`value_regex`, `frame_id`, `poll_interval`, `tare_command`, `zero_command`,
`command_response_timeout`, `expected_frame_rate`.

The rest are read only while configuring, so setting one on a configured node is
**refused with an explanation** rather than accepted and quietly ignored.

```bash
ros2 param set /scale_node frame_id weigh_cell   # -> Set parameter successful
ros2 param set /scale_node baudrate 9600
# -> Setting parameter failed: baudrate is only read while configuring.
#    Run `ros2 lifecycle set scale_node cleanup` then `configure` to apply it.
```

---

# Common tasks

## Wait for a settled reading

The usual pattern in a robot sequence: take only frames where `stable` is true.

```python
import rclpy
from rclpy.node import Node
from hansung_scale_msgs.msg import WeightStamped


class WeighOnce(Node):
    """Wait for a settled reading, print it, shut down."""

    def __init__(self):
        super().__init__('weigh_once')
        self.create_subscription(WeightStamped, '/scale_node/weight_stamped',
                                 self.on_weight, 10)

    def on_weight(self, msg):
        if not msg.stable:
            return                      # still moving
        self.get_logger().info(f'{msg.weight} {msg.unit} ({msg.weight_grams} g)')
        raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(WeighOnce())
    except SystemExit:
        pass
    rclpy.shutdown()
```

If `stable` stays false the scale has not settled yet. Frames keep arriving at
5 Hz regardless, so check `~/raw` for the actual status field.

## Change the namespace or node name

For several scales, or to group them per cell.

```bash
ros2 launch hansung_scale_driver scale.launch.py serial_no:=FTEFY2BT \
  scale_namespace:=/cell1 scale_name:=weigh_station
# -> /cell1/weigh_station/weight_stamped
```

`config/scale_params.yaml` keys off the `/**` wildcard, so renaming does not
break it.

## Use a separate parameter file

```bash
ros2 launch hansung_scale_driver scale.launch.py params_file:=/path/to/my_scale.yaml
```

A launch argument left at its default is **not passed to the node**, so the
value in `params_file` actually applies. (`rs_launch.py` passes all of them
unconditionally, which makes the params file look ignored; that behaviour is
deliberately different here.)

## Lifecycle control

A managed node with `unconfigured -> inactive -> active`. The port is opened
**at configure** and publishing happens **only while active**.

```bash
ros2 lifecycle get /scale_node
ros2 lifecycle set /scale_node deactivate   # port stays open, publishing stops
ros2 lifecycle set /scale_node activate
```

Launch with `autostart:=false` to leave the transitions to an external lifecycle
manager.

## Monitor health

```bash
ros2 topic echo /diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor
```

```yaml
name: 'scale_node: RS232 link'
message: Streaming at 5.0 frames/s
values:
  frame_rate_hz: '5.00'          expected_frame_rate_hz: '5.00'
  frames_received: '1284'        parse_errors: '0'
  dropped_bytes: '0'             last_frame_age_s: '0.14'
  last_weight: 12.7 g            last_status: ST            stable: 'True'
```

A down link (ERROR), stalled frames (WARN) and parse failures (WARN) are
reported apart from each other. This is a plain publisher rather than a
lifecycle one, so it reports "connected, not streaming" **even while inactive**.

## Device information

```bash
ros2 service call /scale_node/device_info hansung_scale_msgs/srv/DeviceInfo
```

```
device_name='Hansung HS-AA series RS232 indicator'
serial_number='FTEFY2BT'   physical_port='/dev/ttyUSB0'
port_id='/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTEFY2BT-if00-port0'
usb_type_descriptor='0403:6001'   serial_settings='2400 8N1'
connected=True   frame_rate=5.0   frames_received=1284   parse_errors=0
```

## When the USB cable is pulled

The driver retries for `reconnect_timeout` seconds. **The node does not die**,
and it reattaches by itself when the adapter comes back.

```
[ERROR] Serial read error: device reports readiness to read but returned no data
[WARN]  Link down; retrying for up to 6.0s...
[ERROR] No serial port matched [serial_no=FTEFY2BT] within 6.0s.
        `ros2 run hansung_scale_driver scale_sniffer --ros-args -p list_ports:=true` shows
        what is actually connected.
[WARN]  Link down; retrying for up to 6.0s...
[INFO]  Reconnected to /dev/ttyUSB0
```

`reconnect_timeout: -1.0` stops the reader at the first read error instead.

---

# Troubleshooting

| Symptom | Cause / check |
|---|---|
| `Permission denied` | the `dialout` group; see [step 1](#1-serial-port-permissions) |
| `No serial port matched` | adapter not connected, or a typo in the selector. Check with `-p list_ports:=true` |
| topics are silent | check the lifecycle state (`ros2 lifecycle get`); `inactive` does not publish |
| no values but the node looks fine | look at `~/raw`. It separates "lines arrive but do not parse" from "nothing arrives" |
| `~/raw` shows garbage | baud or parity mismatch. Sweep 1200/2400/4800/9600 with `scale_sniffer` |
| `parse_errors` climbing | the frame format differs. Use `-p decode:=true` to see how the parser reads it |
| `ros2 param set` refused | a configure-time parameter. `cleanup` then `configure` |
| tare/zero do nothing | this scale accepts no input. Use the physical buttons |

## Find the cause with the sniffer

```bash
# raw byte HEX/ASCII dump
ros2 run hansung_scale_driver scale_sniffer --ros-args -p serial_no:=FTEFY2BT

# dump plus how the current parser reads each line
ros2 run hansung_scale_driver scale_sniffer --ros-args -p serial_no:=FTEFY2BT -p decode:=true
```

```
HEX[57 54 53 54 2b 20 20 31 32 2e 37 30 ...] ASCII['WTST+  12.70   g\r\n']
  LINE 'WTST+  12.70   g' -> HS-AA frame: weight=12.7 unit='g' stable=True status='ST' grams=12.7
```

`decode:=true` is for validating `line_ending` and `value_regex` candidates
**before** committing them to a params file.

---

# RS232 one-way verification

**Manufacturer's answer:** this indicator's RS232 is output-only. Reading data
is all it does.

Recorded so nobody repeats the investigation. Starting from a settled reading of
an object on the pan (12.70 g, baseline spread `0.00`), **43 probes** were sent
while watching four signals at once.

| Signal watched | Meaning |
|---|---|
| value collapses to 0 | tare or zero worked |
| value moves beyond the baseline spread | something happened |
| frame rate drops, or the stream stops | the scale paused to process it |
| a line that is not `WT...` | an error or ACK reply |

| Axis | What was sent |
|---|---|
| Command bytes (25) | `T`/`t`/`T\r\n`/`T\r`, `Z`/`z`/`Z\r\n`/`Z\r`, `Q`, `S`, `SI`, `W`, `P`, `R` (each bare and CRLF), ENQ `05`, ACK `06`, DC1, DC2, bare CR, bare LF |
| Framing (11) | `WTT`/`WTZ`/`WT`, address prefixes `01T`/`00T`/`1T`, attention `@T`/`#T`/`*T`, words `TARE`/`ZERO` |
| Handshake lines (7) | RTS low, DTR low, RTS+DTR low, each with `T`/`Z`, with and without CRLF, and twice in a row |

**Result: no response to any of the 43.** Nothing changed on the scale's display
either. The modem input lines (`CTS`, `DSR`, `DCD`) all read low, so this looks
like a three-wire cable.

> ESC sequences and the `C` family, which can enter calibration or setup mode,
> were **deliberately excluded**. Leaving those modes needs a power cycle, and at
> worst they disturb the calibration.

---

# Tests

```bash
colcon test --packages-select hansung_scale_driver
colcon test-result --verbose --test-result-base build/hansung_scale_driver
```

**All 136 tests run without a scale.** Frame parsing and port selection are pure
functions and are tested directly; the parts that need a real serial fd use a
pty from `os.openpty()`.

| File | Covers |
|---|---|
| `test_protocol.py` | frame parsing, unit conversion, escape decoding, line assembly |
| `test_device.py` | line-setting validation, port selection precedence, waiting for a device |
| `test_connection.py` | real open/read/write/close against a pty |
| `test_scale_node.py` | declared parameters, advertised topics and services, dynamic parameters |
| `test_launch_args.py` | launch parameter table consistency, type conversion |
| `test_flake8.py` / `test_pep257.py` | lint |

---

# Layout

```
hansung_scale/
├── hansung_scale_driver/
│   ├── hansung_scale_driver/
│   │   ├── scale_node.py    # the lifecycle driver node
│   │   ├── raw_sniffer.py   # port listing, raw dump, parse preview
│   │   ├── protocol.py      # frame parsing (no ROS, no pyserial)
│   │   ├── device.py        # port discovery and selection, serial settings, the connection
│   │   └── qos.py           # QoS preset string -> QoSProfile
│   ├── config/scale_params.yaml
│   ├── launch/scale.launch.py
│   └── test/
└── hansung_scale_msgs/              # WeightStamped, DeviceInfo, SendCommand
```

`protocol.py` and `device.py` knowing nothing about ROS is the point. It is what
makes parsing and port selection testable without a scale or a ROS graph, and it
narrows the files to touch when a device with a different protocol is added.

## Mapping to realsense-ros

| realsense2_camera | hansung_scale_driver |
|---|---|
| lifecycle node | same |
| `serial_no` / `usb_port_id` / `device_type` device selection | same, against the USB-serial adapter |
| `wait_for_device_timeout` / `reconnect_timeout` | same |
| `enable_<stream>` / `<stream>_qos` | `enable_*` / `weight_qos`, `raw_qos` |
| `/diagnostics` (temperature, stream rate) | `/diagnostics` (link state, frame rate, parse failures) |
| `~/device_info`, `~/hw_reset` | same |
| hardware-monitor command service | `~/send_command`, with `enable_commands` |
| `configurable_parameters` in `rs_launch.py` | same structure in `scale.launch.py` |
| `realsense2_camera_msgs` | `hansung_scale_msgs` |

---

# Adapting it to another indicator

1. Find the port with `scale_sniffer -p list_ports:=true`, then confirm the real
   frame format with `-p decode:=true`.
2. Add the captured lines to `test_protocol.py` as cases. The parser can then be
   fixed **without the device**.
3. Change the `HS_AA_FRAME` regex in `hansung_scale_driver/protocol.py` to the
   new format, or just adjust the `value_regex` parameter, which is used
   automatically whenever the frame match fails.
4. Set `baudrate`, `parity` and `expected_frame_rate` in
   `config/scale_params.yaml` to the measured values.
5. If the device does accept input, turn on `enable_commands: true` and use
   `~/send_command` to find the command bytes, then put them in `tare_command`
   and `zero_command`. For a request/response device use `poll_mode: true` with
   `poll_command` (and keep `timeout` below `poll_interval`).
