"""
ROS2 lifecycle driver for a Hansung(한성전자저울) HS-AA series RS232 indicator.

Shaped after realsense2_camera's camera node, feature for feature where the
feature makes sense for a serial indicator:

* **Managed node** — unconfigured -> inactive -> active, so the port is only
  held while configured and only streams while active.
* **Device selection without device paths** — `serial_no` / `usb_port_id` /
  `device_type` instead of `/dev/ttyUSB0`, which renumbers on every reboot.
* **`wait_for_device_timeout` / `reconnect_timeout`** — the driver waits for
  the adapter to appear and re-attaches when a yanked USB cable comes back,
  instead of dying on the first read error.
* **Per-topic `enable_*` and `*_qos` parameters.**
* **`/diagnostics`** — link state, measured frame rate, staleness and parse
  errors, on the standard topic so `rqt_robot_monitor` picks it up.
* **Services** — `~/device_info`, `~/hw_reset`, `~/send_command`, plus
  `~/tare` and `~/zero`.

Protocol details and frame parsing live in `hansung_scale_driver.protocol`; port discovery
and the pyserial handle live in `hansung_scale_driver.device`.
"""
import re
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from hansung_scale_msgs.msg import WeightStamped
from hansung_scale_msgs.srv import DeviceInfo, SendCommand

from .device import PortInfo, ScaleConnection, SerialSettings, by_id_path, wait_for_port
from .protocol import DEFAULT_VALUE_REGEX, LineAssembler, decode_escapes, encode_printable, \
    parse_frame
from .qos import QOS_NAMES, qos_profile_from_string

DEVICE_NAME = 'Hansung HS-AA series RS232 indicator'

#: Parameters that may be changed while the node is configured. Everything
#: else needs a cleanup/configure cycle, the same rule realsense-ros applies
#: to its non-dynamic parameters.
DYNAMIC_PARAMS = frozenset({
    'value_regex', 'frame_id', 'poll_interval', 'tare_command', 'zero_command',
    'command_response_timeout', 'expected_frame_rate',
})

#: Publisher key -> (message type, topic, enable parameter, qos parameter).
TOPICS = {
    'weight_stamped': (WeightStamped, '~/weight_stamped', 'enable_weight_stamped', 'weight_qos'),
    'weight': (Float32, '~/weight', 'enable_weight', 'weight_qos'),
    'stable': (Bool, '~/stable', 'enable_stable', 'weight_qos'),
    'unit': (String, '~/unit', 'enable_unit', 'weight_qos'),
    'raw': (String, '~/raw', 'enable_raw', 'raw_qos'),
}


def compile_value_regex(pattern: str):
    """
    Compile `value_regex`, turning re.error into ValueError.

    That lets the caller report a bad params file the same way it reports a
    bad baud rate.
    """
    try:
        return re.compile(pattern)
    except re.error as exc:
        raise ValueError(f'value_regex {pattern!r} does not compile: {exc}')


class ScaleNode(LifecycleNode):
    """
    Managed (lifecycle) node for one RS232 indicator.

    Resolve and open the port in on_configure, start streaming in
    on_activate, stop in on_deactivate, close in on_cleanup.
    """

    def __init__(self):
        super().__init__('scale_node')

        # -- device selection, mirroring realsense-ros --------------------
        self._declare('port', '/dev/ttyUSB0',
                      'Explicit device path or /dev/serial/by-id symlink. Ignored when '
                      'serial_no, usb_port_id or device_type is set.')
        self._declare('serial_no', '',
                      "USB serial number of the RS232 adapter, e.g. 'FTEFY2BT'. Survives "
                      'reboots and replugging, unlike the ttyUSB index.')
        self._declare('usb_port_id', '',
                      "Physical USB location prefix, e.g. '8-1' (also matches '8-1:1.0'). "
                      'For adapters with no serial number: pins whichever adapter is in '
                      'that socket.')
        self._declare('device_type', '',
                      "Regex matched against the adapter's description, manufacturer and "
                      "product strings, e.g. 'ftdi'.")
        self._declare('wait_for_device_timeout', -1.0,
                      'Seconds on_configure waits for a matching port to appear. Negative '
                      'or zero means try once and fail.')
        self._declare('reconnect_timeout', 6.0,
                      'Seconds the reader keeps retrying after the link drops. Negative '
                      'means never retry (stop the reader on the first read error).')
        self._declare('initial_reset', False,
                      'Pulse DTR and flush the input buffer on connect. Some adapters only '
                      'pass data once DTR is asserted.')

        # -- line settings (2400 8N1 confirmed on the HS-AA) --------------
        self._declare('baudrate', 2400, 'Confirmed on the HS-AA by sniffing the real unit.')
        self._declare('bytesize', 8, 'Bits per character: 5, 6, 7 or 8.')
        self._declare('parity', 'NONE',
                      'NONE, EVEN, ODD, MARK or SPACE. Spelled out because bare N/Y are '
                      'YAML 1.1 booleans.')
        self._declare('stopbits', 1.0, 'Stop bits: 1, 1.5 or 2.')
        self._declare('timeout', 1.0, 'pyserial read timeout in seconds.')

        # -- protocol -----------------------------------------------------
        self._declare('line_ending', '\r\n',
                      "Frame terminator. Backslash escapes, or 'hex:0D0A'.")
        self._declare('value_regex', DEFAULT_VALUE_REGEX,
                      'Fallback numeric extractor, used only for lines that do not match '
                      'the WT<status><sign><value><unit> frame.')
        self._declare('frame_id', 'scale_link', 'frame_id stamped into ~/weight_stamped.')

        # -- poll mode (unused on the HS-AA, which streams unprompted) ----
        self._declare('poll_mode', False,
                      'Request/response mode, for indicators that answer only when asked.')
        self._declare('poll_command', '', 'Bytes sent each poll cycle.')
        self._declare('poll_interval', 0.5, 'Seconds between polls.')

        # -- commands -----------------------------------------------------
        self._declare('tare_command', '',
                      'Bytes the ~/tare service sends. Not yet confirmed for the HS-AA; '
                      'use ~/send_command to hunt for it.')
        self._declare('zero_command', '', 'Bytes the ~/zero service sends.')
        self._declare('enable_commands', False,
                      'Advertise the write-side interface (~/cmd, ~/tare, ~/zero, '
                      '~/send_command). Off by default: the HS-AA RS232 port is output '
                      'only, confirmed by the manufacturer, so nothing sent to it can '
                      'ever take effect. Turn it on for an indicator that does listen.')
        self._declare('command_response_timeout', 0.0,
                      'Seconds ~/tare and ~/zero collect a reply for. 0 sends and returns '
                      'immediately, which is right for a continuously streaming indicator.')

        # -- topics: enable flags and QoS ---------------------------------
        for _, _, enable_param, _ in TOPICS.values():
            self._declare(enable_param, True, f'Create the publisher for {enable_param[7:]}.')
        self._declare('weight_qos', 'SYSTEM_DEFAULT',
                      f'QoS preset for the weight topics: {", ".join(QOS_NAMES)}. Use '
                      'SENSOR_DATA for best-effort streaming.')
        self._declare('raw_qos', 'SYSTEM_DEFAULT',
                      f'QoS preset for ~/raw: {", ".join(QOS_NAMES)}.')

        # -- diagnostics --------------------------------------------------
        self._declare('diagnostics_period', 0.0,
                      'Seconds between /diagnostics updates. 0 disables them.')
        self._declare('expected_frame_rate', 5.0,
                      'Frames/s the indicator should produce. Measured at 5.0 (200 ms) on '
                      'the real HS-AA. Drives the diagnostics staleness check.')

        # If true, main() drives this node straight to active on startup
        # instead of waiting for an external lifecycle manager.
        self._declare('autostart', True,
                      'Auto-configure and auto-activate on startup instead of waiting for '
                      '`ros2 lifecycle set scale_node configure`.')

        self.conn = None
        self._assembler = None
        self._value_re = None
        self._pubs = {key: None for key in TOPICS}
        self._diag_pub = None
        self._diag_timer = None
        self._poll_timer = None
        self._poll_command = b''
        self._poll_failures = 0
        # NOT `_services`: rclpy.Node keeps its own service list under that
        # exact attribute name, and rebinding it drops the node's references
        # to the parameter services, which are then garbage-collected off the
        # graph — `ros2 param set` just starts timing out, with no error
        # logged anywhere.
        self._own_services = []
        self._cmd_sub = None

        self._thread = None
        self._stop_event = threading.Event()
        self._active = False

        # Capture buffer for ~/send_command: while this is not None the reader
        # thread copies every chunk into it, so a reply can be collected
        # without having to stop the stream first.
        self._capture = None
        self._capture_lock = threading.Lock()

        self._stats_lock = threading.Lock()
        self._frames = 0
        self._parse_errors = 0
        self._last_reading = None
        self._last_frame_time = None
        self._frame_rate = 0.0
        self._diag_baseline = (0, 0, None)

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _declare(self, name, default, description):
        self.declare_parameter(name, default, ParameterDescriptor(description=description))

    def _param(self, name):
        return self.get_parameter(name).value

    # -- lifecycle transitions -------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # A previous deactivate leaves the event set; _connect() passes it to
        # wait_for_port, which would treat it as "shutting down" and refuse
        # to wait at all.
        self._stop_event.clear()
        try:
            settings = SerialSettings(
                baudrate=self._param('baudrate'), bytesize=self._param('bytesize'),
                parity=self._param('parity'), stopbits=self._param('stopbits'),
                timeout=self._param('timeout'))
            if self._param('expected_frame_rate') <= 0.0:
                raise ValueError('expected_frame_rate must be positive')
            self.conn = ScaleConnection(settings)
            self._assembler = LineAssembler(decode_escapes(self._param('line_ending')))
            self._value_re = compile_value_regex(self._param('value_regex'))
        except ValueError as exc:
            self.get_logger().error(f'Bad configuration: {exc}')
            self.conn = None
            return TransitionCallbackReturn.FAILURE

        if not self._connect(self._param('wait_for_device_timeout')):
            self.conn = None
            return TransitionCallbackReturn.FAILURE

        for key, (msg_type, topic, enable_param, qos_param) in TOPICS.items():
            if not self._param(enable_param):
                continue
            self._pubs[key] = self.create_lifecycle_publisher(
                msg_type, topic, qos_profile_from_string(self._param(qos_param)))

        # ~/hw_reset and ~/device_info are about the adapter and the driver, not
        # about commanding the indicator, so they exist either way.
        self._own_services = [
            self.create_service(Trigger, '~/hw_reset', self.on_hw_reset),
            self.create_service(DeviceInfo, '~/device_info', self.on_device_info),
        ]
        if self._param('enable_commands'):
            self._cmd_sub = self.create_subscription(String, '~/cmd', self.on_cmd, 10)
            self._own_services += [
                self.create_service(Trigger, '~/tare', self.on_tare),
                self.create_service(Trigger, '~/zero', self.on_zero),
                self.create_service(SendCommand, '~/send_command', self.on_send_command),
            ]
        else:
            # Not advertised rather than advertised-and-always-failing: a
            # service in `ros2 service list` is a promise, and on this unit it
            # is one the hardware cannot keep.
            self.get_logger().info(
                'Read-only mode: ~/cmd, ~/tare, ~/zero and ~/send_command are not '
                'advertised. Set enable_commands:=true for an indicator whose RS232 '
                'port accepts input.')
        self._start_diagnostics()

        self.get_logger().info(
            f'Configured {self.conn.info.summary()} @ {self.conn.settings.describe()}')
        if self.conn.info.by_id:
            self.get_logger().info(
                f'Stable selectors for this adapter: port:={self.conn.info.by_id}'
                + (f' or serial_no:={self.conn.info.serial_number}'
                   if self.conn.info.serial_number else ''))
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._assembler.reset()
        with self._stats_lock:
            self._diag_baseline = (self._frames, self._parse_errors, time.monotonic())

        if self._param('poll_mode'):
            self._poll_command = decode_escapes(self._param('poll_command'))
            self._poll_failures = 0
            interval = self._param('poll_interval')
            self._poll_timer = self.create_timer(interval, self.poll_once)
            self.get_logger().info(
                f'Activated: polling every {interval}s with {self._poll_command!r}')
        else:
            self._start_reader()
            self.get_logger().info('Activated: streaming weight data')
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        self._stop_reader()
        if self._poll_timer is not None:
            self.destroy_timer(self._poll_timer)
            self._poll_timer = None
        self.get_logger().info('Deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _start_reader(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, name='scale-reader',
                                        daemon=True)
        self._thread.start()

    def _stop_reader(self):
        """
        Stop the reader thread and wait for it.

        This must complete before any publisher is destroyed, or the thread
        can publish into a freed handle.
        """
        self._stop_event.set()
        if self._thread is not None:
            # Bounded by one pyserial read timeout plus slack for the reconnect
            # poll interval; the reader checks the event on every iteration.
            self._thread.join(timeout=self._param('timeout') + 4.0)
            if self._thread.is_alive():
                self.get_logger().warn('Reader thread did not stop within the join timeout')
            self._thread = None

    def _teardown(self):
        self._active = False
        self._stop_reader()
        for attr in ('_poll_timer', '_diag_timer'):
            timer = getattr(self, attr)
            if timer is not None:
                self.destroy_timer(timer)
                setattr(self, attr, None)
        if self.conn is not None:
            self.conn.close()
            self.conn = None
        for key, pub in self._pubs.items():
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
                self._pubs[key] = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._cmd_sub is not None:
            self.destroy_subscription(self._cmd_sub)
            self._cmd_sub = None
        for srv in self._own_services:
            self.destroy_service(srv)
        self._own_services = []

    # -- connection ------------------------------------------------------
    def _selectors(self) -> dict:
        return {
            'port': self._param('port'),
            'serial_no': self._param('serial_no'),
            'usb_port_id': self._param('usb_port_id'),
            'device_type': self._param('device_type'),
        }

    def _describe_selectors(self) -> str:
        chosen = {key: value for key, value in self._selectors().items() if value}
        return ', '.join(f'{k}={v}' for k, v in chosen.items()) or 'any USB serial adapter'

    def _connect(self, wait_timeout: float, throttle: float = 0.0) -> bool:
        """
        Resolve the selectors and open the port.

        Logs and returns False on failure rather than raising, because all
        three callers — on_configure, the reconnect loop and the poll timer —
        want to decide for themselves what to do about it. `throttle` caps
        how often the failure is logged, for the poll path that retries
        several times a second.
        """
        info = wait_for_port(wait_timeout, stop_event=self._stop_event, **self._selectors())
        if info is None:
            if self._stop_event.is_set():
                return False
            self.get_logger().error(
                f'No serial port matched [{self._describe_selectors()}]'
                + (f' within {wait_timeout}s' if wait_timeout > 0 else '')
                + '. `ros2 run hansung_scale_driver scale_sniffer --ros-args '
                  '-p list_ports:=true` shows what is actually connected.',
                throttle_duration_sec=throttle)
            return False
        try:
            self.conn.open(info)
        except OSError as exc:
            hint = ''
            if 'Permission denied' in str(exc):
                hint = (' The user is not in the dialout group: '
                        '`sudo usermod -aG dialout $USER`, then log out and back in.')
            self.get_logger().error(f'Failed to open {info.device}: {exc}{hint}',
                                    throttle_duration_sec=throttle)
            return False
        if self._param('initial_reset'):
            self.conn.toggle_dtr()
            self.conn.reset_input()
        return True

    # -- reading ---------------------------------------------------------
    def _publish(self, text: str):
        reading = parse_frame(text, self._value_re)
        if reading is None:
            with self._stats_lock:
                self._parse_errors += 1
            self.get_logger().debug(f'No numeric match in: {text!r}')
            return

        stamp = self.get_clock().now().to_msg()
        with self._stats_lock:
            self._frames += 1
            self._last_reading = reading
            self._last_frame_time = time.monotonic()

        pub = self._pubs['weight_stamped']
        if pub is not None:
            msg = WeightStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._param('frame_id')
            msg.weight = float(reading.weight)
            msg.unit = reading.unit
            msg.weight_grams = float(reading.weight_grams)
            msg.stable = reading.stable
            msg.status = reading.status
            pub.publish(msg)

        if self._pubs['weight'] is not None:
            self._pubs['weight'].publish(Float32(data=float(reading.weight)))
        # Only a full frame actually carries stability and unit. Publishing the
        # defaults from a fallback-parsed line would look indistinguishable
        # from a real reading of "unstable, no unit".
        if reading.framed:
            if self._pubs['stable'] is not None:
                self._pubs['stable'].publish(Bool(data=reading.stable))
            if self._pubs['unit'] is not None:
                self._pubs['unit'].publish(String(data=reading.unit))

    def _handle_chunk(self, chunk: bytes) -> bool:
        """Publish every complete line in `chunk`; True if there was one."""
        with self._capture_lock:
            if self._capture is not None:
                self._capture += chunk
        handled = False
        for line in self._assembler.feed(chunk):
            text = line.decode('ascii', errors='replace').strip()
            if not text:
                continue
            handled = True
            if self._pubs['raw'] is not None:
                self._pubs['raw'].publish(String(data=text))
            self._publish(text)
        return handled

    def _read_loop(self):
        reconnect_timeout = self._param('reconnect_timeout')
        while rclpy.ok() and not self._stop_event.is_set():
            if not self.conn.is_open:
                if reconnect_timeout < 0:
                    self.get_logger().error('Link is down and reconnect_timeout is negative; '
                                            'stopping the reader.')
                    return
                self.get_logger().warn(f'Link down; retrying for up to {reconnect_timeout}s...')
                if not self._connect(reconnect_timeout):
                    if self._stop_event.is_set():
                        return
                    continue
                self._assembler.reset()
                self.get_logger().info(f'Reconnected to {self.conn.info.device}')

            try:
                chunk = self.conn.read()
            except OSError as exc:
                # A yanked USB adapter surfaces here, as does a port that was
                # reset underneath us. Drop the handle and let the loop above
                # decide whether to wait for it to come back.
                #
                # OSError rather than serial.SerialException throughout: that
                # exception subclasses IOError, so this catches it too, and it
                # also catches the bare OSError(ENOTTY) that devices without
                # modem control lines raise. An escaped exception inside a
                # service callback takes down the whole node.
                self.get_logger().error(f'Serial read error: {exc}')
                self.conn.close()
                continue
            if chunk:
                self._handle_chunk(chunk)

    def poll_once(self):
        """
        Request one reading, for indicators that answer only when asked.

        Reads until a terminated line arrives or the read timeout expires: a
        single read can come back with a partial frame, and at 2400 baud a
        20-byte reply takes ~80 ms. The assembler is reset alongside the
        driver buffer so a leftover fragment from the previous poll cannot be
        glued onto this reply.

        Keep `timeout` below `poll_interval` in poll mode — this runs on the
        executor thread and blocks it while it waits.
        """
        if self.conn is None:
            return
        if not self.conn.is_open and not self._poll_reconnect():
            return

        deadline = time.monotonic() + self._param('timeout')
        try:
            self.conn.reset_input()
            self._assembler.reset()
            self.conn.write(self._poll_command)
            while time.monotonic() < deadline:
                chunk = self.conn.read(max_bytes=512)
                if chunk and self._handle_chunk(chunk):
                    self._poll_failures = 0
                    return
        except OSError as exc:
            self.get_logger().error(f'Poll failed: {exc}')
            self.conn.close()
            return

        # A pty or adapter that went away mid-poll returns no bytes rather
        # than raising, so silence has to be what triggers recovery here —
        # otherwise poll mode would sit warning forever while streaming mode
        # reconnects.
        self._poll_failures += 1
        give_up = self._poll_failures >= self.POLL_FAILURES_BEFORE_RECONNECT
        # Throttled: a device that is simply gone would otherwise warn on
        # every tick, several times a second at a short poll_interval.
        self.get_logger().warn(
            f'No complete frame within {self._param("timeout")}s of sending '
            f'{self._poll_command!r}'
            + (' (giving up on this handle and reconnecting)' if give_up else ''),
            throttle_duration_sec=5.0)
        if give_up:
            self.conn.close()

    #: Consecutive silent polls before the handle is treated as dead.
    POLL_FAILURES_BEFORE_RECONNECT = 3

    def _poll_reconnect(self) -> bool:
        """Reopen the port between polls, without blocking the executor."""
        if self._param('reconnect_timeout') < 0:
            return False
        # Timeout 0: a single attempt per poll tick. Waiting here would stall
        # every other callback on this executor thread. The log is throttled
        # because poll_interval can be a fraction of a second.
        if not self._connect(0.0, throttle=5.0):
            return False
        self._assembler.reset()
        self._poll_failures = 0
        self.get_logger().info(f'Reconnected to {self.conn.info.device}')
        return True

    # -- commands --------------------------------------------------------
    def _send(self, data: bytes, response_timeout: float = 0.0) -> 'tuple[bool, str, bytes]':
        """
        Write `data` and return (ok, message, reply bytes).

        Collects whatever arrives for `response_timeout` seconds.

        A non-zero timeout blocks the calling executor thread for that long,
        so keep it short; it exists for protocol discovery, not for a control
        loop.
        """
        if self.conn is None or not self.conn.is_open:
            return False, 'serial link is not open', b''

        capturing = response_timeout > 0.0
        if capturing:
            with self._capture_lock:
                self._capture = bytearray()
        try:
            self.conn.write(data)
        except OSError as exc:
            with self._capture_lock:
                self._capture = None
            return False, f'write failed: {exc}', b''

        if not capturing:
            return True, f'sent {len(data)} byte(s)', b''

        deadline = time.monotonic() + response_timeout
        streaming = self._thread is not None and self._thread.is_alive()
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                if streaming:
                    # The reader thread owns the port and copies into _capture.
                    time.sleep(min(0.02, remaining))
                    continue
                try:
                    chunk = self.conn.read(max_bytes=512)
                except OSError as exc:
                    return False, f'read failed: {exc}', b''
                if chunk:
                    with self._capture_lock:
                        self._capture += chunk
            with self._capture_lock:
                reply = bytes(self._capture)
        finally:
            with self._capture_lock:
                self._capture = None
        return True, f'sent {len(data)} byte(s), got {len(reply)} back', reply

    def _send_param(self, param_name: str):
        text = self._param(param_name)
        if not text:
            return False, (f'{param_name} is empty. The HS-AA command bytes are not '
                           'documented; use ~/send_command to find them, then set '
                           f'{param_name}.')
        try:
            data = decode_escapes(text)
        except ValueError as exc:
            return False, f'{param_name} is not decodable: {exc}'
        ok, message, _ = self._send(data, self._param('command_response_timeout'))
        return ok, message

    def on_cmd(self, msg: String):
        if self.conn is None or not self.conn.is_open:
            self.get_logger().warn('~/cmd received while the link is down; ignoring')
            return
        try:
            data = decode_escapes(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f'~/cmd could not decode {msg.data!r}: {exc}')
            return
        ok, message, _ = self._send(data)
        self.get_logger().info(f'~/cmd {data!r}: {message}' if ok
                               else f'~/cmd {data!r} failed: {message}')

    def on_tare(self, request, response):
        response.success, response.message = self._send_param('tare_command')
        return response

    def on_zero(self, request, response):
        response.success, response.message = self._send_param('zero_command')
        return response

    def on_hw_reset(self, request, response):
        """
        Close and reopen the port, pulsing DTR.

        The serial equivalent of realsense-ros's `hw_reset`, for when the
        adapter or the indicator wedges.
        """
        if self.conn is None:
            response.success, response.message = False, 'node is not configured'
            return response

        # Stop the reader first: closing the fd underneath a blocking read
        # would have it race us into its own reconnect.
        streaming = self._thread is not None and self._thread.is_alive()
        if streaming:
            self._stop_reader()
        self.conn.close()
        self._stop_event.clear()

        response.success = self._connect(max(self._param('reconnect_timeout'), 0.0))
        if response.success:
            pulsed = self.conn.toggle_dtr()
            self.conn.reset_input()
            self._assembler.reset()
            response.message = f'reopened {self.conn.info.device}' + (
                '' if pulsed else ' (device has no modem control lines, so DTR was not pulsed)')
        else:
            response.message = 'could not reopen the port; see the node log'
        if streaming:
            self._start_reader()
        return response

    def on_send_command(self, request, response):
        try:
            data = decode_escapes(request.command)
        except ValueError as exc:
            response.success = False
            response.message = f'could not decode {request.command!r}: {exc}'
            return response
        if not data:
            response.success, response.message = False, 'command is empty'
            return response

        response.success, response.message, reply = self._send(data, request.response_timeout)
        response.response = encode_printable(reply)
        response.response_hex = reply.hex(' ')
        self.get_logger().info(f'~/send_command {data!r}: {response.message}')
        return response

    def on_device_info(self, request, response):
        info = (self.conn.info if self.conn is not None else None) or PortInfo(
            device=self._param('port'))
        response.device_name = DEVICE_NAME
        response.serial_number = info.serial_number
        response.physical_port = info.device
        response.port_id = info.by_id or by_id_path(info.device)
        response.usb_type_descriptor = info.vid_pid
        response.serial_settings = self.conn.settings.describe() if self.conn is not None else ''
        response.connected = self.conn is not None and self.conn.is_open
        with self._stats_lock:
            response.frame_rate = self._frame_rate
            response.frames_received = self._frames
            response.parse_errors = self._parse_errors
        return response

    # -- diagnostics -----------------------------------------------------
    def _start_diagnostics(self):
        period = self._param('diagnostics_period')
        if period <= 0.0:
            return
        # /diagnostics is deliberately absolute and not a lifecycle publisher:
        # "configured but not streaming" is exactly the state a monitor needs
        # to see, and rqt_robot_monitor only looks at the global topic.
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._diag_timer = self.create_timer(period, self._publish_diagnostics)

    def _publish_diagnostics(self):
        now = time.monotonic()
        with self._stats_lock:
            frames, errors = self._frames, self._parse_errors
            last_frame, reading = self._last_frame_time, self._last_reading
            base_frames, base_errors, base_time = self._diag_baseline
            elapsed = (now - base_time) if base_time else 0.0
            self._frame_rate = (frames - base_frames) / elapsed if elapsed > 0 else 0.0
            rate, new_errors = self._frame_rate, errors - base_errors
            self._diag_baseline = (frames, errors, now)

        connected = self.conn is not None and self.conn.is_open
        expected = self._param('expected_frame_rate')
        period = 1.0 / expected if expected > 0 else 0.0
        # Three frame periods of silence, floored at 1s so a slow indicator
        # does not make the status flap.
        stale_after = max(3.0 * period, 1.0)
        age = (now - last_frame) if last_frame else None

        status = DiagnosticStatus()
        status.name = f'{self.get_name()}: RS232 link'
        status.hardware_id = str(
            (self.conn.info.by_id or self.conn.info.device)
            if connected and self.conn.info else self._param('port'))
        if not connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Serial link is down'
        elif not self._active:
            status.level = DiagnosticStatus.OK
            status.message = 'Connected, not streaming (lifecycle state is not active)'
        elif age is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'No frames received yet'
        elif age > stale_after:
            status.level = DiagnosticStatus.WARN
            status.message = (f'No frame for {age:.1f}s '
                              f'(expected one every {period:.2f}s)')
        elif new_errors:
            status.level = DiagnosticStatus.WARN
            status.message = f'{new_errors} unparseable line(s) in the last window'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Streaming at {rate:.1f} frames/s'

        status.values = [
            KeyValue(key='port', value=status.hardware_id),
            KeyValue(key='serial_settings',
                     value=self.conn.settings.describe() if self.conn is not None else ''),
            KeyValue(key='frame_rate_hz', value=f'{rate:.2f}'),
            KeyValue(key='expected_frame_rate_hz', value=f'{expected:.2f}'),
            KeyValue(key='frames_received', value=str(frames)),
            KeyValue(key='parse_errors', value=str(errors)),
            KeyValue(key='dropped_bytes',
                     value=str(self._assembler.dropped_bytes if self._assembler else 0)),
            KeyValue(key='last_frame_age_s', value='n/a' if age is None else f'{age:.2f}'),
            KeyValue(key='last_weight', value='n/a' if reading is None
                     else f'{reading.weight} {reading.unit}'.strip()),
            KeyValue(key='last_status',
                     value='n/a' if reading is None else (reading.status or '-')),
            KeyValue(key='stable', value='n/a' if reading is None else str(reading.stable)),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    # -- parameters ------------------------------------------------------
    def _on_set_parameters(self, params) -> SetParametersResult:
        """
        Accept the dynamic parameters at runtime and refuse the rest.

        Once the node is configured, a non-dynamic parameter is rejected the
        way realsense-ros rejects its own, rather than accepting a value that
        will never take effect.
        """
        configured = self.conn is not None
        for param in params:
            if configured and param.name not in DYNAMIC_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=(f'{param.name} is only read while configuring. Run `ros2 lifecycle '
                            'set scale_node cleanup` then `configure` to apply it.'))
            if param.name == 'value_regex':
                try:
                    self._value_re = compile_value_regex(param.value)
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
            elif param.name == 'expected_frame_rate' and param.value <= 0.0:
                return SetParametersResult(successful=False,
                                           reason='expected_frame_rate must be positive')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ScaleNode()
    if node.get_parameter('autostart').value:
        if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error(
                'Auto-configure failed; staying unconfigured. Fix the port/params, then '
                '`ros2 lifecycle set scale_node configure`.')
        else:
            node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
