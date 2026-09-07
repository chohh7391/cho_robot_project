"""
Serial port discovery, selection and connection handling.

realsense-ros never makes you name a device node: you say which camera you
mean with `serial_no`, `usb_port_id` or `device_type`, and it waits for that
device to show up (`wait_for_device_timeout`) and re-attaches when it comes
back (`reconnect_timeout`). USB-RS232 adapters need exactly the same thing —
`/dev/ttyUSB0` and `/dev/ttyUSB1` swap on every reboot, and unplugging the
adapter makes the device node disappear outright — so this module provides
the RS232 equivalents.
"""
import glob
import os
import re
import threading
import time
from dataclasses import dataclass

import serial
import serial.tools.list_ports

BYTESIZE_MAP = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}
# Spelled out rather than 'N'/'E'/'O': bare N/Y/n/y are YAML 1.1 boolean
# literals, so a single-letter code silently becomes True/False the moment it
# passes through a params file or a launch substitution. The single letters
# are still accepted for anyone who already has them in a config.
PARITY_MAP = {
    'NONE': serial.PARITY_NONE, 'N': serial.PARITY_NONE,
    'EVEN': serial.PARITY_EVEN, 'E': serial.PARITY_EVEN,
    'ODD': serial.PARITY_ODD, 'O': serial.PARITY_ODD,
    'MARK': serial.PARITY_MARK, 'M': serial.PARITY_MARK,
    'SPACE': serial.PARITY_SPACE, 'S': serial.PARITY_SPACE,
}
STOPBITS_MAP = {
    1: serial.STOPBITS_ONE,
    1.5: serial.STOPBITS_ONE_POINT_FIVE,
    2: serial.STOPBITS_TWO,
}
PARITY_LETTER = {serial.PARITY_NONE: 'N', serial.PARITY_EVEN: 'E', serial.PARITY_ODD: 'O',
                 serial.PARITY_MARK: 'M', serial.PARITY_SPACE: 'S'}


@dataclass(frozen=True)
class SerialSettings:
    """Line settings, validated once instead of at every use site."""

    baudrate: int = 2400          # confirmed on the HS-AA
    bytesize: int = 8
    parity: str = 'NONE'
    stopbits: float = 1
    timeout: float = 1.0

    def to_kwargs(self) -> dict:
        """Build pyserial constructor kwargs, raising on an unusable combination."""
        try:
            bytesize = BYTESIZE_MAP[int(self.bytesize)]
        except (KeyError, ValueError):
            raise ValueError(f'bytesize must be one of {sorted(BYTESIZE_MAP)}, '
                             f'got {self.bytesize!r}')
        try:
            parity = PARITY_MAP[str(self.parity).upper()]
        except KeyError:
            raise ValueError(f'parity must be one of NONE, EVEN, ODD, MARK, SPACE; '
                             f'got {self.parity!r}')
        try:
            stopbits = STOPBITS_MAP[float(self.stopbits)]
        except (KeyError, ValueError):
            raise ValueError(f'stopbits must be 1, 1.5 or 2, got {self.stopbits!r}')
        if int(self.baudrate) <= 0:
            raise ValueError(f'baudrate must be positive, got {self.baudrate!r}')
        return {
            'baudrate': int(self.baudrate),
            'bytesize': bytesize,
            'parity': parity,
            'stopbits': stopbits,
            'timeout': float(self.timeout),
        }

    def describe(self) -> str:
        """Human-readable form, e.g. '2400 8N1'."""
        kwargs = self.to_kwargs()
        stopbits = kwargs['stopbits']
        stopbits = int(stopbits) if float(stopbits).is_integer() else stopbits
        return (f'{kwargs["baudrate"]} {kwargs["bytesize"]}'
                f'{PARITY_LETTER[kwargs["parity"]]}{stopbits}')


@dataclass(frozen=True)
class PortInfo:
    """What we know about one candidate port, for logging and ~/device_info."""

    device: str
    serial_number: str = ''
    location: str = ''
    vid_pid: str = ''
    description: str = ''
    manufacturer: str = ''
    product: str = ''
    by_id: str = ''

    def summary(self) -> str:
        bits = [self.device]
        if self.vid_pid:
            bits.append(self.vid_pid)
        if self.serial_number:
            bits.append(f'sn={self.serial_number}')
        if self.location:
            bits.append(f'usb_port_id={self.location}')
        if self.description and self.description != 'n/a':
            bits.append(self.description)
        return ' '.join(bits)


def by_id_path(device: str) -> str:
    """
    Return the stable /dev/serial/by-id symlink for `device`, or ''.

    This is the path worth putting in a config file: it is derived from the
    adapter's VID/PID/serial, so it survives replugging and reboots, unlike
    the ttyUSB index.
    """
    try:
        target = os.path.realpath(device)
    except OSError:
        return ''
    for link in glob.glob('/dev/serial/by-id/*'):
        if os.path.realpath(link) == target:
            return link
    return ''


def list_ports() -> 'list[PortInfo]':
    """Every serial port pyserial can see, newest enumeration order aside."""
    infos = []
    for port in serial.tools.list_ports.comports():
        vid_pid = ''
        if port.vid is not None and port.pid is not None:
            vid_pid = f'{port.vid:04x}:{port.pid:04x}'
        infos.append(PortInfo(
            device=port.device,
            serial_number=port.serial_number or '',
            location=port.location or '',
            vid_pid=vid_pid,
            description=port.description or '',
            manufacturer=port.manufacturer or '',
            product=port.product or '',
            by_id=by_id_path(port.device),
        ))
    return sorted(infos, key=lambda info: info.device)


def find_port(port: str = '', serial_no: str = '', usb_port_id: str = '',
              device_type: str = '') -> 'PortInfo | None':
    """
    Resolve the selectors to exactly one port, or None if none matched.

    Mirrors realsense-ros's selection semantics, with `port` added because
    RS232 (unlike librealsense) has no device registry to enumerate against:

    * `serial_no` — exact match on the adapter's USB serial number. The FTDI
      and CH340-class adapters used for RS232 all carry one, so this is the
      selector that survives a reboot.
    * `usb_port_id` — the physical USB location, matched as a prefix of what
      pyserial reports, so '8-1' also matches a '8-1:1.0' location. Use this
      for the cheap adapters that ship without a serial number: it pins
      "whatever is in *that* socket".
    * `device_type` — regex matched against description/manufacturer/product,
      e.g. 'ftdi'.
    * `port` — an explicit device path, or a /dev/serial/by-id symlink. Only
      consulted when none of the three selectors above is set, so setting
      `serial_no` does not silently lose to the default `port` value. A path
      that exists is accepted even when pyserial does not enumerate it, so
      raw `/dev/ttyS0` and socat pty pairs still work.

    The three selectors combine with AND. With nothing set at all, ports with
    a USB identity are preferred over the motherboard `/dev/ttyS*` nodes the
    kernel enumerates whether or not anything is wired to them. When several
    ports still match, the lowest-sorting device path wins so the choice is at
    least stable across restarts.
    """
    candidates = list_ports()

    if serial_no:
        candidates = [info for info in candidates if info.serial_number == serial_no]
    if usb_port_id:
        candidates = [info for info in candidates
                      if info.location and info.location.startswith(usb_port_id)]
    if device_type:
        pattern = re.compile(device_type, re.IGNORECASE)
        candidates = [info for info in candidates
                      if pattern.search(info.description) or pattern.search(info.manufacturer)
                      or pattern.search(info.product)]

    if serial_no or usb_port_id or device_type:
        return candidates[0] if candidates else None

    if port:
        for info in candidates:
            if port in (info.device, info.by_id):
                return info
        if os.path.exists(port):
            resolved = os.path.realpath(port)
            for info in candidates:
                if os.path.realpath(info.device) == resolved:
                    return info
            return PortInfo(device=port, by_id=by_id_path(port))
        return None

    usb = [info for info in candidates if info.by_id or info.vid_pid]
    return (usb or candidates or [None])[0]


def wait_for_port(timeout: float, poll_interval: float = 0.5, stop_event=None,
                  **selectors) -> 'PortInfo | None':
    """
    Poll `find_port` until it matches or `timeout` seconds elapse.

    `timeout <= 0` means a single attempt (realsense-ros's
    `wait_for_device_timeout: -1`, i.e. do not wait). A negative timeout of
    `float('inf')` waits forever. `stop_event` lets a shutdown interrupt the
    wait instead of blocking the whole timeout.
    """
    info = find_port(**selectors)
    if info is not None or timeout <= 0:
        return info
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.wait(poll_interval):
            return None
        elif stop_event is None:
            time.sleep(poll_interval)
        info = find_port(**selectors)
        if info is not None:
            return info
    return None


class ScaleConnection:
    """
    The open port plus the write lock, so the node never touches pyserial.

    Writes come from service/subscription callbacks on the executor thread
    while the reader thread is inside `read()`, hence the lock. pyserial is
    not thread-safe across a concurrent read and write on every platform, but
    reads here are the blocking half and must not be serialized against
    writes, so the lock guards writes and buffer resets only — the same
    trade-off the original single-threaded loop made implicitly.
    """

    def __init__(self, settings: SerialSettings):
        self.settings = settings
        self._kwargs = settings.to_kwargs()   # validate eagerly, before any I/O
        self._ser = None
        self._info = None
        self._write_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def info(self) -> 'PortInfo | None':
        return self._info

    def open(self, info: PortInfo) -> None:
        """Open `info.device`. Raises serial.SerialException on failure."""
        self.close()
        self._ser = serial.Serial(port=info.device, **self._kwargs)
        self._info = info

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:      # noqa: BLE001 - closing a yanked USB device
                pass                # raises all sorts; nothing useful to do
            self._ser = None

    def reset_input(self) -> bool:
        """Discard buffered input. False if the device would not do it."""
        if not self.is_open:
            return False
        try:
            with self._write_lock:
                self._ser.reset_input_buffer()
        except OSError:
            return False
        return True

    def toggle_dtr(self, hold: float = 0.1) -> bool:
        """
        Pulse DTR low then high -- the `initial_reset` analogue.

        Some indicators and adapters only start streaming once DTR is
        asserted.

        Returns False when the device has no modem control lines to toggle —
        a pty, a plain file, some virtual ports — because that raises bare
        OSError(ENOTTY) rather than SerialException, and an unhandled
        exception inside a service callback takes the whole node down.
        """
        if not self.is_open:
            return False
        try:
            with self._write_lock:
                self._ser.dtr = False
                time.sleep(hold)
                self._ser.dtr = True
        except OSError:
            return False
        return True

    def read(self, max_bytes: int = 256) -> bytes:
        """
        Block until at least one byte arrives or the read timeout expires.

        Drains whatever else is already buffered in the same call so a slow
        consumer cannot fall behind one byte at a time.
        """
        if not self.is_open:
            raise serial.SerialException('port is not open')
        waiting = self._ser.in_waiting
        return self._ser.read(min(waiting, max_bytes) if waiting else 1)

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise serial.SerialException('port is not open')
        with self._write_lock:
            written = self._ser.write(data)
            self._ser.flush()
        return written
