r"""
Protocol-discovery tool: dump the raw bytes an indicator sends.

Reading the exact output format (framing, terminator, ASCII layout) off that
dump is how `scale_node` gets configured.

This is how the HS-AA's `WTST+   0.00   g\\r\\n` frame and its 2400 8N1 line
settings were established in the first place, since no protocol document for
the unit is public.

Three modes:

* ``-p list_ports:=true`` — print every serial port with its USB serial
  number and physical location, then exit. Use it to fill in `serial_no` or
  `usb_port_id` instead of hard-coding `/dev/ttyUSB0`.
* default — stream HEX/ASCII of everything that arrives.
* ``-p decode:=true`` — additionally show how `hansung_scale_driver.protocol` would parse
  each assembled line, so a candidate `line_ending` / `value_regex` can be
  checked before it goes into a params file.
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .device import ScaleConnection, SerialSettings, find_port, list_ports
from .protocol import LineAssembler, decode_escapes, parse_frame


def print_port_table():
    """
    Print every serial port, USB adapters first.

    The kernel enumerates 32 `/dev/ttyS*` legacy ports whether or not anything
    is wired to them, so listing in device order buries the one adapter you
    are looking for.
    """
    ports = list_ports()
    if not ports:
        print('No serial ports found.', file=sys.stderr)
        return
    usb = [info for info in ports if info.by_id or info.vid_pid]
    legacy = [info for info in ports if info not in usb]

    header = f'{"DEVICE":<20} {"VID:PID":<10} {"SERIAL":<18} {"USB PORT":<12} DESCRIPTION'
    print(header)
    print('-' * len(header))
    for info in usb + legacy:
        print(f'{info.device:<20} {info.vid_pid or "-":<10} {info.serial_number or "-":<18} '
              f'{info.location or "-":<12} {info.description or "-"}')
    if not usb:
        print('\n# No USB serial adapter found. Everything above is a kernel-enumerated '
              'legacy port.')
        return
    print('\n# Stable selectors for scale_node (any one of these):')
    selectors = []
    for info in usb:
        if info.serial_number:
            selectors.append((f'-p serial_no:={info.serial_number}', info.device))
        if info.location:
            selectors.append((f'-p usb_port_id:={info.location}', info.device))
        if info.by_id:
            selectors.append((f'-p port:={info.by_id}', info.device))
    width = max(len(text) for text, _ in selectors)
    for text, deviceroot in selectors:
        print(f'#   {text:<{width}}   ({deviceroot})')


class RawSniffer(Node):
    """
    Plain (non-lifecycle) node.

    This is a debugging tool, so it opens the port and starts printing the
    moment it is run.
    """

    def __init__(self):
        super().__init__('scale_sniffer')

        self.declare_parameter('list_ports', False)
        self.declare_parameter('decode', False)

        # Same selectors as scale_node, so a working sniffer invocation
        # translates directly into scale_node parameters.
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('serial_no', '')
        self.declare_parameter('usb_port_id', '')
        self.declare_parameter('device_type', '')

        # 2400 is the confirmed HS-AA rate. When sniffing an unknown unit,
        # step through 1200/2400/4800/9600/19200 until the ASCII stops looking
        # like garbage.
        self.declare_parameter('baudrate', 2400)
        self.declare_parameter('bytesize', 8)
        self.declare_parameter('parity', 'NONE')
        self.declare_parameter('stopbits', 1.0)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('line_ending', '\r\n')

        if self.get_parameter('list_ports').value:
            print_port_table()
            raise SystemExit(0)

        self.decode = self.get_parameter('decode').value
        self.assembler = LineAssembler(decode_escapes(self.get_parameter('line_ending').value))
        self.pub = self.create_publisher(String, '~/raw_hex', 10)

        info = find_port(port=self.get_parameter('port').value,
                         serial_no=self.get_parameter('serial_no').value,
                         usb_port_id=self.get_parameter('usb_port_id').value,
                         device_type=self.get_parameter('device_type').value)
        if info is None:
            self.get_logger().error(
                'No matching serial port. Run with -p list_ports:=true to see what is '
                'connected.')
            raise SystemExit(1)

        settings = SerialSettings(
            baudrate=self.get_parameter('baudrate').value,
            bytesize=self.get_parameter('bytesize').value,
            parity=self.get_parameter('parity').value,
            stopbits=self.get_parameter('stopbits').value,
            timeout=self.get_parameter('timeout').value)
        self.conn = ScaleConnection(settings)
        try:
            self.conn.open(info)
        except OSError as exc:
            hint = ''
            if 'Permission denied' in str(exc):
                hint = (' The user is not in the dialout group: '
                        '`sudo usermod -aG dialout $USER`, then log out and back in.')
            self.get_logger().error(f'Failed to open {info.device}: {exc}{hint}')
            raise SystemExit(1)

        self.get_logger().info(
            f'Sniffing {info.summary()} @ {settings.describe()}. Put something on the scale '
            'and watch this output, or echo ~/raw_hex.')
        self.timer = self.create_timer(0.05, self.poll)

    def poll(self):
        try:
            data = self.conn.read(max_bytes=1024) if self.conn.is_open else b''
        except OSError as exc:
            self.get_logger().error(f'Read error: {exc}')
            self.conn.close()
            return
        if not data:
            return

        ascii_str = data.decode('ascii', errors='replace')
        line = f'HEX[{data.hex(" ")}] ASCII[{ascii_str!r}]'
        print(line, flush=True)
        self.pub.publish(String(data=line))

        if not self.decode:
            return
        for raw in self.assembler.feed(data):
            text = raw.decode('ascii', errors='replace').strip()
            reading = parse_frame(text)
            if reading is None:
                print(f'  LINE {text!r} -> no number found', flush=True)
            else:
                kind = 'HS-AA frame' if reading.framed else 'value_regex fallback'
                print(f'  LINE {text!r} -> {kind}: weight={reading.weight} '
                      f'unit={reading.unit!r} stable={reading.stable} '
                      f'status={reading.status!r} grams={reading.weight_grams}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RawSniffer()
    except SystemExit as exc:
        rclpy.shutdown()
        return exc.code
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.conn.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
