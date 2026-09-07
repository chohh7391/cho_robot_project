"""
Unit tests for line settings validation and port selection.

Port selection is tested against a fake `list_ports` rather than real
hardware: the point is the precedence rules between `port`, `serial_no`,
`usb_port_id` and `device_type`, which are pure logic.
"""
import pytest

from hansung_scale_driver import device
from hansung_scale_driver.device import PortInfo, SerialSettings, find_port, wait_for_port

FTDI = PortInfo(device='/dev/ttyUSB0', serial_number='FTEFY2BT', location='8-1',
                vid_pid='0403:6001', description='USB Serial Converter',
                manufacturer='FTDI', product='USB Serial Converter',
                by_id='/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTEFY2BT-if00-port0')
CH340 = PortInfo(device='/dev/ttyUSB1', serial_number='', location='2-4',
                 vid_pid='1a86:7523', description='USB Serial',
                 manufacturer='QinHeng', product='USB Serial')
LEGACY = PortInfo(device='/dev/ttyS0')


@pytest.fixture
def ports(monkeypatch):
    """Replace enumeration with a fixed list; returns it so a test can edit it."""
    listing = [LEGACY, FTDI, CH340]
    monkeypatch.setattr(device, 'list_ports', lambda: list(listing))
    return listing


class TestSerialSettings:

    def test_hs_aa_defaults_describe_as_2400_8n1(self):
        assert SerialSettings().describe() == '2400 8N1'

    def test_describe_reflects_overrides(self):
        assert SerialSettings(baudrate=9600, parity='EVEN', stopbits=2).describe() == '9600 8E2'

    def test_single_letter_parity_still_accepted(self):
        assert SerialSettings(parity='N').describe() == '2400 8N1'

    def test_lowercase_parity_accepted(self):
        assert SerialSettings(parity='even').describe() == '2400 8E1'

    @pytest.mark.parametrize('kwargs', [
        {'parity': 'X'},
        {'bytesize': 9},
        {'stopbits': 3},
        {'baudrate': 0},
        {'baudrate': -1},
    ])
    def test_unusable_settings_raise_before_any_io(self, kwargs):
        with pytest.raises(ValueError):
            SerialSettings(**kwargs).to_kwargs()

    def test_error_names_the_offending_field(self):
        with pytest.raises(ValueError, match='parity'):
            SerialSettings(parity='X').to_kwargs()


class TestFindPort:

    def test_serial_no_selects_that_adapter(self, ports):
        assert find_port(serial_no='FTEFY2BT').device == '/dev/ttyUSB0'

    def test_serial_no_outranks_the_default_port_value(self, ports):
        # The whole point: leaving `port` at its default must not silently
        # beat an explicitly configured selector.
        assert find_port(port='/dev/ttyUSB0', serial_no='').device == '/dev/ttyUSB0'
        assert find_port(port='/dev/ttyUSB0', usb_port_id='2-4').device == '/dev/ttyUSB1'

    def test_usb_port_id_matches_as_a_prefix(self, ports):
        assert find_port(usb_port_id='8-1').device == '/dev/ttyUSB0'
        assert find_port(usb_port_id='8-1:1.0') is None

    def test_device_type_is_a_case_insensitive_regex(self, ports):
        assert find_port(device_type='ftdi').device == '/dev/ttyUSB0'
        assert find_port(device_type='qinheng|prolific').device == '/dev/ttyUSB1'

    def test_selectors_combine_with_and(self, ports):
        assert find_port(serial_no='FTEFY2BT', usb_port_id='2-4') is None

    def test_unmatched_selector_returns_none_rather_than_a_fallback(self, ports):
        assert find_port(serial_no='NOPE') is None

    def test_explicit_device_path(self, ports):
        assert find_port(port='/dev/ttyUSB1').device == '/dev/ttyUSB1'

    def test_by_id_symlink_resolves_to_the_device(self, ports):
        assert find_port(port=FTDI.by_id).device == '/dev/ttyUSB0'

    def test_missing_explicit_path_returns_none(self, ports):
        assert find_port(port='/dev/ttyUSB99') is None

    def test_bare_selection_prefers_usb_over_motherboard_ports(self, ports):
        # /dev/ttyS0 sorts first but is a kernel-enumerated legacy port that
        # is almost never the scale.
        assert find_port().device == '/dev/ttyUSB0'

    def test_bare_selection_falls_back_when_only_legacy_ports_exist(self, ports):
        ports[:] = [LEGACY]
        assert find_port().device == '/dev/ttyS0'

    def test_no_ports_at_all(self, ports):
        ports[:] = []
        assert find_port() is None


class TestWaitForPort:

    def test_returns_immediately_when_already_present(self, ports):
        assert wait_for_port(10.0, serial_no='FTEFY2BT').device == '/dev/ttyUSB0'

    def test_non_positive_timeout_tries_once(self, ports):
        ports[:] = []
        assert wait_for_port(-1.0, serial_no='FTEFY2BT') is None

    def test_picks_up_a_port_that_appears_during_the_wait(self, ports, monkeypatch):
        ports[:] = []
        calls = {'n': 0}

        def appearing():
            calls['n'] += 1
            return [FTDI] if calls['n'] > 2 else []

        monkeypatch.setattr(device, 'list_ports', appearing)
        assert wait_for_port(5.0, poll_interval=0.01,
                             serial_no='FTEFY2BT').device == '/dev/ttyUSB0'

    def test_a_set_stop_event_aborts_the_wait(self, ports):
        import threading
        ports[:] = []
        stop = threading.Event()
        stop.set()
        assert wait_for_port(30.0, poll_interval=0.01, stop_event=stop,
                             serial_no='FTEFY2BT') is None


class TestByIdPath:

    def test_a_path_with_no_symlink_returns_empty(self):
        assert device.by_id_path('/dev/null') == ''

    def test_a_nonexistent_path_returns_empty(self):
        assert device.by_id_path('/dev/definitely-not-here') == ''
