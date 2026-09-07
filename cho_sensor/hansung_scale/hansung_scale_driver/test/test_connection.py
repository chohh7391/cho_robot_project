"""
Integration tests for ScaleConnection against a pty.

`os.openpty()` gives a real character device with a real serial fd, so open /
read / write / close are exercised for real without the scale being plugged
in. It also reproduces the one thing a pty does *differently* from a USB
adapter — no modem control lines — which is worth pinning down, because that
difference used to take the node down.
"""
import os
import time

import pytest

from hansung_scale_driver.device import PortInfo, ScaleConnection, SerialSettings

FRAME = b'WTST+   0.00   g\r\n'


def read_exactly(conn, count, attempts=40):
    """
    Accumulate `count` bytes.

    One `read()` returns only what the driver has already buffered, which
    right after a write on the other end of a pty can be a single byte.
    """
    data = b''
    for _ in range(attempts):
        if len(data) >= count:
            break
        data += conn.read(max_bytes=512)
    return data


@pytest.fixture
def pty_pair():
    """(master fd, ScaleConnection open on the slave end)."""
    master, slave = os.openpty()
    conn = ScaleConnection(SerialSettings(timeout=0.5))
    conn.open(PortInfo(device=os.ttyname(slave)))
    try:
        yield master, conn
    finally:
        conn.close()
        os.close(master)
        os.close(slave)


def test_open_reports_the_port_as_open(pty_pair):
    _, conn = pty_pair
    assert conn.is_open
    assert conn.info.device.startswith('/dev/pts/')


def test_read_returns_what_the_other_end_wrote(pty_pair):
    master, conn = pty_pair
    os.write(master, FRAME)
    assert read_exactly(conn, len(FRAME)) == FRAME


def test_read_drains_several_frames(pty_pair):
    master, conn = pty_pair
    os.write(master, FRAME * 3)
    assert read_exactly(conn, len(FRAME) * 3) == FRAME * 3


def test_read_returns_empty_on_timeout_rather_than_blocking_forever(pty_pair):
    _, conn = pty_pair
    assert conn.read() == b''


def test_write_reaches_the_other_end(pty_pair):
    master, conn = pty_pair
    assert conn.write(b'hex') == 3
    received = b''
    while len(received) < 3:
        received += os.read(master, 16)
    assert received == b'hex'


def test_toggle_dtr_on_a_device_without_modem_lines_reports_failure(pty_pair):
    # Regression: a pty raises bare OSError(ENOTTY) here, not
    # serial.SerialException. Letting that escape killed the node from inside
    # the ~/hw_reset service callback.
    _, conn = pty_pair
    assert conn.toggle_dtr(hold=0.0) is False


def test_reset_input_does_not_raise_on_a_pty(pty_pair):
    _, conn = pty_pair
    conn.reset_input()


def test_reset_input_discards_buffered_data(pty_pair):
    master, conn = pty_pair
    os.write(master, FRAME)
    time.sleep(0.1)          # let the pty actually deliver it before flushing
    conn.reset_input()
    assert conn.read() == b''


def test_close_is_idempotent(pty_pair):
    _, conn = pty_pair
    conn.close()
    conn.close()
    assert not conn.is_open


def test_operations_after_close_raise_rather_than_silently_no_op(pty_pair):
    _, conn = pty_pair
    conn.close()
    with pytest.raises(OSError):
        conn.read()
    with pytest.raises(OSError):
        conn.write(b'x')


def test_no_modem_line_helpers_when_closed(pty_pair):
    _, conn = pty_pair
    conn.close()
    assert conn.toggle_dtr() is False
    assert conn.reset_input() is False


def test_bad_settings_are_rejected_before_the_port_is_touched():
    with pytest.raises(ValueError):
        ScaleConnection(SerialSettings(parity='X'))
