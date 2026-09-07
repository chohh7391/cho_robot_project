r"""
Frame parsing for the Hansung(한성전자저울) HS-AA family.

Kept free of ROS and of pyserial so it can be unit-tested without the
indicator plugged in.

realsense-ros keeps the librealsense conversation in its own layer and lets
the node do nothing but publish; this module is the same split for RS232 —
bytes in, `ScaleReading` out.

Protocol confirmed by sniffing the actual device: 2400 baud, 8N1, continuous
ASCII output, one frame per cycle, no request command needed::

    WT<status:2><sign:1>   <value>   <unit>\\r\\n
    e.g. b'WTST+   0.00   g\\r\\n'

* ``status``: ``ST`` = stable. ``US`` (unstable) / ``OL`` (overload) follow the
  convention for this frame style but have **not** been seen on real hardware,
  so only ``ST`` is treated as authoritative: `stable` is ``status == 'ST'``
  rather than ``status != 'US'``.
* ``sign``: ``+`` or ``-``
* ``value``/``unit``: whitespace-padded, so strip before parsing
"""
import codecs
import math
import re
from dataclasses import dataclass

#: The confirmed HS-AA frame.
HS_AA_FRAME = re.compile(
    r'^WT(?P<status>\S{2})(?P<sign>[+-])\s*(?P<value>\d+\.?\d*)\s*(?P<unit>\S*)$')

#: Generic numeric extractor, used when a line does not match HS_AA_FRAME.
DEFAULT_VALUE_REGEX = r'[-+]?\d+\.?\d*'

STATUS_STABLE = 'ST'
STATUS_UNSTABLE = 'US'
STATUS_OVERLOAD = 'OL'

#: Grams per unit, for the units weighing indicators in this class emit.
#: Anything not listed leaves `weight_grams` as NaN rather than guessing.
UNIT_TO_GRAMS = {
    'mg': 1e-3,
    'g': 1.0,
    'kg': 1e3,
    'ct': 0.2,                # metric carat
    'gn': 0.06479891,         # grain
    'oz': 28.349523125,
    'ozt': 31.1034768,        # troy ounce
    'dwt': 1.55517384,        # pennyweight
    'lb': 453.59237,
}


@dataclass(frozen=True)
class ScaleReading:
    """
    One parsed line.

    `framed` distinguishes a full HS-AA frame (status/unit/sign all known)
    from a line that only matched the generic numeric fallback, where
    everything except `weight` is a default.
    """

    weight: float
    unit: str = ''
    stable: bool = False
    status: str = ''
    framed: bool = False

    @property
    def weight_grams(self) -> float:
        """`weight` in grams, or NaN when `unit` is not in UNIT_TO_GRAMS."""
        factor = UNIT_TO_GRAMS.get(self.unit.strip().lower())
        if factor is None:
            return math.nan
        return self.weight * factor

    @property
    def overload(self) -> bool:
        return self.status == STATUS_OVERLOAD


def parse_frame(text: str, value_re: 're.Pattern | None' = None) -> 'ScaleReading | None':
    """
    Parse one line of indicator output.

    Tries the confirmed HS-AA frame first. If that fails — a different
    Hansung model, or another brand of indicator wired up later — falls back
    to pulling the first number out of the line with `value_re`, so this
    still works as a plain "one number per line" driver.

    Returns None when the line carries no number at all.
    """
    text = text.strip()
    if not text:
        return None

    match = HS_AA_FRAME.match(text)
    if match:
        value = float(match.group('value'))
        if match.group('sign') == '-':
            value = -value
        status = match.group('status')
        return ScaleReading(
            weight=value,
            unit=match.group('unit'),
            stable=status == STATUS_STABLE,
            status=status,
            framed=True,
        )

    match = (value_re or re.compile(DEFAULT_VALUE_REGEX)).search(text)
    if not match:
        return None
    try:
        return ScaleReading(weight=float(re.sub(r'\s+', '', match.group())))
    except ValueError:
        return None


def decode_escapes(text: str) -> bytes:
    r"""
    Turn a parameter string into the raw bytes to put on the wire.

    Two forms, so control bytes can be expressed either way in YAML:

    * ``'hex:05'`` / ``'hex:1B 40'`` — literal hex, for ENQ, ESC and friends
    * ``'T\\r\\n'`` — backslash escapes, for printable commands
    """
    if not text:
        return b''
    if text.startswith('hex:'):
        return bytes.fromhex(text[4:].replace(' ', ''))
    return codecs.decode(text, 'unicode_escape').encode('latin1')


def encode_printable(data: bytes) -> str:
    """
    Render bytes for a log line or a service response.

    Printable ASCII is kept as-is; everything else is shown as a period.
    """
    return ''.join(chr(b) if 0x20 <= b < 0x7F else '.' for b in data)


class LineAssembler:
    """
    Split a byte stream into terminator-delimited lines.

    A serial read returns whatever happened to be in the FIFO, so frames
    arrive split across reads and several frames arrive in one read. Keeping
    the leftover here (rather than inline in the read loop) is what makes the
    framing testable.
    """

    #: Cap on unterminated data. At 2400 baud a ~20-byte frame arrives every
    #: ~100 ms; anything past this means the terminator is misconfigured or
    #: the line is noise, and we drop it instead of growing without bound.
    MAX_BUFFER = 4096

    def __init__(self, terminator: bytes = b'\r\n'):
        if not terminator:
            raise ValueError('terminator must be at least one byte')
        self.terminator = terminator
        self._buf = bytearray()
        self.dropped_bytes = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, chunk: bytes) -> 'list[bytes]':
        """Add `chunk` and return every complete line it finished."""
        self._buf += chunk
        lines = []
        while True:
            index = self._buf.find(self.terminator)
            if index < 0:
                break
            lines.append(bytes(self._buf[:index]))
            del self._buf[:index + len(self.terminator)]
        if len(self._buf) > self.MAX_BUFFER:
            self.dropped_bytes += len(self._buf)
            self._buf.clear()
        return lines
