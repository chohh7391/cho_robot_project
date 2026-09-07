"""
Unit tests for the frame parser.

These are the tests that matter most on this package: the HS-AA protocol was
reverse-engineered from a byte dump rather than a datasheet, so the frame
layout is the part most likely to be wrong or to drift. They need no serial
port and no ROS graph.
"""
import math

import pytest

from hansung_scale_driver.protocol import (
    LineAssembler,
    decode_escapes,
    encode_printable,
    parse_frame,
)


class TestParseFrame:

    def test_confirmed_stable_frame(self):
        # The exact bytes captured from the real HS-AA unit.
        reading = parse_frame('WTST+   0.00   g')
        assert reading.framed
        assert reading.weight == 0.0
        assert reading.unit == 'g'
        assert reading.stable
        assert reading.status == 'ST'

    def test_trailing_terminator_is_stripped(self):
        assert parse_frame('WTST+   1.25   g\r\n').weight == 1.25

    def test_negative_sign(self):
        reading = parse_frame('WTST-  12.34   g')
        assert reading.weight == -12.34
        assert reading.stable

    def test_unstable_status_is_not_stable(self):
        reading = parse_frame('WTUS+  12.34   g')
        assert reading.status == 'US'
        assert not reading.stable

    def test_unknown_status_is_not_assumed_stable(self):
        # Only ST is confirmed, so anything else must read as unsettled
        # rather than "not US, therefore stable".
        reading = parse_frame('WTZZ+   5.00   g')
        assert reading.status == 'ZZ'
        assert not reading.stable

    def test_overload_status(self):
        reading = parse_frame('WTOL+   0.00   g')
        assert reading.overload

    def test_kg_unit_is_reported_verbatim(self):
        reading = parse_frame('WTST+   1.50  kg')
        assert reading.unit == 'kg'
        assert reading.weight == 1.5

    def test_frame_without_unit(self):
        reading = parse_frame('WTST+   7.00')
        assert reading.framed
        assert reading.unit == ''
        assert reading.weight == 7.0

    def test_integer_value(self):
        assert parse_frame('WTST+     42   g').weight == 42.0

    def test_fallback_pulls_a_number_out_of_an_unknown_line(self):
        reading = parse_frame('ST,GS,+  103.5 g')
        assert not reading.framed
        assert reading.weight == 103.5
        # A fallback line carries no status/unit information, and must not
        # pretend otherwise.
        assert reading.unit == ''
        assert reading.status == ''
        assert not reading.stable

    def test_fallback_honours_a_custom_regex(self):
        import re
        reading = parse_frame('net=-8.75kg gross=100', re.compile(r'-?\d+\.\d+'))
        assert reading.weight == -8.75

    @pytest.mark.parametrize('line', ['', '   ', '\r\n', 'no digits at all', 'ERR'])
    def test_lines_with_no_number_return_none(self, line):
        assert parse_frame(line) is None


class TestUnitConversion:

    @pytest.mark.parametrize('unit,expected', [
        ('g', 2.5), ('kg', 2500.0), ('mg', 0.0025), ('ct', 0.5),
    ])
    def test_known_units_convert_to_grams(self, unit, expected):
        reading = parse_frame(f'WTST+   2.50  {unit}')
        assert reading.weight_grams == pytest.approx(expected)

    def test_unit_matching_is_case_insensitive(self):
        assert parse_frame('WTST+   1.00  KG').weight_grams == pytest.approx(1000.0)

    def test_unknown_unit_is_nan_rather_than_a_guess(self):
        assert math.isnan(parse_frame('WTST+   1.00  qq').weight_grams)

    def test_fallback_reading_has_no_unit_so_no_grams(self):
        assert math.isnan(parse_frame('just 5.0 here').weight_grams)


class TestDecodeEscapes:

    def test_empty_string_is_no_bytes(self):
        assert decode_escapes('') == b''

    def test_backslash_escapes(self):
        assert decode_escapes('T\\r\\n') == b'T\r\n'

    def test_hex_prefix(self):
        assert decode_escapes('hex:05') == b'\x05'

    def test_hex_prefix_ignores_spacing(self):
        assert decode_escapes('hex:1B 40') == b'\x1b@'

    def test_odd_length_hex_is_rejected(self):
        with pytest.raises(ValueError):
            decode_escapes('hex:5')

    def test_high_bytes_survive_the_round_trip(self):
        assert decode_escapes('hex:ff') == b'\xff'


class TestEncodePrintable:

    def test_printable_ascii_is_preserved(self):
        assert encode_printable(b'WTST+') == 'WTST+'

    def test_control_bytes_become_periods(self):
        assert encode_printable(b'A\r\n\x00\xffB') == 'A....B'


class TestLineAssembler:

    def test_frame_split_across_two_reads(self):
        assembler = LineAssembler()
        assert assembler.feed(b'WTST+   0.00   g\r\nWTST+   1') == [b'WTST+   0.00   g']
        assert assembler.feed(b'.00   g\r\n') == [b'WTST+   1.00   g']

    def test_several_frames_in_one_read(self):
        assembler = LineAssembler()
        assert len(assembler.feed(b'A\r\nB\r\nC\r\n')) == 3

    def test_partial_data_is_withheld_until_terminated(self):
        assembler = LineAssembler()
        assert assembler.feed(b'WTST+   0.00   g') == []

    def test_custom_terminator(self):
        assembler = LineAssembler(b'\n')
        assert assembler.feed(b'a\nb\n') == [b'a', b'b']

    def test_multibyte_terminator_is_not_left_in_the_line(self):
        assembler = LineAssembler(b'\r\n')
        assert assembler.feed(b'x\r\n')[0] == b'x'

    def test_reset_discards_the_partial_line(self):
        assembler = LineAssembler()
        assembler.feed(b'garbage')
        assembler.reset()
        assert assembler.feed(b'ok\r\n') == [b'ok']

    def test_unterminated_flood_is_dropped_rather_than_buffered_forever(self):
        assembler = LineAssembler()
        assembler.feed(b'x' * (LineAssembler.MAX_BUFFER + 1))
        assert assembler.dropped_bytes > 0
        assert assembler.feed(b'ok\r\n') == [b'ok']

    def test_empty_terminator_is_rejected(self):
        with pytest.raises(ValueError):
            LineAssembler(b'')
