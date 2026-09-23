"""Decoding a sensor_msgs/Image payload without cv_bridge."""
import numpy as np
import pytest

from cho_pour_stream.image import UnsupportedEncoding, to_gray


def test_mono8_respects_row_padding():
    # A publisher is free to pad rows. Reshaping by width instead of step shears
    # the image a pixel per row, which never fails -- it just quietly raises the
    # noise floor of every frame difference taken afterwards.
    height, width, step = 4, 3, 5
    data = bytes(range(height * step))
    gray = to_gray('mono8', height, width, step, data)
    assert gray.shape == (height, width)
    assert gray[1].tolist() == [5, 6, 7]


def test_rgb_and_bgr_are_not_the_same_picture():
    pixels = bytes([255, 0, 0] * 4)
    rgb = to_gray('rgb8', 2, 2, 6, pixels)
    bgr = to_gray('bgr8', 2, 2, 6, pixels)
    assert rgb[0, 0] != bgr[0, 0]


def test_mono16_keeps_the_high_byte():
    data = bytes([0x00, 0x7F] * 4)
    gray = to_gray('mono16', 2, 2, 4, data)
    assert np.all(gray == 0x7F)


def test_mono16_respects_row_padding():
    # 2 px of mono16 is 4 bytes; this publisher pads each row to 6.
    data = bytes([0x00, 0x11, 0x00, 0x22, 0xFF, 0xFF] * 2)
    gray = to_gray('mono16', 2, 2, 6, data)
    assert gray[0].tolist() == [0x11, 0x22]


def test_mono16_short_data_is_refused():
    with pytest.raises(UnsupportedEncoding):
        to_gray('mono16', 4, 4, 8, bytes(8))


def test_yuy2_takes_the_luma_plane():
    data = bytes([90, 128, 200, 128] * 2)
    gray = to_gray('yuv422_yuy2', 2, 2, 4, data)
    assert gray[0].tolist() == [90, 200]


def test_an_unknown_encoding_is_refused_rather_than_guessed():
    with pytest.raises(UnsupportedEncoding):
        to_gray('32FC1', 2, 2, 16, bytes(32))


def test_short_data_is_refused():
    with pytest.raises(UnsupportedEncoding):
        to_gray('mono8', 100, 100, 100, bytes(10))
