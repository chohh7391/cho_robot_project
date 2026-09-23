"""
Decode a ``sensor_msgs/Image`` payload into a 2-D greyscale array.

No ROS and no OpenCV. The detector needs one thing from an image -- a plane of
intensities -- and getting it with numpy alone keeps this package's dependency
list to numpy, keeps the whole decode path testable from a bytes object, and
avoids pulling cv_bridge in for a crop and a subtraction.

The one thing worth being careful about is ``step``. A publisher is free to pad
each row, and several do; reshaping by ``width`` instead of by ``step`` then
shears the image by a pixel per row, which looks like a faint diagonal texture
and quietly raises the noise floor of every frame difference taken afterwards.
"""
from __future__ import annotations

import numpy as np

#: Luma weights. Any fixed set would do for a frame difference, but these match
#: what every other tool will show the operator when they look at the stream.
_R, _G, _B = 0.299, 0.587, 0.114

_BGR_LIKE = {'bgr8': 3, 'bgra8': 4}
_RGB_LIKE = {'rgb8': 3, 'rgba8': 4}


class UnsupportedEncoding(ValueError):
    """Raised for an encoding this module cannot turn into intensities."""


def to_gray(encoding: str, height: int, width: int, step: int, data: bytes) -> np.ndarray:
    """
    Return an ``(height, width)`` uint8 intensity array.

    Raises :class:`UnsupportedEncoding` rather than guessing. A wrong guess here
    does not fail loudly later -- it produces a plausible-looking image with the
    channels transposed, and the detector then works slightly worse for reasons
    nobody finds.
    """
    if height <= 0 or width <= 0:
        raise UnsupportedEncoding(f'image has no extent: {width}x{height}')

    raw = np.frombuffer(data, dtype=np.uint8)
    encoding = encoding.lower()

    if encoding in ('mono8', '8uc1'):
        return _rows(raw, height, width, step, 1)[:, :, 0]

    if encoding in ('mono16', '16uc1'):
        rows = _two_byte_rows(raw, height, width, step, 'mono16')
        # Little-endian is what every driver in this workspace publishes; the
        # high byte alone is plenty for a frame difference.
        return np.ascontiguousarray(rows.reshape(height, width, 2)[:, :, 1])

    if encoding in _BGR_LIKE:
        planes = _rows(raw, height, width, step, _BGR_LIKE[encoding])
        return _luma(planes[:, :, 2], planes[:, :, 1], planes[:, :, 0])

    if encoding in _RGB_LIKE:
        planes = _rows(raw, height, width, step, _RGB_LIKE[encoding])
        return _luma(planes[:, :, 0], planes[:, :, 1], planes[:, :, 2])

    if encoding in ('yuv422_yuy2', 'yuyv'):
        # Y is every other byte, which is exactly the plane wanted here.
        rows = _two_byte_rows(raw, height, width, step, 'yuy2')
        return np.ascontiguousarray(rows[:, 0::2])

    raise UnsupportedEncoding(
        f'cannot make intensities from encoding {encoding!r}. Mono is what this wants: on the '
        'OAK-D Pro W publish the global-shutter mono pair, which is the right sensor for a '
        'falling stream anyway.')


def _two_byte_rows(raw: np.ndarray, height: int, width: int, step: int, label: str) -> np.ndarray:
    """Rows of a two-bytes-per-pixel encoding, padding stripped."""
    needed = width * 2
    if step < needed:
        raise UnsupportedEncoding(f'step {step} is too small for {label} {width} px')
    if raw.size < height * step:
        raise UnsupportedEncoding(
            f'image data is {raw.size} bytes, short of the {height * step} its header describes')
    return raw[:height * step].reshape(height, step)[:, :needed]


def _rows(raw: np.ndarray, height: int, width: int, step: int, channels: int) -> np.ndarray:
    needed = width * channels
    if step < needed:
        raise UnsupportedEncoding(f'step {step} is too small for {width} px x {channels} channels')
    if raw.size < height * step:
        raise UnsupportedEncoding(
            f'image data is {raw.size} bytes, short of the {height * step} its header describes')
    return raw[:height * step].reshape(height, step)[:, :needed].reshape(height, width, channels)


def _luma(r: np.ndarray, g: np.ndarray, b: np.ndarray) -> np.ndarray:
    out = _R * r.astype(np.float32) + _G * g.astype(np.float32) + _B * b.astype(np.float32)
    return out.astype(np.uint8)
