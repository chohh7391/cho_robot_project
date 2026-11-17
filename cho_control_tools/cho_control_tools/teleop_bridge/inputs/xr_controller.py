# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The XR motion controllers (Meta Quest 3, Pico 4 Ultra) as Isaac Capture publishes them.

NVIDIA's reference ROS 2 publisher, ``examples/teleop_ros2`` of
github.com/NVIDIA/IsaacCapture at tag v1.4.145 run with ``mode:=controller_raw``,
publishes ``xr_teleop/controller_data``: a ``std_msgs/ByteMultiArray`` holding
one msgpack map, one byte per element (its ``messages.py``,
``build_controller_msg``). The keys read here, each once per side ``left_`` /
``right_``:

- ``grip_position`` / ``aim_position`` [m] and ``grip_orientation`` /
  ``aim_orientation`` (x, y, z, w), in the XR world frame (OpenXR stage: x right,
  y up, z towards the operator), not the robot's;
- ``squeeze_value`` and ``trigger_value`` in [0, 1];
- ``primary_click``, ``secondary_click``, ``thumbstick_click``, ``menu_click``,
  0 or 1 (on a Quest: A / B on the right, X / Y on the left);
- ``is_active``, false while the controller is not connected.

A side is valid only when it is active and its pose is finite with a unit
quaternion; anything else leaves it without a pose, never guessed. The payload
carries no OpenXR pose-validity bits, only ``is_active``, so a connected
controller that lost tracking can still report a stale pose: the clutch's
jump guard is what catches that.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import msgpack
import numpy as np


SIDES = ('left', 'right')
BUTTONS = ('primary_click', 'secondary_click', 'thumbstick_click', 'menu_click')
POSE_SOURCES = ('grip', 'aim')
# How far from 1 a quaternion's norm may be before the pose is refused, not normalised.
UNIT_QUATERNION_TOLERANCE = 1e-2


@dataclass(frozen=True)
class HandSample:
    """One controller in one frame; ``position`` and ``orientation`` are None when it has no valid pose."""

    position: np.ndarray | None = None
    orientation: np.ndarray | None = None
    squeeze: float = 0.0
    trigger: float = 0.0

    @property
    def valid(self) -> bool:
        return self.position is not None and self.orientation is not None


@dataclass(frozen=True)
class InputFrame:
    """One decoded message: ``hands`` by side, ``buttons`` by ``<side>_<button>``."""

    hands: dict = field(default_factory=dict)
    buttons: dict = field(default_factory=dict)
    stamp_ns: int = 0


def button_names():
    """Every button name a frame carries, as the bridge's config names them."""
    return tuple(f'{side}_{button}' for side in SIDES for button in BUTTONS)


def payload_bytes(data) -> bytes:
    """The bytes of a ``ByteMultiArray.data``: rclpy gives one ``bytes`` per element."""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    parts = list(data)
    if parts and isinstance(parts[0], int):
        return bytes(value & 0xFF for value in parts)
    return b''.join(parts)


def decode(data, pose_source='grip', button_threshold=0.5) -> InputFrame:
    """Decode one ``controller_data`` message's ``data``.

    Raises ValueError when the payload is not a msgpack map; a side with a
    missing or malformed key is only left invalid.
    """
    if pose_source not in POSE_SOURCES:
        raise ValueError(f"pose_source '{pose_source}' is not one of {list(POSE_SOURCES)}")
    try:
        payload = msgpack.unpackb(payload_bytes(data), raw=False, strict_map_key=False)
    # msgpack's unpack errors (ExtraData, FormatError, StackError, truncated input)
    # are all ValueErrors; an unhashable map key is a TypeError.
    except (ValueError, TypeError) as exc:
        raise ValueError(f'controller_data is not msgpack: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError(f'controller_data is a msgpack {type(payload).__name__}, not a map')
    hands = {side: _hand(payload, side, pose_source) for side in SIDES}
    buttons = {}
    for side in SIDES:
        active = payload.get(f'{side}_is_active') is True
        for button in BUTTONS:
            name = f'{side}_{button}'
            buttons[name] = active and _fraction(payload.get(name)) >= button_threshold
    stamp = payload.get('timestamp')
    return InputFrame(hands, buttons, stamp if isinstance(stamp, int) and not isinstance(stamp, bool) else 0)


def _hand(payload, side, pose_source):
    if payload.get(f'{side}_is_active') is not True:
        return HandSample()
    squeeze = _fraction(payload.get(f'{side}_squeeze_value'))
    trigger = _fraction(payload.get(f'{side}_trigger_value'))
    position = _vector(payload.get(f'{side}_{pose_source}_position'), 3)
    orientation = _vector(payload.get(f'{side}_{pose_source}_orientation'), 4)
    if position is None or orientation is None:
        return HandSample(squeeze=squeeze, trigger=trigger)
    norm = float(np.linalg.norm(orientation))
    if abs(norm - 1.0) > UNIT_QUATERNION_TOLERANCE:
        return HandSample(squeeze=squeeze, trigger=trigger)
    return HandSample(position, orientation / norm, squeeze, trigger)


def _fraction(value):
    """A finite number clipped to [0, 1]; anything else is 0 (not pressed)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return 0.0
    return min(max(float(value), 0.0), 1.0)


def _vector(value, size):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        return None
    if not all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value):
        return None
    vector = np.asarray(value, dtype=float)
    return vector if np.all(np.isfinite(vector)) else None
