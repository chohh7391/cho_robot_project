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

"""teleop_bridge's xr_controller input: Isaac Capture's controller_data, decoded or refused, never guessed."""

import importlib.util
import math

import numpy as np
import pytest

# A skip mark, not a module-level importorskip: under this workspace's pytest that
# skipped every other test module of the session along with this one.
HAVE_MSGPACK = importlib.util.find_spec('msgpack') is not None
pytestmark = pytest.mark.skipif(not HAVE_MSGPACK, reason='python3-msgpack (a declared exec_depend) is not installed')
if HAVE_MSGPACK:
    import msgpack
    from cho_control_tools.teleop_bridge.fake_xr import payload, to_message
    from cho_control_tools.teleop_bridge.inputs import xr_controller


LEFT_POSE = ((-0.2, 1.1, -0.4), (0.0, 0.0, 0.0, 1.0), 0.8, 0.3)


def frame(data, **kwargs):
    return xr_controller.decode(to_message(data).data, **kwargs)


def test_a_teleop_ros2_message_decodes_both_hands_and_the_buttons():
    data = payload({'left': LEFT_POSE}, {'right_primary_click'}, 123)
    decoded = frame(data)
    left = decoded.hands['left']
    assert left.valid
    np.testing.assert_allclose(left.position, LEFT_POSE[0])
    np.testing.assert_allclose(left.orientation, LEFT_POSE[1])
    assert left.squeeze == pytest.approx(0.8)
    assert left.trigger == pytest.approx(0.3)
    assert decoded.hands['right'].valid
    assert decoded.buttons['right_primary_click']
    assert not decoded.buttons['right_secondary_click']
    assert set(decoded.buttons) == set(xr_controller.button_names())
    assert decoded.stamp_ns == 123


def test_the_aim_pose_is_read_when_asked():
    data = payload({'left': LEFT_POSE}, set(), 0)
    data['left_aim_position'] = [1.0, 2.0, 3.0]
    np.testing.assert_allclose(frame(data, pose_source='aim').hands['left'].position, (1.0, 2.0, 3.0))
    np.testing.assert_allclose(frame(data).hands['left'].position, LEFT_POSE[0])


def test_an_inactive_hand_has_no_pose_nothing_pressed_and_no_buttons():
    data = payload({'left': LEFT_POSE}, {'left_primary_click'}, 0)
    data['left_is_active'] = False
    decoded = frame(data)
    assert not decoded.hands['left'].valid
    assert decoded.hands['left'].squeeze == 0.0
    assert not decoded.buttons['left_primary_click']


@pytest.mark.parametrize('key,value', [
    ('left_grip_orientation', [0.0, 0.0, 0.0, 2.0]),
    ('left_grip_orientation', [0.0, 0.0, 1.0]),
    ('left_grip_position', [0.0, math.nan, 0.0]),
    ('left_grip_position', 'here'),
    ('left_grip_position', None),
])
def test_a_malformed_pose_leaves_the_hand_invalid_but_keeps_its_squeeze(key, value):
    data = payload({'left': LEFT_POSE}, set(), 0)
    data[key] = value
    hand = frame(data).hands['left']
    assert not hand.valid
    assert hand.squeeze == pytest.approx(0.8)


def test_a_slightly_off_unit_quaternion_is_normalised():
    data = payload({'left': ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.005), 0.0, 0.0)}, set(), 0)
    np.testing.assert_allclose(frame(data).hands['left'].orientation, (0.0, 0.0, 0.0, 1.0))


@pytest.mark.parametrize('value,expected', [(math.inf, 0.0), (-0.5, 0.0), (1.7, 1.0), ('x', 0.0), (True, 0.0)])
def test_values_are_clipped_or_read_as_released(value, expected):
    data = payload({'left': LEFT_POSE}, set(), 0)
    data['left_squeeze_value'] = value
    assert frame(data).hands['left'].squeeze == expected


# A msgpack array [1, 2, 3], a byte msgpack never uses, nothing.
@pytest.mark.parametrize('raw', [b'\x93\x01\x02\x03', b'\xc1\xc1', b''])
def test_a_payload_that_is_not_a_msgpack_map_is_refused(raw):
    with pytest.raises(ValueError):
        xr_controller.decode(raw)


def test_payload_bytes_takes_rclpy_bytes_ints_and_raw_bytes():
    assert xr_controller.payload_bytes([b'\x01', b'\xff']) == b'\x01\xff'
    assert xr_controller.payload_bytes([1, -1]) == b'\x01\xff'
    assert xr_controller.payload_bytes(b'\x01') == b'\x01'


def test_an_unknown_pose_source_is_refused():
    with pytest.raises(ValueError):
        xr_controller.decode(msgpack.packb({}), pose_source='palm')
