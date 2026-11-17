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

"""teleop_bridge's clutch: engaging never moves the arm, and the hand's motion maps through xr_to_base."""

from dataclasses import dataclass

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cho_control_tools.teleop_bridge.clutch import Clutch, ClutchConfig


# The default mapping: XR forward (-z) -> +x, XR left (-x) -> +y, XR up (+y) -> +z.
XR_TO_BASE = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
HOME_POSITION = np.array([0.4, 0.1, 0.5])
HOME_QUATERNION = Rotation.from_euler('x', 180, degrees=True).as_quat()
IDENTITY = (0.0, 0.0, 0.0, 1.0)


@dataclass
class Hand:
    position: tuple = (0.2, 1.0, -0.5)
    orientation: tuple = IDENTITY
    squeeze: float = 1.0
    trigger: float = 0.0

    @property
    def valid(self):
        return self.position is not None and self.orientation is not None


def seeded(**config):
    clutch = Clutch(ClutchConfig(xr_to_base=XR_TO_BASE, **config))
    clutch.seed(HOME_POSITION, HOME_QUATERNION)
    # A press held through the seed has to be released first: start released.
    clutch.update(Hand(squeeze=0.0), enabled=True)
    return clutch


def moved(position, delta):
    return tuple(np.asarray(position) + np.asarray(delta))


def same_rotation(first, second):
    return Rotation.from_quat(first).inv() * Rotation.from_quat(second)


def assert_target(target, position, quaternion=HOME_QUATERNION):
    assert target is not None
    np.testing.assert_allclose(target[0], position, atol=1e-12)
    assert same_rotation(target[1], quaternion).magnitude() < 1e-9


def test_nothing_is_held_before_a_seed():
    clutch = Clutch(ClutchConfig(xr_to_base=XR_TO_BASE))
    assert clutch.update(Hand(), enabled=True) is None
    assert not clutch.engaged


def test_a_seeded_clutch_holds_the_seed_while_released():
    clutch = seeded()
    assert_target(clutch.update(Hand(squeeze=0.0, position=(5.0, 5.0, 5.0)), enabled=True), HOME_POSITION)


def test_engaging_does_not_move_the_target_wherever_the_hand_is():
    clutch = seeded()
    target = clutch.update(Hand(position=(3.0, -2.0, 7.0), orientation=Rotation.from_euler('y', 40,
                                degrees=True).as_quat()), enabled=True)
    assert clutch.engaged
    assert_target(target, HOME_POSITION)


@pytest.mark.parametrize('xr_delta,base_delta', [
    ((0.0, 0.0, -0.05), (0.05, 0.0, 0.0)),   # forward
    ((-0.05, 0.0, 0.0), (0.0, 0.05, 0.0)),   # left
    ((0.0, 0.05, 0.0), (0.0, 0.0, 0.05)),    # up
])
def test_hand_motion_maps_through_xr_to_base(xr_delta, base_delta):
    clutch = seeded()
    start = Hand()
    clutch.update(start, enabled=True)
    target = clutch.update(Hand(position=moved(start.position, xr_delta)), enabled=True)
    assert_target(target, HOME_POSITION + np.asarray(base_delta))


def test_position_scale_scales_the_motion():
    clutch = seeded(position_scale=0.5)
    start = Hand()
    clutch.update(start, enabled=True)
    target = clutch.update(Hand(position=moved(start.position, (0.0, 0.0, -0.08))), enabled=True)
    assert_target(target, HOME_POSITION + np.array([0.04, 0.0, 0.0]))


def test_turning_the_hand_about_xr_up_turns_the_arm_about_base_z():
    clutch = seeded()
    clutch.update(Hand(), enabled=True)
    turned = Rotation.from_euler('y', 20, degrees=True).as_quat()
    target = clutch.update(Hand(orientation=turned), enabled=True)
    expected = (Rotation.from_euler('z', 20, degrees=True) * Rotation.from_quat(HOME_QUATERNION)).as_quat()
    assert_target(target, HOME_POSITION, expected)


def test_without_follow_rotation_the_orientation_stays():
    clutch = seeded(follow_rotation=False)
    clutch.update(Hand(), enabled=True)
    target = clutch.update(Hand(orientation=Rotation.from_euler('y', 20, degrees=True).as_quat()), enabled=True)
    assert_target(target, HOME_POSITION)


def test_released_the_target_stays_and_engaging_again_does_not_jump():
    clutch = seeded()
    start = Hand()
    clutch.update(start, enabled=True)
    clutch.update(Hand(position=moved(start.position, (0.0, 0.0, -0.05))), enabled=True)
    after_first = HOME_POSITION + np.array([0.05, 0.0, 0.0])
    # Release and carry the hand somewhere else: the arm stays.
    assert_target(clutch.update(Hand(squeeze=0.0, position=(1.0, 2.0, 3.0)), enabled=True), after_first)
    assert not clutch.engaged
    # Engage again there: no jump, then the motion continues from where the arm is.
    assert_target(clutch.update(Hand(position=(1.0, 2.0, 3.0)), enabled=True), after_first)
    target = clutch.update(Hand(position=(1.0, 2.05, 3.0)), enabled=True)
    assert_target(target, after_first + np.array([0.0, 0.0, 0.05]))


def test_a_press_held_through_the_seed_must_be_released_first():
    clutch = Clutch(ClutchConfig(xr_to_base=XR_TO_BASE))
    clutch.seed(HOME_POSITION, HOME_QUATERNION)
    clutch.update(Hand(), enabled=True)
    assert not clutch.engaged
    clutch.update(Hand(squeeze=0.0), enabled=True)
    clutch.update(Hand(), enabled=True)
    assert clutch.engaged


def test_the_squeeze_has_hysteresis():
    clutch = seeded(engage_threshold=0.6, release_threshold=0.4)
    clutch.update(Hand(squeeze=0.5), enabled=True)
    assert not clutch.engaged
    clutch.update(Hand(squeeze=0.0), enabled=True)
    clutch.update(Hand(squeeze=0.7), enabled=True)
    assert clutch.engaged
    clutch.update(Hand(squeeze=0.5), enabled=True)
    assert clutch.engaged
    clutch.update(Hand(squeeze=0.3), enabled=True)
    assert not clutch.engaged


@pytest.mark.parametrize('lost', [
    Hand(position=None),
    None,
])
def test_a_hand_without_a_pose_lets_go_and_needs_a_new_press(lost):
    clutch = seeded()
    start = Hand()
    clutch.update(start, enabled=True)
    clutch.update(Hand(position=moved(start.position, (0.0, 0.05, 0.0))), enabled=True)
    held = HOME_POSITION + np.array([0.0, 0.0, 0.05])
    assert_target(clutch.update(lost, enabled=True), held)
    assert not clutch.engaged
    if lost is not None:
        # The squeeze never let go: tracking coming back does not re-engage.
        clutch.update(Hand(position=(9.0, 9.0, 9.0)), enabled=True)
        assert not clutch.engaged


def test_disabled_lets_go():
    clutch = seeded()
    clutch.update(Hand(), enabled=True)
    clutch.update(Hand(), enabled=False)
    assert not clutch.engaged
    clutch.update(Hand(), enabled=True)
    assert not clutch.engaged


def test_a_jump_lets_go_and_holds_the_last_target():
    clutch = seeded(max_input_step=0.1)
    start = Hand()
    clutch.update(start, enabled=True)
    clutch.update(Hand(position=moved(start.position, (0.0, 0.0, -0.05))), enabled=True)
    held = HOME_POSITION + np.array([0.05, 0.0, 0.0])
    target = clutch.update(Hand(position=moved(start.position, (0.0, 0.0, -0.3))), enabled=True)
    assert not clutch.engaged
    assert_target(target, held)
    clutch.update(Hand(position=moved(start.position, (0.0, 0.0, -0.3))), enabled=True)
    assert not clutch.engaged


def test_a_turn_jump_lets_go():
    clutch = seeded(max_input_rotation_step=0.5)
    clutch.update(Hand(), enabled=True)
    clutch.update(Hand(orientation=Rotation.from_euler('x', 60, degrees=True).as_quat()), enabled=True)
    assert not clutch.engaged


def test_the_workspace_clamps_the_target():
    clutch = seeded(workspace_min=(0.0, -1.0, 0.0), workspace_max=(0.45, 1.0, 1.0))
    start = Hand()
    clutch.update(start, enabled=True)
    target = clutch.update(Hand(position=moved(start.position, (0.0, 0.0, -0.09))), enabled=True)
    assert_target(target, (0.45, 0.1, 0.5))


def test_reset_forgets_the_target():
    clutch = seeded()
    clutch.reset()
    assert clutch.target is None


@pytest.mark.parametrize('config', [
    {'xr_to_base': np.eye(3) * 2.0},
    {'xr_to_base': np.diag([1.0, 1.0, -1.0])},
    {'xr_to_base': XR_TO_BASE, 'engage_threshold': 0.3, 'release_threshold': 0.4},
    {'xr_to_base': XR_TO_BASE, 'position_scale': 0.0},
    {'xr_to_base': XR_TO_BASE, 'workspace_min': (0.0, 0.0, 0.0)},
    {'xr_to_base': XR_TO_BASE, 'workspace_min': (1.0, 0.0, 0.0), 'workspace_max': (0.0, 1.0, 1.0)},
])
def test_bad_configs_are_refused(config):
    with pytest.raises(ValueError):
        ClutchConfig(**config)
